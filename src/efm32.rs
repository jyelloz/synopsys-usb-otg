use core::marker::PhantomData;
use core::ops::Deref;
use core::cell::Cell;

use cortex_m::asm::bkpt;
use cortex_m_semihosting::hprintln;

use usb_device::{
    Result,
    UsbDirection::{
        self,
        *,
    },
    UsbError,
    bus::PollResult,
    endpoint::{EndpointType, EndpointAddress},
};

use crate::{
    UsbPeripheral,
    endpoint::{EndpointIn, Endpoint},
    ral::{
        read_reg,
        write_reg,
        modify_reg,
        otg_global,
        otg_device,
        otg_pwrclk,
        endpoint0_out,
        endpoint_in,
        endpoint_out,
    },
    target::{
        UsbRegisters,
        interrupt::{self, Mutex, CriticalSection},
    },
    transition::EndpointDescriptor,
};

struct HardwiredEndpointAllocator<P> {
    allocated: u8,
    _marker: PhantomData<P>,
}

impl <P: UsbPeripheral> HardwiredEndpointAllocator<P> {

    fn new() -> Self {
        Self {
            allocated: 0,
            _marker: PhantomData,
        }
    }

    fn allocate(
        &mut self,
        ep_type: EndpointType,
        _max_packet_size: u16,
        _interval: u8,
    ) -> Result<usize> {
        if ep_type != EndpointType::Control {
            return Err(UsbError::Unsupported);
        }
        if self.allocated as usize >= P::ENDPOINT_COUNT {
            Err(UsbError::EndpointOverflow)
        } else {
            self.allocated += 1;
            Ok(self.allocated as usize)
        }
    }

    fn deconfigure_all(&self, regs: &UsbRegisters) {
        for i in 0..self.allocated {
        }
    }

}

pub struct USB<P> {
    allocator_in: HardwiredEndpointAllocator<P>,
    allocator_out: HardwiredEndpointAllocator<P>,
    ctrl_out: EndpointOut,
    ctrl_in: EndpointIn,
    regs: Mutex<UsbRegisters>,
}

impl <P: UsbPeripheral> USB<P> {

    pub fn new() -> Self {
        let out_descriptor = EndpointDescriptor {
            address: EndpointAddress::from_parts(0, Out),
            ep_type: EndpointType::Control,
            max_packet_size: 64,
            interval: 0,
        };
        let in_descriptor = EndpointDescriptor {
            address: EndpointAddress::from_parts(0, In),
            ..out_descriptor
        };
        Self {
            allocator_in: HardwiredEndpointAllocator::new(),
            allocator_out: HardwiredEndpointAllocator::new(),
            ctrl_out: EndpointOut::new::<P>(out_descriptor),
            ctrl_in: EndpointIn::new::<P>(in_descriptor),
            regs: Mutex::new(UsbRegisters::new::<P>()),
        }
    }

    fn enable(&self, cs: &CriticalSection) {
        let regs = self.regs.borrow(cs);

        // Wait for AHB ready
        while read_reg!(otg_global, regs.global(), GRSTCTL, AHBIDL) == 0 {}

        // Configure OTG as device
        modify_reg!(otg_global, regs.global(), GUSBCFG,
            SRPCAP: 0, // SRP capability is not enabled
            FDMOD: 1 // Force device mode
        );

        // Perform core soft-reset
        while read_reg!(otg_global, regs.global(), GRSTCTL, AHBIDL) == 0 {}
        modify_reg!(otg_global, regs.global(), GRSTCTL, CSRST: 1);
        while read_reg!(otg_global, regs.global(), GRSTCTL, CSRST) == 1 {}

        modify_reg!(otg_global, regs.global(), GUSBCFG, TRDT: 5);

        // Enable PHY clock
        write_reg!(otg_pwrclk, regs.pwrclk(), PCGCCTL, 0);

        // Soft disconnect device
        modify_reg!(otg_device, regs.device(), DCTL, SDIS: 1);

        // Setup USB speed and frame interval
        modify_reg!(otg_device, regs.device(), DCFG,
            PFIVL: 0,
            DSPD: 3
        );

        // unmask EP interrupts
        write_reg!(otg_device, regs.device(), DIEPMSK, XFRCM: 1);

        // unmask core interrupts
        write_reg!(otg_global, regs.global(), GINTMSK,
            USBRST: 1,
            ENUMDNEM: 1,
            USBSUSPM: 1,
            WUIM: 1,
            IEPINT: 1,
            RXFLVLM: 1
        );

        // clear pending interrupts
        write_reg!(otg_global, regs.global(), GINTSTS, 0xffffffff);

        // unmask global interrupt
        modify_reg!(otg_global, regs.global(), GAHBCFG, GINT: 1);

        // connect(true)
        modify_reg!(otg_device, regs.device(), DCTL, SDIS: 0);

    }

    fn set_device_address(&self, regs: &UsbRegisters, addr: u32) {
        if addr != 0 {
            bkpt();
        }
        modify_reg!(otg_device, regs.device(), DCFG, DAD: addr);
    }

    fn poll(&self, cs: &CriticalSection) -> PollResult {

        let regs = self.regs.borrow(cs);
        let (wakeup, suspend, enum_done, reset, iep, rxflvl) = read_reg!(
            otg_global,
            regs.global(),
            GINTSTS,
            WKUPINT, USBSUSP, ENUMDNE, USBRST, IEPINT, RXFLVL
        );

        if reset != 0 {
            write_reg!(otg_global, regs.global(), GINTSTS, USBRST: 1);
            self.handle_reset(regs);
            return PollResult::None;
        }

        if enum_done != 0 {
            write_reg!(otg_global, regs.global(), GINTSTS, ENUMDNE: 1);
            self.handle_enum(regs);
            return PollResult::Reset;
        }
        if wakeup != 0 {
            write_reg!(otg_global, regs.global(), GINTSTS, WKUPINT: 1);
            return PollResult::Resume;
        }
        if suspend != 0 {
            write_reg!(otg_global, regs.global(), GINTSTS, USBSUSP: 1);
            return PollResult::Suspend;
        }

        let mut ep_out = 0;
        let mut ep_in_complete = 0;
        let mut ep_setup = 0;

        let (epnum, data_size, status) = read_reg!(
            otg_global,
            regs.global(),
            GRXSTSR,
            EPNUM,
            BCNT,
            PKTSTS
        );
        if rxflvl != 0 {
            match status {
                0x02 => { // OUT received
                    ep_out |= 1 << epnum;
                },
                0x06 => { // SETUP received
                    // flushing TX if something stuck in control endpoint
                    ep_setup |= 1 << epnum;
                },
                0x03 | 0x04 => { // OUT or SETUP completed
                    read_reg!(otg_global, regs.global(), GRXSTSP); // pop GRXSTSP
                    self.ctrl_out.set_bytes_to_read(cs, None);
                    return PollResult::None;
                },
                _ => {
                    read_reg!(otg_global, regs.global(), GRXSTSP); // pop GRXSTSP
                    self.ctrl_out.set_bytes_to_read(cs, None);
                },
            }

            if iep != 0 {
                for ep in 0..3 {
                    let ep_regs = regs.endpoint_in(ep as usize);
                    if read_reg!(endpoint_in, ep_regs, DIEPINT, XFRC) != 0 {
                        write_reg!(endpoint_in, ep_regs, DIEPINT, XFRC: 1);
                        ep_in_complete |= 1 << ep;
                    }
                }
            }

            if (ep_out | ep_in_complete | ep_setup) != 0 {
                return PollResult::Data {
                    ep_out,
                    ep_in_complete,
                    ep_setup,
                }
            }
        }

        PollResult::None
    }

    // 14.4.4.1.1
    fn handle_reset(&self, regs: &UsbRegisters) {

        // 1.
        modify_reg!(otg_device, regs.device(), DOEPCTL0, SNAK: 1);
        modify_reg!(otg_device, regs.device(), DOEPCTL1, SNAK: 1);
        modify_reg!(otg_device, regs.device(), DOEPCTL2, SNAK: 1);
        modify_reg!(otg_device, regs.device(), DOEPCTL3, SNAK: 1);

        // 2.
        modify_reg!(otg_device, regs.device(), DAINTMSK,
            IEPM: (1 << 0),
            OEPM: (1 << 0)
        );
        modify_reg!(otg_device, regs.device(), DOEPMSK,
            XFRCM: 1,
            STUPM: 1
        );
        modify_reg!(otg_device, regs.device(), DIEPMSK,
            XFRCM: 1,
            TOM: 1
        );

        // 3.
        modify_reg!(otg_global, regs.global(), GINTMSK,
            NPTXFEM: 1,
            RXFLVLM: 1
        );

        // 4.
        modify_reg!(otg_global, regs.global(), GRXFSIZ,
            RXFD: 256
        );
        write_reg!(otg_global, regs.global(), DIEPTXF0,
            TX0FD: 64,
            TX0FSA: 512
        );

        // 5.
        modify_reg!(otg_device, regs.device(), DOEPTSIZ0,
            STUPCNT: 3
        );

    }

    // 14.4.4.1.2
    fn handle_enum(&self, regs: &UsbRegisters) {
        modify_reg!(otg_device, regs.device(),
            DIEPCTL0,
            MPSIZ: 0
        );
        modify_reg!(otg_device, regs.device(),
            DOEPCTL0,
            EPENA: 1
        );
    }

    fn reset(&self, regs: &UsbRegisters) {
        self.configure_all(regs);
        self.set_device_address(regs, 0);
    }

    fn configure_all(&self, regs: &UsbRegisters) {
        let rx_fifo_size_words = 256usize;
        write_reg!(otg_global, regs.global(),
            GRXFSIZ,
            rx_fifo_size_words as u32
        );

        let fifo_size = 16usize;
        let fifo_top = rx_fifo_size_words;
        write_reg!(otg_global, regs.global(), DIEPTXF0,
            TX0FD: fifo_size as u32,
            TX0FSA: fifo_top as u32
        );

        // flush
        modify_reg!(otg_global, regs.global(), GRSTCTL,
            RXFFLSH: 1,
            TXFFLSH: 1,
            TXFNUM: 0x10
        );
        while read_reg!(otg_global, regs.global(), GRSTCTL, RXFFLSH, TXFFLSH) != (0, 0) {}

        // enabling EP TX interrupt
        let ctrl_in = &self.ctrl_in;
        modify_reg!(otg_device, regs.device(),
            DAINTMSK,
            |v| v | (0x0001 << ctrl_in.address().index())
        );
        ctrl_in.configure_();

        let ctrl_out = &self.ctrl_out;
        // enable RX interrupt on EP0
        modify_reg!(otg_device, regs.device(), DAINTMSK, |v| v | 0x00010000);
        ctrl_out.configure_();

    }

    fn deconfigure_all(&self, regs: &UsbRegisters) {

        // disable interrupts
        modify_reg!(otg_device, regs.device(),
            DAINTMSK,
            IEPM: 0,
            OEPM: 0
        );

        self.allocator_in.deconfigure_all(regs);
        self.allocator_out.deconfigure_all(regs);

    }

    fn flush_txfifo(&self, epnum: u32, regs: &UsbRegisters) {
        let ep = regs.endpoint_in(epnum as usize);
        if read_reg!(endpoint_in, ep, DIEPTSIZ, PKTCNT) == 0 {
            return;
        }
        modify_reg!(otg_device, regs.device(), DIEPCTL0, SNAK: 1);
        while read_reg!(endpoint_in, ep, DIEPINT, INEPNE) != 0 {}

        while read_reg!(otg_global, regs.global(), GRSTCTL, AHBIDL) != 0 {}

        modify_reg!(otg_global, regs.global(), GRSTCTL,
            TXFNUM: epnum,
            TXFFLSH: 1
        );
        while read_reg!(otg_global, regs.global(), GRSTCTL, TXFFLSH) == 1 {}
    }

}

impl <P: UsbPeripheral> usb_device::bus::UsbBus for USB<P> {

    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        if let Some(_) = ep_addr.filter(|addr| addr.index() == 0) {
            let address = EndpointAddress::from_parts(0, ep_dir);
            match ep_dir {
                In => {
                    let endpoint = &self.ctrl_in;
                    interrupt::free(|cs| endpoint.configure(cs));
                },
                Out => {
                    let endpoint = &self.ctrl_out;
                    interrupt::free(|cs| endpoint.configure(cs));
                },
            };
            Ok(address)
        } else {
            let index = match ep_dir {
                In => self.allocator_in.allocate(ep_type, max_packet_size, interval)?,
                Out => self.allocator_out.allocate(ep_type, max_packet_size, interval)?,
            };
            Ok(EndpointAddress::from_parts(index, ep_dir))
        }
    }

    fn set_device_address(&self, addr: u8) {
        if addr != 0 {
            bkpt();
        }
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            self.set_device_address(regs, addr.into());
        });
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.index() >= P::ENDPOINT_COUNT {
            return;
        }
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            crate::endpoint::set_stalled(*regs, ep_addr, stalled)
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.index() >= P::ENDPOINT_COUNT {
            return true;
        }

        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            crate::endpoint::is_stalled(*regs, ep_addr)
        })
    }

    fn enable(&mut self) {
        interrupt::free(|cs| {
            P::enable();
            USB::enable(self, cs);
        });
    }

    fn suspend(&self) { }

    fn resume(&self) { }

    fn reset(&self) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            self.reset(regs);
        });
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        let index = ep_addr.index();
        if !ep_addr.is_out() || index >= P::ENDPOINT_COUNT {
            return Err(UsbError::InvalidEndpoint);
        }

        if index == 0 {
            let max_packet_size = 64usize;
            let ep = &self.ctrl_out;
            let bytes_to_read = ep.get_bytes_to_read();
            if bytes_to_read.is_none() {
                return Err(UsbError::WouldBlock);
            }
            let bytes_to_read = bytes_to_read.unwrap();
            let len = buf.len()
                .min(max_packet_size as usize)
                .min(bytes_to_read);
            let buf = &mut buf[..len];
            let ep = &self.ctrl_out;
            let result = ep.read(buf)?;
            let remaining_bytes = Some(bytes_to_read - result)
                .filter(|i| *i > 0);
            interrupt::free(|cs| ep.set_bytes_to_read(cs, remaining_bytes));
            Ok(result)
        } else {
            // TODO
            Err(UsbError::InvalidEndpoint)
        }
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        let index = ep_addr.index();
        if !ep_addr.is_in() || index >= P::ENDPOINT_COUNT {
            return Err(UsbError::InvalidEndpoint);
        }

        if index == 0 {
            let len = buf.len();
            self.ctrl_in.write(buf)?;
            Ok(len)
        } else {
            bkpt();
            // TODO
            // Err(UsbError::Unsupported)
            Ok(buf.len())
        }
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| self.poll(cs))
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;

}

struct EndpointOut {
    common: Endpoint,
    bytes_to_read: Mutex<Cell<Option<usize>>>,
}

impl EndpointOut {
    pub fn new<P: UsbPeripheral>(descriptor: EndpointDescriptor) -> Self {
        Self {
            common: Endpoint::new::<P>(descriptor),
            bytes_to_read: Mutex::new(Cell::new(None)),
        }
    }

    pub fn configure(&self, _cs: &CriticalSection) {
        self.configure_();
    }
    fn configure_(&self) {
        if self.index() == 0 {
            let mpsiz = match self.descriptor.max_packet_size {
                8 => 0b11,
                16 => 0b10,
                32 => 0b01,
                64 => 0b00,
                _ => panic!("unsupported max packet size"),
            };

            let regs = self.usb.endpoint0_out();
            write_reg!(endpoint0_out, regs, DOEPTSIZ0,
                STUPCNT: 1,
                PKTCNT: 1,
                XFRSIZ: self.descriptor.max_packet_size as u32
            );
            modify_reg!(endpoint0_out, regs, DOEPCTL0,
                MPSIZ: mpsiz as u32,
                EPENA: 1,
                CNAK: 1
            );
        } else {
            let regs = self.usb.endpoint_out(self.index() as usize);
            write_reg!(endpoint_out, regs, DOEPCTL,
                SD0PID_SEVNFRM: 1,
                CNAK: 1,
                EPENA: 1,
                USBAEP: 1,
                EPTYP: self.descriptor.ep_type as u32,
                MPSIZ: self.descriptor.max_packet_size as u32
            );
        }
    }

    pub fn deconfigure(&self, _cs: &CriticalSection) {
        self.deconfigure_()
    }

    fn deconfigure_(&self) {
        let regs = self.usb.endpoint_out(self.index() as usize);

        // deactivating endpoint
        modify_reg!(endpoint_out, regs, DOEPCTL, USBAEP: 0);

        // disabling endpoint
        if read_reg!(endpoint_out, regs, DOEPCTL, EPENA) != 0 && self.index() != 0 {
            modify_reg!(endpoint_out, regs, DOEPCTL, EPDIS: 1)
        }

        // clean EP interrupts
        write_reg!(endpoint_out, regs, DOEPINT, 0xff);
    }

    fn set_bytes_to_read(&self, cs: &CriticalSection, value: Option<usize>) {
        self.bytes_to_read.borrow(cs).set(value)
    }
    fn get_bytes_to_read(&self) -> Option<usize> {
        interrupt::free(|cs| self.bytes_to_read.borrow(cs).get())
    }

    /// Pops one word off the RxFIFO
    pub fn take(&self) -> [u8; 4] {
        self.usb.fifo(self.index() as usize)
            .read()
            .to_ne_bytes()
    }

    pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
        let bytes_to_read = self.get_bytes_to_read();
        if bytes_to_read.is_none() {
            return Err(UsbError::WouldBlock);
        }
        let bytes_to_read = bytes_to_read.unwrap();
        if buf.len() < bytes_to_read {
            return Err(UsbError::BufferOverflow)
        }
        if bytes_to_read == 0 {
            return Ok(0);
        }
        interrupt::free(|_| {
            for buf in buf.chunks_exact_mut(4).rev() {
                let word = self.take();
                buf.copy_from_slice(&word);
            };
            let unaligned_bytes = buf.chunks_exact_mut(4)
                .into_remainder();
            //XXX: this might need to go first?
            match unaligned_bytes.len() {
                    len @ 1..=3 => {
                        bkpt();
                        let word = self.take();
                        unaligned_bytes.copy_from_slice(&word[..len]);
                    },
                    _ => { },
            }
            Ok(bytes_to_read)
        })
    }

}

impl Deref for EndpointOut {
    type Target = Endpoint;
    fn deref(&self) -> &Self::Target {
        &self.common
    }
}
