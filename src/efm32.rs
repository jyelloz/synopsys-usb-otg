use core::marker::PhantomData;
use core::cell::RefCell;

use cortex_m::asm::bkpt;

use usb_device::{
    Result,
    UsbDirection,
    UsbError,
    bus::PollResult,
    endpoint::{EndpointType, EndpointAddress},
};

use cortex_m_semihosting::hprintln;

use crate::{
    UsbPeripheral,
    ral::{
        read_reg,
        write_reg,
        modify_reg,
        otg_global,
        otg_device,
        otg_pwrclk,
        endpoint_in,
    },
    target::{
        UsbRegisters,
        interrupt::{self, Mutex, CriticalSection},
    },
};

struct SetupPacket {
    buf: [u8; 128],
    length: Option<usize>,
}

impl SetupPacket {

    fn buf(&self) -> Option<&[u8]> {
        if let Some(length) = self.length {
            Some(&self.buf[..length])
        } else {
            None
        }
    }

    fn mut_buf(&mut self) -> &mut [u8] {
        &mut self.buf
    }

    fn update(&mut self, length: Option<usize>) {
        self.length = length;
    }

}

impl Default for SetupPacket {
    fn default() -> Self {
        Self {
            buf: [0; 128],
            length: None,
        }
    }
}

pub struct USB<P> {
    regs: Mutex<UsbRegisters>,
    setup_packet: Mutex<RefCell<SetupPacket>>,
    _marker: PhantomData<P>,
}

impl <P: UsbPeripheral> USB<P> {

    pub fn new() -> Self {
        Self {
            regs: Mutex::new(UsbRegisters::new::<P>()),
            _marker: PhantomData,
            setup_packet: Mutex::new(RefCell::new(Default::default())),
        }
    }

    fn enable(&self, regs: &UsbRegisters) {

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
            // return PollResult::None;
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
                    let mut setup = self.setup_packet.borrow(cs).borrow_mut();
                    let buf = setup.mut_buf();
                    let length = self.read_fifo(regs, buf).ok();
                    setup.update(length);
                    read_reg!(otg_global, regs.global(), GRXSTSP); // pop GRXSTSP
                    modify_reg!(otg_device, regs.device(), DOEPCTL0, CNAK: 1, EPENA: 1);
                    ep_setup |= 1 << epnum;
                },
                0x03 | 0x04 => { // OUT or SETUP completed
                    read_reg!(otg_global, regs.global(), GRXSTSP); // pop GRXSTSP
                },
                _ => {
                    read_reg!(otg_global, regs.global(), GRXSTSP); // pop GRXSTSP
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
            EPENA: 1,
            CNAK: 1
        );
    }

    fn reset(&self, regs: &UsbRegisters) {
        self.configure_all(regs);
        self.set_device_address(regs, 0);
    }

    fn configure_all(&self, _regs: &UsbRegisters) {
    }

    fn _deconfigure_all(&self, _regs: &UsbRegisters) {

    }

    fn write(&self, regs: &UsbRegisters, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        let len = buf.len();
        let index = ep_addr.index() as usize;
        let ep_regs = regs.endpoint_in(index);
        let available_space = read_reg!(endpoint_in, ep_regs,
            DTXFSTS,
            INEPTFSAV
        ) as usize;
        if len > available_space {
            return Err(UsbError::BufferOverflow);
        }
        let packet_count = if len == available_space { 2 } else { 1 };
        modify_reg!(endpoint_in, ep_regs,
            DIEPTSIZ,
            PKTCNT: 1,
            XFRSIZ: len as u32
        );
        modify_reg!(endpoint_in, ep_regs, DIEPCTL, EPENA: 1, CNAK: 1);
        crate::target::fifo_write(*regs, index, buf);
        // bkpt();
        Ok(len)
    }

    fn read_fifo(&self, regs: &UsbRegisters, buf: &mut[u8]) -> Result<usize> {
        let data_size = read_reg!(otg_global, regs.global(), GRXSTSP, BCNT);
        let len = buf.len().min(data_size as usize);
        let buf = &mut buf[..len];
        let fifo = regs.fifo(0);
        for chunk in buf.chunks_exact_mut(4) {
            let word = fifo.read().to_ne_bytes();
            // bkpt();
            chunk.copy_from_slice(&word);
        }
        let remainder = buf.chunks_exact_mut(4)
            .into_remainder();
        if let len @ 1..=3 = remainder.len() {
            bkpt();
            let last_word = fifo.read().to_ne_bytes();
            remainder.copy_from_slice(&last_word[..len]);
        }
        Ok(len)
    }

    fn read(&self, cs: &CriticalSection, _ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        let len = buf.len();
        if len == 0 {
            return Ok(0);
        }
        let mut setup_packet = self.setup_packet.borrow(cs).borrow_mut();
        if let Some(setup) = setup_packet.buf() {
            let setup_len = setup.len();
            if len < setup_len {
                return Err(UsbError::BufferOverflow);
            }
            let buf = &mut buf[..setup_len];
            buf.copy_from_slice(setup);
            setup_packet.update(None);
            Ok(setup_len)
        } else {
            let regs = self.regs.borrow(cs);
            self.read_fifo(regs, buf)
        }
    }

}

impl <P: UsbPeripheral> usb_device::bus::UsbBus for USB<P> {

    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        _ep_addr: Option<EndpointAddress>,
        _ep_type: EndpointType,
        _max_packet_size: u16,
        _interval: u8,
    ) -> Result<EndpointAddress> {
        Ok(EndpointAddress::from_parts(0, ep_dir))
    }

    fn set_device_address(&self, addr: u8) {
        if addr != 0 { bkpt(); }
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            self.set_device_address(regs, addr.into());
        });
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.index() >= P::ENDPOINT_COUNT {
            return;
        }
        // if stalled { bkpt(); }
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
            let regs = self.regs.borrow(cs);
            P::enable();
            USB::enable(self, regs);
        });
    }

    fn suspend(&self) { }

    fn resume(&self) { }

    fn reset(&self) {
        interrupt::free(|cs| self.reset(self.regs.borrow(cs)));
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        interrupt::free(|cs| self.read(cs, ep_addr, buf))
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        interrupt::free(|cs| self.write(self.regs.borrow(cs), ep_addr, buf))
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| self.poll(cs))
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;

}
