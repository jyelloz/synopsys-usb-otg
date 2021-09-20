use core::marker::PhantomData;

use cortex_m::asm::bkpt;

use usb_device::{
    Result,
    UsbDirection,
    UsbError,
    bus::PollResult,
    endpoint::{EndpointType, EndpointAddress},
};

use crate::{
    UsbPeripheral,
    endpoint::{
        EndpointIn,
        EndpointOut,
    },
    endpoint_memory::{
        EndpointBuffer,
        EndpointBufferState,
    },
    ral::{
        read_reg,
        write_reg,
        modify_reg,
        otg_global,
        otg_device,
        otg_pwrclk,
        endpoint_in,
        endpoint_out,
    },
    target::{
        UsbRegisters,
        interrupt::{self, Mutex, CriticalSection},
    },
    transition::EndpointDescriptor,
};

struct EndpointStack<P> {
    endpoints_in: [Option<EndpointIn>; 4],
    endpoints_out: [Option<EndpointOut>; 4],
    input_buffers: [ [u32; 32]; 4],
    _peripheral_marker: PhantomData<P>,
}

impl <P: UsbPeripheral> EndpointStack<P> {

    fn new() -> Self {
        Self {
            endpoints_in: [None, None, None, None],
            endpoints_out: [None, None, None, None],
            input_buffers: [ [0; 32]; 4],
            _peripheral_marker: PhantomData,
        }
    }

    fn endpoint_buffer_for_index(&mut self, index: usize) -> EndpointBuffer {
        let p = &mut self.input_buffers[index] as *mut [u32];
        let static_buffer = unsafe {
            &mut * p as &'static mut [u32]
        };
        EndpointBuffer::new(static_buffer)
    }

    fn allocate_ep0(&mut self, dir: UsbDirection, max_packet_size: u16) -> Result<EndpointAddress> {
        let address = EndpointAddress::from_parts(0, dir);
        let descriptor = EndpointDescriptor {
            address,
            max_packet_size,
            ep_type: EndpointType::Control,
            interval: 0,
        };
        match dir {
            UsbDirection::In => {
                self.endpoints_in[0] = Some(EndpointIn::new::<P>(descriptor));
            },
            UsbDirection::Out => {
                let buffer = self.endpoint_buffer_for_index(0);
                self.endpoints_out[0] = Some(EndpointOut::new::<P>(descriptor, buffer));
            },
        }
        Ok(address)
    }

    fn allocate(
        &mut self,
        ep_dir: UsbDirection,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        for i in 1..P::ENDPOINT_COUNT {
            let address = EndpointAddress::from_parts(i, ep_dir);
            if self.is_allocated(address) {
                continue;
            }
            let descriptor = EndpointDescriptor {
                address,
                ep_type,
                max_packet_size,
                interval,
            };
            match ep_dir {
                UsbDirection::In => {
                    let endpoint = EndpointIn::new::<P>(descriptor);
                    self.endpoints_in[i] = Some(endpoint);
                },
                UsbDirection::Out => {
                    let buffer = self.endpoint_buffer_for_index(i);
                    let endpoint = EndpointOut::new::<P>(descriptor, buffer);
                    self.endpoints_out[i] = Some(endpoint);
                },
            }
            return Ok(address);
        }
        Err(UsbError::EndpointOverflow)
    }

    fn is_allocated(&self, addr: EndpointAddress) -> bool {
        let index = addr.index();
        if index >= P::ENDPOINT_COUNT {
            return false;
        }
        match addr.direction() {
            UsbDirection::In => self.endpoints_in[index].is_some(),
            UsbDirection::Out => self.endpoints_out[index].is_some(),
        }
    }

    fn is_stalled(
        &self,
        regs: &UsbRegisters,
        addr: EndpointAddress,
    ) -> bool {
        if addr.index() >= P::ENDPOINT_COUNT {
            return true;
        }
        crate::endpoint::is_stalled(*regs, addr)
    }

    fn set_stalled(
        &self,
        regs: &UsbRegisters,
        addr: EndpointAddress,
        stalled: bool,
    ) {
        if addr.index() >= P::ENDPOINT_COUNT {
            return;
        }
        crate::endpoint::set_stalled(*regs, addr, stalled)
    }

    fn configure_all(&self, cs: &CriticalSection) {
        for i in 0..P::ENDPOINT_COUNT {
            if let Some(ep) = self.endpoints_in[i].as_ref() {
                ep.configure(cs);
            }
            if let Some(ep) = self.endpoints_out[i].as_ref() {
                ep.configure(cs);
            }
        }
    }

    fn write(&self, addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if addr.direction() != UsbDirection::In {
            return Err(UsbError::Unsupported);
        }
        let index = addr.index();
        if let Some(ep) = self.endpoints_in[index as usize].as_ref() {
            ep.write(buf)?;
            Ok(buf.len())
        } else {
            Err(UsbError::InvalidEndpoint)
        }
    }

    fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        self.endpoints_out[addr.index()].as_ref().unwrap().read(buf)
    }
}

pub struct USB<P> {
    regs: Mutex<UsbRegisters>,
    endpoints: EndpointStack<P>,
    _marker: PhantomData<P>,
}

impl <P: UsbPeripheral> USB<P> {

    pub fn new() -> Self {
        Self {
            regs: Mutex::new(UsbRegisters::new::<P>()),
            endpoints: EndpointStack::new(),
            _marker: PhantomData,
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
            GRXSTSP,
            EPNUM,
            BCNT,
            PKTSTS
        );
        if rxflvl != 0 {
            match status {
                0x02 => { // OUT received
                    let ep_regs = regs.endpoint_out(epnum as usize);
                    let ep = &self.endpoints.endpoints_out[epnum as usize];
                    if let Some(ep) = ep {
                        let mut buffer = ep.buffer.borrow(cs).borrow_mut();
                        let result = buffer.fill_from_fifo(
                            *regs,
                            data_size as u16,
                            false,
                        );
                        match result {
                            Ok(_) => {
                                modify_reg!(endpoint_out, ep_regs, DOEPCTL, CNAK: 1, EPENA: 1);
                                ep_out |= 1 << epnum;
                            },
                            Err(UsbError::WouldBlock) => {
                                bkpt();
                            },
                            _ => {
                                bkpt();
                            }
                        }
                    }
                },
                0x06 => { // SETUP received
                    let ep_out = regs.endpoint_out(epnum as usize);
                    let ep_in = regs.endpoint_in(epnum as usize);
                    if read_reg!(endpoint_in, ep_in, DIEPTSIZ, PKTCNT) != 0 {
                        modify_reg!(otg_global, regs.global(), GRSTCTL, TXFNUM: epnum, TXFFLSH: 1);
                        while read_reg!(otg_global, regs.global(), GRSTCTL, TXFFLSH) == 1 {}
                    }
                    let ep = &self.endpoints.endpoints_out[epnum as usize];
                    if let Some(ep) = ep {
                        let mut buffer = ep.buffer.borrow(cs).borrow_mut();
                        let result = buffer.fill_from_fifo(
                            *regs,
                            data_size as u16,
                            true,
                        );
                        match result {
                            Ok(_) => {
                                modify_reg!(endpoint_out, ep_out, DOEPCTL, CNAK: 1, EPENA: 1);
                                ep_setup |= 1 << epnum;
                            },
                            Err(UsbError::WouldBlock) => {
                                bkpt();
                            },
                            _ => {
                                bkpt();
                            }
                        }
                    }
                },
                0x03 | 0x04 => { // OUT completed | SETUP completed
                    modify_reg!(otg_device, regs.device(), DOEPTSIZ0, STUPCNT: 1);
                    modify_reg!(otg_device, regs.device(), DOEPCTL0, CNAK: 1, EPENA: 1);
                    return PollResult::None;
                },
                _ => { },
            }

            if status == 0x02 || status == 0x06 {
                if iep != 0 {
                    for ep in 0..3usize {
                        let ep_regs = regs.endpoint_in(ep);
                        if read_reg!(endpoint_in, ep_regs, DIEPINT, XFRC) != 0 {
                            write_reg!(endpoint_in, ep_regs, DIEPINT, XFRC: 1);
                            ep_in_complete |= 1 << ep;
                        }
                    }
                }
            }

            for ep in &self.endpoints.endpoints_out {
                if let Some(ep) = ep {
                    match ep.buffer_state() {
                        EndpointBufferState::DataOut => {
                            ep_out |= 1 << ep.address().index();
                        },
                        EndpointBufferState::DataSetup => {
                            ep_setup |= 1 << ep.address().index();
                        },
                        EndpointBufferState::Empty => {},
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
            STUPCNT: 1
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

    fn reset(&self, cs: &CriticalSection) {
        self.endpoints.configure_all(cs);
        let regs = self.regs.borrow(cs);
        self.set_device_address(regs, 0);
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
        if let Some(0) = ep_addr.map(|a| a.index()) {
            self.endpoints.allocate_ep0(ep_dir, max_packet_size)
        } else {
            self.endpoints.allocate(ep_dir, ep_type, max_packet_size, interval)
        }
    }

    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            self.set_device_address(regs, addr.into());
        });
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            self.endpoints.set_stalled(regs, ep_addr, stalled)
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            self.endpoints.is_stalled(regs, ep_addr)
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
        interrupt::free(|cs| self.reset(cs));
    }

    fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        self.endpoints.read(addr, buf)
    }

    fn write(&self, addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        self.endpoints.write(addr, buf)
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| self.poll(cs))
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;

}
