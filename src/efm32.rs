use core::marker::PhantomData;

use usb_device::{
    Result,
    UsbDirection,
    UsbError,
    bus::PollResult,
    endpoint::{EndpointType, EndpointAddress},
};

use crate::{
    UsbPeripheral,
    endpoint_memory::EndpointBufferState,
    endpoint_allocator::EndpointAllocator,
    ral::{
        read_reg,
        write_reg,
        modify_reg,
        otg_global,
        otg_global_dieptxfx,
        otg_device,
        otg_pwrclk,
        endpoint_in,
    },
    target::{
        UsbRegisters,
        interrupt::{self, Mutex, CriticalSection},
    },
};

pub struct USB<P> {
    regs: Mutex<UsbRegisters>,
    allocator: EndpointAllocator<P>,
    _marker: PhantomData<P>,
}

impl <P: UsbPeripheral> USB<P> {

    pub fn new(rx_buffer: &'static mut [u32]) -> Self {
        Self {
            regs: Mutex::new(UsbRegisters::new::<P>()),
            allocator: EndpointAllocator::new(rx_buffer),
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

    fn set_device_address(&self, cs: &CriticalSection, addr: u32) {
        let regs = self.regs.borrow(cs);
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
            self.handle_reset(cs);
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
                    let ep_regs = regs.endpoint_in(epnum as usize);
                    if read_reg!(endpoint_in, ep_regs, DIEPTSIZ, PKTCNT) != 0 {
                        self.flush_tx_buffer(regs, epnum as usize);
                    }
                    ep_setup |= 1 << epnum;
                },
                0x03 | 0x04 => { // OUT completed | SETUP completed
                    read_reg!(otg_global, regs.global(), GRXSTSP);
                    modify_reg!(otg_device, regs.device(), DOEPTSIZ0, STUPCNT: 1);
                    modify_reg!(otg_device, regs.device(), DOEPCTL0, CNAK: 1, EPENA: 1);
                },
                _ => {
                    read_reg!(otg_global, regs.global(), GRXSTSP);
                },
            }

            if let 0x02 | 0x06 = status {
                let ep = &self.allocator.endpoints_out()[epnum as usize];
                if let Some(ep) = ep {
                    if ep.is_empty() {
                        read_reg!(otg_global, regs.global(), GRXSTSP);
                        let is_setup = status == 0x06;
                        ep.fill_from_fifo(cs, data_size as u16, is_setup).ok();
                    }
                }
            }
        }

        if iep != 0 {
            for ep in self.allocator.endpoints_in() {
                if let Some(ep) = ep {
                    let index = ep.address().index();
                    let ep_regs = regs.endpoint_in(index);
                    if read_reg!(endpoint_in, ep_regs, DIEPINT, XFRC) != 0 {
                        write_reg!(endpoint_in, ep_regs, DIEPINT, XFRC: 1);
                        ep_in_complete |= 1 << index;
                    }
                }
            }
        }

        for ep in self.allocator.endpoints_out() {
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
            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        } else {
            PollResult::None
        }
    }

    // 14.4.4.1.1
    fn handle_reset(&self, cs: &CriticalSection) {

        let regs = self.regs.borrow(cs);

        self.deconfigure_all(cs);
        self.flush_rx_buffer(regs);

        // 1.
        modify_reg!(otg_device, regs.device(), DOEPCTL0, SNAK: 1);
        modify_reg!(otg_device, regs.device(), DOEPCTL1, SNAK: 1);
        modify_reg!(otg_device, regs.device(), DOEPCTL2, SNAK: 1);
        modify_reg!(otg_device, regs.device(), DOEPCTL3, SNAK: 1);

        // 2.
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

    fn reset(&self, cs: &CriticalSection) {
        self.configure_all(cs);
        self.set_device_address(cs, 0);
    }

    fn flush_rx_buffer(&self, regs: &UsbRegisters) {
        modify_reg!(otg_global, regs.global(), GRSTCTL, RXFFLSH: 1);
        while read_reg!(otg_global, regs.global(), GRSTCTL, RXFFLSH) != 0 {}
    }

    fn flush_tx_buffer(&self, regs: &UsbRegisters, endpoint_number: usize) {
        modify_reg!(otg_global, regs.global(),
            GRSTCTL,
            TXFFLSH: 1,
            TXFNUM: endpoint_number as u32
        );
        while read_reg!(otg_global, regs.global(), GRSTCTL, TXFFLSH) != 0 {}
    }

    fn flush_all_tx_buffers(&self, regs: &UsbRegisters) {
        self.flush_tx_buffer(regs, 0x10)
    }

    fn force_reset(&self, cs: &CriticalSection) -> Result<()> {

        self.deconfigure_all(cs);
        self.reset(cs);
        let regs = self.regs.borrow(cs);

        modify_reg!(otg_device, regs.device(), DCTL, SDIS: 1);
        // 35 cycles should be ok but giving it more time.
        cortex_m::asm::delay(100);
        modify_reg!(otg_device, regs.device(), DCTL, SDIS: 0);
        Ok(())
    }

    fn configure_all(&self, cs: &CriticalSection) {

        let regs = self.regs.borrow(cs);

        let rx_fifo_size = self.allocator.total_rx_buffer_size_words() + 30;

        write_reg!(otg_global, regs.global(),
            GRXFSIZ,
            rx_fifo_size as u32
        );

        let mut fifo_top = rx_fifo_size;
        let fifo_size = self.allocator.tx_fifo_size_words(0);

        write_reg!(otg_global, regs.global(), DIEPTXF0,
            TX0FD: fifo_size as u32,
            TX0FSA: fifo_top as u32
        );
        fifo_top += fifo_size;

        if let Some(ep) = &self.allocator.endpoints_in()[0] {
            ep.configure(cs);
        }
        if let Some(ep) = &self.allocator.endpoints_out()[0] {
            ep.configure(cs);
        }

        self.flush_rx_buffer(regs);
        self.flush_all_tx_buffers(regs);

        for i in 1..P::ENDPOINT_COUNT {
            if let Some(ep) = &self.allocator.endpoints_in()[i] {
                let index = ep.address().index();
                let dieptxfx = regs.dieptxfx(i);
                let fifo_size = self.allocator.tx_fifo_size_words(i);
                write_reg!(otg_global_dieptxfx, dieptxfx, DIEPTXFx,
                    INEPTXFD: fifo_size as u32,
                    INEPTXSA: fifo_top as u32
                );
                fifo_top += fifo_size;
                modify_reg!(otg_device, regs.device(), DAINTMSK, |v| v | (0x0001 << index));
                ep.configure(cs);
            }
            if let Some(ep) = &self.allocator.endpoints_out()[i] {
                ep.configure(cs);
            }
        }
    }

    pub fn deconfigure_all(&self, cs: &CriticalSection) {
        let regs = self.regs.borrow(cs);

        // disable interrupts
        modify_reg!(otg_device, regs.device(), DAINTMSK, IEPM: 0, OEPM: 0);

        for ep in &self.allocator.endpoints_in()[1..] {
            if let Some(ep) = ep {
                ep.deconfigure(cs);
            }
        }

        for ep in &self.allocator.endpoints_out()[1..] {
            if let Some(ep) = ep {
                ep.deconfigure(cs);
            }
        }
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
        self.allocator.alloc_ep(ep_dir, ep_addr, ep_type, max_packet_size, interval)
    }

    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| self.set_device_address(cs, addr.into()));
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            crate::endpoint::set_stalled(*regs, ep_addr, stalled)
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
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

    fn suspend(&self) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            write_reg!(otg_global, regs.global(),
                GINTMSK,
                USBRST: 1,
                ENUMDNEM: 1,
                WUIM: 1
            );
        });
    }


    fn resume(&self) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);
            write_reg!(otg_global, regs.global(),
                GINTMSK,
                USBRST: 1,
                ENUMDNEM: 1,
                USBSUSPM: 1,
                WUIM: 1,
                IEPINT: 1,
                RXFLVLM: 1
            );
        });
    }

    fn reset(&self) {
        interrupt::free(|cs| self.reset(cs));
    }

    fn force_reset(&self) -> Result<()> {
        interrupt::free(|cs| self.force_reset(cs))
    }

    fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !addr.is_out() || addr.index() >= P::ENDPOINT_COUNT {
            return Err(UsbError::InvalidEndpoint);
        }
        if let Some(ep) = &self.allocator.endpoints_out()[addr.index()] {
            ep.read(buf)
        } else {
            Err(UsbError::InvalidEndpoint)
        }
    }

    fn write(&self, addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !addr.is_in() || addr.index() >= P::ENDPOINT_COUNT {
            return Err(UsbError::InvalidEndpoint);
        }
        if let Some(ep) = &self.allocator.endpoints_in()[addr.index()] {
            ep.write(buf).map(|_| buf.len())
        } else {
            Err(UsbError::InvalidEndpoint)
        }
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| self.poll(cs))
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;

}
