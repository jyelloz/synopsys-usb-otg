use core::marker::PhantomData;
use usb_device::{Result, UsbDirection, UsbError};
use usb_device::endpoint::{EndpointType, EndpointAddress};
use crate::endpoint::{EndpointIn, EndpointOut};
use crate::endpoint_memory::EndpointMemoryAllocator;
use crate::transition::{EndpointConfig, EndpointDescriptor};
use crate::UsbPeripheral;

pub struct EndpointAllocator<USB> {
    bitmap_in: u8,
    bitmap_out: u8,
    endpoints_in: [Option<EndpointIn>; 9],
    endpoints_out: [Option<EndpointOut>; 9],
    memory_allocator: EndpointMemoryAllocator<USB>,
    _marker: PhantomData<USB>,
}

impl<USB: UsbPeripheral> EndpointAllocator<USB> {
    pub fn new(memory: &'static mut [u32]) -> Self {
        assert!(USB::ENDPOINT_COUNT <= 9);
        Self {
            bitmap_in: 0,
            bitmap_out: 0,
            // [None; 9] requires Copy
            endpoints_in: [None, None, None, None, None, None, None, None, None],
            endpoints_out: [None, None, None, None, None, None, None, None, None],
            memory_allocator: EndpointMemoryAllocator::new(memory),
            _marker: PhantomData
        }
    }

    fn alloc_number(bitmap: &mut u8, number: Option<u8>) -> Result<u8> {
        if let Some(number) = number {
            if number as usize >= USB::ENDPOINT_COUNT {
                return Err(UsbError::InvalidEndpoint);
            }
            if *bitmap & (1 << number) == 0 {
                *bitmap |= 1 << number;
                Ok(number)
            } else {
                Err(UsbError::InvalidEndpoint)
            }
        } else {
            // Skip EP0
            for number in 1..USB::ENDPOINT_COUNT {
                if *bitmap & (1 << number) == 0 {
                    *bitmap |= 1 << number;
                    return Ok(number as u8)
                }
            }
            Err(UsbError::EndpointOverflow)
        }
    }

    fn alloc(bitmap: &mut u8, config: &EndpointConfig, direction: UsbDirection) -> Result<EndpointDescriptor> {
        let number = Self::alloc_number(bitmap, config.number)?;
        let address = EndpointAddress::from_parts(number as usize, direction);
        Ok(EndpointDescriptor {
            address,
            ep_type: config.ep_type,
            max_packet_size: config.max_packet_size,
            interval: config.interval
        })
    }

    fn alloc_in(&mut self, config: &EndpointConfig) -> Result<EndpointIn> {
        let descr = Self::alloc(&mut self.bitmap_in, config, UsbDirection::In)?;

        self.memory_allocator.allocate_tx_buffer(descr.address.index() as u8, descr.max_packet_size as usize)?;
        let ep = EndpointIn::new::<USB>(descr);

        Ok(ep)
    }

    fn alloc_out(&mut self, config: &EndpointConfig) -> Result<EndpointOut> {
        let descr = Self::alloc(&mut self.bitmap_out, config, UsbDirection::Out)?;

        let buffer = self.memory_allocator.allocate_rx_buffer(descr.max_packet_size as usize)?;
        let ep = EndpointOut::new::<USB>(descr, buffer);

        Ok(ep)
    }

    pub fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8) -> Result<EndpointAddress>
    {
        let ep_type = unsafe { core::mem::transmute(ep_type) };
        let number = ep_addr.map(|a| a.index() as u8);

        let config = EndpointConfig {
            ep_type,
            max_packet_size,
            interval,
            number,
            pair_of: None
        };
        match ep_dir {
            UsbDirection::Out => {
                let ep = self.alloc_out(&config)?;
                let address = ep.address();
                self.endpoints_out[address.index()] = Some(ep);
                Ok(address)
            },
            UsbDirection::In => {
                let ep = self.alloc_in(&config)?;
                let address = ep.address();
                self.endpoints_in[address.index()] = Some(ep);
                Ok(address)
            },
        }
    }

    pub fn total_rx_buffer_size_words(&self) -> u16 {
        self.memory_allocator.total_rx_buffer_size_words()
    }

    pub fn tx_fifo_size_words(&self, ep_number: usize) -> u16 {
        self.memory_allocator.tx_fifo_size_words(ep_number)
    }

    pub fn endpoints_in(&self) -> &[Option<EndpointIn>] {
        &self.endpoints_in
    }

    pub fn endpoints_out(&self) -> &[Option<EndpointOut>] {
        &self.endpoints_out
    }

}
