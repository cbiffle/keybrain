#![no_std]
#![no_main]

extern crate panic_semihosting;
extern crate stm32l4;

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::convert::TryInto;

use byteorder::LittleEndian;
use zerocopy::{U16, FromBytes, AsBytes, Unaligned};
use cortex_m_rt::entry;
use stm32l4::stm32l4x2 as device;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use smart_default::SmartDefault;

const ROW_COUNT: usize = 8;

static ROW_PATTERNS: [u32; ROW_COUNT] = [
    (1 << 0) | (0xFF ^ (1 << 0)) << 16,
    (1 << 1) | (0xFF ^ (1 << 1)) << 16,
    (1 << 2) | (0xFF ^ (1 << 2)) << 16,
    (1 << 3) | (0xFF ^ (1 << 3)) << 16,
    (1 << 4) | (0xFF ^ (1 << 4)) << 16,
    (1 << 5) | (0xFF ^ (1 << 5)) << 16,
    (1 << 6) | (0xFF ^ (1 << 6)) << 16,
    (1 << 7) | (0xFF ^ (1 << 7)) << 16,
];

static KEYS: [[u8; 2]; ROW_COUNT] = [
    [0x04, 0x05],
    [0x06, 0x07],
    [0x06, 0x07],
    [0x06, 0x07],
    [0x06, 0x07],
    [0x06, 0x07],
    [0x06, 0x07],
    [0x06, 0x07],
];

#[entry]
fn main() -> ! {
    let p = unsafe { device::Peripherals::steal() };

    // Turn on ports A and B and diagnostics on C.
    p.RCC.ahb2enr.write(|w| w.gpioaen().set_bit().gpioben().set_bit().gpiocen().set_bit());

    // Configure PC7:0 as diagnostic outputs.
    p.GPIOC.moder.write(|w| {
        w.moder0().output()
            .moder1().output()
            .moder2().output()
            .moder3().output()
            .moder4().output()
            .moder5().output()
            .moder6().output()
            .moder7().output()
    });

    // Configure SCANOUT0:1 as outputs.
    p.GPIOA.moder.write(|w| {
        w.moder0().output()
            .moder1().output()
            .moder2().output()
            .moder3().output()
            .moder4().output()
            .moder5().output()
            .moder6().output()
            .moder7().output()
    });

    // Configure SCANIN0:1 as inputs.
    p.GPIOB.moder.write(|w| {
        w.moder0().input()
            .moder1().input()
    });
    // With pull-downs for when they're floating.
    p.GPIOB.pupdr.write(|w| {
        w.pupdr0().pull_down()
            .pupdr1().pull_down()
    });

    //p.GPIOA.bsrr.write(|w| w.bs0().set_bit());

    // Scale the CPU up to our max frequency of 80MHz.
    //
    // We come out of reset at 4MHz on MSI and the board appears stable at that
    // speed. We'd like to move over to HSI16 for greater accuracy, and then
    // raise that to 80 with the PLL.
    //
    // HSI16 is already in a suitable range for the PLL, but could be further
    // divided if it enables us to improve (reduce) VCO frequency. However, the
    // CPU runs on the PLL's R-tap, and the R-tap has very few divisor options.
    // This means our VCO frequency is the constrained part. The easiest choice
    // without using fractional mode is a div-by-4 from a VCO frequency of
    // 320MHz -- div-by-2 and div-by-6 would each put the frequency out of
    // range. (This part does not have an adjustable VCO input frequency range.)
    //
    // 16MHz input * 20 = 320MHz VCO
    // 320MHz VCO / 4 = 80MHz clock
    //
    // That gives us DIVN1=20, DIVP1=3.
    //
    // We'll turn off the Q and R taps for now.
    
    const MHZ: u32 = 80;

    // Getting to 80MHz requires the CPU to be volted at its highest setting of
    // VOS1. It _appears_ to come out of reset at VOS1. So that was easy.

    // Turn on HSI16.
    p.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while !p.RCC.cr.read().hsirdy().bit() {}

    // Configure PLL1.
    p.RCC.pllcfgr.write(|w| unsafe {
        w.pllpdiv().bits(0)
            .pllr().bits(0b01)
            .pllren().set_bit()
            .pllq().bits(0b11)
            .pllqen().clear_bit()
            .pllp().clear_bit()
            .pllpen().clear_bit()
            .plln().bits(20)
            .pllm().bits(0)
            .pllsrc().bits(0b10)
    });
    // Switch on PLL1.
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    while !p.RCC.cr.read().pllrdy().bit() {}

    // Adjust Flash wait states for target frequency.
    p.FLASH.acr.modify(|_, w| unsafe { w.latency().bits(0b100) });
    while p.FLASH.acr.read().latency() != 0b100 {}

    // Adjust bus clock dividers for target frequency - no changes should be
    // needed.

    // Switch CPU frequency.
    p.RCC.cfgr.modify(|_, w| unsafe { w.sw().bits(0b11) });
    while p.RCC.cfgr.read().sws() != 0b11 {}

    // k cool

    // Turn on HSI48.
    p.RCC.crrcr.modify(|_, w| {
        w.hsi48on().set_bit()
    });
    while !p.RCC.crrcr.read().hsi48rdy().bit() {
        // spin
    }
    // Use HSI48 as the 48MHz reference.
    p.RCC.ccipr.modify(|_, w| unsafe {
        w.clk48sel().bits(0b00)
    });

    // Power up the CRS and PWR
    p.RCC.apb1enr1.modify(|_, w| w.crsen().set_bit().pwren().set_bit());
    cortex_m::asm::dsb();
    // Turn on the USB peripheral
    p.PWR.cr2.modify(|_, w| w.usv().set_bit());

    // Turn on the USB peripheral.
    p.RCC.apb1enr1.modify(|_, w| w.usbfsen().set_bit());
    cortex_m::asm::dsb();

    // Mux the USB pins to the proper role.
    p.GPIOA.moder.modify(|_, w| {
        w.moder11().alternate()
            .moder12().alternate()
    });
    p.GPIOA.afrh.modify(|_, w| {
        w.afrh11().af10()
            .afrh12().af10()
    });

    // Do any configuration needed to get the sync signal generated (TBD?)
    p.USB.cntr.modify(|_,w| w.pdwn().clear_bit());
    cortex_m::asm::delay(80);
    p.USB.cntr.modify(|_,w| w.fres().clear_bit());
    p.USB.istr.write(|w| unsafe { w.bits(0) });
    p.USB.bcdr.modify(|_, w| {
        w.dppu().set_bit()
    });

    // Enable the CRS
    p.CRS.cr.modify(|_, w| {
        w.autotrimen().set_bit()
            .cen().set_bit()
    });

    let btable = get_btable();
    let buffers = get_buffers();

    setup_usb_buffers(&p.USB, btable, buffers);

    let mut device = Device::default();

    let scan_results = scan::begin_dma_scan(
        &p.GPIOA,
        &ROW_PATTERNS,
        &p.GPIOB,
        get_scan_buffer(),
        &p.RCC,
        p.DMA1,
        p.TIM2,
        MHZ,
    );

    loop {
        let istr = p.USB.istr.read();
        if istr.reset().bit() {
            device.reset(&p.USB);
            continue;
        }
        if istr.ctr().bit() {
            p.GPIOC.bsrr.write(|w| w.bs0().set_bit());
            // Correct Transfer
            let ep = istr.ep_id().bits() as usize;
            if istr.dir().bit() {
                // OUT/SETUP
                let epr = p.USB.epr[ep].read();
                // Clear CTR_RX by writing zero. Preserve other bits,
                // including toggles. This is a real bitch with svd2rust.
                p.USB.epr[ep].modify(|_, w| {
                    zero_toggles(w).ctr_rx().bit(false)
                });
                // Distinguish types
                if epr.setup().bit() {
                    device.on_setup(ep, &p.USB);
                } else {
                    device.on_out(ep, &p.USB);
                }
            } else {
                // Clear CTR_TX by writing 0, preserving toggles.
                p.USB.epr[ep].modify(|_, w| {
                    zero_toggles(w)
                        .ctr_tx().bit(false)
                });
                device.on_in(ep, &p.USB, &scan_results);
            }
            p.GPIOC.bsrr.write(|w| w.br0().set_bit());
        }
    }
}

/// Zeroes the toggle bits in the value being written so they aren't
/// inadvertantly changed.
fn zero_toggles(w: &mut device::usb::epr::W) -> &mut device::usb::epr::W {
    w.dtog_rx().bit(false)
        .stat_rx().bits(0)
        .dtog_tx().bit(false)
        .stat_tx().bits(0)
}

fn setup_usb_buffers(
    usb: &device::USB,
    btable: &'static mut [BtableSlot; 8],
    buffers: [&'static mut [MaybeUninit<u16>; 32]; 4],
) {
    let [b0, b1, b2, b3] = buffers;

    btable[0].configure(&mut b0[..], &mut b1[..]);
    btable[1].configure(&mut b2[..], &mut b3[..]);

    // Load base of descriptor table.
    let btable_off = (btable.as_mut_ptr() as usize).wrapping_sub(USB_SRAM_BASE);
    debug_assert!(btable_off < USB_SRAM_SIZE);
    usb.btable.write(|w| unsafe { w.bits(btable_off as u32) });
}

const USB_SRAM_BASE: usize = 0x4000_6C00;
const USB_SRAM_SIZE: usize = 1024;

fn read_usb_sram_16(addr: u16) -> u16 {
    assert!(addr < 0x3FF);

    unsafe {
        core::ptr::read_volatile(
            (USB_SRAM_BASE + usize::from(addr)) as *mut u16,
        )
    }
}

fn write_usb_sram_16(addr: u16, data: u16) {
    assert!(addr < 0x3FF);

    unsafe {
        core::ptr::write_volatile(
            (USB_SRAM_BASE + usize::from(addr)) as *mut u16,
            data,
        )
    }
}

fn write_usb_sram_8(addr: u16, data: u8) {
    let word = read_usb_sram_16(addr & !1);
    let word = if addr & 1 != 0 {
        (word & 0xFF) | u16::from(data) << 8
    } else {
        (word & 0xFF00) | u16::from(data)
    };
    write_usb_sram_16(addr & !1, word);
}

fn write_usb_sram_bytes(mut addr: u16, mut data: &[u8]) {
    assert!(addr < 0x400);
    assert!(addr as usize + data.len() <= 0x400);

    if addr & 1 != 0 && !data.is_empty() {
        write_usb_sram_8(addr, data[0]);
        addr += 1;
        data = &data[1..];
    }

    for (i, d) in data.chunks(2).enumerate() {
        if d.len() == 2 {
            unsafe {
                core::ptr::write_volatile(
                    (USB_SRAM_BASE + usize::from(addr) + 2 * i) as *mut u16,
                    u16::from_le_bytes(d.try_into().unwrap()),
                )
            }
        } else {
            unsafe {
                core::ptr::write_volatile(
                    (USB_SRAM_BASE + usize::from(addr) + 2 * i) as *mut u16,
                    u16::from(d[0]),
                )
            }
        }
    }
}

fn read_usb_sram<T: Sized>(addr: u16) -> T
where T: FromBytes
{
    assert!(addr < 0x400);
    assert!(addr + (core::mem::size_of::<T>() as u16) <= 0x400);

    let mut buffer: MaybeUninit<T> = MaybeUninit::uninit();
    let buffer_bytes = buffer.as_mut_ptr() as *mut u8;
    let src_addr = (USB_SRAM_BASE + usize::from(addr)) as *const u8;

    for i in 0..core::mem::size_of::<T>() {
        unsafe {
            buffer_bytes.add(i).write(src_addr.add(i).read());
        }
    }
    unsafe {
        buffer.assume_init()
    }
}

#[derive(Clone, Debug, Default, FromBytes, AsBytes, Unaligned)]
#[repr(C)]
pub struct SetupPacket {
    pub request_type: RequestType,
    pub request: u8,
    pub value: U16<LittleEndian>,
    pub index: U16<LittleEndian>,
    pub length: U16<LittleEndian>,
}

#[derive(Copy, Clone, Debug, Default, FromBytes, AsBytes, Unaligned)]
#[repr(transparent)]
pub struct RequestType(u8);

impl RequestType {
    pub fn data_phase_direction(self) -> Dir {
        if self.0 & 0x80 == 0 {
            Dir::HostToDevice
        } else {
            Dir::DeviceToHost
        }
    }

    pub fn type_(self) -> RequestTypeType {
        match (self.0 >> 5) & 0b11 {
            0 => RequestTypeType::Standard,
            1 => RequestTypeType::Class,
            2 => RequestTypeType::Vendor,
            _ => RequestTypeType::Reserved,
        }
    }

    pub fn recipient(self) -> Recipient {
        match self.0 & 0x1F {
            0 => Recipient::Device,
            1 => Recipient::Interface,
            2 => Recipient::Endpoint,
            3 => Recipient::Other,
            x => Recipient::Reserved(x),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Dir {
    HostToDevice,
    DeviceToHost,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RequestTypeType {
    Standard = 0,
    Class = 1,
    Vendor = 2,
    Reserved = 3,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Recipient {
    Device,
    Interface,
    Endpoint,
    Other,
    Reserved(u8),
}

fn get_ep_rx_offset(usb: &device::USB, ep: usize) -> u16 {
    assert!(ep < 8);

    let table = usb.btable.read().bits() as u16;
    read_usb_sram_16(table + ep as u16 * 8 + 4)
}

fn get_ep_tx_offset(usb: &device::USB, ep: usize) -> u16 {
    assert!(ep < 8);

    let table = usb.btable.read().bits() as u16;
    read_usb_sram_16(table + ep as u16 * 8 + 0)
}

fn set_ep_tx_count(usb: &device::USB, ep: usize, count: u16) {
    assert!(ep < 8);

    let table = usb.btable.read().bits() as u16;
    write_usb_sram_16(table + ep as u16 * 8 + 2, count)
}

#[derive(Copy, Clone, Debug, FromPrimitive, AsBytes, Unaligned)]
#[repr(u8)]
enum DescriptorType {
    Device = 1,
    Configuration = 2,
    String = 3,
    Interface = 4,
    Endpoint = 5,
}

#[derive(Copy, Clone, Debug, FromPrimitive)]
#[repr(u8)]
enum StdRequestCode {
    GetStatus = 0,
    ClearFeature = 1,
    SetFeature = 3,
    SetAddress = 5,
    GetDescriptor = 6,
    SetDescriptor = 7,
    GetConfiguration = 8,
    SetConfiguration = 9,
    GetInterface = 10,
    SetInterface = 11,
    SynchFrame = 12,
}

#[derive(Clone, Debug, AsBytes, Unaligned, SmartDefault)]
#[repr(C)]
struct DeviceDescriptor {
    #[default = 18]
    length: u8,
    #[default(DescriptorType::Device)]
    type_: DescriptorType,
    #[default(U16::new(0x0101))]
    usb_version: U16<LittleEndian>,
    device_class: u8,
    device_subclass: u8,
    device_protocol: u8,
    max_packet_size0: u8,
    vendor: U16<LittleEndian>,
    product: U16<LittleEndian>,
    device_version: U16<LittleEndian>,
    manufacturer_string: u8,
    product_string: u8,
    serial_string: u8,
    num_configurations: u8,
}

#[derive(Clone, Debug, AsBytes, Unaligned, SmartDefault)]
#[repr(C)]
struct ConfigDescriptor {
    #[default = 9]
    length: u8,
    #[default(DescriptorType::Configuration)]
    type_: DescriptorType,
    total_length: U16<LittleEndian>,
    num_interfaces: u8,
    configuration_value: u8,
    configuration_string: u8,
    attributes: u8,
    max_power: u8,
}

#[derive(Clone, Debug, AsBytes, Unaligned, SmartDefault)]
#[repr(C)]
struct InterfaceDescriptor {
    #[default = 9]
    length: u8,
    #[default(DescriptorType::Interface)]
    type_: DescriptorType,
    interface_number: u8,
    alternate_setting: u8,
    num_endpoints: u8,
    interface_class: u8,
    interface_subclass: u8,
    interface_protocol: u8,
    interface_string: u8,
}

#[derive(Clone, Debug, AsBytes, Unaligned, SmartDefault)]
#[repr(C)]
struct EndpointDescriptor {
    #[default = 7]
    length: u8,
    #[default(DescriptorType::Endpoint)]
    type_: DescriptorType,
    endpoint_address: u8,
    attributes: u8,
    max_packet_size: U16<LittleEndian>,
    interval: u8,
}

#[derive(Clone, Debug, AsBytes, Unaligned, SmartDefault)]
#[repr(C)]
struct CompoundConfig {
    config: ConfigDescriptor,
    iface: InterfaceDescriptor,
    hid: hid::HidDescriptor,
    ep: EndpointDescriptor,
}

fn device_get_descriptor(
    setup: &SetupPacket,
    offset: u16,
) -> Result<usize, ()> {
    let dtype = DescriptorType::from_u16(setup.value.get() >> 8)
        .ok_or(())?;
    let idx = setup.value.get() as u8;

    let write = |bytes| {
        write_usb_sram_bytes(offset, bytes);
        Ok(bytes.len())
    };

    match (dtype, idx) {
        (DescriptorType::Device, 0) => write(DeviceDescriptor {
            usb_version: U16::new(0x01_01),
            max_packet_size0: 64,
            vendor: U16::new(0xdead),
            product: U16::new(0xbeef),
            device_version: U16::new(0x03_14),
            manufacturer_string: 1,
            product_string: 1,
            serial_string: 1,
            num_configurations: 1,
            ..DeviceDescriptor::default()
        }.as_bytes()),
        (DescriptorType::Configuration, 0) => {
            let desc = CompoundConfig {
                config: ConfigDescriptor {
                    total_length: U16::new(core::mem::size_of::<CompoundConfig>() as u16),
                    num_interfaces: 1,
                    configuration_value: 1,
                    configuration_string: 1,
                    attributes: 0x80,
                    max_power: 50,
                    ..ConfigDescriptor::default()
                },
                iface: InterfaceDescriptor {
                    interface_number: 0,
                    alternate_setting: 0,
                    num_endpoints: 1,
                    interface_class: 3,
                    interface_subclass: 1,
                    interface_protocol: 1,
                    interface_string: 1,
                    ..InterfaceDescriptor::default()
                },
                hid: hid::HidDescriptor {
                    hid_version: U16::new(0x0101),
                    country_code: 0,
                    descriptor_type: 0x22,
                    descriptor_length: U16::new(62),
                    ..hid::HidDescriptor::default()
                },
                ep: EndpointDescriptor {
                    endpoint_address: 0x81,
                    attributes: 3,
                    max_packet_size: U16::new(8),
                    interval: 1,
                    ..EndpointDescriptor::default()
                },
            };
            write(desc.as_bytes())
        },
        (DescriptorType::String, _) => {
            // String
            match idx {
                0 => {
                    // LangID set
                    write(&[
                        4, // bLength
                        DescriptorType::String as u8, // bDescriptorType
                        0x09, 0x04 // en_US
                    ])
                }
                1 => {
                    // The one bogus string
                    write(&[
                        16, // bLength
                        DescriptorType::String as u8, // bDescriptorType
                        0x59, 0x00,
                        0x4f, 0x00,
                        0x55, 0x00,
                        0x52, 0x00,
                        0x4d, 0x00,
                        0x4f, 0x00,
                        0x4d, 0x00,
                    ])
                }
                _ => {
                    return Err(());
                }
            }
        }
        _ => {
            // Huh?
            return Err(());
        }
    }
}

#[derive(Debug, Default)]
#[repr(C)]
pub struct BtableSlot {
    addr_tx: u16,
    count_tx: u16,
    addr_rx: u16,
    count_rx: u16,
}

impl BtableSlot {
    /// Configures this slot for a given `tx` (host IN) and `rx` (host OUT)
    /// buffer. This is intended to be used once, during table setup.
    ///
    /// Requirements:
    /// - Both references must be valid, i.e. not aliasing, aligned, etc.
    /// - Both buffers must reside entirely within USBSRAM.
    /// - The `rx` buffer must be at least 2 bytes long.
    pub fn configure(
        &mut self,
        tx: &'static mut [MaybeUninit<u16>],
        rx: &'static mut [MaybeUninit<u16>],
    ) {
        // Record the offset of `tx` in USBSRAM; its length is immaterial at
        // this point.
        let tx_offset = (tx.as_ptr() as usize).wrapping_sub(USB_SRAM_BASE);
        debug_assert!(tx_offset < USB_SRAM_SIZE);
        self.addr_tx = tx_offset as u16;

        // Record both the base and length of `rx`. Only certain lengths can be
        // recorded; `rx.len()` will be rounded _down._
        let rx_offset = (rx.as_ptr() as usize).wrapping_sub(USB_SRAM_BASE);
        debug_assert!(rx_offset < USB_SRAM_SIZE);
        self.addr_rx = rx_offset as u16;
        let (bl_size, num_block) = if rx.len() <= 62 {
            debug_assert!(rx.len() >= 2);
            (0, rx.len() as u16 >> 1)
        } else {
            debug_assert!(rx.len() <= USB_SRAM_SIZE);
            (1, (rx.len() as u16 / 32) - 1)
        };
        self.count_rx = (num_block << 10) | (bl_size << 15);
    }

    /// Adjusts the `COUNT_TX` field to determine the size of packet that will
    /// be served up in response to the next IN request to this endpoint.
    pub fn set_tx_available(&mut self, n: usize) {
        debug_assert!(n <= usize::from(core::u16::MAX));
        self.count_tx = n as u16;
    }

    /// Reads the `COUNT_RX` field, giving the length of the last OUT/SETUP
    /// packet received at this endpoint.
    pub fn rx_available(&self) -> usize {
        usize::from(self.count_rx & 0x3FF)
    }
}

/// Produces a reference to a statically allocated buffer table located in
/// USBSRAM, the first time it is called. If you call it again, it will panic.
fn get_btable() -> &'static mut [BtableSlot; 8] {
    static TAKEN: AtomicBool = AtomicBool::new(false);

    assert!(!TAKEN.swap(true, Ordering::SeqCst));

    #[link_section = ".usbram"]
    static mut BTABLE: MaybeUninit<[BtableSlot; 8]> = MaybeUninit::uninit();

    let array: &mut [MaybeUninit<BtableSlot>; 8] = unsafe {
        core::mem::transmute(&mut BTABLE)
    };

    for slot in array.iter_mut() {
        unsafe {
            core::ptr::write(slot.as_mut_ptr(), BtableSlot::default());
        }
    }

    let initialized: &mut [BtableSlot; 8] = unsafe {
        core::mem::transmute(array)
    };
    initialized
}

fn get_buffers() -> [&'static mut [MaybeUninit<u16>; 32]; 4] {
    static TAKEN: AtomicBool = AtomicBool::new(false);

    assert!(!TAKEN.swap(true, Ordering::SeqCst));

    #[link_section = ".usbram"]
    static mut BUFFERS: MaybeUninit<[[u16; 32]; 4]> = MaybeUninit::uninit();

    let array: &mut [MaybeUninit<[u16; 32]>; 8] = unsafe {
        core::mem::transmute(&mut BUFFERS)
    };

    // Even though we're *acting* like this is uninitialized, let's go ahead and
    // initialize it.
    for slot in array.iter_mut() {
        unsafe {
            core::ptr::write(slot.as_mut_ptr(), [0; 32]);
        }
    }

    unsafe {
        [
            core::mem::transmute(&mut array[0]),
            core::mem::transmute(&mut array[1]),
            core::mem::transmute(&mut array[2]),
            core::mem::transmute(&mut array[3]),
        ]
    }
}

/// Protocol states for the outermost (Device) layer.
///
/// These follow the names used in Figure 9-1 in the USB 2.0 specification, even
/// though I think they are bad names.
#[derive(Copy, Clone, Debug, SmartDefault)]
enum DeviceState {
    /// Because we're a bus-powered device, we start out in the "powered" state
    /// -- since before we're powered, we don't have any state at all!
    ///
    /// In this state we are expecting a bus reset, but we may also be
    /// suspended.
    #[default]
    Powered,

    /// In the Default state, we are attached to the bus and can accept packets,
    /// but only to set up further states. At this point the device responds to
    /// address 0 and only address 0.
    Default,

    /// In the Address state, we have been assigned an address, but no
    /// configuration has been selected.
    Address,

    /// In the Configured state, a configuration has been selected, and the
    /// interface/endpoints other than EP0 are serving requests.
    Configured,

    // Suspended state will go here once I support suspension.
}

#[derive(Clone, Debug, Default)]
struct Device {
    state: DeviceState,
    pending_address: Option<u8>,

    iface: hid::Hid,
}

impl Device {
    pub fn reset(&mut self, usb: &device::USB) {
        usb.istr.write(|w| unsafe {
            //             v--- RESET
            w.bits(0b0111_1011_1000_0000)
        });

        // Set up control EP 0 for enumeration.
        usb.epr[0].modify(|r, w| {
            w.ea().bits(0)
                .ep_type().bits(0b01) // CONTROL
                .ep_kind().clear_bit() // not STATUS_OUT

                // Note: these bits are toggled by writing 1 for some goddamn
                // reason, so we set them as follows. I'd love to extract a
                // utility function for this but svd2rust has ensured that this
                // is impossible.
                .dtog_tx().bit(r.dtog_tx().bit()) // clear bit by toggle
                .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID

                .dtog_rx().bit(r.dtog_rx().bit()) // clear bit by toggle
                .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID (can receive)
        });

        // Configure to respond to address 0.
        usb.daddr.write(|w| w.ef().set_bit());
        self.state = DeviceState::Default;
        self.pending_address = None;

        self.iface = hid::Hid::default();
    }

    pub fn on_out(&mut self, ep: usize, usb: &device::USB) {
        // TODO we just completely don't support OUT right now
        configure_response(usb, ep, Status::Stall, Status::Stall);
    }

    pub fn on_setup(&mut self, ep: usize, usb: &device::USB) {
        if ep == 0 {
            // Reset any previously received address, if the host didn't take us
            // through to the status phase.
            self.pending_address = None;

            // Collect the setup packet from USBSRAM.
            let rxbuf = get_ep_rx_offset(usb, ep);
            let setup = read_usb_sram::<SetupPacket>(rxbuf);

            // Dispatch the different protocol layers.
            match (setup.request_type.type_(), setup.request_type.recipient()) {
                // Messages to us, the device layer:
                (RequestTypeType::Standard, Recipient::Device) => match (setup.request_type.data_phase_direction(), StdRequestCode::from_u8(setup.request)) {
                    (Dir::HostToDevice, Some(StdRequestCode::SetAddress)) => {
                        // Record pending address to be set in the status phase.
                        self.pending_address = Some(setup.value.get() as u8 & 0x7F);
                        // The host will send an IN for the status phase; make
                        // sure we don't send anything in response.
                        set_ep_tx_count(usb, ep, 0);
                        // Configure the endpoint to handle either just an IN,
                        // or an OUT followed by an IN, because it appears that
                        // the host is technically permitted to send a
                        // zero-length data phase before status.
                        configure_response(usb, ep, Status::Valid, Status::Valid);
                    }

                    (Dir::DeviceToHost, Some(StdRequestCode::GetDescriptor)) => {
                        let txoff = get_ep_tx_offset(usb, ep);
                        match device_get_descriptor(&setup, txoff) {
                            Ok(len) => {
                                // A descriptor has been deposited at txoff in
                                // USBSRAM; configure DMA to send as much of it
                                // as the host requested.
                                set_ep_tx_count(usb, ep, setup.length.get().min(len as u16));
                                // ACK the next thing.
                                configure_response(usb, ep, Status::Valid, Status::Valid);
                            }
                            Err(_) => {
                                // We don't have a descriptor matching this
                                // request.
                                // Do not transmit any data.
                                set_ep_tx_count(usb, ep, 0);
                                // In fact, stall.
                                configure_response(usb, ep, Status::Stall, Status::Stall);
                            }
                        }
                    }
                    (Dir::HostToDevice, Some(StdRequestCode::SetConfiguration)) => {
                        set_ep_tx_count(usb, ep, 0);
                        self.iface.on_set_config(usb);
                        configure_response(usb, 0, Status::Valid, Status::Valid);
                        self.state = DeviceState::Configured;
                    }
                    _ => {
                        // Unsupported
                        // Update transmittable count.
                        set_ep_tx_count(usb, ep, 0);
                        // Set a stall condition.
                        configure_response(usb, 0, Status::Stall, Status::Stall);
                    }
                },
                (RequestTypeType::Standard, Recipient::Interface) => {
                    self.iface.on_setup_iface_std(&setup, usb);
                }
                (RequestTypeType::Class, Recipient::Interface) => {
                    self.iface.on_setup_iface_class(&setup, usb);
                }
                _ => {
                    // Unsupported
                    // Update transmittable count.
                    set_ep_tx_count(usb, ep, 0);
                    // Set a stall condition.
                    configure_response(usb, ep, Status::Stall, Status::Stall);
                },
            }
        } else {
            // TODO SETUP to other endpoints
            configure_response(usb, ep, Status::Stall, Status::Stall);
        }
    }

    pub fn on_in(&mut self, ep: usize, usb: &device::USB, scan_results: &[AtomicU32]) {
        if ep == 0 {
            // This indicates completion of e.g. a descriptor transfer or an
            // empty status phase.

            if let Some(addr) = self.pending_address.take() {
                usb.daddr.write(|w| w.ef().set_bit().add().bits(addr));
                if addr == 0 {
                    self.state = DeviceState::Default;
                } else {
                    self.state = DeviceState::Address;
                }
            }

            configure_response(usb, ep, Status::Valid, Status::Valid);
        } else {
            self.iface.on_in(ep, usb, scan_results);
        }
    }
}

enum Status {
    Disabled = 0b00,
    Stall = 0b01,
    Nak = 0b10,
    Valid = 0b11,
}

fn configure_response(usb: &device::USB, ep: usize, tx: Status, rx: Status) {
    usb.epr[ep].modify(|r, w| {
        zero_toggles(w)
            .stat_tx().bits(r.stat_tx().bits() ^ tx as u8)
            .stat_rx().bits(r.stat_rx().bits() ^ rx as u8)
    });
}

fn get_scan_buffer() -> &'static mut [AtomicU32; ROW_COUNT] {
    static TAKEN: AtomicBool = AtomicBool::new(false);

    assert!(!TAKEN.swap(true, Ordering::SeqCst));

    static mut BUFFER: MaybeUninit<[AtomicU32; ROW_COUNT]> = MaybeUninit::uninit();

    let array: &mut [MaybeUninit<AtomicU32>; ROW_COUNT] = unsafe {
        core::mem::transmute(&mut BUFFER)
    };

    for slot in array.iter_mut() {
        unsafe {
            core::ptr::write(slot.as_mut_ptr(), AtomicU32::new(0));
        }
    }

    unsafe {
        core::mem::transmute(array)
    }

}

mod hid;
mod scan;