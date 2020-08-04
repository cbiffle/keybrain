#![no_std]
#![no_main]

extern crate panic_semihosting;
extern crate stm32l4;

use core::convert::TryInto;

use byteorder::LittleEndian;
use zerocopy::{U16, U32, FromBytes, AsBytes, Unaligned};
use cortex_m_rt::entry;
use stm32l4::stm32l4x2 as device;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use smart_default::SmartDefault;

#[entry]
fn main() -> ! {
    let p = unsafe { device::Peripherals::steal() };

    // Turn on ports A and B.
    p.RCC.ahb2enr.write(|w| w.gpioaen().set_bit().gpioben().set_bit());

    // Configure SCANOUT0:1 as outputs.
    p.GPIOA.moder.write(|w| {
        w.moder0().output()
            .moder1().output()
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

    p.GPIOA.bsrr.write(|w| w.bs0().set_bit());

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

    if true {
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
        //p.USB.daddr.write(|w| w.ef().set_bit());

        // Enable the CRS
        p.CRS.cr.modify(|_, w| {
            w.autotrimen().set_bit()
                .cen().set_bit()
        });
    }

    // Output CRS status signals on PA7:0
    let mut pending_address = None;
    let mut scan_divider = 0;
    let mut scan_state = false;
    let mut keys_down = [[false; 2]; 2];
    static KEYS: [[u8; 2]; 2] = [
        [0x04, 0x05],
        [0x06, 0x07],
    ];
    loop {
        if scan_divider == 0 {
            // Read result of last scan.
            let scan_in = p.GPIOB.idr.read().bits() & 0b11;
            keys_down[scan_state as usize][0] = scan_in & 0b01 != 0;
            keys_down[scan_state as usize][1] = scan_in & 0b10 != 0;

            scan_state = !scan_state;

            // Advance key scan state machine
            p.GPIOA.bsrr.write(|w| unsafe {
                let bits = u32::from(scan_state);
                let bits = (bits << 1) | (bits ^ 1);
                w.bits(bits | (bits ^ 0b11) << 16)
            });

            scan_divider = 100;
        } else {
            scan_divider -= 1;
        }

        let istr = p.USB.istr.read();
        if istr.reset().bit() {
            setup_usb(&p.USB);
            continue;
        }
        if istr.ctr().bit() {
            // Correct Transfer
            let ep = istr.ep_id().bits() as usize;
            if istr.dir().bit() {
                // OUT/SETUP
                let epr = p.USB.epr[ep].read();
                // Clear CTR_RX by writing zero. Preserve other bits,
                // including toggles. This is a real bitch with svd2rust.
                p.USB.epr[ep].modify(|r, w| {
                    zero_toggles(w).ctr_rx().bit(false)
                });
                // Distinguish types
                if epr.setup().bit() {
                    // SETUP

                    if ep == 0 {
                        // Just in case
                        pending_address = None;

                        // Collect request from packet memory.
                        let rxbuf = get_ep_rx_offset(&p.USB, ep);
                        let setup = read_usb_sram::<SetupPacket>(rxbuf);

                        match (setup.request_type.type_(), setup.request_type.recipient()) {
                            (RequestTypeType::Standard, Recipient::Device) => match (setup.request_type.data_phase_direction(), StdRequestCode::from_u8(setup.request)) {
                                (Dir::HostToDevice, Some(StdRequestCode::SetAddress)) => {
                                    pending_address = Some(setup.value.get() as u8 & 0x7F);
                                    write_usb_sram_16(2, 0);
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                                            // Also reception to allow for status
                                            // packet
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID
                                    });
                                }
                                (Dir::DeviceToHost, Some(StdRequestCode::GetDescriptor)) => {
                                    let temp_buffer = unsafe {
                                        core::slice::from_raw_parts_mut(
                                            (USB_SRAM_BASE + 64) as *mut u16,
                                            64 / 2,
                                        )
                                    };
                                    match device_get_descriptor(&setup, temp_buffer) {
                                        Ok(len) => {
                                            write_usb_sram_16(2, setup.length.get().min(len as u16));
                                            p.USB.epr[ep].modify(|r, w| {
                                                zero_toggles(w)
                                                    .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                                                    // Also reception to allow for status
                                                    // packet
                                                    .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID
                                            });
                                        }
                                        Err(_) => {
                                            // Do not transmit any data.
                                            write_usb_sram_16(2, 0);
                                            // In fact, stall.
                                            p.USB.epr[ep].modify(|r, w| {
                                                zero_toggles(w)
                                                    .stat_tx().bits(r.stat_tx().bits() ^ 0b01) // STALL
                                                    // Also reception to allow for status
                                                    // packet
                                                    .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL
                                            });
                                        }
                                    }
                                }
                                (Dir::HostToDevice, Some(StdRequestCode::SetConfiguration)) => {
                                    write_usb_sram_16(2, 0);
                                    // Prepare empty report.
                                    write_usb_sram_16(192, 0);
                                    write_usb_sram_16(192 + 2, 0);
                                    write_usb_sram_16(192 + 4, 0);
                                    write_usb_sram_16(192 + 6, 0);

                                    // Set up EP1 for HID
                                    p.USB.epr[1].modify(|r, w| {
                                        w.ea().bits(1)
                                            .ep_type().bits(0b11) // INTERRUPT
                                            .ep_kind().clear_bit() // not used

                                            // Note: these bits are toggled by writing 1 for some goddamn
                                            // reason, so we set them as follows. I'd love to extract a utility
                                            // function for this but svd2rust has ensured that this is
                                            // impossible.
                                            .dtog_tx().bit(r.dtog_tx().bit()) // clear bit by toggle
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID

                                            .dtog_rx().bit(r.dtog_rx().bit()) // clear bit by toggle
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL (can't receive)
                                    });
                                    p.USB.epr[0].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                                            // Also reception to allow for status
                                            // packet
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID
                                    });
                                }
                                _ => {
                                    // Unsupported
                                    // Update transmittable count.
                                    write_usb_sram_16(2, 0);
                                    // Set a stall condition.
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b01) // STALL
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL
                                    });
                                }
                            },
                            (RequestTypeType::Standard, Recipient::Interface) => match (setup.request_type.data_phase_direction(), StdRequestCode::from_u8(setup.request)) {
                                (Dir::DeviceToHost, Some(StdRequestCode::GetDescriptor)) => {
                                    match HidClassDescriptorType::from_u16(setup.value.get() >> 8) {
                                        Some(HidClassDescriptorType::Report) => {
                                            // HID Report Descriptor
                                            let desc = [
                                                0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
                                                0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x03, 0x75, 0x01,
                                                0x05, 0x08, 0x19, 0x01, 0x29, 0x03, 0x91, 0x02, 0x95, 0x05, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
                                                0x75, 0x08, 0x26, 0xff, 0x00, 0x05, 0x07, 0x19, 0x00, 0x29, 0x91, 0x81, 0x00, 0xc0,
                                            ];
                                            write_usb_sram_bytes(64, &desc);
                                            // Update transmittable count.
                                            write_usb_sram_16(2, setup.length.get().min(desc.len() as u16));
                                        }
                                        _ => {
                                            // Unknown kind of descriptor.
                                            write_usb_sram_16(2, 0);
                                        }
                                    }
                                    // Enable transmission.
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                                            // Also reception to allow for status
                                            // packet
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID
                                    });
                                }
                                _ => {
                                    // Unsupported
                                    // Update transmittable count.
                                    write_usb_sram_16(2, 0);
                                    // Set a stall condition.
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b01) // STALL
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL
                                    });
                                }
                            }
                            (RequestTypeType::Class, Recipient::Interface) => match (setup.request_type.data_phase_direction(), HidRequestCode::from_u8(setup.request)) {
                                (Dir::HostToDevice, Some(HidRequestCode::SetIdle)) => {
                                    write_usb_sram_16(2, 0);
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                                            // Also reception to allow for status
                                            // packet
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID
                                    });
                                }
                                (Dir::HostToDevice, Some(HidRequestCode::SetReport)) => {
                                    // whatever
                                    write_usb_sram_16(2, 0);
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                                            // Also reception to allow for status
                                            // packet
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID
                                    });
                                }
                                _ => {
                                    // Unsupported
                                    // Update transmittable count.
                                    write_usb_sram_16(2, 0);
                                    // Set a stall condition.
                                    p.USB.epr[ep].modify(|r, w| unsafe {
                                        zero_toggles(w)
                                            .stat_tx().bits(r.stat_tx().bits() ^ 0b01) // STALL
                                            .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL
                                    });
                                }
                            }
                            _ => {
                                // Unsupported
                                // Update transmittable count.
                                write_usb_sram_16(2, 0);
                                // Set a stall condition.
                                p.USB.epr[ep].modify(|r, w| unsafe {
                                    zero_toggles(w)
                                        .stat_tx().bits(r.stat_tx().bits() ^ 0b01) // STALL
                                        .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL
                                });
                            },
                        }
                    }
                } else {
                    // OUT
                    if ep == 0 {
                        // lolwut
                        p.USB.epr[ep].modify(|r, w| unsafe {
                            zero_toggles(w)
                                .stat_tx().bits(r.stat_tx().bits() ^ 0b01) // STALL
                                .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL
                        });
                    }
                }
            } else {
                // IN
                if ep == 0 {
                    // Clear CTR_TX by writing 0, preserving toggles.
                    p.USB.epr[ep].modify(|r, w| {
                        zero_toggles(w)
                            .ctr_tx().bit(false)
                    });

                    if let Some(addr) = pending_address.take() {
                        p.USB.daddr.write(|w| w.ef().set_bit().add().bits(addr))
                    }

                    p.USB.epr[ep].modify(|r, w| unsafe {
                        zero_toggles(w)
                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                    });
                } else {
                    // Clear CTR_TX by writing 0, preserving toggles.
                    p.USB.epr[ep].modify(|r, w| {
                        zero_toggles(w)
                            .ctr_tx().bit(false)
                    });

                    let mut write_idx = 2;
                    for (dn_row, code_row) in keys_down.iter().zip(&KEYS) {
                        for (dn, code) in dn_row.iter().zip(code_row) {
                            if *dn {
                                write_usb_sram_8(192 + write_idx, *code);
                                write_idx += 1;
                            }
                        }
                    }
                    while write_idx < 8 {
                        write_usb_sram_8(192 + write_idx, 0);
                        write_idx += 1;
                    }

                    p.USB.epr[ep].modify(|r, w| unsafe {
                        zero_toggles(w)
                            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID
                    });
                }
            }
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

fn setup_usb(usb: &device::USB) {
    usb.istr.write(|w| unsafe {
        //             v--- RESET
        w.bits(0b0111_1011_1000_0000)
    });

    // Buffer layout:
    // - Buffer table occupies 8 * 8 = 64 bytes
    // - EP 0 IN (TX) buffer is next at 64 bytes
    // - Then EP 0 OUT at 64

    write_usb_sram_16(0, 64); // EP 0 IN
    write_usb_sram_16(2, 0);  // no bytes valid
    write_usb_sram_16(4, 128);  // EP 0 OUT
    write_usb_sram_16(6, (1 << 15) | (1 << 10));  // 64 bytes

    write_usb_sram_16(8, 192); // EP 0 IN
    write_usb_sram_16(10, 8);  // 8 bytes valid
    write_usb_sram_16(12, 256);  // EP 0 OUT
    write_usb_sram_16(14, 0);  // no bytes

    // Load base of descriptor table.
    usb.btable.write(|w| unsafe { w.bits(0) });

    // Set up control EP 0 for enumeration.
    usb.epr[0].modify(|r, w| {
        w.ea().bits(0)
            .ep_type().bits(0b01) // CONTROL
            .ep_kind().clear_bit() // not STATUS_OUT

            // Note: these bits are toggled by writing 1 for some goddamn
            // reason, so we set them as follows. I'd love to extract a utility
            // function for this but svd2rust has ensured that this is
            // impossible.
            .dtog_tx().bit(r.dtog_tx().bit()) // clear bit by toggle
            .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID

            .dtog_rx().bit(r.dtog_rx().bit()) // clear bit by toggle
            .stat_rx().bits(r.stat_rx().bits() ^ 0b11) // VALID (can receive)
    });

    usb.daddr.write(|w| w.ef().set_bit());
}

const USB_SRAM_BASE: usize = 0x4000_6C00;

fn read_usb_sram_8(addr: u16) -> u8 {
    assert!(addr < 0x400);

    unsafe {
        core::ptr::read_volatile(
            (USB_SRAM_BASE + usize::from(addr)) as *mut u8,
        )
    }
}

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

fn write_high_8(dest: &mut u16, data: u8) {
    *dest = (*dest & 0xFF) | u16::from(data) << 8
}

fn write_low_8(dest: &mut u16, data: u8) {
    *dest = (*dest & 0xFF00) | u16::from(data)
}

/// Writes bytes from `src` into `dest` using `u16` accesses.
///
/// Most bytes will be packed into `u16`s and stored directly; the final byte,
/// if the length is odd, will be stored with a read-modify-write operation.
///
/// `dest` must be at least as large as `src` measured in bytes; otherwise this
/// panics. On return, the first `src.len()` bytes of `dest` are initialized.
#[inline(never)]
fn write_bytes(dest: &mut [u16], src: &[u8]) {
    assert!(dest.len() >= (src.len() + 1) / 2);
    for (d, schunk) in dest.iter_mut().zip(src.chunks_exact(2)) {
        *d = u16::from_le_bytes(schunk.try_into().unwrap());
    }
    if src.len() & 1 != 0 {
        // Handle trailing odd byte
        write_low_8(&mut dest[src.len() / 2], src[src.len() - 1])
    }
}

/// Stores `value` into the prefix of `dest` and returns the valid length *in
/// bytes*.
fn write_t(dest: &mut [u16], value: &impl AsBytes) -> usize {
    let bytes = value.as_bytes();
    write_bytes(dest, bytes);
    bytes.len()
}

fn read_usb_sram<T: Sized>(addr: u16) -> T
where T: FromBytes
{
    assert!(addr < 0x400);
    assert!(addr + (core::mem::size_of::<T>() as u16) <= 0x400);
    use core::mem::MaybeUninit;

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

fn write_usb_sram<T: Sized>(addr: u16, value: T)
where T: AsBytes
{
    assert!(addr < 0x400);
    assert!(addr + (core::mem::size_of::<T>() as u16) <= 0x400);

    write_usb_sram_bytes(addr, value.as_bytes());
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

fn get_ep_tx_offset(usb: &device::USB, ep: u8) -> u16 {
    assert!(ep < 8);

    let table = usb.btable.read().bits() as u16;
    read_usb_sram_16(table + u16::from(ep) * 8)
}

fn get_ep_rx_offset(usb: &device::USB, ep: usize) -> u16 {
    assert!(ep < 8);

    let table = usb.btable.read().bits() as u16;
    read_usb_sram_16(table + ep as u16 * 8 + 4)
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

#[derive(Copy, Clone, Debug, FromPrimitive, AsBytes, Unaligned)]
#[repr(u8)]
enum HidClassDescriptorType {
    Hid = 0x21,
    Report = 0x22,
    Physical = 0x33,
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

#[derive(Copy, Clone, Debug, FromPrimitive)]
#[repr(u8)]
enum HidRequestCode {
    GetReport = 1,
    GetIdle = 2,
    GetProtocol = 3,
    SetReport = 9,
    SetIdle = 0xA,
    SetProtocol = 0xB,
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
struct HidDescriptor {
    #[default = 9]
    length: u8,
    #[default(HidClassDescriptorType::Hid)]
    type_: HidClassDescriptorType,
    hid_version: U16<LittleEndian>,
    country_code: u8,
    #[default = 1]
    num_descriptors: u8,
    descriptor_type: u8,
    descriptor_length: U16<LittleEndian>,
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
    hid: HidDescriptor,
    ep: EndpointDescriptor,
}

fn device_get_descriptor(
    setup: &SetupPacket,
    buffer: &mut [u16],
) -> Result<usize, ()> {
    let dtype = DescriptorType::from_u16(setup.value.get() >> 8);
    let dtype = if let Some(d) = dtype {
        d
    } else {
        panic!("wtf: {:?}", setup);
    };
    let idx = setup.value.get() as u8;

    let mut write = |bytes| {
        write_bytes(buffer, bytes);
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
                hid: HidDescriptor {
                    hid_version: U16::new(0x0101),
                    country_code: 0,
                    descriptor_type: 0x22,
                    descriptor_length: U16::new(62),
                    ..HidDescriptor::default()
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
