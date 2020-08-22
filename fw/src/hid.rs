use super::*;

#[derive(Copy, Clone, Debug, FromPrimitive, AsBytes, Unaligned)]
#[repr(u8)]
pub enum HidClassDescriptorType {
    Hid = 0x21,
    Report = 0x22,
    Physical = 0x33,
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
pub struct HidDescriptor {
    #[default = 9]
    pub length: u8,
    #[default(HidClassDescriptorType::Hid)]
    pub type_: HidClassDescriptorType,
    pub hid_version: U16<LittleEndian>,
    pub country_code: u8,
    #[default = 1]
    pub num_descriptors: u8,
    pub descriptor_type: u8,
    pub descriptor_length: U16<LittleEndian>,
}


#[derive(Debug, Default, Clone)]
pub struct Hid {
}

static BOOT_KBD_DESC: [u8; 62] = [
    0x05, 0x01,       //  Usage Page (Desktop),
    0x09, 0x06,       //  Usage (Keyboard),
    0xA1, 0x01,       //  Collection (Application),
    0x05, 0x07,       //      Usage Page (Keyboard),
    0x19, 0xE0,       //      Usage Minimum (KB Leftcontrol),
    0x29, 0xE7,       //      Usage Maximum (KB Right GUI),
    0x15, 0x00,       //      Logical Minimum (0),
    0x25, 0x01,       //      Logical Maximum (1),
    0x75, 0x01,       //      Report Size (1),
    0x95, 0x08,       //      Report Count (8),
    0x81, 0x02,       //      Input (Variable),
    0x95, 0x01,       //      Report Count (1),
    0x75, 0x08,       //      Report Size (8),
    0x81, 0x01,       //      Input (Constant),
    0x95, 0x03,       //      Report Count (3),
    0x75, 0x01,       //      Report Size (1),
    0x05, 0x08,       //      Usage Page (LED),
    0x19, 0x01,       //      Usage Minimum (01h),
    0x29, 0x03,       //      Usage Maximum (03h),
    0x91, 0x02,       //      Output (Variable),
    0x95, 0x05,       //      Report Count (5),
    0x75, 0x01,       //      Report Size (1),
    0x91, 0x01,       //      Output (Constant),
    0x95, 0x06,       //      Report Count (6),
    0x75, 0x08,       //      Report Size (8),
    0x26, 0xFF, 0x00, //      Logical Maximum (255),
    0x05, 0x07,       //      Usage Page (Keyboard),
    0x19, 0x00,       //      Usage Minimum (None),
    0x29, 0x91,       //      Usage Maximum (KB LANG2),
    0x81, 0x00,       //      Input,
    0xC0              //  End Collection
];

impl Hid {
    pub fn on_set_config(&mut self, usb: &device::USB) {
        // Prepare empty report for EP1.
        let txoff = get_ep_tx_offset(usb, 1);
        write_usb_sram_16(txoff, 0);
        write_usb_sram_16(txoff + 2, 0);
        write_usb_sram_16(txoff + 4, 0);
        write_usb_sram_16(txoff + 6, 0);
        set_ep_tx_count(usb, 1, 8);

        // Set up EP1 for HID
        usb.epr[1].modify(|r, w| {
            w.ea().bits(1)
                .ep_type().bits(0b11) // INTERRUPT
                .ep_kind().clear_bit() // not used

                // Note: these bits are toggled by writing 1 for some goddamn
                // reason, so we set them as follows. I'd love to extract a
                // utility function for this but svd2rust has ensured that this
                // is impossible.
                .dtog_tx().bit(r.dtog_tx().bit()) // clear bit by toggle
                .stat_tx().bits(r.stat_tx().bits() ^ 0b11) // VALID

                .dtog_rx().bit(r.dtog_rx().bit()) // clear bit by toggle
                .stat_rx().bits(r.stat_rx().bits() ^ 0b01) // STALL (can't receive)
        });
    }

    pub fn on_setup_iface_std(&mut self, setup: &SetupPacket, usb: &device::USB) {
        match (setup.request_type.data_phase_direction(), StdRequestCode::from_u8(setup.request)) {
            (Dir::DeviceToHost, Some(StdRequestCode::GetDescriptor)) => {
                match HidClassDescriptorType::from_u16(setup.value.get() >> 8) {
                    Some(HidClassDescriptorType::Report) => {
                        // HID Report Descriptor
                        let desc = &BOOT_KBD_DESC;
                        write_usb_sram_bytes(get_ep_tx_offset(usb, 0), desc);
                        // Update transmittable count.
                        set_ep_tx_count(usb, 0, setup.length.get().min(desc.len() as u16));
                    }
                    _ => {
                        // Unknown kind of descriptor.
                        // TODO stall
                        set_ep_tx_count(usb, 0, 0);
                    }
                }
                // Enable transmission.
                configure_response(usb, 0, Status::Valid, Status::Valid);
            }
            _ => {
                // Unsupported
                // Update transmittable count.
                set_ep_tx_count(usb, 0, 0);
                // Set a stall condition.
                configure_response(usb, 0, Status::Stall, Status::Stall);
            }
        }
    }

    pub fn on_setup_iface_class(&mut self, setup: &SetupPacket, usb: &device::USB) {
        match (setup.request_type.data_phase_direction(), HidRequestCode::from_u8(setup.request)) {
            (Dir::HostToDevice, Some(HidRequestCode::SetIdle)) => {
                set_ep_tx_count(usb, 0, 0);
                configure_response(usb, 0, Status::Valid, Status::Valid);
            }
            (Dir::HostToDevice, Some(HidRequestCode::SetReport)) => {
                // whatever
                set_ep_tx_count(usb, 0, 0);
                configure_response(usb, 0, Status::Valid, Status::Valid);
            }
            (Dir::HostToDevice, Some(HidRequestCode::SetProtocol)) => {
                // whatever - our report protocol matches the boot protocol so
                // it's all the same to us.
                set_ep_tx_count(usb, 0, 0);
                configure_response(usb, 0, Status::Valid, Status::Valid);
            }
            _ => {
                // Unsupported
                // Update transmittable count.
                set_ep_tx_count(usb, 0, 0);
                // Set a stall condition.
                configure_response(usb, 0, Status::Stall, Status::Stall);
            }
        }
    }

    pub fn on_in(&mut self, ep: usize, usb: &device::USB, debounce: &[[debounce::KeyState; COLS]; ROW_COUNT], kbd: &kbd::Kbd) {
        // The host has just read a HID report. Prepare the next one.
        // TODO this introduces one stage of queueing delay; the reports
        // should be generated asynchronously.

        let cfg = kbd.read_configuration(debounce);

        // We have a key status matrix. We want a packed list of keycodes.
        // Scan the matrix to convert.
        let mut keys_written = 0;
        let mut packet = BootKbdPacket::default();

        let map = kbd.map_for_config(cfg);

        for (scan_row, code_row) in debounce.iter().zip(map) {
            // Note that this formulation will ignore any high-order bits if
            // code_row is narrower than 16, and any extra entries in code_row
            // beyond 16.
            for (deb, code) in scan_row.iter().zip(code_row) {
                if deb.is_closed() {
                    let value = kbd.rewrite_key(cfg, *code);

                    if value >= K::LC {
                        // Handle as modifier.
                        let bit_number = value as u8 - K::LC as u8;
                        packet.modifiers |= 1 << bit_number;
                    } else if value > K::__ {
                        // Handle as key (0 means no key or dead key).
                        if keys_written < 6 {
                            packet.keys_down[keys_written] = value as u8;
                            keys_written += 1;
                        } else if keys_written == 6 {
                            // We have key overflow. Scribble over the packet
                            // once and then advance keys_written to 7 so we
                            // don't do it for further keys. (Continue scanning
                            // to pick up modifiers.)
                            for byte in &mut packet.keys_down {
                                *byte = 0x01;
                            }
                            keys_written += 1;
                        }
                    }
                }
            }
        }
        let txoff = get_ep_tx_offset(usb, 1);
        write_usb_sram_bytes(txoff, packet.as_bytes());

        configure_response(usb, ep, Status::Valid, Status::Valid);
    }
}

#[derive(Debug, Default, AsBytes)]
#[repr(C)]
struct BootKbdPacket {
    modifiers: u8,
    reserved: u8,
    keys_down: [u8; 6],
}

/// HID Keyboard Usage codes.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
#[repr(u8)]
pub enum K {
    __ = 0,

    A = 0x04,
    B = 0x05,
    C = 0x06,
    D = 0x07,
    E = 0x08,
    F = 0x09,
    G = 0x0A,
    H = 0x0B,
    I = 0x0C,
    J = 0x0D,
    K = 0x0E,
    L = 0x0F,
    M = 0x10,
    N = 0x11,
    O = 0x12,
    P = 0x13,
    Q = 0x14,
    R = 0x15,
    S = 0x16,
    T = 0x17,
    U = 0x18,
    V = 0x19,
    W = 0x1A,
    X = 0x1B,
    Y = 0x1C,
    Z = 0x1D,

    _1 = 0x1E,
    _2 = 0x1F,
    _3 = 0x20,
    _4 = 0x21,
    _5 = 0x22,
    _6 = 0x23,
    _7 = 0x24,
    _8 = 0x25,
    _9 = 0x26,
    _0 = 0x27,

    En = 0x28,
    Es = 0x29,
    Bs = 0x2A,
    Tb = 0x2B,
    Sp = 0x2C,

    Mn = 0x2D,
    Eq = 0x2E,
    Lb = 0x2F,
    Rb = 0x30,
    Bh = 0x31,
    Sc = 0x33,
    Ap = 0x34,
    Gv = 0x35,
    Cm = 0x36,
    Pd = 0x37,
    Sl = 0x38,
    CL = 0x39,

    F1 = 0x3A,
    F2 = 0x3B,
    F3 = 0x3C,
    F4 = 0x3D,
    F5 = 0x3E,
    F6 = 0x3F,
    F7 = 0x40,
    F8 = 0x41,
    F9 = 0x42,
    F10 = 0x43,
    F11 = 0x44,
    F12 = 0x45,

    PS = 0x46,
    SL = 0x47,
    Pa = 0x48,
    In = 0x49,
    Hm = 0x4A,
    PU = 0x4B,
    Dl = 0x4C,
    Ed = 0x4D,
    PD = 0x4E,

    Rt = 0x4F,
    Lt = 0x50,
    Dn = 0x51,
    Up = 0x52,

    Cp = 0x65,

    VM = 0x7F,
    VU = 0x80,
    VD = 0x81,

    LC = 0xE0,
    LS = 0xE1,
    LA = 0xE2,
    LU = 0xE3,
    RC = 0xE4,
    RS = 0xE5,
    RA = 0xE6,
    RU = 0xE7,
}
