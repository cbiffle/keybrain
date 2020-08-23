//! Keyboard hardware interface.

use core::sync::atomic::AtomicU32;
use core::cell::Cell;

use super::device;
use super::scan;
use super::hid;
use super::debounce;

pub const COLS: usize = 16;
pub const ROW_COUNT: usize = 8;

#[cfg(feature = "v1")]
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

#[cfg(not(feature = "v1"))]
static ROW_PATTERNS: [u32; ROW_COUNT] = [
    // Bit-reversed 3-bit counter starting at pin 4.
    (0b000 << 4) | (!0b000 << 4) << 16,
    (0b100 << 4) | (!0b100 << 4) << 16,
    (0b010 << 4) | (!0b010 << 4) << 16,
    (0b110 << 4) | (!0b110 << 4) << 16,
    (0b001 << 4) | (!0b001 << 4) << 16,
    (0b101 << 4) | (!0b101 << 4) << 16,
    (0b011 << 4) | (!0b011 << 4) << 16,
    (0b111 << 4) | (!0b111 << 4) << 16,
];

// Matrix in logical order:
//
//  0 1  2  3  4   5  6  7  8  9  10 11 12 13 14 15
// PD __ __ _7 F10 _0 _9 _8 Ed _4 F5 PS D1 _3 _2 _1 H0
// __ Rt Dn N  F12 Sl __ __ Lt B  __ RA D3 __ __ __ H1
// __ __ __ M  En  __ Pd Cm __ V  RC __ D4 C  X  Z  H2
// __ __ Sp H  F11 Ap __ F6 Up G  __ LA D5 F4 88 Es H3
// __ RS __ J  Bh  Sc L  K  __ F  RU __ D6 D  S  A  H4
// __ LS __ Y  Bs  Lb F7 Rb __ T  LU Cp __ F3 CL Tb H5
// __ __ __ U  __  P  O  I  __ R  Pa SL __ E  W  Q  H6
// PU In Dl _6 F9  Mn F8 Eq Hm _5 LC __ D2 F2 F1 Gv H7

static KEYS: [[hid::K; COLS]; ROW_COUNT] = {
    use hid::K::*;
    [
    // 6 8   5   10  9   13  14  11  15  12  7   4    3   1   2    0
    [__, Up, Ap, __, G,  F4, __, LA, Es, __, F6, F11, H,  __, Sp, __], // H3
    [L,  __, Sc, RU, F,  D,  S,  __, A,  __, K,  Bh,  J,  RS, __, __], // H4
    [F7, __, Lb, LU, T,  F3, CL, Cp, Tb, __, Rb, Bs,  Y,  LS, __, __], // H5
    [O,  __, P , Pa, R,  E,  W,  SL, Q,  __, I,  __,  U,  __, __, __], // H6
    [_9, Ed, _0, F5, _4, _3, _2, PS, _1, __, _8, F10, _7, __, __, PD], // H0
    [F8, Hm, Mn, LC, _5, F2, F1, __, Gv, __, Eq, F9,  _6, In, Dl, PU], // H7
    [__, Lt, Sl, __, B,  __, __, RA, __, __, __, F12, N,  Rt, Dn, __], // H1
    [Pd, __, __, RC, V,  C,  X,  __, Z,  __, Cm, En,  M,  __, __, __], // H2
]
};

static FNKEYS: [[hid::K; COLS]; ROW_COUNT] = {
    use hid::K::*;
    [
    // 6 8   5   10  9   13  14  11  15  12  7   4    3   1   2    0
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, __], // H3
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, __], // H4
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, __], // H5
    [__, __, __, VM, __, __, __, __, __, __, __, __, __, __, __, __], // H6
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, VD], // H0
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, VU], // H7
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, __], // H1
    [__, __, __, __, __, __, __, __, __, __, __, __, __, __, __, __], // H2
]
};

pub struct Kbd<'a> {
    gpioa: &'a device::GPIOA,
    gpiob: device::GPIOB,
    gpioc: &'a device::GPIOC,
    scan_results: &'static [AtomicU32],
    backlight_on: Cell<bool>,
    f12_down: Cell<bool>,
}

impl<'a> Kbd<'a> {
    pub fn configure(
        gpioa: &'a device::GPIOA,
        gpiob: device::GPIOB,
        gpioc: &'a device::GPIOC,
        scan_buffer: &'static mut [AtomicU32],
        rcc: &device::RCC,
        dma1: device::DMA1,
        tim2: device::TIM2,
        timer_mhz: u32,
    ) -> Self {
        // Turn on GPIO ports for scanout and readback.
        rcc.ahb2enr.modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit());
        cortex_m::asm::dsb();

        // Configure port A.
        cfg_if::cfg_if! {
            if #[cfg(feature = "v1")] {
                // One output line per column, backlight on A8. Backlight is
                // active high; ensure that we don't raise it.
                gpioa.bsrr.write(|w| {
                    w.br8().set_bit()
                });
                gpioa.moder.write(|w| {
                    w.moder0().output()
                        .moder1().output()
                        .moder2().output()
                        .moder3().output()
                        .moder4().output()
                        .moder5().output()
                        .moder6().output()
                        .moder7().output()
                        .moder8().output()
                });
            } else {
                // Three scan output lines, plus status LEDs PA0:1 and backlight
                // on A7.

                // Backlight is active high; ensure that we don't raise it.
                // Status LEDs are active low; don't blink them either.
                p.GPIOA.bsrr.write(|w| {
                    w.br7().set_bit()
                        .bs0().set_bit()
                        .bs1().set_bit()
                });
                p.GPIOA.moder.write(|w| {
                    w.moder0().output()
                        .moder1().output()
                        .moder4().output()
                        .moder5().output()
                        .moder6().output()
                        .moder7().output()
                });
            }
        }

        // Apply pull-downs to make floating low-side inputs predictable.
        gpiob.pupdr.write(|w| {
            w.pupdr0().pull_down()
                .pupdr1().pull_down()
                .pupdr2().pull_down()
                .pupdr3().pull_down()
                .pupdr4().pull_down()
                .pupdr5().pull_down()
                .pupdr6().pull_down()
                .pupdr7().pull_down()
                .pupdr8().pull_down()
                .pupdr9().pull_down()
                .pupdr10().pull_down()
                .pupdr11().pull_down()
                .pupdr12().pull_down()
                .pupdr13().pull_down()
                .pupdr14().pull_down()
                .pupdr15().pull_down()
        });
        // And make them inputs.
        gpiob.moder.write(|w| {
            w.moder0().input()
                .moder1().input()
                .moder2().input()
                .moder3().input()
                .moder4().input()
                .moder5().input()
                .moder6().input()
                .moder7().input()
                .moder8().input()
                .moder9().input()
                .moder10().input()
                .moder11().input()
                .moder12().input()
                .moder13().input()
                .moder14().input()
                .moder15().input()
        });

        // Also turn on and configure port C if we're using it.
        #[cfg(feature = "v1")]
        {
            rcc.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

            cortex_m::asm::dsb();

            // LEDs are active low; avoid blinking them.
            gpioc.bsrr.write(|w| {
                w.bs13().set_bit()
                    .bs14().set_bit()
            });
            // Make them outputs.
            gpioc.moder.write(|w| {
                w.moder13().output()
                    .moder14().output()
            });
        }

        let scan_results = scan::begin_dma_scan(
            gpioa,
            &ROW_PATTERNS,
            &gpiob,
            scan_buffer,
            rcc,
            dma1,
            tim2,
            timer_mhz,
        );

        Kbd {
            gpioa,
            gpiob,
            gpioc,
            scan_results,
            backlight_on: Cell::new(false),
            f12_down: Cell::new(false),
        }
    }

    pub fn set_hid_leds(&self, mask: u8) {
        // CAPS
        self.set_left_status_led(mask & 0b10 != 0);
        // SCROLL
        self.set_right_status_led(mask & 0b100 != 0);
    }

    pub fn set_left_status_led(&self, state: bool) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "v1")] {
                // PC13, active low
                if state {
                    self.gpioc.bsrr.write(|w| w.br13().set_bit());
                } else {
                    self.gpioc.bsrr.write(|w| w.bs13().set_bit());
                }
            } else {
                // PA0, active low
                if state {
                    self.gpioa.bsrr.write(|w| w.br0().set_bit());
                } else {
                    self.gpioa.bsrr.write(|w| w.bs0().set_bit());
                }
            }
        }
    }

    pub fn set_right_status_led(&self, state: bool) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "v1")] {
                // PC14, active low
                if state {
                    self.gpioc.bsrr.write(|w| w.br14().set_bit());
                } else {
                    self.gpioc.bsrr.write(|w| w.bs14().set_bit());
                }
            } else {
                // PA1, active low
                if state {
                    self.gpioa.bsrr.write(|w| w.br1().set_bit());
                } else {
                    self.gpioa.bsrr.write(|w| w.bs1().set_bit());
                }
            }
        }
    }

    pub fn set_global_backlight(&self, level: u8) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "v1")] {
                // PA8 / TIM1_CH1, active high
                // PWM not currently implemented, use simple thresholding.
                if level > 127 {
                    self.gpioa.bsrr.write(|w| w.bs8().set_bit());
                } else {
                    self.gpioa.bsrr.write(|w| w.br8().set_bit());
                }
            } else {
                // PA7 / TIM1_CH1N, active high
                // PWM not currently implemented, use simple thresholding.
                if level > 127 {
                    self.gpioa.bsrr.write(|w| w.bs7().set_bit());
                } else {
                    self.gpioa.bsrr.write(|w| w.br7().set_bit());
                }
            }
        }
    }

    pub fn scan_results(&self) -> &[AtomicU32] {
        self.scan_results
    }

    pub fn read_configuration(&self, debounce: &[[debounce::KeyState; COLS]; ROW_COUNT]) -> u32 {
        let f12 = debounce[6][11].is_closed();
        let fn_enabled = debounce[6][9].is_closed();
        let fn_ = debounce[2][7].is_closed();
        if f12 && fn_enabled && fn_ {
            if self.f12_down.get() {
                // already handled this
            } else {
                let bl = !self.backlight_on.get();
                self.backlight_on.set(bl);
                self.f12_down.set(true);
                self.set_global_backlight(if bl { 255 } else { 0 });
            }
        } else {
            self.f12_down.set(false);
        }

        u32::from(debounce[4][9].is_closed()) // DIP1
            | u32::from(debounce[5][9].is_closed()) << 1 // DIP2
            | u32::from(fn_enabled) << 2 // DIP3
            | u32::from(debounce[7][9].is_closed()) << 3 // DIP4
            | u32::from(debounce[0][9].is_closed()) << 4 // DIP5
            | u32::from(debounce[1][9].is_closed()) << 5 // DIP6
            | u32::from(fn_) << 6 // fn
    }

    pub fn map_for_config(&self, config: u32) -> &[[hid::K; COLS]; ROW_COUNT] {
        const DIP3_AND_FN: u32 = (1 << 2) | (1 << 6);
        if config & DIP3_AND_FN == DIP3_AND_FN {
            &FNKEYS
        } else {
            &KEYS
        }
    }

    pub fn rewrite_key(&self, config: u32, mut key: hid::K) -> hid::K {
        use hid::K;

        const DIP1: u32 = 1 << 0;
        const DIP2: u32 = 1 << 1;
        const DIP3: u32 = 1 << 2;

        if config & DIP1 != 0 {
            // Swap left ctrl and caps lock
            match key {
                K::LC => key = K::CL,
                K::CL => key = K::LC,
                _ => (),
            }
        }
        if config & DIP2 != 0 {
            // Swap alt and super
            match key {
                K::LA => key = K::LU,
                K::LU => key = K::LA,
                K::RA => key = K::RU,
                K::RU => key = K::RA,
                _ => (),
            }
        }
        if config & DIP3 != 0 {
            // Fn becomes dead from USB's perspective
            if key == K::Cp {
                key = K::__
            }
        }
        key
    }
    
}
