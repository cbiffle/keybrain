use core::sync::atomic::AtomicU32;

use stm32l4::stm32l4x2 as device;

/// Sets up DMA and timer to automatically scan the keyboard matrix.
///
/// `drive_port`'s BSRR will be driven regularly with successive entries from
/// `drive_pattern`. At each step, the low 16 bits in each word determine which
/// GPIOs should be driven high, and the high 16 bits which should be driven
/// low. (Any GPIOs whose bits are not set in either half of the word are
/// unchanged, allowing them to be used for something else.)
///
/// After each step is written to `drive_port`, there is a delay for lines to
/// settle, and then `read_port`'s IDR is copied into the element of `output`
/// that corresponds to the `drive_pattern` entry.
///
/// Before calling this function, you need to have powered up both GPIO ports
/// and configured driven pins as outputs, and read pins as inputs with
/// pulldowns (if external pulldowns are not provided).
///
/// Corner cases:
///
/// - `drive_port` and `read_port` can be the same port (though you'll need to
///   alter the types to achieve this, because svd2rust doesn't let us treat
///   GPIO ports generically).
/// - `drive_pattern` and `output` should be the same length, but if they
///   aren't, the minimum of the two lengths will be used.
pub fn begin_dma_scan(
    drive_port: &device::GPIOA,
    drive_pattern: &'static [u32],
    read_port: &device::GPIOB,
    output: &'static mut [AtomicU32],
    rcc: &device::RCC,
    dma1: device::DMA1,
    tim2: device::TIM2,
) -> &'static [AtomicU32] {
    let row_count = drive_pattern.len().min(output.len());

    // Turn on DMA1.
    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
    // Configure CH2 and CH5 to take DRQs from TIM2.
    dma1.cselr.write(|w| {
        w.c2s().bits(0b100) // TIM2_UP
            .c5s().bits(0b100) // TIM2_CH1
    });
    // Arrange CH2 (update) to transfer from the patterns array into BSRR.
    dma1.cndtr2.write(|w| unsafe { w.bits(row_count as u32) });
    dma1.cmar2.write(|w| unsafe { w.bits(drive_pattern.as_ptr() as u32) });
    dma1.cpar2.write(|w| unsafe { w.bits(&drive_port.bsrr as *const _ as u32) });
    dma1.ccr2.write(|w| unsafe {
        // Circular transfer forever.
        w.circ().set_bit()
            // Memory to peripheral.
            .dir().set_bit()
            // Read words from memory.
            .msize().bits(0b10)
            // And increment after reads.
            .minc().set_bit()
            // Write words to peripheral.
            .psize().bits(0b10)
            // Don't increment that side.
            .pinc().clear_bit()
            // On!
            .en().set_bit()
    });
    // Arrange CH5 to transfer in response to TIM2_CH1. We'll use this to write
    // to the scan outputs.
    dma1.cndtr5.write(|w| unsafe { w.bits(row_count as u32) });
    dma1.cpar5.write(|w| unsafe { w.bits(&read_port.idr as *const _ as u32) });
    dma1.cmar5.write(|w| unsafe { w.bits(output.as_mut_ptr() as u32) });
    dma1.ccr5.write(|w| unsafe {
        // Circular transfer forever.
        w.circ().set_bit()
            // Peripheral to memory.
            .dir().clear_bit()
            // Read words from peripheral.
            .psize().bits(0b10)
            // Don't increment that side.
            .pinc().clear_bit()
            // Write words to memory.
            .msize().bits(0b10)
            // And increment after writes.
            .minc().set_bit()
            // On!
            .en().set_bit()
    });

    // Turn on TIM2.
    rcc.apb1enr1.modify(|_, w| w.tim2en().set_bit());
    // Configure TIM2 to roll over every, say, 2kHz for now.
    // Prescaler will clock the timer at 80MHz / 80 = 1MHz.
    tim2.psc.write(|w| unsafe { w.bits(80 - 1) });
    // Timer rolls over at 1MHz / 500 = 2kHz.
    tim2.arr.write(|w| unsafe { w.bits(500 - 1) });
    // CH1 event happens halfway through because why not.
    tim2.ccr1.write(|w| unsafe { w.bits(250) });
    // We want DRQs on both CC1 and UP.
    tim2.dier.write(|w| w.ude().set_bit().cc1de().set_bit());
    // Generate an update to get all the registers loaded.
    tim2.egr.write(|w| w.ug().set_bit());
    // Let's roll.
    tim2.cr1.write(|w| w.cen().set_bit());

    output
}
