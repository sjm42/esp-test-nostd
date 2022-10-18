// main.rs

#![no_std]
#![no_main]

extern crate alloc;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;

use esp32c3_hal::{
    clock::ClockControl, pac::Peripherals, prelude::*, timer::TimerGroup, Delay, Rtc, IO,
};
use esp_backtrace as _;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

/*
fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}
*/

#[riscv_rt::entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let system = p.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(p.RTC_CNTL);
    let timer_group0 = TimerGroup::new(p.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(p.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    let mut delay = Delay::new(&clocks);

    let io = IO::new(p.GPIO, p.IO_MUX);

    let mut led1 = io.pins.gpio3.into_push_pull_output();
    let mut led2 = io.pins.gpio4.into_push_pull_output();
    let mut led3 = io.pins.gpio5.into_push_pull_output();

    let mut i = 0u64;
    let mut state = false;

    loop {
        if state {
            match i % 4 {
                0 => {
                    led1.set_high().ok();
                }
                1 => {
                    led2.set_high().ok();
                }
                2 => {
                    led3.set_high().ok();
                }
                _ => {}
            };
            i += 1;
        } else {
            led1.set_low().ok();
            led2.set_low().ok();
            led3.set_low().ok();
        }
        state = !state;

        delay.delay_ms(500u32);
    }
}

// EOF
