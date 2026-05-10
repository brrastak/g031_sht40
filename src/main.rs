#![deny(unsafe_code)]
#![no_main]
#![no_std]


use defmt;
// Global logger
use defmt_rtt as _;
use hal::rtc::{self, Rtc};
use stm32g0xx_hal as hal;
use panic_probe as _;

use g031_sht40::bsp::{Board, Display, Sensor, Led};
use g031_sht40::data::{Data, UpdateStatus};


#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EXTI0_1])]
mod app {

    use super::*;


    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        display: Display,
        sensor: Sensor,
        prev_data: Data,
        rtc: Rtc,
        led: Led,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        
        let board = Board::new(cx.device, cx.core);
        defmt::info!("Initialization finished!");

        let display = board.display;
        let sensor = board.sensor;
        let rtc = board.rtc;
        let led = board.led;

        let prev_data = Data::empty();

        system_task::spawn().ok();

        (
            Shared {},
            Local {
                display,
                sensor,
                prev_data,
                rtc,
                led,
            },
        )
    }

    #[task(local = [display, sensor, prev_data], priority = 1)]
    async fn system_task(cx: system_task::Context) {
        let system_task::LocalResources {
            display,
            sensor,
            prev_data,
            ..
        } = cx.local;

        let data = sensor.read();

        let update_status = data.check_update(prev_data);
        if update_status == UpdateStatus::NoUpdate {
            defmt::info!("No update needed");
            return;
        }

        display.full_update(&data);

        *prev_data = data;
    }

    #[task(binds = RTC_TAMP, local = [rtc], priority = 2)]
    fn wake_up(cx: wake_up::Context) {
        let wake_up::LocalResources { rtc, .. } = cx.local;

        rtc.unpend(rtc::Event::AlarmA);

        system_task::spawn().ok();
    }

    /// Turns on the LED for a short time when the button is pressed
    /// to indicate that the device is powered
    #[task(binds = EXTI4_15, local = [led], priority = 2)]
    fn is_alive(cx: is_alive::Context) {
        let is_alive::LocalResources { led, .. } = cx.local;

        led.blink();

        led.exti_unpend();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

}
