#![deny(unsafe_code)]
#![no_main]
#![no_std]


use cortex_m::singleton;
use defmt;
// Global logger
use defmt_rtt as _;
use embedded_graphics::prelude::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_waveshare::{
    epd1in02::{Epd1in02, Display1in02},
    prelude::*
};
use fixed::types::*;
// use hal::{
//     clocks::Clock,
//     fugit::RateExtU32,
//     gpio,
//     spi,
//     rtc,
//     Sio,
// };
// use inverted_pin::InvertedPin;
use hal::{
    gpio::{gpioa, gpioa::PA, Analog, Input, PullUp, PullDown, OpenDrain, Output, PushPull, PB7, PB6, PA5, PA7},
    i2c::{Config, I2c},
    pac::{SPI1, TIM16, TIM17, I2C1, TIM14},
    prelude::*,
    rcc,
    spi::{self, Mode, Phase, Polarity, SpiBus, NoMiso},
    timer::delay::Delay,
};
use inverted_pin::InvertedPin;
use panic_probe as _;
// use panic_halt as _;
use rtic_monotonics::systick::prelude::*;
use sht4x::{Sht4x, Precision};
use stm32g0xx_hal as hal;
use u8g2_fonts::{fonts, types::*, FontRenderer};

use g031_sht40::deactivate::{ActivatedEpdPins, DeactivatedEpdPins};


systick_monotonic!(Mono, 1000);


#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EXTI0_1])]
mod app {

    use super::*;


    type SpiDev = ExclusiveDevice<SpiBus<SPI1, (PA5<Analog>, NoMiso, PA7<Analog>)>, PA<Output<PushPull>>, &'static mut Delay<TIM17>>;
    type Epd = Epd1in02<SpiDev, PA<Input<PullDown>>, PA<Output<PushPull>>, PA<Output<PushPull>>, Delay<TIM16>>;
    type Sensor = sht4x::Sht4x<I2c<I2C1, PB7<Output<OpenDrain>>, PB6<Output<OpenDrain>>>, Delay<TIM14>>;


    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        epd: Epd,
        spi_dev: SpiDev,
        epd_delay: Delay<TIM16>,
        // rtc: rtc::RealTimeClock,
        sensor: Sensor,
        sensor_delay: Delay<TIM14>,
        en_sensor: InvertedPin<gpioa::PA<Output<OpenDrain>>>,
        en_epd: gpioa::PA<Output<PushPull>>,
        rcc: rcc::Rcc,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        // Configure clock to 1MHz
        let mut rcc = cx.device.RCC.freeze(rcc::Config::hsi(rcc::Prescaler::Div16));

        Mono::start(cx.core.SYST, rcc.clocks.sys_clk.to_Hz());
        // TODO: Fix: for unknown reason the first Mono activation takes above ten seconds
        // At least let it happen when nothing is still started
        Mono.delay_ms(1);

        let gpioa = cx.device.GPIOA.split(&mut rcc);
        let gpiob = cx.device.GPIOB.split(&mut rcc);
        // let led = gpioa.pa4.into_push_pull_output().downgrade();

        // Epd display
        let mut en_epd = gpioa.pa0.into_push_pull_output_in_state(PinState::High).downgrade();
        let epd_busy = gpioa.pa3.into_pull_down_input().downgrade();
        let epd_reset = gpioa.pa2.into_push_pull_output().downgrade();
        let epd_dc = gpioa.pa8.into_push_pull_output().downgrade();
        let epd_cs = gpioa.pa6.into_push_pull_output().downgrade();
        let spi_clk = gpioa.pa5;
        let spi_mosi = gpioa.pa7;

        let mut spi = cx.device.SPI1.spi(
            (spi_clk, spi::NoMiso, spi_mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            100.kHz(),
            &mut rcc,
        );
        // Set transmit-only mode
        spi.half_duplex_enable(true);
        spi.half_duplex_output_enable(true);

        let tim17 = cx.device.TIM17;
        let spi_delay: &'static mut _ = singleton!(: hal::timer::delay::Delay<hal::pac::TIM17> = tim17.delay(&mut rcc)).unwrap();
        let mut spi_dev = ExclusiveDevice::new(
            spi,
            epd_cs,
            spi_delay
        )
        .expect("SpiDevice error");

        en_epd.set_high().ok();
        let mut epd_delay = cx.device.TIM16.delay(&mut rcc);
        let mut epd = Epd1in02::new(
            &mut spi_dev,
            epd_busy,
            epd_dc,
            epd_reset,
            &mut epd_delay,
            None
        )
        .expect("Epd init error");
        epd.clear_frame(&mut spi_dev, &mut epd_delay).expect("Epd error");
        epd.display_frame(&mut spi_dev, &mut epd_delay).expect("Epd error");
        epd.sleep(&mut spi_dev, &mut epd_delay).expect("Epd error");
        en_epd.set_low().ok();
        
        let _ = DeactivatedEpdPins::steal_epd_pins(&mut rcc);

        // Sht40 sensor
        let en_sensor = gpioa.pa12.into_open_drain_output_in_state(PinState::High).downgrade();
        let en_sensor = InvertedPin::new(en_sensor);
        let sda = gpiob.pb7.into_open_drain_output_in_state(PinState::High);
        let scl = gpiob.pb6.into_open_drain_output_in_state(PinState::High);
        let i2c = cx.device
            .I2C1
            .i2c(sda, scl, Config::new(100.kHz()), &mut rcc);
        let sensor: Sensor = Sht4x::new(i2c);
        let sensor_delay = cx.device.TIM14.delay(&mut rcc);

        // // Prepare the RTC for the example using the 1/1/0 (Day/Month/Year) at 0:00:00 as the initial
        // // day and time (it may not have been a Monday but it doesn't matter for this example.).
        // let mut rtc = hal::rtc::RealTimeClock::new(
        //     cx.device.RTC,
        //     clocks.rtc_clock,
        //     &mut resets,
        //     rtc::DateTime {
        //         year: 0,
        //         month: 1,
        //         day: 1,
        //         day_of_week: rtc::DayOfWeek::Monday,
        //         hour: 0,
        //         minute: 0,
        //         second: 0,
        //     },
        // )
        // .unwrap();

        // // Trigger the IRQ once a minute
        // rtc.schedule_alarm(rtc::DateTimeFilter::default().second(0));
        // rtc.enable_interrupt();

        // // Let the core enter deep-sleep while waiting on wfi
        // let mut scb = cx.core.SCB;
        // scb.set_sleepdeep();
        
        defmt::info!("Initialization finished!");

        main_task::spawn().ok();
        // heartbeat::spawn().ok();

        (
            Shared {},
            Local {
                epd,
                spi_dev,
                epd_delay,
                // rtc,
                sensor,
                sensor_delay,
                en_sensor,
                en_epd,
                rcc,
            },
        )
    }


    #[task(local = [epd, spi_dev, epd_delay, sensor, sensor_delay, en_sensor, en_epd, rcc], priority = 1)]
    async fn main_task(cx: main_task::Context) {

        let main_task::LocalResources
            {epd, spi_dev, epd_delay, sensor, sensor_delay, en_sensor, en_epd, rcc, ..} = cx.local;

        let mut frame = Display1in02::default();
        frame.set_rotation(DisplayRotation::Rotate180);
        
        let font = FontRenderer::new::<fonts::u8g2_font_logisoso24_tf>();

        let mut prev_temperature = I16F16::ZERO;
        let mut prev_humidity = I16F16::ZERO;

        let mut epd_pins = DeactivatedEpdPins::steal_epd_pins(rcc);

        loop {
            let mut buf = [0u8; 40];

            en_sensor.set_high().ok();
            // Delay after switching sensor on
            Mono::delay(2.millis()).await;
            let measurement = sensor.measure(Precision::High, sensor_delay);
            en_sensor.set_low().ok();

            let message = match measurement {
                Ok(measurement) => {
                    let (temperature, humidity) = decode_sensor_data(measurement);

                    defmt::info!("Temperature: {}°C Humidity: {}%", temperature, humidity);
                    // if temperature == prev_temperature && humidity == prev_humidity {

                    //     // hw_config::sleep();
                    //     Mono::delay(5000.millis()).await;
                    //     continue;
                    // }

                    prev_temperature = temperature;
                    prev_humidity = humidity;

                    format_no_std::show(
                        &mut buf,
                        format_args!("{}°C\n{}%",
                            temperature,
                            humidity,
                        )).unwrap()
                },
                Err(_) => {
                    defmt::info!("Sensor reading error");
                    "Error"
                },
            };

            // Render a message
            frame.clear(epd.background_color().clone()).ok();
            font.render_aligned(
                message,
                frame.bounding_box().center(),
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                // Inversed color works with both light and dark background
                FontColor::Transparent(epd.background_color().inverse()),
                &mut frame,
            )
            .expect("Render error");

            // Display a message
            let activated_epd_pins = epd_pins.reactivate();
            en_epd.set_high().ok();
            epd.wake_up(spi_dev, epd_delay).expect("Epd error");
            epd
                .update_and_display_frame(spi_dev, frame.buffer(), epd_delay)
                .expect("Epd error");
            epd.sleep(spi_dev, epd_delay).expect("Epd error");
            en_epd.set_low().ok();
            epd_pins = activated_epd_pins.deactivate();

            Mono::delay(10.secs()).await;
            // hw_config::sleep();
        }
    }


    // #[task(binds = RTC_IRQ, local = [rtc], priority = 2)]
    // fn wake_up(cx: wake_up::Context) {

    //     let wake_up::LocalResources
    //         {rtc, ..} = cx.local;

    //     rtc.clear_interrupt();
    // }


    // Blink on-board LED
    // #[task(local = [led], priority = 1)]
    // async fn heartbeat(cx: heartbeat::Context) {

    //     let heartbeat::LocalResources
    //         {led, ..} = cx.local;

    //     loop {
    //         led.toggle().ok();
    //         defmt::info!("Blink!");
    //         Mono::delay(1000.millis()).await;
    //     }
    // }


    #[idle]
    fn idle(_: idle::Context) -> ! {

        loop {
            cortex_m::asm::wfi();
        }
    }
}


fn decode_sensor_data(measurement: sht4x::Measurement) -> (I16F16, I16F16) {

    // Keep one decimal digit in a fractional part
    let temperature = measurement
        .temperature_celsius() * 10;
    let temperature = temperature.round() / 10;

    let humidity = measurement
        .humidity_percent()
        .round();

    (temperature, humidity)
}
