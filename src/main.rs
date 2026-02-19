#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m::singleton;
use defmt;
// Global logger
use defmt_rtt as _;
use embedded_graphics::prelude::*;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_waveshare::{
    epd1in02::{Display1in02, Epd1in02},
    prelude::*,
};
use fixed::types::*;
use hal::{
    gpio::{
        Analog, Input, OpenDrain, Output, PA5, PA7, PB6, PB7, PullDown, PushPull, gpioa, gpioa::PA,
    },
    i2c::{Config, I2c},
    pac::{I2C1, SPI1, TIM3, TIM14, TIM16, TIM17},
    power::{LowPowerMode, PowerMode},
    prelude::*,
    rcc,
    rtc::{Alarm, Event, Rtc},
    spi::{self, Mode, NoMiso, Phase, Polarity, SpiBus},
    time::{Date, Month, MonthDay, Year},
    timer::delay::Delay,
};
use inverted_pin::InvertedPin;
use panic_probe as _;
use sht4x::{Precision, Sht4x};
use stm32g0xx_hal as hal;
use u8g2_fonts::{FontRenderer, fonts, types::*};

use g031_sht40::deactivate::DeactivatedEpdPins;

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [EXTI0_1])]
mod app {

    use super::*;

    type SpiDev = ExclusiveDevice<
        SpiBus<SPI1, (PA5<Analog>, NoMiso, PA7<Analog>)>,
        PA<Output<PushPull>>,
        &'static mut Delay<TIM17>,
    >;
    type Epd = Epd1in02<
        SpiDev,
        PA<Input<PullDown>>,
        PA<Output<PushPull>>,
        PA<Output<PushPull>>,
        Delay<TIM16>,
    >;
    type Sensor =
        sht4x::Sht4x<I2c<I2C1, PB7<Output<OpenDrain>>, PB6<Output<OpenDrain>>>, Delay<TIM14>>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        epd: Epd,
        spi_dev: SpiDev,
        epd_delay: Delay<TIM16>,
        sensor: Sensor,
        sensor_delay: Delay<TIM14>,
        en_sensor: InvertedPin<gpioa::PA<Output<OpenDrain>>>,
        en_epd: gpioa::PA<Output<PushPull>>,
        rcc: rcc::Rcc,
        rtc: Rtc,
        delay: Delay<TIM3>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Configure clock to 1MHz
        let mut rcc = cx
            .device
            .RCC
            .freeze(rcc::Config::hsi(rcc::Prescaler::Div16));

        let gpioa = cx.device.GPIOA.split(&mut rcc);
        let gpiob = cx.device.GPIOB.split(&mut rcc);

        // Epd display
        let mut en_epd = gpioa
            .pa0
            .into_push_pull_output_in_state(PinState::High)
            .downgrade();
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
        let spi_delay: &'static mut _ =
            singleton!(: hal::timer::delay::Delay<hal::pac::TIM17> = tim17.delay(&mut rcc))
                .unwrap();
        let mut spi_dev = ExclusiveDevice::new(spi, epd_cs, spi_delay).expect("SpiDevice error");

        en_epd.set_high().ok();
        let mut epd_delay = cx.device.TIM16.delay(&mut rcc);
        let mut epd = Epd1in02::new(
            &mut spi_dev,
            epd_busy,
            epd_dc,
            epd_reset,
            &mut epd_delay,
            None,
        )
        .expect("Epd init error");
        epd.clear_frame(&mut spi_dev, &mut epd_delay)
            .expect("Epd error");
        epd.display_frame(&mut spi_dev, &mut epd_delay)
            .expect("Epd error");
        epd.sleep(&mut spi_dev, &mut epd_delay).expect("Epd error");
        en_epd.set_low().ok();

        let _ = DeactivatedEpdPins::steal_epd_pins(&mut rcc);

        // Sht40 sensor
        let en_sensor = gpioa
            .pa12
            .into_open_drain_output_in_state(PinState::High)
            .downgrade();
        let en_sensor = InvertedPin::new(en_sensor);
        let sda = gpiob.pb7.into_open_drain_output_in_state(PinState::High);
        let scl = gpiob.pb6.into_open_drain_output_in_state(PinState::High);
        let i2c = cx
            .device
            .I2C1
            .i2c(sda, scl, Config::new(100.kHz()), &mut rcc);
        let sensor: Sensor = Sht4x::new(i2c);
        let sensor_delay = cx.device.TIM14.delay(&mut rcc);

        // Delay
        let delay = cx.device.TIM3.delay(&mut rcc);

        // RTC
        let date = Date::new(Year(0), Month(0), MonthDay(0));
        // Every time as seconds value turns 10
        let alarm = Alarm::new().set_seconds(10);
        let mut rtc = cx.device.RTC.constrain(&mut rcc);
        rtc.set_date(&date);
        rtc.set_alarm_a(alarm);
        rtc.listen(Event::AlarmA);

        // Let the core enter deep-sleep while waiting on wfi
        let mut power = cx.device.PWR.constrain(&mut rcc);
        power.set_mode(PowerMode::UltraLowPower(LowPowerMode::StopMode2));
        let mut scb = cx.core.SCB;
        scb.set_sleepdeep();
        cortex_m::asm::wfi();

        defmt::info!("Initialization finished!");

        main_task::spawn().ok();

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
                rtc,
                delay,
            },
        )
    }

    #[task(local = [epd, spi_dev, epd_delay, sensor, sensor_delay, en_sensor, en_epd, rcc, delay], priority = 1)]
    async fn main_task(cx: main_task::Context) {
        let main_task::LocalResources {
            epd,
            spi_dev,
            epd_delay,
            sensor,
            sensor_delay,
            en_sensor,
            en_epd,
            rcc,
            delay,
            ..
        } = cx.local;

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
            delay.delay(2.millis());
            let measurement = sensor.measure(Precision::High, sensor_delay);
            en_sensor.set_low().ok();

            let message = match measurement {
                Ok(measurement) => {
                    let (temperature, humidity) = decode_sensor_data(measurement);

                    defmt::info!("Temperature: {}°C Humidity: {}%", temperature, humidity);
                    // if temperature == prev_temperature && humidity == prev_humidity {

                    //     cortex_m::asm::wfi();
                    //     continue;
                    // }

                    prev_temperature = temperature;
                    prev_humidity = humidity;

                    format_no_std::show(&mut buf, format_args!("{}°C\n{}%", temperature, humidity,))
                        .unwrap()
                }
                Err(_) => {
                    defmt::info!("Sensor reading error");
                    "Error"
                }
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
            epd.update_and_display_frame(spi_dev, frame.buffer(), epd_delay)
                .expect("Epd error");
            epd.sleep(spi_dev, epd_delay).expect("Epd error");
            en_epd.set_low().ok();
            epd_pins = activated_epd_pins.deactivate();

            cortex_m::asm::wfi();
        }
    }

    #[task(binds = RTC_TAMP, local = [rtc], priority = 2)]
    fn wake_up(cx: wake_up::Context) {
        let wake_up::LocalResources { rtc, .. } = cx.local;

        rtc.unpend(Event::AlarmA);
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

fn decode_sensor_data(measurement: sht4x::Measurement) -> (I16F16, I16F16) {
    // Keep one decimal digit in a fractional part
    let temperature = measurement.temperature_celsius() * 10;
    let temperature = temperature.round() / 10;

    let humidity = measurement.humidity_percent().round();

    (temperature, humidity)
}
