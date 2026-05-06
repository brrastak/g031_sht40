
use cortex_m::singleton;
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_waveshare::{
    epd1in02::{Display1in02, Epd1in02},
    prelude::*,
};
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
use sht4x::{Precision, Sht4x};
use stm32g0xx_hal as hal;


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
type Sht =
    sht4x::Sht4x<I2c<I2C1, PB7<Output<OpenDrain>>, PB6<Output<OpenDrain>>>, Delay<TIM14>>;

pub struct Sensor {
    sensor: Sht,
    en_sensor: InvertedPin<gpioa::PA<Output<OpenDrain>>>,
    inner_delay: Delay<TIM14>,
    extern_delay: Delay<TIM3>,
}

pub struct Display {
    epd: Epd,
    en_epd: gpioa::PA<Output<PushPull>>,
    spi_dev: SpiDev,
    epd_delay: Delay<TIM16>,
}

pub struct Board {
    pub display: Display,
    pub sensor: Sensor,
    pub rcc: rcc::Rcc,
    pub rtc: Rtc,
}

impl Board {
    
    /// Configure clock to 1MHz,
    /// initialize sensor and display,
    /// configure RTC to wake up every 1 minute,
    /// and set up the board to enter deep-sleep on wfi.
    pub fn new(periph: hal::pac::Peripherals, core: cortex_m::Peripherals) -> Self {

        // Configure clock to 1MHz
        let mut rcc = periph
            .RCC
            .freeze(rcc::Config::hsi(rcc::Prescaler::Div16));

        let gpioa = periph.GPIOA.split(&mut rcc);
        let gpiob = periph.GPIOB.split(&mut rcc);

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

        let mut spi = periph.SPI1.spi(
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

        let tim17 = periph.TIM17;
        let spi_delay: &'static mut _ =
            singleton!(: hal::timer::delay::Delay<hal::pac::TIM17> = tim17.delay(&mut rcc))
                .unwrap();
        let mut spi_dev = ExclusiveDevice::new(spi, epd_cs, spi_delay).expect("SpiDevice error");

        en_epd.set_high().ok();
        let mut epd_delay = periph.TIM16.delay(&mut rcc);
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

        // Sht40 sensor
        let en_sensor = gpioa
            .pa12
            .into_open_drain_output_in_state(PinState::High)
            .downgrade();
        let en_sensor = InvertedPin::new(en_sensor);
        let sda = gpiob.pb7.into_open_drain_output_in_state(PinState::High);
        let scl = gpiob.pb6.into_open_drain_output_in_state(PinState::High);
        let i2c = periph
            .I2C1
            .i2c(sda, scl, Config::new(100.kHz()), &mut rcc);
        let sensor: Sht = Sht4x::new(i2c);
        let inner_delay = periph.TIM14.delay(&mut rcc);
        let extern_delay = periph.TIM3.delay(&mut rcc);

        // RTC
        let date = Date::new(Year(0), Month(0), MonthDay(0));
        // Every time as seconds value turns 0
        let alarm = Alarm::new().set_seconds(0);
        let mut rtc = periph.RTC.constrain(&mut rcc);
        rtc.set_date(&date);
        rtc.set_alarm_a(alarm);
        rtc.listen(Event::AlarmA);

        // Let the core enter deep-sleep while waiting on wfi
        let mut power = periph.PWR.constrain(&mut rcc);
        power.set_mode(PowerMode::UltraLowPower(LowPowerMode::StopMode2));
        let mut scb = core
        .SCB;
        scb.set_sleepdeep();

        // Initialize hardware components and create SensorSystem and DisplaySystem instances
        let sensor = Sensor {
            sensor,
            en_sensor,
            inner_delay,
            extern_delay,
        };
        
        let display = Display {
            epd,
            en_epd,
            spi_dev,
            epd_delay,
        };
        
        Board {
            display,
            sensor,
            rcc,
            rtc,
        }
    }
}
