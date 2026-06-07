//! EPD display related stuff

use embedded_graphics::prelude::*;
use epd_waveshare::{
    epd1in02::{self, Epd1in02},
    graphics::VarDisplay,
    prelude::*,
};
use hal::{
    gpio::{Input, Output, PullDown, PushPull, gpioa, gpioa::PA},
    prelude::*,
    rcc,
};
use stm32g0xx_hal as hal;
use u8g2_fonts::{FontRenderer, fonts, types::*};

use crate::bsp::{Epd, EpdDelay, EpdSpiDev};
use crate::data::{Data, UpdateStatus};
use crate::deactivate::{ActivatedEpdPins, DeactivatedEpdPins};

// type Frame = epd_waveshare::graphics::Display<80, 128, false, 1280, Color>;
type PartialFrame<'a> = VarDisplay<'a, Color>;
type Font = fonts::u8g2_font_logisoso24_tf;

const FONT: FontRenderer = FontRenderer::new::<Font>();
const TEXT_HEIGHT: u32 = FONT
    .get_font_bounding_box(VerticalPosition::Baseline)
    .size
    .height;
const TEXT_WIDTH: u32 = epd1in02::WIDTH;
/// Size of the frame buffer for one line of text
const FRAME_SIZE: usize = TEXT_HEIGHT as usize * TEXT_WIDTH as usize / 8;
/// Temperature text line Y coordinate
const TEMPERATURE_LINE_Y: u32 = epd1in02::HEIGHT / 2;
/// Humidity text line Y coordinate
const HUMIDITY_LINE_Y: u32 = epd1in02::HEIGHT / 2 - TEXT_HEIGHT;
/// Y coordinate for error message when data update fails
const ERROR_LINE_Y: u32 = epd1in02::HEIGHT / 2 - TEXT_HEIGHT / 2;
/// Y coordinate to update both lines
const BOTH_LINES_Y: u32 = HUMIDITY_LINE_Y;
/// Number of updates after which the screen will be fully cleared to prevent ghosting effect
const CLEAR_NUMBER_OF_UPDATES: u32 = 100;

pub enum RenderTarget {
    Temperature,
    Humidity,
    Error,
}

/// Display related HW
pub struct Display {
    epd: Epd,
    en_epd: gpioa::PA<Output<PushPull>>,
    spi_dev: EpdSpiDev,
    epd_delay: EpdDelay,
    /// For DeactivatedEpdPins
    pub rcc: rcc::Rcc,
    temperature_frame_buf: [u8; FRAME_SIZE],
    humidity_frame_buf: [u8; FRAME_SIZE],
    update_counter: u32,
}

impl Display {
    /// Create new Display instance and initialize EPD
    /// Arguments:
    /// - `spi_dev`: SPI device for EPD communication
    /// - `epd_busy`: EPD busy pin
    /// - `epd_dc`: EPD data/command pin
    /// - `epd_reset`: EPD reset pin
    /// - `epd_delay`: Delay provider
    /// - `en_epd`: GPIO pin to enable/disable EPD power
    /// - `rcc`: Reset and clock control for pin reconfiguration when deactivating EPD pins
    pub fn new(
        mut spi_dev: EpdSpiDev,
        epd_busy: PA<Input<PullDown>>,
        epd_dc: PA<Output<PushPull>>,
        epd_reset: PA<Output<PushPull>>,
        mut epd_delay: EpdDelay,
        mut en_epd: PA<Output<PushPull>>,
        mut rcc: rcc::Rcc,
    ) -> Self {
        en_epd.set_high().ok();

        let mut epd = Epd1in02::new(
            &mut spi_dev,
            epd_busy,
            epd_dc,
            epd_reset,
            &mut epd_delay,
            None,
        )
        .expect("EPD init error");
        epd.sleep(&mut spi_dev, &mut epd_delay).expect("EPD error");
        en_epd.set_low().ok();
        // Without that power consumption is ~50mA
        let _ = DeactivatedEpdPins::steal_epd_pins(&mut rcc);

        let background_color = epd.background_color().get_byte_value();

        Display {
            epd,
            en_epd,
            spi_dev,
            epd_delay,
            rcc,
            temperature_frame_buf: [background_color; FRAME_SIZE],
            humidity_frame_buf: [background_color; FRAME_SIZE],
            update_counter: 0,
        }
    }

    pub fn partial_update(&mut self, data: &Data, update_status: UpdateStatus) {
        if update_status == UpdateStatus::NoUpdate {
            defmt::info!("No update needed");
            return;
        }

        let background_color = self.epd.background_color().get_byte_value();
        let mut temperature_frame_buf = self.temperature_frame_buf;
        let mut humidity_frame_buf = self.humidity_frame_buf;

        // Activate EPD
        let activated_pins = self.activate_pins();
        self.en_epd.set_high().ok();
        self.epd
            .wake_up(&mut self.spi_dev, &mut self.epd_delay)
            .expect("EPD error");

        // Proceed full refresh
        self.update_counter += 1;
        if self.update_counter >= CLEAR_NUMBER_OF_UPDATES
            || update_status == UpdateStatus::UpdateBothFull
            || update_status == UpdateStatus::UpdateError
        {
            defmt::info!("Full screen refresh");
            self.epd
                .clear_frame(&mut self.spi_dev, &mut self.epd_delay)
                .expect("EPD error");
            self.update_counter = 0;
        }

        let mut render_and_update = |prev_frame_buf: &mut [u8; FRAME_SIZE],
                                     target: RenderTarget| {
            let mut new_frame_buf = [background_color; FRAME_SIZE];
            let new_frame = self.render_partial_frame(data, &mut new_frame_buf, &target);
            let y = match target {
                RenderTarget::Temperature => TEMPERATURE_LINE_Y,
                RenderTarget::Humidity => HUMIDITY_LINE_Y,
                RenderTarget::Error => ERROR_LINE_Y,
            };
            self.update_partial_frame(prev_frame_buf, new_frame, y);
            // Save buffer for the next update
            match target {
                RenderTarget::Temperature => {
                    self.temperature_frame_buf = new_frame_buf;
                }
                RenderTarget::Humidity => {
                    self.humidity_frame_buf = new_frame_buf;
                }
                RenderTarget::Error => (),
            }
        };

        match update_status {
            UpdateStatus::UpdateTemperature => {
                render_and_update(&mut temperature_frame_buf, RenderTarget::Temperature);
                self.epd
                    .display_partial_frame(
                        &mut self.spi_dev,
                        &mut self.epd_delay,
                        0,
                        TEMPERATURE_LINE_Y,
                        TEXT_WIDTH,
                        TEXT_HEIGHT,
                    )
                    .expect("EPD error");
            }
            UpdateStatus::UpdateHumidity => {
                render_and_update(&mut humidity_frame_buf, RenderTarget::Humidity);
                self.epd
                    .display_partial_frame(
                        &mut self.spi_dev,
                        &mut self.epd_delay,
                        0,
                        HUMIDITY_LINE_Y,
                        TEXT_WIDTH,
                        TEXT_HEIGHT,
                    )
                    .expect("EPD error");
            }
            UpdateStatus::UpdateBoth | UpdateStatus::UpdateBothFull => {
                render_and_update(&mut temperature_frame_buf, RenderTarget::Temperature);
                render_and_update(&mut humidity_frame_buf, RenderTarget::Humidity);
                self.epd
                    .display_partial_frame(
                        &mut self.spi_dev,
                        &mut self.epd_delay,
                        0,
                        BOTH_LINES_Y,
                        TEXT_WIDTH,
                        TEXT_HEIGHT * 2,
                    )
                    .expect("EPD error");
            }
            UpdateStatus::UpdateError => {
                render_and_update(&mut [background_color; FRAME_SIZE], RenderTarget::Error);
                self.epd
                    .display_partial_frame(
                        &mut self.spi_dev,
                        &mut self.epd_delay,
                        0,
                        BOTH_LINES_Y,
                        TEXT_WIDTH,
                        TEXT_HEIGHT * 2,
                    )
                    .expect("EPD error");
            }
            UpdateStatus::NoUpdate => unreachable!(),
        }

        // Deactivate EPD to save power
        self.epd
            .sleep(&mut self.spi_dev, &mut self.epd_delay)
            .expect("EPD error");
        self.deactivate_pins(activated_pins);
    }

    fn update_partial_frame<'a>(
        &mut self,
        prev_frame_buf: &'a [u8; FRAME_SIZE],
        new_frame: VarDisplay<'a, Color>,
        y: u32,
    ) {
        self.epd
            .update_partial_old_frame(
                &mut self.spi_dev,
                &mut self.epd_delay,
                prev_frame_buf,
                0,
                y,
                TEXT_WIDTH,
                TEXT_HEIGHT,
            )
            .expect("EPD error");
        self.epd
            .update_partial_new_frame(
                &mut self.spi_dev,
                &mut self.epd_delay,
                new_frame.buffer(),
                0,
                y,
                TEXT_WIDTH,
                TEXT_HEIGHT,
            )
            .expect("EPD error");
    }

    /// Deactive all EPD-related pins to decrease power consumption during sleep mode
    fn deactivate_pins(&mut self, activated_pins: ActivatedEpdPins) {
        self.en_epd.set_low().ok();

        _ = activated_pins.deactivate();
    }

    /// Reactivate previously deactivated pins
    fn activate_pins(&mut self) -> ActivatedEpdPins {
        let epd_pins = DeactivatedEpdPins::steal_epd_pins(&mut self.rcc);
        let activated_pins = epd_pins.reactivate();

        self.en_epd.set_high().ok();

        activated_pins
    }

    fn render_partial_frame<'a>(
        &mut self,
        data: &Data,
        buf: &'a mut [u8],
        target: &RenderTarget,
    ) -> PartialFrame<'a> {
        let mut frame = VarDisplay::new(TEXT_WIDTH, TEXT_HEIGHT, buf, false).unwrap();

        frame.set_rotation(DisplayRotation::Rotate180);

        let font = FontRenderer::new::<Font>();

        let mut buf = [0u8; 20];
        let message = match target {
            RenderTarget::Temperature => data.format_temperature_into_str(&mut buf),
            RenderTarget::Humidity => data.format_humidity_into_str(&mut buf),
            RenderTarget::Error => "Error",
        };

        frame.clear(self.epd.background_color().clone()).ok();
        font.render_aligned(
            message,
            frame.bounding_box().center(),
            VerticalPosition::Center,
            HorizontalAlignment::Center,
            // Inversed color works with both light and dark background
            FontColor::Transparent(self.epd.background_color().inverse()),
            &mut frame,
        )
        .expect("Render error");

        frame
    }
}
