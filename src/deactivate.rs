//! Makes illegal copy of EPD pins to deactivate them during sleep mode

use embedded_hal::digital::PinState;
use hal::{
    gpio::{Floating, GpioExt, Input, Output, PA2, PA6, PA8, PushPull},
    pac,
    rcc::Rcc,
};
use stm32g0xx_hal as hal;

pub struct DeactivatedEpdPins {
    epd_reset: PA2<Input<Floating>>,
    epd_dc: PA8<Input<Floating>>,
    epd_cs: PA6<Input<Floating>>,
}

pub struct ActivatedEpdPins {
    epd_reset: PA2<Output<PushPull>>,
    epd_dc: PA8<Output<PushPull>>,
    epd_cs: PA6<Output<PushPull>>,
}

impl DeactivatedEpdPins {
    pub fn steal_epd_pins(rcc: &mut Rcc) -> Self {
        let device = unsafe { pac::Peripherals::steal() };
        let gpioa = device.GPIOA.split(rcc);

        let epd_reset = gpioa.pa2.into_floating_input();
        let epd_dc = gpioa.pa8.into_floating_input();
        let epd_cs = gpioa.pa6.into_floating_input();

        DeactivatedEpdPins {
            epd_reset,
            epd_dc,
            epd_cs,
        }
    }

    pub fn reactivate(self) -> ActivatedEpdPins {
        let epd_reset = self
            .epd_reset
            .into_push_pull_output_in_state(PinState::High);
        let epd_dc = self.epd_dc.into_push_pull_output_in_state(PinState::High);
        let epd_cs = self.epd_cs.into_push_pull_output_in_state(PinState::High);
        ActivatedEpdPins {
            epd_reset,
            epd_dc,
            epd_cs,
        }
    }
}

impl ActivatedEpdPins {
    pub fn deactivate(self) -> DeactivatedEpdPins {
        let epd_reset = self.epd_reset.into_floating_input();
        let epd_dc = self.epd_dc.into_floating_input();
        let epd_cs = self.epd_cs.into_floating_input();
        DeactivatedEpdPins {
            epd_reset,
            epd_dc,
            epd_cs,
        }
    }
}
