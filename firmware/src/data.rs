//! Data processing for SHT4x sensor measurements

use fixed::types::*;

#[derive(Debug, PartialEq, Clone, Default)]
pub struct Data {
    pub temperature: I16F16,
    pub humidity: I16F16,
}

#[derive(Debug, PartialEq)]
pub enum UpdateStatus {
    NoUpdate,
    UpdateTemperature,
    UpdateHumidity,
    UpdateBoth,
    // Error recovery: clear screen before full update to remove error message
    UpdateBothFull,
    UpdateError,
}

impl Data {
    pub fn from_sensor_measurement<Err>(
        measurement: Result<sht4x::Measurement, Err>,
    ) -> Option<Self> {
        match measurement {
            Ok(m) => Some(Data::from_values(
                m.temperature_celsius(),
                m.humidity_percent(),
            )),
            Err(_) => None,
        }
    }

    fn from_values(temperature: I16F16, humidity: I16F16) -> Self {
        // Keep one decimal digit in a fractional part
        let temperature = temperature * 10;
        let temperature = temperature.round() / 10;

        let humidity = humidity.round();

        Data {
            temperature,
            humidity,
        }
    }

    pub fn check_update(current: Option<&Self>, prev: Option<&Data>) -> UpdateStatus {
        if current.is_none() {
            return UpdateStatus::UpdateError;
        }

        if prev.is_none() {
            return UpdateStatus::UpdateBothFull;
        }

        let current = current.unwrap();
        let prev = prev.unwrap();

        let temperature_changed = prev.temperature != current.temperature;
        let humidity_changed = prev.humidity != current.humidity;

        match (temperature_changed, humidity_changed) {
            (false, false) => UpdateStatus::NoUpdate,
            (true, false) => UpdateStatus::UpdateTemperature,
            (false, true) => UpdateStatus::UpdateHumidity,
            (true, true) => UpdateStatus::UpdateBoth,
        }
    }

    pub fn format_temperature_into_str<'a>(&self, buf: &'a mut [u8; 20]) -> &'a str {
        format_no_std::show(buf, format_args!("{}°C", self.temperature))
            .expect("Buffer for formatted data is too small")
    }

    pub fn format_humidity_into_str<'a>(&self, buf: &'a mut [u8; 20]) -> &'a str {
        format_no_std::show(buf, format_args!("{}%", self.humidity))
            .expect("Buffer for formatted data is too small")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::{assert_eq, assert_ne};

    #[test]
    fn round_temperature() {
        let data = Data::from_values(I16F16::from_num(23.456), I16F16::from_num(56.789));

        let mut buf = [0u8; 20];
        let formatted = data.format_temperature_into_str(&mut buf);
        assert_eq!(formatted, "23.5°C");
    }

    #[test]
    fn round_humidity() {
        let data = Data::from_values(I16F16::from_num(23.456), I16F16::from_num(56.789));

        let mut buf = [0u8; 20];
        let formatted = data.format_humidity_into_str(&mut buf);
        assert_eq!(formatted, "57%");
    }

    #[test]
    fn format_temperature() {
        let data = Data {
            temperature: I16F16::from_num(36.6),
            humidity: I16F16::from_num(57),
        };

        let mut buf = [0u8; 20];
        let formatted = data.format_temperature_into_str(&mut buf);
        assert_eq!(formatted, "36.6°C");
    }

    #[test]
    fn format_humidity() {
        let data = Data {
            temperature: I16F16::from_num(36.6),
            humidity: I16F16::from_num(57),
        };

        let mut buf = [0u8; 20];
        let formatted = data.format_humidity_into_str(&mut buf);
        assert_eq!(formatted, "57%");
    }

    #[test]
    fn update_temperature() {
        let prev = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };
        let new = Data {
            temperature: I16F16::from_num(24.0),
            humidity: I16F16::from_num(56),
        };

        assert_eq!(
            Data::check_update(Some(&new), Some(&prev)),
            UpdateStatus::UpdateTemperature
        );
    }

    #[test]
    fn update_humidity() {
        let prev = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };
        let new = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(57),
        };

        assert_eq!(
            Data::check_update(Some(&new), Some(&prev)),
            UpdateStatus::UpdateHumidity
        );
    }

    #[test]
    fn update_both() {
        let prev = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };
        let new = Data {
            temperature: I16F16::from_num(24.0),
            humidity: I16F16::from_num(57),
        };

        assert_eq!(
            Data::check_update(Some(&new), Some(&prev)),
            UpdateStatus::UpdateBoth
        );
    }

    #[test]
    fn no_update() {
        let prev = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };
        let new = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };

        assert_eq!(
            Data::check_update(Some(&new), Some(&prev)),
            UpdateStatus::NoUpdate
        );
    }

    #[test]
    fn handle_none() {
        assert_eq!(Data::check_update(None, None), UpdateStatus::UpdateError);
    }

    #[test]
    fn handle_none_to_some() {
        let new = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };

        assert_eq!(
            Data::check_update(Some(&new), None),
            UpdateStatus::UpdateBothFull
        );
    }

    #[test]
    fn handle_some_to_none() {
        let prev = Data {
            temperature: I16F16::from_num(23.0),
            humidity: I16F16::from_num(56),
        };

        assert_eq!(
            Data::check_update(None, Some(&prev)),
            UpdateStatus::UpdateError
        );
    }
}
