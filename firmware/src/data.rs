//! Data processing for SHT4x sensor measurements

use fixed::types::*;


/// Contains temperature and humidity
pub struct Data(Option<DataInner>);

#[derive(Debug, PartialEq, Clone)]
struct DataInner {
    temperature: I16F16,
    humidity: I16F16,
}

#[derive(Debug, PartialEq)]
pub enum UpdateStatus {
    NoUpdate,
    UpdateTemperature,
    UpdateHumidity,
    UpdateBoth,
}

impl Data {

    pub fn empty() -> Self {
        Data(None)
    }

    pub fn from_sensor_measurement<Err>(measurement: Result<sht4x::Measurement, Err>) -> Self {

        match measurement {
            Ok(m) => Data::from_values(
                m.temperature_celsius(),
                m.humidity_percent()
            ),
            Err(_) => Data(None),
        }
    }

    fn from_values(temperature: I16F16, humidity: I16F16) -> Self {

        // Keep one decimal digit in a fractional part
        let temperature = temperature * 10;
        let temperature = temperature.round() / 10;

        let humidity = humidity.round();

        Data(Some(DataInner { temperature, humidity }))
    }

    pub fn check_update(&self, prev: &Data) -> UpdateStatus {

        match (prev.0.as_ref(), (*self).0.as_ref()) {
            (Some(DataInner { temperature: prev_temp, humidity: prev_hum }), Some(DataInner { temperature: new_temp, humidity: new_hum })) => {
                let temp_changed = prev_temp != new_temp;
                let hum_changed = prev_hum != new_hum;

                match (temp_changed, hum_changed) {
                    (false, false) => UpdateStatus::NoUpdate,
                    (true, false) => UpdateStatus::UpdateTemperature,
                    (false, true) => UpdateStatus::UpdateHumidity,
                    (true, true) => UpdateStatus::UpdateBoth,
                }
            },
            _ => UpdateStatus::UpdateBoth,
        }
    }

    pub fn format_into_str<'a>(&self, buf: &'a mut [u8; 40]) -> &'a str {

        match self.0.as_ref() {
            Some(DataInner { temperature, humidity }) => {
                defmt::info!("Temperature: {}°C Humidity: {}%", temperature, humidity);
                format_no_std::show(buf, format_args!("{}°C\n{}%", temperature, humidity,))
                    .expect("Buffer for formatted data is too small")
            }
            None => {
                defmt::info!("Sensor reading error");
                "Error"
            }
        }
    }

    pub fn format_temperature_into_str<'a>(&self, buf: &'a mut [u8; 20]) -> &'a str {

        match self.0.as_ref() {
            Some(DataInner { temperature, humidity }) => {
                defmt::info!("Temperature: {}°C Humidity: {}%", temperature, humidity);
                format_no_std::show(buf, format_args!("{}°C", temperature))
                    .expect("Buffer for formatted data is too small")
            }
            None => {
                defmt::info!("Sensor reading error");
                "Error"
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::{assert_eq, assert_ne};


    #[test]
    fn round_values() {
        let data = Data::from_values(I16F16::from_num(23.456), I16F16::from_num(56.789));

        let mut buf = [0u8; 40];
        let formatted = data.format_into_str(&mut buf);
        assert_eq!(formatted, "23.5°C\n57%");
    }

    #[test]
    fn format_valid_data() {
        let data = Data(Some(DataInner { temperature: I16F16::from_num(36.6), humidity: I16F16::from_num(57) }));

        let mut buf = [0u8; 40];
        let formatted = data.format_into_str(&mut buf);
        assert_eq!(formatted, "36.6°C\n57%");
    }

    #[test]
    fn format_not_valid_data() {
        let data = Data(None);

        let mut buf = [0u8; 40];
        let formatted = data.format_into_str(&mut buf);
        assert_eq!(formatted, "Error");
    }

    #[test]
    fn update_temperature() {
        let prev = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));
        let new = Data(Some(DataInner { temperature: I16F16::from_num(24.0), humidity: I16F16::from_num(56) }));
        
        assert_eq!(new.check_update(&prev), UpdateStatus::UpdateTemperature);
    }

    #[test]
    fn update_humidity() {
        let prev = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));
        let new = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(57) }));

        assert_eq!(new.check_update(&prev), UpdateStatus::UpdateHumidity);
    }

    #[test]
    fn update_both() {
        let prev = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));
        let new = Data(Some(DataInner { temperature: I16F16::from_num(24.0), humidity: I16F16::from_num(57) }));

        assert_eq!(new.check_update(&prev), UpdateStatus::UpdateBoth);
    }

    #[test]
    fn no_update() {
        let prev = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));
        let new = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));

        assert_eq!(new.check_update(&prev), UpdateStatus::NoUpdate);
    }

    #[test]
    fn handle_none() {
        let prev = Data(None);
        let new = Data(None);
        assert_eq!(new.check_update(&prev), UpdateStatus::UpdateBoth);
    }

    #[test]
    fn handle_none_to_some() {
        let prev = Data(None);
        let new = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));
        assert_eq!(new.check_update(&prev), UpdateStatus::UpdateBoth);
    }

    #[test]
    fn handle_some_to_none() {
        let prev = Data(Some(DataInner { temperature: I16F16::from_num(23.0), humidity: I16F16::from_num(56) }));
        let new = Data(None);
        assert_eq!(new.check_update(&prev), UpdateStatus::UpdateBoth);
    }
}
