//! Fixed-point conversion helpers.

use super::{AccelRaw, GyroRaw, Sample, TemperatureRaw};
use super::scale::{accel_lsb_per_g, gyro_lsb_per_dps, temperature_lsb_per_celsius};
use crate::config::common::{AccelRange, GyroRange};

/// Fixed-point number type used for sensor conversions (I32F32).
pub type Fixed = crate::fixed_crate::types::I32F32;

/// Fixed-point accelerometer sample in g.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AccelFixed {
    /// X-axis acceleration in g.
    pub x: Fixed,
    /// Y-axis acceleration in g.
    pub y: Fixed,
    /// Z-axis acceleration in g.
    pub z: Fixed,
}

/// Fixed-point gyroscope sample in dps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GyroFixed {
    /// X-axis angular rate in dps.
    pub x: Fixed,
    /// Y-axis angular rate in dps.
    pub y: Fixed,
    /// Z-axis angular rate in dps.
    pub z: Fixed,
}

/// Fixed-point temperature sample in deg C.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TemperatureFixed {
    /// Temperature in degrees Celsius.
    pub celsius: Fixed,
}

/// Converts accelerometer raw counts to g.
pub fn accel_to_g(raw: AccelRaw, range: AccelRange) -> AccelFixed {
    let scale = Fixed::from_num(accel_lsb_per_g(range));
    AccelFixed {
        x: Fixed::from_num(raw.x) / scale,
        y: Fixed::from_num(raw.y) / scale,
        z: Fixed::from_num(raw.z) / scale,
    }
}

/// Converts gyroscope raw counts to dps.
pub fn gyro_to_dps(raw: GyroRaw, range: GyroRange) -> GyroFixed {
    let scale = Fixed::from_num(gyro_lsb_per_dps(range));
    GyroFixed {
        x: Fixed::from_num(raw.x) / scale,
        y: Fixed::from_num(raw.y) / scale,
        z: Fixed::from_num(raw.z) / scale,
    }
}

/// Converts temperature raw counts to degrees Celsius.
///
/// The datasheet specifies 1 LSB = 1/256 deg C.
pub fn temperature_celsius(raw: TemperatureRaw) -> TemperatureFixed {
    let scale = Fixed::from_num(temperature_lsb_per_celsius());
    TemperatureFixed {
        celsius: Fixed::from_num(raw.value) / scale,
    }
}

/// Converts a timestamped accelerometer sample to g.
pub fn accel_sample_to_g(sample: Sample<AccelRaw>, range: AccelRange) -> Sample<AccelFixed> {
    Sample {
        timestamp: sample.timestamp,
        data: accel_to_g(sample.data, range),
    }
}

/// Converts a timestamped gyroscope sample to dps.
pub fn gyro_sample_to_dps(sample: Sample<GyroRaw>, range: GyroRange) -> Sample<GyroFixed> {
    Sample {
        timestamp: sample.timestamp,
        data: gyro_to_dps(sample.data, range),
    }
}

/// Converts a timestamped temperature sample to degrees Celsius.
pub fn temperature_sample_celsius(sample: Sample<TemperatureRaw>) -> Sample<TemperatureFixed> {
    Sample {
        timestamp: sample.timestamp,
        data: temperature_celsius(sample.data),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::common::{AccelRange, GyroRange};
    use crate::data::Timestamp;

    #[test]
    fn accel_conversion_matches_scale() {
        let raw = AccelRaw {
            x: 16_384,
            y: -16_384,
            z: 0,
        };
        let fixed = accel_to_g(raw, AccelRange::G2);
        assert_eq!(fixed.x, Fixed::from_num(1));
        assert_eq!(fixed.y, Fixed::from_num(-1));
        assert_eq!(fixed.z, Fixed::from_num(0));
    }

    #[test]
    fn gyro_conversion_matches_scale() {
        let raw = GyroRaw {
            x: 2_048,
            y: -2_048,
            z: 0,
        };
        let fixed = gyro_to_dps(raw, GyroRange::Dps16);
        assert_eq!(fixed.x, Fixed::from_num(1));
        assert_eq!(fixed.y, Fixed::from_num(-1));
        assert_eq!(fixed.z, Fixed::from_num(0));
    }

    #[test]
    fn temperature_conversion_matches_scale() {
        let raw = TemperatureRaw { value: 256 };
        let fixed = temperature_celsius(raw);
        assert_eq!(fixed.celsius, Fixed::from_num(1));
    }

    #[test]
    fn sample_conversion_preserves_timestamp() {
        let sample = Sample {
            timestamp: Timestamp { ticks: 123 },
            data: AccelRaw { x: 0, y: 0, z: 0 },
        };
        let fixed = accel_sample_to_g(sample, AccelRange::G8);
        assert_eq!(fixed.timestamp.ticks, 123);
    }
}
