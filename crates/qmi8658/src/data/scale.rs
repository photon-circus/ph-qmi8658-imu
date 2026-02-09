//! Integer scaling helpers for raw sensor data.

use crate::config::common::{AccelRange, GyroRange};

/// Ratio representing a scale factor without floating-point math.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScaleFactor {
    /// Scale numerator.
    pub numerator: i32,
    /// Scale denominator.
    pub denominator: i32,
}

impl ScaleFactor {
    /// Creates a new scale ratio.
    pub const fn new(numerator: i32, denominator: i32) -> Self {
        Self {
            numerator,
            denominator,
        }
    }
}

/// Returns the accelerometer sensitivity in LSB/g.
pub const fn accel_lsb_per_g(range: AccelRange) -> i32 {
    match range {
        AccelRange::G2 => 16_384,
        AccelRange::G4 => 8_192,
        AccelRange::G8 => 4_096,
        AccelRange::G16 => 2_048,
    }
}

/// Returns the gyroscope sensitivity in LSB/dps.
pub const fn gyro_lsb_per_dps(range: GyroRange) -> i32 {
    match range {
        GyroRange::Dps16 => 2_048,
        GyroRange::Dps32 => 1_024,
        GyroRange::Dps64 => 512,
        GyroRange::Dps128 => 256,
        GyroRange::Dps256 => 128,
        GyroRange::Dps512 => 64,
        GyroRange::Dps1024 => 32,
        GyroRange::Dps2048 => 16,
    }
}

/// Returns the temperature sensitivity in LSB per degree Celsius.
pub const fn temperature_lsb_per_celsius() -> i32 {
    256
}

/// Returns the accelerometer scale in milli-g per LSB as a ratio.
pub const fn accel_mg_per_lsb(range: AccelRange) -> ScaleFactor {
    ScaleFactor::new(1000, accel_lsb_per_g(range))
}

/// Returns the gyroscope scale in milli-deg/s per LSB as a ratio.
pub const fn gyro_mdps_per_lsb(range: GyroRange) -> ScaleFactor {
    ScaleFactor::new(1000, gyro_lsb_per_dps(range))
}

/// Returns the temperature scale in milli-deg C per LSB as a ratio.
pub const fn temperature_mdegc_per_lsb() -> ScaleFactor {
    ScaleFactor::new(1000, temperature_lsb_per_celsius())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accel_lsb_per_g_values() {
        assert_eq!(accel_lsb_per_g(AccelRange::G2), 16_384);
        assert_eq!(accel_lsb_per_g(AccelRange::G16), 2_048);
    }

    #[test]
    fn gyro_lsb_per_dps_values() {
        assert_eq!(gyro_lsb_per_dps(GyroRange::Dps16), 2_048);
        assert_eq!(gyro_lsb_per_dps(GyroRange::Dps2048), 16);
    }

    #[test]
    fn temperature_scale_values() {
        assert_eq!(temperature_lsb_per_celsius(), 256);
        assert_eq!(
            temperature_mdegc_per_lsb(),
            ScaleFactor::new(1000, 256)
        );
    }
}
