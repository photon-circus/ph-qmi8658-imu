//! Configuration helpers for the QMI8658.

pub(crate) mod common;
pub(crate) mod mode;

pub use common::{AccelConfig, AccelOutputDataRate, AccelRange, LowPassFilterMode};
pub use common::{GyroConfig, GyroOutputDataRate, GyroRange};
pub use mode::OperatingMode;
pub(crate) use mode::{OperatingModeStateMachine, TransitionDelay, transition_delay};

use crate::error::Error;
use crate::register::{ctrl5, ctrl7};

/// QMI8658 configuration settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Accelerometer configuration (range + output data rate). None disables the accelerometer.
    pub accel: Option<AccelConfig>,
    /// Gyroscope configuration (range + output data rate). None disables the gyroscope.
    pub gyro: Option<GyroConfig>,
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    /// Creates a default configuration.
    pub const fn new() -> Self {
        Self {
            accel: Some(AccelConfig::DEFAULT),
            gyro: Some(GyroConfig::DEFAULT),
        }
    }

    /// Sets the accelerometer configuration.
    #[must_use]
    pub const fn with_accel_config(mut self, accel: AccelConfig) -> Self {
        self.accel = Some(accel);
        self
    }

    /// Disables the accelerometer.
    #[must_use]
    pub const fn without_accel(mut self) -> Self {
        self.accel = None;
        self
    }

    /// Sets the gyroscope configuration.
    #[must_use]
    pub const fn with_gyro_config(mut self, gyro: GyroConfig) -> Self {
        self.gyro = Some(gyro);
        self
    }

    /// Disables the gyroscope.
    #[must_use]
    pub const fn without_gyro(mut self) -> Self {
        self.gyro = None;
        self
    }

    pub(crate) const fn accel_enabled(self) -> bool {
        self.accel.is_some()
    }

    pub(crate) const fn gyro_enabled(self) -> bool {
        self.gyro.is_some()
    }

    pub(crate) const fn ctrl2_value(self) -> u8 {
        match self.accel {
            Some(accel) => accel.ctrl2_value(),
            None => 0,
        }
    }

    pub(crate) const fn ctrl3_value(self) -> u8 {
        match self.gyro {
            Some(gyro) => gyro.ctrl3_value(),
            None => 0,
        }
    }

    pub(crate) const fn ctrl5_value(self) -> u8 {
        let mut value = 0;
        if let Some(gyro) = self.gyro
            && let Some(mode) = gyro.lpf
        {
            value |= ctrl5::G_LPF_EN;
            value |= (mode.bits() << ctrl5::G_LPF_MODE_SHIFT) & ctrl5::G_LPF_MODE_MASK;
        }
        if let Some(accel) = self.accel
            && let Some(mode) = accel.lpf
        {
            value |= ctrl5::A_LPF_EN;
            value |= (mode.bits() << ctrl5::A_LPF_MODE_SHIFT) & ctrl5::A_LPF_MODE_MASK;
        }
        value
    }

    pub(crate) const fn ctrl7_value(self) -> u8 {
        let mut value = 0;
        if self.accel_enabled() {
            value |= ctrl7::A_EN;
        }
        if self.gyro_enabled() {
            value |= ctrl7::G_EN;
        }
        value
    }

    pub(crate) fn validate(self) -> Result<(), Error> {
        if let (Some(accel), Some(_gyro)) = (self.accel, self.gyro)
            && accel.odr.is_low_power()
        {
            return Err(Error::InvalidData);
        }
        Ok(())
    }

    pub(crate) const fn target_mode(self) -> OperatingMode {
        match (self.accel, self.gyro) {
            (None, None) => OperatingMode::PowerOnDefault,
            (Some(accel), None) => {
                if accel.odr.is_low_power() {
                    OperatingMode::LowPowerAccel
                } else {
                    OperatingMode::AccelOnly
                }
            }
            (None, Some(_)) => OperatingMode::GyroOnly,
            (Some(_), Some(_)) => OperatingMode::AccelGyroOnly,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::common::{AccelConfig, AccelOutputDataRate, AccelRange, GyroConfig};
    use super::*;

    #[test]
    fn validate_rejects_low_power_accel_with_gyro() {
        let accel = AccelConfig::new(AccelRange::G8, AccelOutputDataRate::LowPowerHz21);
        let config = Config::new().with_accel_config(accel);
        assert_eq!(config.validate(), Err(Error::InvalidData));
    }

    #[test]
    fn validate_allows_low_power_accel_without_gyro() {
        let accel = AccelConfig::new(AccelRange::G8, AccelOutputDataRate::LowPowerHz21);
        let config = Config::new().with_accel_config(accel).without_gyro();
        assert_eq!(config.validate(), Ok(()));
    }

    #[test]
    fn target_mode_tracks_config() {
        let accel = AccelConfig::new(AccelRange::G8, AccelOutputDataRate::LowPowerHz21);
        let gyro = GyroConfig::DEFAULT;
        let config = Config::new().with_accel_config(accel).without_gyro();
        assert_eq!(config.target_mode(), OperatingMode::LowPowerAccel);

        let config = Config::new().with_gyro_config(gyro).without_accel();
        assert_eq!(config.target_mode(), OperatingMode::GyroOnly);

        let config = Config::new().without_accel().without_gyro();
        assert_eq!(config.target_mode(), OperatingMode::PowerOnDefault);
    }
}
