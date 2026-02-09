//! Interrupt routing and status decoding.

use crate::register::{ctrl8, status_int, status0, status1};

/// Interrupt pin selection (device pins).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptPin {
    /// Interrupt pin 1.
    Int1,
    /// Interrupt pin 2.
    Int2,
}

/// Interrupt routing and feature enable configuration (CTRL8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Use STATUSINT.bit7 for CTRL9 handshake when set.
    pub ctrl9_handshake_statusint: bool,
    /// Route motion detection events to the selected pin.
    pub motion_pin: InterruptPin,
    /// Enable pedometer interrupt generation.
    pub pedometer: bool,
    /// Enable significant motion interrupt generation.
    pub significant_motion: bool,
    /// Enable no motion interrupt generation.
    pub no_motion: bool,
    /// Enable any motion interrupt generation.
    pub any_motion: bool,
    /// Enable tap interrupt generation.
    pub tap: bool,
}

impl InterruptConfig {
    /// Default interrupt configuration.
    pub const DEFAULT: Self = Self {
        ctrl9_handshake_statusint: false,
        motion_pin: InterruptPin::Int1,
        pedometer: false,
        significant_motion: false,
        no_motion: false,
        any_motion: false,
        tap: false,
    };

    /// Creates a new interrupt configuration.
    pub const fn new() -> Self {
        Self::DEFAULT
    }

    /// Sets the pin used for motion detection events.
    #[must_use]
    pub const fn with_motion_pin(mut self, pin: InterruptPin) -> Self {
        self.motion_pin = pin;
        self
    }

    /// Enables or disables the CTRL9 handshake on STATUSINT.
    #[must_use]
    pub const fn with_ctrl9_handshake_statusint(mut self, enable: bool) -> Self {
        self.ctrl9_handshake_statusint = enable;
        self
    }

    /// Enables or disables tap interrupts.
    #[must_use]
    pub const fn with_tap(mut self, enable: bool) -> Self {
        self.tap = enable;
        self
    }

    /// Enables or disables any-motion interrupts.
    #[must_use]
    pub const fn with_any_motion(mut self, enable: bool) -> Self {
        self.any_motion = enable;
        self
    }

    /// Enables or disables no-motion interrupts.
    #[must_use]
    pub const fn with_no_motion(mut self, enable: bool) -> Self {
        self.no_motion = enable;
        self
    }

    /// Enables or disables significant-motion interrupts.
    #[must_use]
    pub const fn with_significant_motion(mut self, enable: bool) -> Self {
        self.significant_motion = enable;
        self
    }

    /// Enables or disables pedometer interrupts.
    #[must_use]
    pub const fn with_pedometer(mut self, enable: bool) -> Self {
        self.pedometer = enable;
        self
    }

    pub(crate) const fn ctrl8_value(self) -> u8 {
        let mut value = 0;
        if self.ctrl9_handshake_statusint {
            value |= ctrl8::CTRL9_HANDSHAKE_STATUSINT;
        }
        if matches!(self.motion_pin, InterruptPin::Int1) {
            value |= ctrl8::MOTION_INT_INT1;
        }
        if self.pedometer {
            value |= ctrl8::PEDOMETER_EN;
        }
        if self.significant_motion {
            value |= ctrl8::SIGNIFICANT_MOTION_EN;
        }
        if self.no_motion {
            value |= ctrl8::NO_MOTION_EN;
        }
        if self.any_motion {
            value |= ctrl8::ANY_MOTION_EN;
        }
        if self.tap {
            value |= ctrl8::TAP_EN;
        }
        value
    }
}

impl Default for InterruptConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Decoded interrupt and status flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptStatus {
    /// CTRL9 command done.
    pub cmd_done: bool,
    /// Data locked flag.
    pub data_locked: bool,
    /// Data available flag.
    pub data_available: bool,
    /// Accelerometer data ready.
    pub accel_ready: bool,
    /// Gyroscope data ready.
    pub gyro_ready: bool,
    /// AttitudeEngine data ready.
    pub ae_ready: bool,
    /// Tap detected.
    pub tap: bool,
    /// Any motion detected.
    pub any_motion: bool,
    /// No motion detected.
    pub no_motion: bool,
    /// Significant motion detected.
    pub significant_motion: bool,
    /// Wake on Motion detected.
    pub wake_on_motion: bool,
    /// Pedometer detected.
    pub pedometer: bool,
}

impl InterruptStatus {
    pub(crate) const fn from_regs(status_int_reg: u8, status0_reg: u8, status1_reg: u8) -> Self {
        Self {
            cmd_done: (status_int_reg & status_int::CMD_DONE) != 0
                || (status1_reg & status1::CMD_DONE) != 0,
            data_locked: (status_int_reg & status_int::LOCKED) != 0,
            data_available: (status_int_reg & status_int::AVAIL) != 0,
            accel_ready: (status0_reg & status0::ACCEL_AVAIL) != 0,
            gyro_ready: (status0_reg & status0::GYRO_AVAIL) != 0,
            ae_ready: (status0_reg & status0::AE_AVAIL) != 0,
            tap: (status1_reg & status1::TAP) != 0,
            any_motion: (status1_reg & status1::ANY_MOTION) != 0,
            no_motion: (status1_reg & status1::NO_MOTION) != 0,
            significant_motion: (status1_reg & status1::SIGNIFICANT_MOTION) != 0,
            wake_on_motion: (status1_reg & status1::WOM) != 0,
            pedometer: (status1_reg & status1::PEDOMETER) != 0,
        }
    }

    /// Returns true if any status bit is set.
    pub const fn any(self) -> bool {
        self.cmd_done
            || self.data_locked
            || self.data_available
            || self.accel_ready
            || self.gyro_ready
            || self.ae_ready
            || self.tap
            || self.any_motion
            || self.no_motion
            || self.significant_motion
            || self.wake_on_motion
            || self.pedometer
    }
}

/// Error returned when waiting on an interrupt pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptWaitError<E> {
    /// Interrupt pin was not provided to the driver.
    Missing,
    /// Underlying pin error.
    Pin(E),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::register::{status_int, status0, status1};

    #[test]
    fn decode_interrupt_status() {
        let status = InterruptStatus::from_regs(
            status_int::AVAIL,
            status0::ACCEL_AVAIL | status0::GYRO_AVAIL,
            status1::TAP | status1::ANY_MOTION | status1::CMD_DONE | status1::WOM,
        );

        assert!(status.cmd_done);
        assert!(status.data_available);
        assert!(status.accel_ready);
        assert!(status.gyro_ready);
        assert!(status.tap);
        assert!(status.any_motion);
        assert!(status.wake_on_motion);
        assert!(!status.no_motion);
        assert!(status.any());
    }
}
