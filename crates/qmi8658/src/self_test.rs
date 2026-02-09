//! Self-test helpers for accelerometer and gyroscope.

use crate::error::Error;
use crate::interrupt::InterruptWaitError;

/// Self-test axis readings.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestAxis {
    /// X-axis self-test reading.
    pub x: i16,
    /// Y-axis self-test reading.
    pub y: i16,
    /// Z-axis self-test reading.
    pub z: i16,
}

impl SelfTestAxis {
    pub(crate) fn from_bytes(bytes: [u8; 6], big_endian: bool) -> Self {
        let (x, y, z) = if big_endian {
            (
                i16::from_be_bytes([bytes[0], bytes[1]]),
                i16::from_be_bytes([bytes[2], bytes[3]]),
                i16::from_be_bytes([bytes[4], bytes[5]]),
            )
        } else {
            (
                i16::from_le_bytes([bytes[0], bytes[1]]),
                i16::from_le_bytes([bytes[2], bytes[3]]),
                i16::from_le_bytes([bytes[4], bytes[5]]),
            )
        };
        Self { x, y, z }
    }

    fn abs_exceeds(self, threshold_lsb: i16) -> bool {
        abs_i16(self.x) >= threshold_lsb
            && abs_i16(self.y) >= threshold_lsb
            && abs_i16(self.z) >= threshold_lsb
    }
}

/// Self-test report with raw data and pass/fail evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestReport {
    /// Raw self-test axis readings.
    pub raw: SelfTestAxis,
    /// Pass/fail threshold in LSB.
    pub threshold_lsb: i16,
    /// Whether all axes exceeded the threshold.
    pub passed: bool,
}

impl SelfTestReport {
    pub(crate) fn accel(raw: SelfTestAxis) -> Self {
        let threshold = accel_threshold_lsb();
        Self {
            raw,
            threshold_lsb: threshold,
            passed: raw.abs_exceeds(threshold),
        }
    }

    pub(crate) fn gyro(raw: SelfTestAxis) -> Self {
        let threshold = gyro_threshold_lsb();
        Self {
            raw,
            threshold_lsb: threshold,
            passed: raw.abs_exceeds(threshold),
        }
    }
}

/// Self-test error wrapper (driver errors + interrupt wait errors).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestError<E> {
    /// A driver-level error occurred.
    Driver(Error),
    /// An interrupt wait error occurred.
    Interrupt(InterruptWaitError<E>),
}

impl<E> From<Error> for SelfTestError<E> {
    fn from(err: Error) -> Self {
        Self::Driver(err)
    }
}

impl<E> From<InterruptWaitError<E>> for SelfTestError<E> {
    fn from(err: InterruptWaitError<E>) -> Self {
        Self::Interrupt(err)
    }
}

const fn accel_threshold_lsb() -> i16 {
    // 200 mg threshold, data format is signed Q5.11 (1 g = 2048 LSB).
    // 200 mg = 0.2 g => 0.2 * 2048 = 409.6 -> 410 LSB.
    ((200i32 * 2048 + 999) / 1000) as i16
}

const fn gyro_threshold_lsb() -> i16 {
    // 300 dps threshold, data format is signed Q12.4 (1 dps = 16 LSB).
    (300i32 * 16) as i16
}

const fn abs_i16(value: i16) -> i16 {
    if value < 0 {
        (-(value as i32)) as i16
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accel_threshold_is_expected() {
        assert_eq!(accel_threshold_lsb(), 410);
    }

    #[test]
    fn gyro_threshold_is_expected() {
        assert_eq!(gyro_threshold_lsb(), 4800);
    }

    #[test]
    fn report_passes_when_above_threshold() {
        let raw = SelfTestAxis {
            x: 500,
            y: -500,
            z: 600,
        };
        let report = SelfTestReport::accel(raw);
        assert!(report.passed);
    }
}
