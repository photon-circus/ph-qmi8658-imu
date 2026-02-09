use crate::register::{ctrl2, ctrl3};

/// Low-pass filter bandwidth selection (percent of ODR).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LowPassFilterMode {
    /// 2.62% of ODR.
    OdrPercent2_62,
    /// 3.59% of ODR.
    OdrPercent3_59,
    /// 5.32% of ODR.
    OdrPercent5_32,
    /// 14.0% of ODR.
    OdrPercent14_0,
}

impl LowPassFilterMode {
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::OdrPercent2_62 => 0b00,
            Self::OdrPercent3_59 => 0b01,
            Self::OdrPercent5_32 => 0b10,
            Self::OdrPercent14_0 => 0b11,
        }
    }
}

/// Accelerometer full-scale range selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelRange {
    /// +/-2 g range.
    G2,
    /// +/-4 g range.
    G4,
    /// +/-8 g range.
    G8,
    /// +/-16 g range.
    G16,
}

impl AccelRange {
    /// Returns the full-scale range in g.
    pub const fn g(self) -> u16 {
        match self {
            Self::G2 => 2,
            Self::G4 => 4,
            Self::G8 => 8,
            Self::G16 => 16,
        }
    }

    /// Returns the CTRL2 range bits.
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::G2 => 0b000,
            Self::G4 => 0b001,
            Self::G8 => 0b010,
            Self::G16 => 0b011,
        }
    }
}

/// Accelerometer output data rate selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelOutputDataRate {
    /// 8000 Hz output data rate (normal mode).
    Hz8000,
    /// 4000 Hz output data rate (normal mode).
    Hz4000,
    /// 2000 Hz output data rate (normal mode).
    Hz2000,
    /// 1000 Hz output data rate (normal mode).
    Hz1000,
    /// 500 Hz output data rate (normal mode).
    Hz500,
    /// 250 Hz output data rate (normal mode).
    Hz250,
    /// 125 Hz output data rate (normal mode).
    Hz125,
    /// 62.5 Hz output data rate (normal mode).
    Hz62_5,
    /// 31.25 Hz output data rate (normal mode).
    Hz31_25,
    /// 128 Hz output data rate (low power mode).
    LowPowerHz128,
    /// 21 Hz output data rate (low power mode).
    LowPowerHz21,
    /// 11 Hz output data rate (low power mode).
    LowPowerHz11,
    /// 3 Hz output data rate (low power mode).
    LowPowerHz3,
}

impl AccelOutputDataRate {
    /// Returns the accel-only output data rate in milli-hertz.
    pub const fn hz_milli(self) -> u32 {
        match self {
            Self::Hz8000 => 8_000_000,
            Self::Hz4000 => 4_000_000,
            Self::Hz2000 => 2_000_000,
            Self::Hz1000 => 1_000_000,
            Self::Hz500 => 500_000,
            Self::Hz250 => 250_000,
            Self::Hz125 => 125_000,
            Self::Hz62_5 => 62_500,
            Self::Hz31_25 => 31_250,
            Self::LowPowerHz128 => 128_000,
            Self::LowPowerHz21 => 21_000,
            Self::LowPowerHz11 => 11_000,
            Self::LowPowerHz3 => 3_000,
        }
    }

    /// Returns the 6DOF output data rate in milli-hertz when accel+gyro are enabled.
    /// Low-power settings are accel-only and return None.
    pub(crate) const fn hz_milli_6dof(self) -> Option<u32> {
        if self.is_low_power() {
            None
        } else {
            Some(Self::scale_0_94(self.hz_milli()))
        }
    }

    /// Returns the effective output data rate in milli-hertz for the current mode.
    pub(crate) const fn effective_hz_milli(self, gyro_enabled: bool) -> u32 {
        if gyro_enabled {
            match self.hz_milli_6dof() {
                Some(value) => value,
                None => self.hz_milli(),
            }
        } else {
            self.hz_milli()
        }
    }

    /// Returns true if the ODR setting selects low-power mode.
    pub(crate) const fn is_low_power(self) -> bool {
        matches!(
            self,
            Self::LowPowerHz128 | Self::LowPowerHz21 | Self::LowPowerHz11 | Self::LowPowerHz3
        )
    }

    const fn scale_0_94(value_milli: u32) -> u32 {
        ((value_milli as u64) * 94 / 100) as u32
    }

    /// Returns the CTRL2 ODR bits.
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Hz8000 => 0b0000,
            Self::Hz4000 => 0b0001,
            Self::Hz2000 => 0b0010,
            Self::Hz1000 => 0b0011,
            Self::Hz500 => 0b0100,
            Self::Hz250 => 0b0101,
            Self::Hz125 => 0b0110,
            Self::Hz62_5 => 0b0111,
            Self::Hz31_25 => 0b1000,
            Self::LowPowerHz128 => 0b1100,
            Self::LowPowerHz21 => 0b1101,
            Self::LowPowerHz11 => 0b1110,
            Self::LowPowerHz3 => 0b1111,
        }
    }

    /// Returns the data-read delay in microseconds for sync sample mode.
    pub(crate) const fn data_read_delay_us(self) -> u32 {
        match self {
            Self::Hz8000 | Self::Hz4000 => 2,
            Self::Hz2000 => 4,
            Self::Hz1000 => 6,
            Self::Hz500 => 12,
            Self::Hz250 => 24,
            Self::Hz125 | Self::Hz62_5 | Self::Hz31_25 => 48,
            Self::LowPowerHz128 => 40,
            Self::LowPowerHz21 => 100,
            Self::LowPowerHz11 => 200,
            Self::LowPowerHz3 => 270,
        }
    }
}

/// Combined accelerometer configuration (range + ODR).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelConfig {
    /// Accelerometer full-scale range.
    pub range: AccelRange,
    /// Accelerometer output data rate.
    pub odr: AccelOutputDataRate,
    /// Accelerometer low-pass filter mode (None disables LPF).
    pub lpf: Option<LowPassFilterMode>,
}

impl AccelConfig {
    /// Default accelerometer configuration.
    pub const DEFAULT: Self = Self {
        range: AccelRange::G8,
        odr: AccelOutputDataRate::Hz1000,
        lpf: None,
    };

    /// Creates a new accelerometer configuration.
    pub const fn new(range: AccelRange, odr: AccelOutputDataRate) -> Self {
        Self {
            range,
            odr,
            lpf: None,
        }
    }

    /// Returns a new configuration with the provided range.
    #[must_use]
    pub const fn with_range(self, range: AccelRange) -> Self {
        Self { range, ..self }
    }

    /// Returns a new configuration with the provided output data rate.
    #[must_use]
    pub const fn with_odr(self, odr: AccelOutputDataRate) -> Self {
        Self { odr, ..self }
    }

    /// Returns a new configuration with the low-pass filter enabled.
    #[must_use]
    pub const fn with_lpf(self, mode: LowPassFilterMode) -> Self {
        Self {
            lpf: Some(mode),
            ..self
        }
    }

    /// Returns a new configuration with the low-pass filter disabled.
    #[must_use]
    pub const fn without_lpf(self) -> Self {
        Self { lpf: None, ..self }
    }

    /// Builds the CTRL2 register value for this configuration.
    pub(crate) const fn ctrl2_value(self) -> u8 {
        ((self.range.bits() << ctrl2::A_FS_SHIFT) & ctrl2::A_FS_MASK)
            | (self.odr.bits() & ctrl2::A_ODR_MASK)
    }

    /// Builds the CTRL2 register value including self-test when requested.
    #[allow(dead_code)]
    pub(crate) const fn ctrl2_value_with_self_test(self, enable_self_test: bool) -> u8 {
        if enable_self_test {
            ctrl2::A_ST | self.ctrl2_value()
        } else {
            self.ctrl2_value()
        }
    }
}

impl Default for AccelConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Gyroscope full-scale range selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroRange {
    /// +/-16 deg/s.
    Dps16,
    /// +/-32 deg/s.
    Dps32,
    /// +/-64 deg/s.
    Dps64,
    /// +/-128 deg/s.
    Dps128,
    /// +/-256 deg/s.
    Dps256,
    /// +/-512 deg/s.
    Dps512,
    /// +/-1024 deg/s.
    Dps1024,
    /// +/-2048 deg/s.
    Dps2048,
}

impl GyroRange {
    /// Returns the full-scale range in degrees per second.
    pub const fn dps(self) -> u16 {
        match self {
            Self::Dps16 => 16,
            Self::Dps32 => 32,
            Self::Dps64 => 64,
            Self::Dps128 => 128,
            Self::Dps256 => 256,
            Self::Dps512 => 512,
            Self::Dps1024 => 1024,
            Self::Dps2048 => 2048,
        }
    }

    /// Returns the CTRL3 range bits.
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Dps16 => 0b000,
            Self::Dps32 => 0b001,
            Self::Dps64 => 0b010,
            Self::Dps128 => 0b011,
            Self::Dps256 => 0b100,
            Self::Dps512 => 0b101,
            Self::Dps1024 => 0b110,
            Self::Dps2048 => 0b111,
        }
    }
}

/// Gyroscope output data rate selection.
///
/// Actual rates follow the gyro natural frequency (~0.94x nominal).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroOutputDataRate {
    /// 8000 Hz output data rate (normal mode).
    Hz8000,
    /// 4000 Hz output data rate (normal mode).
    Hz4000,
    /// 2000 Hz output data rate (normal mode).
    Hz2000,
    /// 1000 Hz output data rate (normal mode).
    Hz1000,
    /// 500 Hz output data rate (normal mode).
    Hz500,
    /// 250 Hz output data rate (normal mode).
    Hz250,
    /// 125 Hz output data rate (normal mode).
    Hz125,
    /// 62.5 Hz output data rate (normal mode).
    Hz62_5,
    /// 31.25 Hz output data rate (normal mode).
    Hz31_25,
}

impl GyroOutputDataRate {
    /// Returns the output data rate in milli-hertz.
    pub const fn hz_milli(self) -> u32 {
        match self {
            Self::Hz8000 => 7_520_000,
            Self::Hz4000 => 3_760_000,
            Self::Hz2000 => 1_880_000,
            Self::Hz1000 => 940_000,
            Self::Hz500 => 470_000,
            Self::Hz250 => 235_000,
            Self::Hz125 => 117_500,
            Self::Hz62_5 => 58_750,
            Self::Hz31_25 => 29_375,
        }
    }

    /// Returns the CTRL3 ODR bits.
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Hz8000 => 0b0000,
            Self::Hz4000 => 0b0001,
            Self::Hz2000 => 0b0010,
            Self::Hz1000 => 0b0011,
            Self::Hz500 => 0b0100,
            Self::Hz250 => 0b0101,
            Self::Hz125 => 0b0110,
            Self::Hz62_5 => 0b0111,
            Self::Hz31_25 => 0b1000,
        }
    }

    /// Returns the data-read delay in microseconds for sync sample mode.
    pub(crate) const fn data_read_delay_us(self) -> u32 {
        match self {
            Self::Hz8000 | Self::Hz4000 => 2,
            Self::Hz2000 => 4,
            Self::Hz1000 => 6,
            Self::Hz500 | Self::Hz250 | Self::Hz125 | Self::Hz62_5 | Self::Hz31_25 => 12,
        }
    }
}

/// Combined gyroscope configuration (range + ODR).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroConfig {
    /// Gyroscope full-scale range.
    pub range: GyroRange,
    /// Gyroscope output data rate.
    pub odr: GyroOutputDataRate,
    /// Gyroscope low-pass filter mode (None disables LPF).
    pub lpf: Option<LowPassFilterMode>,
}

impl GyroConfig {
    /// Default gyroscope configuration.
    pub const DEFAULT: Self = Self {
        range: GyroRange::Dps512,
        odr: GyroOutputDataRate::Hz1000,
        lpf: None,
    };

    /// Creates a new gyroscope configuration.
    pub const fn new(range: GyroRange, odr: GyroOutputDataRate) -> Self {
        Self {
            range,
            odr,
            lpf: None,
        }
    }

    /// Returns a new configuration with the provided range.
    #[must_use]
    pub const fn with_range(self, range: GyroRange) -> Self {
        Self { range, ..self }
    }

    /// Returns a new configuration with the provided output data rate.
    #[must_use]
    pub const fn with_odr(self, odr: GyroOutputDataRate) -> Self {
        Self { odr, ..self }
    }

    /// Returns a new configuration with the low-pass filter enabled.
    #[must_use]
    pub const fn with_lpf(self, mode: LowPassFilterMode) -> Self {
        Self {
            lpf: Some(mode),
            ..self
        }
    }

    /// Returns a new configuration with the low-pass filter disabled.
    #[must_use]
    pub const fn without_lpf(self) -> Self {
        Self { lpf: None, ..self }
    }

    /// Builds the CTRL3 register value for this configuration.
    pub(crate) const fn ctrl3_value(self) -> u8 {
        ((self.range.bits() << ctrl3::G_FS_SHIFT) & ctrl3::G_FS_MASK)
            | (self.odr.bits() & ctrl3::G_ODR_MASK)
    }

    /// Builds the CTRL3 register value including self-test when requested.
    #[allow(dead_code)]
    pub(crate) const fn ctrl3_value_with_self_test(self, enable_self_test: bool) -> u8 {
        if enable_self_test {
            ctrl3::G_ST | self.ctrl3_value()
        } else {
            self.ctrl3_value()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accel_odr_values() {
        assert_eq!(AccelOutputDataRate::Hz1000.hz_milli(), 1_000_000);
        assert_eq!(AccelOutputDataRate::Hz500.hz_milli(), 500_000);
        assert_eq!(AccelOutputDataRate::LowPowerHz21.hz_milli(), 21_000);
    }

    #[test]
    fn accel_odr_effective_for_6dof() {
        let odr = AccelOutputDataRate::Hz1000;
        assert_eq!(odr.hz_milli_6dof(), Some(940_000));
        assert_eq!(odr.effective_hz_milli(true), 940_000);
        assert_eq!(odr.effective_hz_milli(false), 1_000_000);

        let low_power = AccelOutputDataRate::LowPowerHz21;
        assert_eq!(low_power.hz_milli_6dof(), None);
        assert_eq!(low_power.effective_hz_milli(true), 21_000);
    }

    #[test]
    fn gyro_odr_values() {
        assert_eq!(GyroOutputDataRate::Hz1000.hz_milli(), 940_000);
        assert_eq!(GyroOutputDataRate::Hz500.hz_milli(), 470_000);
    }

    #[test]
    fn data_read_delay_table() {
        assert_eq!(AccelOutputDataRate::Hz1000.data_read_delay_us(), 6);
        assert_eq!(AccelOutputDataRate::LowPowerHz3.data_read_delay_us(), 270);
        assert_eq!(GyroOutputDataRate::Hz8000.data_read_delay_us(), 2);
        assert_eq!(GyroOutputDataRate::Hz31_25.data_read_delay_us(), 12);
    }
}

impl Default for GyroConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}
