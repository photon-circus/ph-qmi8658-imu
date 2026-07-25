use crate::register::ctrl9;

/// Groups of controllable pull-up resistors (per datasheet Table 30).
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PullUpGroup {
    /// Auxiliary pin group: SDx, SCx, and RESV-NC (Pin 10).
    Aux,
    /// SDx pin individual control.
    Sdx,
    /// CS (Chip Select) pin.
    Cs,
    /// SCL and SDA pins, shared by I2C and SPI host interfaces.
    SclSda,
}

/// Configuration for pull-up resistor control.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PullUpConfig {
    /// Auxiliary group pull-up state (true: disabled, false: enabled).
    aux_disable: bool,
    /// SDx pin pull-up state (true: disabled, false: enabled).
    sdx_disable: bool,
    /// CS pin pull-up state (true: disabled, false: enabled).
    cs_disable: bool,
    /// I2C group pull-up state (true: disabled, false: enabled).
    scl_sda_disable: bool,
}

/// Bit mapping for pull-up disable flags in the CAL1_L register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
enum PullUpDisableBits {
    /// Auxiliary group disable bit (bit 0).
    Aux = 0x01,
    /// SDx pin disable bit (bit 1).
    Sdx = 0x02,
    /// CS pin disable bit (bit 2).
    Cs = 0x04,
    /// I2C group disable bit (bit 3).
    SclSda = 0x08,
}

impl PullUpConfig {
    /// Default configuration: All pull-up resistors enabled (matches POR state).
    pub const DEFAULT: Self = Self {
        aux_disable: false,
        sdx_disable: false,
        cs_disable: false,
        scl_sda_disable: false,
    };

    /// Creates a new default pull-up configuration.
    pub const fn new() -> Self {
        Self::DEFAULT
    }

    /// Sets the pull-up state for a specific group.
    /// - `group`: The target pull-up resistor group.
    /// - `disable`: true to disable, false to enable.
    #[must_use]
    pub const fn with_group(mut self, group: PullUpGroup, disable: bool) -> Self {
        match group {
            PullUpGroup::Aux => self.aux_disable = disable,
            PullUpGroup::Sdx => self.sdx_disable = disable,
            PullUpGroup::Cs => self.cs_disable = disable,
            PullUpGroup::SclSda => self.scl_sda_disable = disable,
        }
        self
    }

    /// Disables all pull-up resistors (recommended for lowest power consumption).
    #[must_use]
    pub const fn disable_all(mut self) -> Self {
        self.aux_disable = true;
        self.sdx_disable = true;
        self.cs_disable = true;
        self.scl_sda_disable = true;
        self
    }

    /// Generates the raw CAL1_L register value for CTRL9 commands.
    /// Mapping: Bits 0-3 correspond to group disable flags (1 = disabled, 0 = enabled).
    pub(crate) const fn cal1_l(self) -> u8 {
        let mut bits = 0x00;
        if self.aux_disable {
            bits |= PullUpDisableBits::Aux as u8;
        }
        if self.sdx_disable {
            bits |= PullUpDisableBits::Sdx as u8;
        }
        if self.cs_disable {
            bits |= PullUpDisableBits::Cs as u8;
        }
        if self.scl_sda_disable {
            bits |= PullUpDisableBits::SclSda as u8;
        }
        bits & 0x0F // Ensure upper 4 bits remain 0 as they are reserved.
    }

    pub(crate) const fn ctrl9_cmd(&self) -> u8 {
        ctrl9::CMD_SET_RPU
    }
}

impl Default for PullUpConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cal1_l_generates_correct_bits() {
        let config = PullUpConfig::new()
            .with_group(PullUpGroup::Aux, true)
            .with_group(PullUpGroup::Sdx, true)
            .with_group(PullUpGroup::Cs, true);
        assert_eq!(
            config.cal1_l(),
            PullUpDisableBits::Aux as u8
                | PullUpDisableBits::Sdx as u8
                | PullUpDisableBits::Cs as u8
        );

        // Test: Disable all
        let config = PullUpConfig::new().disable_all();
        assert_eq!(
            config.cal1_l(),
            PullUpDisableBits::Aux as u8
                | PullUpDisableBits::Sdx as u8
                | PullUpDisableBits::Cs as u8
                | PullUpDisableBits::SclSda as u8
        );

        // Test: Default (All enabled)
        let config = PullUpConfig::default();
        assert_eq!(config.cal1_l(), 0x00);
    }

    #[test]
    fn with_group_updates_config_correctly() {
        let config = PullUpConfig::new()
            .with_group(PullUpGroup::Cs, true)
            .with_group(PullUpGroup::SclSda, true);
        assert_eq!(config.cal1_l(), 0x0C);
    }

    #[test]
    fn each_group_maps_to_its_documented_bit() {
        assert_eq!(
            PullUpConfig::new()
                .with_group(PullUpGroup::Aux, true)
                .cal1_l(),
            0x01
        );
        assert_eq!(
            PullUpConfig::new()
                .with_group(PullUpGroup::Sdx, true)
                .cal1_l(),
            0x02
        );
        assert_eq!(
            PullUpConfig::new()
                .with_group(PullUpGroup::Cs, true)
                .cal1_l(),
            0x04
        );
        assert_eq!(
            PullUpConfig::new()
                .with_group(PullUpGroup::SclSda, true)
                .cal1_l(),
            0x08
        );
        assert_eq!(PullUpConfig::new().disable_all().cal1_l() & 0xF0, 0);
    }
}
