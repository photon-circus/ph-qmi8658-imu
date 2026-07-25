use crate::register::ctrl9;

/// Groups of controllable pull-up resistors (per datasheet Table 30).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PullUpGroup {
    /// Auxiliary pin group: SDx, SCx, and RESV-NC (Pin 10).
    Aux,
    /// SDx pin individual control.
    Sdx,
    /// CS (Chip Select) pin.
    Cs,
    /// I2C pin group: SCL and SDA.
    I2c,
}

/// Configuration for pull-up resistor control.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PullUpConfig {
    /// Auxiliary group pull-up state (true: disabled, false: enabled).
    pub aux_disable: bool,
    /// SDx pin pull-up state (true: disabled, false: enabled).
    pub sdx_disable: bool,
    /// CS pin pull-up state (true: disabled, false: enabled).
    pub cs_disable: bool,
    /// I2C group pull-up state (true: disabled, false: enabled).
    pub i2c_disable: bool,
}

/// Bit mapping for pull-up disable flags in the CAL1_L register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PullUpDisableBits {
    /// Auxiliary group disable bit (bit 0).
    Aux = 0x01,
    /// SDx pin disable bit (bit 1).
    Sdx = 0x02,
    /// CS pin disable bit (bit 2).
    Cs = 0x04,
    /// I2C group disable bit (bit 3).
    I2c = 0x08,
}

impl PullUpConfig {
    /// Default configuration: All pull-up resistors enabled (matches POR state).
    pub const DEFAULT: Self = Self {
        aux_disable: false,
        sdx_disable: false,
        cs_disable: false,
        i2c_disable: false,
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
            PullUpGroup::I2c => self.i2c_disable = disable,
        }
        self
    }

    /// Disables all pull-up resistors (recommended for lowest power consumption).
    #[must_use]
    pub const fn disable_all(mut self) -> Self {
        self.aux_disable = true;
        self.sdx_disable = true;
        self.cs_disable = true;
        self.i2c_disable = true;
        self
    }

    /// Disables SPI-related pull-ups only (SDx, CS, and Aux groups).
    /// Useful for low-power SPI operation while maintaining I2C compatibility.
    #[must_use]
    pub const fn disable_spi_related(mut self) -> Self {
        self.aux_disable = true;
        self.sdx_disable = true;
        self.cs_disable = true;
        self.i2c_disable = false;
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
        if self.i2c_disable {
            bits |= PullUpDisableBits::I2c as u8;
        }
        bits & 0x0F // Ensure upper 4 bits remain 0 as they are reserved.
    }

    pub(crate) const fn cal1_h(self) -> u8 {
        // Note: CAL1_H is typically not required for the SET_RPU command,
        // as only the lower 4 bits of the payload are defined.
        0x00
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
        // Test: Disable SPI related (Aux+Sdx+Cs), enable I2C
        let config = PullUpConfig::new().disable_spi_related();
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
                | PullUpDisableBits::I2c as u8
        );

        // Test: Default (All enabled)
        let config = PullUpConfig::default();
        assert_eq!(config.cal1_l(), 0x00);
    }

    #[test]
    fn with_group_updates_config_correctly() {
        let config = PullUpConfig::new()
            .with_group(PullUpGroup::Cs, true)
            .with_group(PullUpGroup::I2c, true);
        assert!(config.cs_disable);
        assert!(config.i2c_disable);
        assert!(!config.aux_disable);
        assert!(!config.sdx_disable);
        assert_eq!(config.cal1_l(), 0x0C);
    }
}
