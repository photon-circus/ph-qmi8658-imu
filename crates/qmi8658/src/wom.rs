//! Wake on Motion (WoM) configuration helpers.

use crate::interrupt::InterruptPin;

/// Initial interrupt level for Wake on Motion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WomInterruptLevel {
    /// Active-low interrupt level.
    Low,
    /// Active-high interrupt level.
    High,
}

/// Wake on Motion configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WomConfig {
    /// WoM threshold in mg (1 mg/LSB). 0 disables WoM.
    pub threshold_mg: u8,
    /// Blanking time in accelerometer samples (0..63).
    pub blanking_samples: u8,
    /// Interrupt pin used for WoM.
    pub pin: InterruptPin,
    /// Initial interrupt level for WoM.
    pub initial_level: WomInterruptLevel,
}

impl WomConfig {
    /// Default WoM configuration (threshold disabled, INT1 low, no blanking).
    pub const DEFAULT: Self = Self {
        threshold_mg: 0,
        blanking_samples: 0,
        pin: InterruptPin::Int1,
        initial_level: WomInterruptLevel::Low,
    };

    /// Creates a new WoM configuration with the provided threshold in mg.
    pub const fn new(threshold_mg: u8) -> Self {
        Self {
            threshold_mg,
            ..Self::DEFAULT
        }
    }

    /// Sets the threshold in mg.
    #[must_use]
    pub const fn with_threshold(mut self, threshold_mg: u8) -> Self {
        self.threshold_mg = threshold_mg;
        self
    }

    /// Sets the interrupt blanking time in accel samples (0..63).
    #[must_use]
    pub const fn with_blanking_samples(mut self, samples: u8) -> Self {
        self.blanking_samples = samples;
        self
    }

    /// Sets the interrupt pin and initial level for WoM events.
    #[must_use]
    pub const fn with_interrupt(mut self, pin: InterruptPin, level: WomInterruptLevel) -> Self {
        self.pin = pin;
        self.initial_level = level;
        self
    }

    pub(crate) const fn cal1_l(self) -> u8 {
        self.threshold_mg
    }

    pub(crate) const fn cal1_h(self) -> u8 {
        let blanking = self.blanking_samples & 0x3F;
        let select = match (self.pin, self.initial_level) {
            (InterruptPin::Int1, WomInterruptLevel::Low) => 0b00,
            (InterruptPin::Int1, WomInterruptLevel::High) => 0b10,
            (InterruptPin::Int2, WomInterruptLevel::Low) => 0b01,
            (InterruptPin::Int2, WomInterruptLevel::High) => 0b11,
        };
        (select << 6) | blanking
    }
}

impl Default for WomConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cal1_h_packs_select_and_blanking() {
        let config = WomConfig::new(10)
            .with_blanking_samples(0xAA)
            .with_interrupt(InterruptPin::Int2, WomInterruptLevel::High);
        assert_eq!(config.cal1_h(), 0b11_101010);
    }
}
