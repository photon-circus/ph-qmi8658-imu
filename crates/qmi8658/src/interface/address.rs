//! I2C address definitions for the QMI8658.

/// QMI8658 I2C addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Qmi8658Address {
    /// Primary address: 0x6A (SA0 = low).
    Primary,
    /// Secondary address: 0x6B (SA0 = high).
    Secondary,
}

impl Qmi8658Address {
    /// Returns the 7-bit I2C address.
    pub const fn addr(self) -> u8 {
        match self {
            Self::Primary => 0x6A,
            Self::Secondary => 0x6B,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_address() {
        assert_eq!(Qmi8658Address::Primary.addr(), 0x6A);
        assert_eq!(Qmi8658Address::Secondary.addr(), 0x6B);
    }
}
