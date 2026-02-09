//! Error type for the QMI8658 driver.

/// Error type for QMI8658 operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus communication error (I2C, SPI, etc.).
    Bus,
    /// Sensor not responding or not present.
    NotPresent,
    /// Invalid chip ID or wrong device.
    WrongDevice,
    /// Data not ready.
    NotReady,
    /// Invalid data or configuration.
    InvalidData,
    /// Operation not supported.
    Unsupported,
}
