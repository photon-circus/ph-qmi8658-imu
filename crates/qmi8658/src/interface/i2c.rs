//! I2C interface adapter for the QMI8658.

use embedded_hal_async::i2c::{I2c, Operation};

use super::Qmi8658Address;
use super::{Interface, InterfaceSettings, sealed};
use crate::error::Error;

/// I2C interface configuration (address + serial settings).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2cConfig {
    pub(crate) address: u8,
    pub(crate) auto_increment: bool,
    pub(crate) big_endian: bool,
}

impl I2cConfig {
    /// Creates a new I2C configuration for the provided address.
    pub const fn new(address: u8) -> Self {
        Self {
            address,
            auto_increment: true,
            big_endian: true,
        }
    }

    /// Sets the I2C address.
    #[must_use]
    pub const fn with_address(mut self, address: u8) -> Self {
        self.address = address;
        self
    }

    /// Enables or disables address auto-increment.
    #[must_use]
    pub const fn with_auto_increment(mut self, enable: bool) -> Self {
        self.auto_increment = enable;
        self
    }

    /// Sets the serial read endianness (CTRL1.BE).
    #[must_use]
    pub const fn with_big_endian(mut self, enable: bool) -> Self {
        self.big_endian = enable;
        self
    }

    pub(crate) const fn interface_settings(self) -> InterfaceSettings {
        InterfaceSettings::new(self.auto_increment, self.big_endian, false)
    }
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self::new(Qmi8658Address::Primary.addr())
    }
}

/// I2C register interface.
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Creates a new I2C interface with the given bus and 7-bit address.
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Changes the 7-bit I2C address.
    pub fn set_address(&mut self, address: u8) {
        self.address = address;
    }

    /// Releases the underlying I2C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Interface for I2cInterface<I2C>
where
    I2C: I2c,
{
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buffer = [0u8];
        self.read_regs(reg, &mut buffer).await?;
        Ok(buffer[0])
    }

    async fn read_regs(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Ok(());
        }
        self.i2c
            .write_read(self.address, &[reg], buffer)
            .await
            .map_err(|_| Error::Bus)
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        let buffer = [reg, value];
        self.i2c
            .write(self.address, &buffer)
            .await
            .map_err(|_| Error::Bus)
    }

    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        let reg_buffer = [reg];
        let mut ops = [Operation::Write(&reg_buffer), Operation::Write(data)];
        self.i2c
            .transaction(self.address, &mut ops)
            .await
            .map_err(|_| Error::Bus)
    }
}

impl<I2C> sealed::Sealed for I2cInterface<I2C> {}
