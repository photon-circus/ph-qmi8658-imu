//! SPI interface adapter for the QMI8658.
//!
//! Experimental: the SPI transport has not been validated on hardware yet.

use embedded_hal_async::spi::{Operation, SpiDevice};

use super::{Interface, InterfaceSettings, sealed};
use crate::error::Error;

/// SPI interface configuration (experimental).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiConfig {
    pub(crate) auto_increment: bool,
    pub(crate) big_endian: bool,
    pub(crate) three_wire: bool,
}

impl SpiConfig {
    /// Creates a new SPI configuration (4-wire, auto-increment, big-endian).
    pub const fn new() -> Self {
        Self {
            auto_increment: true,
            big_endian: true,
            three_wire: false,
        }
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

    /// Enables 3-wire SPI mode (CTRL1.SIM).
    #[must_use]
    pub const fn with_three_wire(mut self, enable: bool) -> Self {
        self.three_wire = enable;
        self
    }

    pub(crate) const fn interface_settings(self) -> InterfaceSettings {
        InterfaceSettings::new(self.auto_increment, self.big_endian, self.three_wire)
    }
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// SPI register interface (experimental).
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Creates a new SPI interface with the given bus.
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Releases the underlying SPI bus.
    pub fn release(self) -> SPI {
        self.spi
    }
}

const SPI_READ_MASK: u8 = 0x80;

const fn spi_addr_read(reg: u8) -> u8 {
    (reg & 0x7F) | SPI_READ_MASK
}

const fn spi_addr_write(reg: u8) -> u8 {
    reg & 0x7F
}

impl<SPI> Interface for SpiInterface<SPI>
where
    SPI: SpiDevice,
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
        let addr = spi_addr_read(reg);
        let addr_buf = [addr];
        let mut ops = [Operation::Write(&addr_buf), Operation::Read(buffer)];
        self.spi.transaction(&mut ops).await.map_err(|_| Error::Bus)
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        let addr = spi_addr_write(reg);
        let buffer = [addr, value];
        self.spi.write(&buffer).await.map_err(|_| Error::Bus)
    }

    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        let addr = spi_addr_write(reg);
        let addr_buf = [addr];
        let mut ops = [Operation::Write(&addr_buf), Operation::Write(data)];
        self.spi.transaction(&mut ops).await.map_err(|_| Error::Bus)
    }
}

impl<SPI> sealed::Sealed for SpiInterface<SPI> {}
