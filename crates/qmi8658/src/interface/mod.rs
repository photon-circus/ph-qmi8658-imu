//! Interface abstraction for register I/O.

pub(crate) mod address;
pub(crate) mod i2c;
pub(crate) mod spi;

pub use address::Qmi8658Address;
pub use i2c::{I2cConfig, I2cInterface};
pub use spi::{SpiConfig, SpiInterface};

use crate::error::Error;
use crate::register::ctrl1;

pub(crate) mod sealed {
    pub trait Sealed {}
}

/// Minimal async register I/O for the device core.
#[allow(async_fn_in_trait)]
pub trait Interface: sealed::Sealed {
    /// Reads a single register.
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error>;
    /// Reads a contiguous block of registers into `buffer`.
    async fn read_regs(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error>;
    /// Writes a single register.
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error>;
    /// Writes a contiguous block of registers from `data`.
    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error>;
}

/// Serial interface settings applied via CTRL1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct InterfaceSettings {
    pub(crate) auto_increment: bool,
    pub(crate) big_endian: bool,
    pub(crate) spi_3_wire: bool,
}

impl InterfaceSettings {
    pub(crate) const fn new(auto_increment: bool, big_endian: bool, spi_3_wire: bool) -> Self {
        Self {
            auto_increment,
            big_endian,
            spi_3_wire,
        }
    }

    pub(crate) const fn ctrl1_value(self) -> u8 {
        let mut value = 0;
        if self.spi_3_wire {
            value |= ctrl1::SIM;
        }
        if self.auto_increment {
            value |= ctrl1::ADDR_AI;
        }
        if self.big_endian {
            value |= ctrl1::BE;
        }
        value
    }
}

impl Default for InterfaceSettings {
    fn default() -> Self {
        Self::new(true, true, false)
    }
}
