extern crate std;

use std::vec::Vec;

use embedded_hal_async::delay::DelayNs;

use crate::error::Error;
use crate::interface::{Interface, sealed};

#[derive(Clone, Debug)]
pub(crate) struct MockInterface {
    regs: [u8; 256],
    writes: Vec<(u8, u8)>,
    write_bursts: Vec<(u8, Vec<u8>)>,
}

impl Default for MockInterface {
    fn default() -> Self {
        Self {
            regs: [0u8; 256],
            writes: Vec::new(),
            write_bursts: Vec::new(),
        }
    }
}

impl MockInterface {
    pub(crate) fn with_reg(mut self, reg: u8, value: u8) -> Self {
        self.set_reg(reg, value);
        self
    }

    pub(crate) fn set_reg(&mut self, reg: u8, value: u8) {
        self.regs[reg as usize] = value;
    }

    pub(crate) fn writes(&self) -> &[(u8, u8)] {
        &self.writes
    }

    #[allow(dead_code)]
    pub(crate) fn write_bursts(&self) -> &[(u8, Vec<u8>)] {
        &self.write_bursts
    }
}

impl Interface for MockInterface {
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        Ok(self.regs[reg as usize])
    }

    async fn read_regs(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Ok(());
        }
        for (offset, slot) in buffer.iter_mut().enumerate() {
            let addr = reg.wrapping_add(offset as u8);
            *slot = self.regs[addr as usize];
        }
        Ok(())
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        self.regs[reg as usize] = value;
        self.writes.push((reg, value));
        Ok(())
    }

    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        for (offset, value) in data.iter().enumerate() {
            let addr = reg.wrapping_add(offset as u8);
            self.regs[addr as usize] = *value;
        }
        self.write_bursts.push((reg, data.to_vec()));
        Ok(())
    }
}

impl sealed::Sealed for MockInterface {}

#[derive(Default, Debug)]
pub(crate) struct MockDelay {
    pub(crate) calls: u32,
    pub(crate) last_ns: Option<u32>,
}

impl DelayNs for MockDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.calls += 1;
        self.last_ns = Some(ns);
    }
}
