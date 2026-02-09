//! QMI8658 driver implementation.
//!
//! This module provides a minimal async driver for the QMI8658.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

use crate::config::Config;
use crate::config::OperatingMode;
use crate::data::{AccelRaw, GyroRaw, RawBlock, Sample, TemperatureRaw, Timestamp};
use crate::data::{FifoConfig, FifoFrameFormat, FifoReadout, FifoStatus};
use crate::device::DeviceCore;
use crate::error::Error;
use crate::interface::Interface;
use crate::interface::{I2cConfig, I2cInterface};
use crate::interface::{SpiConfig, SpiInterface};
use crate::interrupt::{InterruptConfig, InterruptStatus, InterruptWaitError};
use crate::register::Register;
use crate::self_test::{SelfTestError, SelfTestReport};
use crate::wom::WomConfig;

const SELF_TEST_DELAY_NS: u32 = 1_000_000;

/// QMI8658 6-axis IMU driver.
pub struct Qmi8658<I, INT1 = (), INT2 = ()> {
    core: DeviceCore<I>,
    int1: Option<INT1>,
    int2: Option<INT2>,
}

/// I2C type alias for the QMI8658 driver.
pub type Qmi8658I2c<I2C, INT1 = (), INT2 = ()> = Qmi8658<I2cInterface<I2C>, INT1, INT2>;
/// SPI type alias for the QMI8658 driver (experimental).
pub type Qmi8658Spi<SPI, INT1 = (), INT2 = ()> = Qmi8658<SpiInterface<SPI>, INT1, INT2>;

impl<I2C, INT1, INT2> Qmi8658<I2cInterface<I2C>, INT1, INT2>
where
    I2C: I2c,
{
    /// Creates a new I2C-based driver with default settings.
    pub fn new_i2c(i2c: I2C, int1: Option<INT1>, int2: Option<INT2>) -> Self {
        Self::with_i2c_config(i2c, int1, int2, Config::default(), I2cConfig::default())
    }

    /// Creates a new I2C-based driver with a custom configuration.
    pub fn with_i2c_config(
        i2c: I2C,
        int1: Option<INT1>,
        int2: Option<INT2>,
        config: Config,
        i2c_config: I2cConfig,
    ) -> Self {
        let interface = I2cInterface::new(i2c, i2c_config.address);
        let core = DeviceCore::new(interface, config, i2c_config.interface_settings());
        Self { core, int1, int2 }
    }

    /// Updates the I2C address used by the interface.
    pub fn set_i2c_address(&mut self, address: u8) {
        self.core.set_i2c_address(address);
    }

    /// Attempts initialization for one or more I2C addresses.
    pub async fn init_with_addresses<D: DelayNs>(
        &mut self,
        delay: &mut D,
        addresses: &[u8],
    ) -> Result<u8, Error> {
        let mut last_err = None;
        for &address in addresses {
            self.set_i2c_address(address);
            match self.init(delay).await {
                Ok(()) => return Ok(address),
                Err(Error::WrongDevice) => return Err(Error::WrongDevice),
                Err(err) => last_err = Some(err),
            }
        }
        Err(last_err.unwrap_or(Error::NotPresent))
    }

    /// Releases the I2C bus, consuming the driver.
    pub fn release(self) -> I2C {
        self.core.release().release()
    }

    /// Releases the I2C bus and interrupt pins, consuming the driver.
    pub fn release_with_ints(self) -> (I2C, Option<INT1>, Option<INT2>) {
        let interface = self.core.release();
        (interface.release(), self.int1, self.int2)
    }
}

impl<SPI, INT1, INT2> Qmi8658<SpiInterface<SPI>, INT1, INT2>
where
    SPI: SpiDevice,
{
    /// Creates a new SPI-based driver (experimental).
    pub fn new_spi(spi: SPI, int1: Option<INT1>, int2: Option<INT2>) -> Self {
        Self::with_spi_config(spi, int1, int2, Config::default(), SpiConfig::default())
    }

    /// Creates a new SPI-based driver with a custom configuration (experimental).
    pub fn with_spi_config(
        spi: SPI,
        int1: Option<INT1>,
        int2: Option<INT2>,
        config: Config,
        spi_config: SpiConfig,
    ) -> Self {
        let interface = SpiInterface::new(spi);
        let core = DeviceCore::new(interface, config, spi_config.interface_settings());
        Self { core, int1, int2 }
    }

    /// Releases the SPI bus, consuming the driver.
    pub fn release(self) -> SPI {
        self.core.release().release()
    }

    /// Releases the SPI bus and interrupt pins, consuming the driver.
    pub fn release_with_ints(self) -> (SPI, Option<INT1>, Option<INT2>) {
        let interface = self.core.release();
        (interface.release(), self.int1, self.int2)
    }
}

impl<I, INT1, INT2> Qmi8658<I, INT1, INT2>
where
    I: Interface,
{
    /// Returns the current sensor configuration.
    pub const fn config(&self) -> Config {
        self.core.config()
    }

    /// Updates the sensor configuration.
    pub fn set_config(&mut self, config: Config) {
        self.core.set_config(config);
    }

    /// Returns the current operating mode.
    pub fn operating_mode(&self) -> OperatingMode {
        self.core.operating_mode()
    }

    /// Returns whether sync sample mode is enabled.
    pub fn sync_sample_enabled(&self) -> bool {
        self.core.sync_sample_enabled()
    }

    /// Enables or disables sync sample mode.
    pub async fn set_sync_sample(&mut self, enable: bool) -> Result<(), Error> {
        self.core.set_sync_sample(enable).await
    }

    /// Sets the operating mode and returns the required transition delay in nanoseconds.
    pub async fn set_mode(&mut self, target: OperatingMode) -> Result<u32, Error> {
        self.core.set_mode(target).await
    }

    /// Initializes the device (soft reset, verify WHO_AM_I, apply config).
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.core.init(delay).await
    }

    /// Performs a software reset.
    pub async fn soft_reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.core.soft_reset(delay).await
    }

    /// Verifies the device WHO_AM_I register.
    pub async fn verify_device(&mut self) -> Result<(), Error> {
        self.core.verify_device().await
    }

    /// Applies the current sensor configuration to the device.
    pub async fn apply_config(&mut self) -> Result<(), Error> {
        // Ordering: disable sensors (CTRL7=0), write CTRL1/2/3/5, then re-enable via CTRL7.
        self.core.apply_config().await
    }

    /// Applies the interrupt routing and enable configuration (CTRL8).
    pub async fn apply_interrupt_config(&mut self, config: InterruptConfig) -> Result<(), Error> {
        self.core.apply_interrupt_config(config).await
    }

    /// Reads and decodes interrupt/status registers.
    ///
    /// Note: reading STATUS0/STATUS1/STATUSINT clears their latched bits.
    pub async fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error> {
        self.core.read_interrupt_status().await
    }

    /// Applies FIFO configuration (watermark, mode, size).
    pub async fn apply_fifo_config(&mut self, config: FifoConfig) -> Result<(), Error> {
        self.core.apply_fifo_config(config).await
    }

    /// Applies Wake on Motion configuration (CAL1_L/H + CTRL9 command).
    pub async fn apply_wom_config(&mut self, config: WomConfig) -> Result<(), Error> {
        self.core.apply_wom_config(config).await
    }

    /// Applies accelerometer host delta offsets (signed Q4.12) and issues CTRL9.
    pub async fn apply_accel_host_delta_offset(
        &mut self,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.core.apply_accel_host_delta_offset(x, y, z).await
    }

    /// Applies accelerometer host delta offsets and waits for CTRL9 completion.
    pub async fn apply_accel_host_delta_offset_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.core
            .apply_accel_host_delta_offset_with_delay(delay, x, y, z)
            .await
    }

    /// Applies gyroscope host delta offsets (signed Q11.5) and issues CTRL9.
    pub async fn apply_gyro_host_delta_offset(
        &mut self,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.core.apply_gyro_host_delta_offset(x, y, z).await
    }

    /// Applies gyroscope host delta offsets and waits for CTRL9 completion.
    pub async fn apply_gyro_host_delta_offset_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.core
            .apply_gyro_host_delta_offset_with_delay(delay, x, y, z)
            .await
    }

    /// Copies gyro bias from CAL registers to FIFO and issues CTRL9.
    ///
    /// Values are signed Q11.5 in CAL1/2/3 (GX/GY/GZ).
    pub async fn copy_gyro_bias_to_fifo(&mut self, x: i16, y: i16, z: i16) -> Result<(), Error> {
        self.core.copy_gyro_bias_to_fifo(x, y, z).await
    }

    /// Copies gyro bias from CAL registers to FIFO and waits for CTRL9 completion.
    pub async fn copy_gyro_bias_to_fifo_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.core
            .copy_gyro_bias_to_fifo_with_delay(delay, x, y, z)
            .await
    }

    /// Reads gyro bias values (GX/GY/GZ) from FIFO after a bias copy.
    ///
    /// Returned values are signed Q11.5.
    pub async fn read_gyro_bias_from_fifo<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<GyroRaw, Error> {
        self.core.read_gyro_bias_from_fifo(delay).await
    }

    /// Copies gyro bias into FIFO and reads it back.
    pub async fn copy_gyro_bias_and_read<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<GyroRaw, Error> {
        self.copy_gyro_bias_to_fifo_with_delay(delay, x, y, z)
            .await?;
        self.read_gyro_bias_from_fifo(delay).await
    }

    /// Runs on-demand calibration (CTRL9) after disabling sensors, then restores config.
    pub async fn run_on_demand_calibration<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error> {
        self.core.run_on_demand_calibration(delay).await
    }

    /// Applies Wake on Motion configuration and waits for CTRL9 completion.
    pub async fn apply_wom_config_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        config: WomConfig,
    ) -> Result<(), Error> {
        self.core.apply_wom_config_with_delay(delay, config).await
    }

    /// Enters Wake on Motion mode using the current accel configuration.
    ///
    /// The current `Config` must be accel-only with a low-power ODR. The WoM
    /// threshold must be non-zero.
    pub async fn enable_wom<D: DelayNs>(
        &mut self,
        delay: &mut D,
        config: WomConfig,
    ) -> Result<(), Error> {
        self.core.enable_wom(delay, config).await
    }

    /// Exits Wake on Motion mode by disabling sensors and clearing the threshold.
    pub async fn disable_wom<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.core.disable_wom(delay).await
    }

    /// Enables/disables AHB clock gating via the CTRL9 command.
    pub async fn set_ahb_clock_gating(&mut self, enable: bool) -> Result<(), Error> {
        self.core.set_ahb_clock_gating(enable).await
    }

    /// Enables/disables AHB clock gating and waits for CTRL9 completion.
    pub async fn set_ahb_clock_gating_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        enable: bool,
    ) -> Result<(), Error> {
        self.core
            .set_ahb_clock_gating_with_delay(delay, enable)
            .await
    }

    /// Returns the current FIFO status and sample count.
    pub async fn fifo_status(&mut self) -> Result<FifoStatus, Error> {
        self.core.fifo_status().await
    }

    /// Issues the CTRL9 FIFO reset command.
    pub async fn reset_fifo(&mut self) -> Result<(), Error> {
        self.core.reset_fifo().await
    }

    /// Issues the CTRL9 FIFO reset command and waits for completion.
    pub async fn reset_fifo_with_delay<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.core.reset_fifo_with_delay(delay).await
    }

    /// Requests access to FIFO data (sets FIFO_RD_MODE via CTRL9).
    pub async fn request_fifo_read(&mut self) -> Result<(), Error> {
        self.core.request_fifo_read().await
    }

    /// Ensures FIFO_RD_MODE is set in FIFO_CTRL.
    pub async fn enable_fifo_read_mode(&mut self) -> Result<(), Error> {
        self.core.enable_fifo_read_mode().await
    }

    /// Clears FIFO_RD_MODE after FIFO readout completes.
    pub async fn finish_fifo_read(&mut self) -> Result<(), Error> {
        self.core.finish_fifo_read().await
    }

    /// Waits for CTRL9 command completion by polling the CmdDone bit.
    pub async fn wait_ctrl9_done<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.core.wait_ctrl9_done(delay).await
    }

    /// Reads FIFO data bytes from the FIFO_DATA register and returns the sample timestamp.
    pub async fn read_fifo_data(&mut self, buffer: &mut [u8]) -> Result<Timestamp, Error> {
        self.core.read_fifo_data(buffer).await
    }

    /// Performs a full FIFO read transaction (CTRL9 request + read + clear).
    ///
    /// The returned byte count is aligned to full FIFO frames. If the caller
    /// buffer is smaller than the available FIFO data, the read is truncated.
    pub async fn read_fifo_burst<D: DelayNs>(
        &mut self,
        delay: &mut D,
        buffer: &mut [u8],
    ) -> Result<FifoReadout, Error> {
        self.core.read_fifo_burst(delay, buffer).await
    }

    /// Returns the FIFO frame format based on enabled sensors.
    pub fn fifo_frame_format(&self) -> FifoFrameFormat {
        self.core.fifo_frame_format()
    }

    /// Waits for INT1 to go high.
    pub async fn wait_int1_high(&mut self) -> Result<(), InterruptWaitError<INT1::Error>>
    where
        INT1: Wait,
    {
        match self.int1.as_mut() {
            Some(pin) => pin.wait_for_high().await.map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for INT1 to go low.
    pub async fn wait_int1_low(&mut self) -> Result<(), InterruptWaitError<INT1::Error>>
    where
        INT1: Wait,
    {
        match self.int1.as_mut() {
            Some(pin) => pin.wait_for_low().await.map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for a rising edge on INT1.
    pub async fn wait_int1_rising_edge(&mut self) -> Result<(), InterruptWaitError<INT1::Error>>
    where
        INT1: Wait,
    {
        match self.int1.as_mut() {
            Some(pin) => pin
                .wait_for_rising_edge()
                .await
                .map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for a falling edge on INT1.
    pub async fn wait_int1_falling_edge(&mut self) -> Result<(), InterruptWaitError<INT1::Error>>
    where
        INT1: Wait,
    {
        match self.int1.as_mut() {
            Some(pin) => pin
                .wait_for_falling_edge()
                .await
                .map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for any edge on INT1.
    pub async fn wait_int1_any_edge(&mut self) -> Result<(), InterruptWaitError<INT1::Error>>
    where
        INT1: Wait,
    {
        match self.int1.as_mut() {
            Some(pin) => pin
                .wait_for_any_edge()
                .await
                .map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for INT2 to go high.
    pub async fn wait_int2_high(&mut self) -> Result<(), InterruptWaitError<INT2::Error>>
    where
        INT2: Wait,
    {
        match self.int2.as_mut() {
            Some(pin) => pin.wait_for_high().await.map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for INT2 to go low.
    pub async fn wait_int2_low(&mut self) -> Result<(), InterruptWaitError<INT2::Error>>
    where
        INT2: Wait,
    {
        match self.int2.as_mut() {
            Some(pin) => pin.wait_for_low().await.map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for a rising edge on INT2.
    pub async fn wait_int2_rising_edge(&mut self) -> Result<(), InterruptWaitError<INT2::Error>>
    where
        INT2: Wait,
    {
        match self.int2.as_mut() {
            Some(pin) => pin
                .wait_for_rising_edge()
                .await
                .map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for a falling edge on INT2.
    pub async fn wait_int2_falling_edge(&mut self) -> Result<(), InterruptWaitError<INT2::Error>>
    where
        INT2: Wait,
    {
        match self.int2.as_mut() {
            Some(pin) => pin
                .wait_for_falling_edge()
                .await
                .map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Waits for any edge on INT2.
    pub async fn wait_int2_any_edge(&mut self) -> Result<(), InterruptWaitError<INT2::Error>>
    where
        INT2: Wait,
    {
        match self.int2.as_mut() {
            Some(pin) => pin
                .wait_for_any_edge()
                .await
                .map_err(InterruptWaitError::Pin),
            None => Err(InterruptWaitError::Missing),
        }
    }

    /// Reads the sample timestamp.
    pub async fn read_timestamp(&mut self) -> Result<Timestamp, Error> {
        self.core.read_timestamp().await
    }

    /// Reads timestamp, temperature, accelerometer, and gyroscope in one burst.
    pub async fn read_raw_block(&mut self) -> Result<RawBlock, Error> {
        self.core.read_raw_block().await
    }

    /// Runs the accelerometer self-test and returns the raw data plus pass/fail.
    ///
    /// This flow requires INT2 to be wired and provided to the driver.
    /// The current `Config` must include an accelerometer configuration.
    pub async fn run_accel_self_test<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<SelfTestReport, SelfTestError<INT2::Error>>
    where
        INT2: Wait,
    {
        let accel = self.core.config().accel.ok_or(Error::InvalidData)?;
        let ctrl2_st = accel.ctrl2_value_with_self_test(true);
        let ctrl2_off = accel.ctrl2_value_with_self_test(false);

        self.core.write_reg(Register::Ctrl7, 0).await?;
        delay.delay_ns(SELF_TEST_DELAY_NS).await;
        self.core.write_reg(Register::Ctrl2, ctrl2_st).await?;

        if let Err(err) = self.wait_int2_high().await {
            let _ = self.core.write_reg(Register::Ctrl2, ctrl2_off).await;
            let _ = self.core.apply_config().await;
            return Err(SelfTestError::from(err));
        }

        self.core.write_reg(Register::Ctrl2, ctrl2_off).await?;

        if let Err(err) = self.wait_int2_low().await {
            let _ = self.core.apply_config().await;
            return Err(SelfTestError::from(err));
        }

        let raw = self.core.read_self_test_axes().await?;
        let report = SelfTestReport::accel(raw);
        self.core.apply_config().await?;
        Ok(report)
    }

    /// Runs the gyroscope self-test and returns the raw data plus pass/fail.
    ///
    /// This flow requires INT2 to be wired and provided to the driver.
    /// The current `Config` must include a gyroscope configuration.
    pub async fn run_gyro_self_test<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<SelfTestReport, SelfTestError<INT2::Error>>
    where
        INT2: Wait,
    {
        let gyro = self.core.config().gyro.ok_or(Error::InvalidData)?;
        let ctrl3_st = gyro.ctrl3_value_with_self_test(true);
        let ctrl3_off = gyro.ctrl3_value_with_self_test(false);

        self.core.write_reg(Register::Ctrl7, 0).await?;
        delay.delay_ns(SELF_TEST_DELAY_NS).await;
        self.core.write_reg(Register::Ctrl3, ctrl3_st).await?;

        if let Err(err) = self.wait_int2_high().await {
            let _ = self.core.write_reg(Register::Ctrl3, ctrl3_off).await;
            let _ = self.core.apply_config().await;
            return Err(SelfTestError::from(err));
        }

        self.core.write_reg(Register::Ctrl3, ctrl3_off).await?;

        if let Err(err) = self.wait_int2_low().await {
            let _ = self.core.apply_config().await;
            return Err(SelfTestError::from(err));
        }

        let raw = self.core.read_self_test_axes().await?;
        let report = SelfTestReport::gyro(raw);
        self.core.apply_config().await?;
        Ok(report)
    }

    /// Waits for a new sync sample to be available in STATUSINT.
    pub async fn wait_for_sync_sample<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.core.wait_for_sync_sample(delay).await
    }

    /// Reads a sync-locked sensor sample using the data-lock flow.
    pub async fn read_sync_sample<D: DelayNs>(&mut self, delay: &mut D) -> Result<RawBlock, Error> {
        self.core.read_sync_sample(delay).await
    }

    /// Reads raw accelerometer data (X, Y, Z) with a timestamp.
    pub async fn read_accel_raw(&mut self) -> Result<Sample<AccelRaw>, Error> {
        self.core.read_accel_raw().await
    }

    /// Reads raw gyroscope data (X, Y, Z) with a timestamp.
    pub async fn read_gyro_raw(&mut self) -> Result<Sample<GyroRaw>, Error> {
        self.core.read_gyro_raw().await
    }

    /// Reads raw temperature data with a timestamp.
    pub async fn read_temperature_raw(&mut self) -> Result<Sample<TemperatureRaw>, Error> {
        self.core.read_temperature_raw().await
    }

    /// Sets the operating mode and waits for the transition delay.
    pub async fn set_mode_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        target: OperatingMode,
    ) -> Result<(), Error> {
        self.core.set_mode_with_delay(delay, target).await
    }

    /// Returns the nominal sample period derived from the configured ODRs.
    pub fn sample_period_ns(&self) -> Option<u32> {
        self.core.sample_period_ns()
    }

    /// Returns a reference to the INT1 pin, if provided.
    pub fn int1(&self) -> Option<&INT1> {
        self.int1.as_ref()
    }

    /// Returns a mutable reference to the INT1 pin, if provided.
    pub fn int1_mut(&mut self) -> Option<&mut INT1> {
        self.int1.as_mut()
    }

    /// Returns a reference to the INT2 pin, if provided.
    pub fn int2(&self) -> Option<&INT2> {
        self.int2.as_ref()
    }

    /// Returns a mutable reference to the INT2 pin, if provided.
    pub fn int2_mut(&mut self) -> Option<&mut INT2> {
        self.int2.as_mut()
    }

    /// Takes the INT1 pin out of the driver, leaving None.
    pub fn take_int1(&mut self) -> Option<INT1> {
        self.int1.take()
    }

    /// Takes the INT2 pin out of the driver, leaving None.
    pub fn take_int2(&mut self) -> Option<INT2> {
        self.int2.take()
    }

    #[allow(dead_code)]
    pub(crate) async fn read_reg(&mut self, reg: Register) -> Result<u8, Error> {
        self.core.read_reg(reg).await
    }

    #[allow(dead_code)]
    pub(crate) async fn read_regs(
        &mut self,
        reg: Register,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.core.read_regs(reg, buffer).await
    }

    #[allow(dead_code)]
    pub(crate) async fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Error> {
        self.core.write_reg(reg, value).await
    }

    #[allow(dead_code)]
    pub(crate) async fn write_regs(&mut self, reg: Register, data: &[u8]) -> Result<(), Error> {
        self.core.write_regs(reg, data).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;
    use crate::data::{FifoConfig, FifoMode, FifoSize};
    use crate::interface::InterfaceSettings;
    use crate::interrupt::{InterruptConfig, InterruptPin};
    use crate::register::Register;
    use crate::testing::MockInterface;
    use futures::executor::block_on;

    #[test]
    fn apply_fifo_config_writes_fifo_registers() {
        let interface = MockInterface::default();
        let config = Config::new();
        let settings = InterfaceSettings::new(true, true, false);
        let core = DeviceCore::new(interface, config, settings);
        let mut driver: Qmi8658<MockInterface, (), ()> = Qmi8658 {
            core,
            int1: None,
            int2: None,
        };

        let fifo = FifoConfig::new(FifoMode::Stream, FifoSize::Samples64, 8);
        block_on(driver.apply_fifo_config(fifo)).expect("fifo config");

        let interface = driver.core.release();
        let writes = interface.writes();
        assert_eq!(writes.len(), 2);
        assert_eq!(writes[0], (Register::FifoWtmTh.addr(), fifo.watermark));
        assert_eq!(writes[1], (Register::FifoCtrl.addr(), fifo.ctrl_value()));
    }

    #[test]
    fn apply_interrupt_config_writes_ctrl8() {
        let interface = MockInterface::default();
        let config = Config::new();
        let settings = InterfaceSettings::new(true, true, false);
        let core = DeviceCore::new(interface, config, settings);
        let mut driver: Qmi8658<MockInterface, (), ()> = Qmi8658 {
            core,
            int1: None,
            int2: None,
        };

        let irq = InterruptConfig::new()
            .with_motion_pin(InterruptPin::Int2)
            .with_any_motion(true);
        block_on(driver.apply_interrupt_config(irq)).expect("interrupt config");

        let interface = driver.core.release();
        let writes = interface.writes();
        assert_eq!(writes, [(Register::Ctrl8.addr(), irq.ctrl8_value())]);
    }
}
