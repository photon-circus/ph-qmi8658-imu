//! Device core operations for the QMI8658.

use embedded_hal_async::delay::DelayNs;

use crate::config::Config;
use crate::config::{OperatingMode, OperatingModeStateMachine, TransitionDelay, transition_delay};
use crate::data::{
    AccelRaw, GyroRaw, RawBlock, Sample, TemperatureRaw, Timestamp, decode_raw_block,
};
use crate::data::{FifoConfig, FifoFrameFormat, FifoReadout, FifoStatus, fifo_read_len};
use crate::error::Error;
use crate::interface::I2cInterface;
use crate::interface::{Interface, InterfaceSettings};
use crate::interrupt::{InterruptConfig, InterruptStatus};
use crate::pull::PullUpConfig;
use crate::register::{Register, ctrl7, ctrl9, fifo_ctrl, reset, status_int, who_am_i};
use crate::self_test::SelfTestAxis;
use crate::wom::WomConfig;

pub(crate) struct DeviceCore<I> {
    interface: I,
    config: Config,
    settings: InterfaceSettings,
    mode: OperatingModeStateMachine,
    ctrl7_flags: u8,
    drdy_enabled: bool,
}

impl<I> DeviceCore<I>
where
    I: Interface,
{
    pub(crate) fn new(interface: I, config: Config, settings: InterfaceSettings) -> Self {
        Self {
            interface,
            config,
            settings,
            mode: OperatingModeStateMachine::new(OperatingMode::PowerOnDefault),
            ctrl7_flags: 0,
            drdy_enabled: true,
        }
    }

    pub(crate) const fn config(&self) -> Config {
        self.config
    }

    pub(crate) fn set_config(&mut self, config: Config) {
        self.config = config;
    }

    #[allow(dead_code)]
    pub(crate) const fn interface_settings(&self) -> InterfaceSettings {
        self.settings
    }

    pub(crate) const fn operating_mode(&self) -> OperatingMode {
        self.mode.mode()
    }

    pub(crate) async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.soft_reset(delay).await?;
        self.verify_device().await?;
        self.apply_config().await
    }

    pub(crate) async fn soft_reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.write_reg(Register::Reset, reset::SOFT_RESET).await?;

        const POLL_RETRIES: u8 = 20;
        const POLL_DELAY_NS: u32 = 1_000_000;
        let mut read_succeeded = false;

        delay.delay_ns(POLL_DELAY_NS).await;
        for attempt in 0..POLL_RETRIES {
            match self.interface.read_reg(reset::RESET_DONE_REG).await {
                Ok(reset::RESET_DONE) => {
                    self.mode = OperatingModeStateMachine::new(OperatingMode::PowerOnDefault);
                    return Ok(());
                }
                Ok(_) => read_succeeded = true,
                Err(_) => {}
            }

            if attempt + 1 < POLL_RETRIES {
                delay.delay_ns(POLL_DELAY_NS).await;
            }
        }

        if read_succeeded {
            Err(Error::NotReady)
        } else {
            Err(Error::Bus)
        }
    }

    pub(crate) async fn verify_device(&mut self) -> Result<(), Error> {
        let who = self.read_reg(Register::WhoAmI).await?;
        if who != who_am_i::EXPECTED {
            return Err(Error::WrongDevice);
        }
        Ok(())
    }

    pub(crate) async fn apply_config(&mut self) -> Result<(), Error> {
        self.config.validate()?;
        self.write_reg(Register::Ctrl7, 0).await?;
        self.write_reg(Register::Ctrl1, self.settings.ctrl1_value())
            .await?;
        self.write_reg(Register::Ctrl2, self.config.ctrl2_value())
            .await?;
        self.write_reg(Register::Ctrl3, self.config.ctrl3_value())
            .await?;
        self.write_reg(Register::Ctrl5, self.config.ctrl5_value())
            .await?;
        let ctrl7_value = self.compose_ctrl7(self.config.ctrl7_value());
        self.write_reg(Register::Ctrl7, ctrl7_value).await?;
        self.mode.set_mode(self.config.target_mode());
        Ok(())
    }

    /// Applies Pull-up resistor configuration (CAL1_L + CTRL9 command).
    pub(crate) async fn apply_pull_up_config(&mut self, config: PullUpConfig) -> Result<(), Error> {
        self.write_reg(Register::Cal1L, config.cal1_l()).await?;
        self.write_reg(Register::Ctrl9, config.ctrl9_cmd()).await
    }

    /// Applies Pull-up configuration and waits for CTRL9 command completion.
    pub(crate) async fn apply_pull_up_config_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        config: PullUpConfig,
    ) -> Result<(), Error> {
        self.apply_pull_up_config(config).await?;
        self.wait_ctrl9_done(delay).await
    }

    pub(crate) const fn sync_sample_enabled(&self) -> bool {
        (self.ctrl7_flags & ctrl7::SYNC_SMPL) != 0
    }

    pub(crate) const fn drdy_enabled(&self) -> bool {
        self.drdy_enabled
    }

    pub(crate) async fn set_drdy_enabled(&mut self, enable: bool) -> Result<(), Error> {
        self.drdy_enabled = enable;

        if self.mode.mode() == OperatingMode::PowerOnDefault {
            return Ok(());
        }

        let ctrl7_value = self.ctrl7_for_mode(self.mode.mode())?;
        self.write_reg(Register::Ctrl7, ctrl7_value).await
    }

    pub(crate) async fn set_sync_sample(&mut self, enable: bool) -> Result<(), Error> {
        if enable {
            self.ctrl7_flags |= ctrl7::SYNC_SMPL;
        } else {
            self.ctrl7_flags &= !ctrl7::SYNC_SMPL;
        }

        if self.mode.mode() == OperatingMode::PowerOnDefault {
            return Ok(());
        }

        let ctrl7_value = self.ctrl7_for_mode(self.mode.mode())?;
        self.write_reg(Register::Ctrl7, ctrl7_value).await
    }

    pub(crate) async fn set_mode(&mut self, target: OperatingMode) -> Result<u32, Error> {
        let delay_token = self.set_mode_internal(target).await?;
        Ok(self.transition_delay_ns(delay_token).unwrap_or(0))
    }

    async fn set_mode_internal(&mut self, target: OperatingMode) -> Result<TransitionDelay, Error> {
        let delay = transition_delay(self.mode.mode(), target).ok_or(Error::Unsupported)?;
        let ctrl7_value = self.ctrl7_for_mode(target)?;
        self.write_reg(Register::Ctrl7, ctrl7_value).await?;
        self.mode.set_mode(target);
        Ok(delay)
    }

    pub(crate) async fn set_mode_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        target: OperatingMode,
    ) -> Result<(), Error> {
        let delay_ns = self.set_mode(target).await?;
        if delay_ns > 0 {
            delay.delay_ns(delay_ns).await;
        }
        Ok(())
    }

    /// Applies the interrupt routing and enable configuration (CTRL8).
    pub(crate) async fn apply_interrupt_config(
        &mut self,
        config: InterruptConfig,
    ) -> Result<(), Error> {
        self.write_reg(Register::Ctrl8, config.ctrl8_value()).await
    }

    /// Reads and decodes interrupt/status registers.
    ///
    /// Note: reading STATUS0/STATUS1/STATUSINT clears their latched bits.
    pub(crate) async fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error> {
        let status_int = self.read_reg(Register::StatusInt).await?;
        let status0 = self.read_reg(Register::Status0).await?;
        let status1 = self.read_reg(Register::Status1).await?;
        Ok(InterruptStatus::from_regs(status_int, status0, status1))
    }

    /// Applies FIFO configuration (watermark, mode, size).
    pub(crate) async fn apply_fifo_config(&mut self, config: FifoConfig) -> Result<(), Error> {
        self.write_reg(Register::FifoWtmTh, config.watermark)
            .await?;
        self.write_reg(Register::FifoCtrl, config.ctrl_value())
            .await?;
        Ok(())
    }

    /// Applies Wake on Motion configuration (CAL1_L/H + CTRL9 command).
    pub(crate) async fn apply_wom_config(&mut self, config: WomConfig) -> Result<(), Error> {
        self.write_reg(Register::Cal1L, config.cal1_l()).await?;
        self.write_reg(Register::Cal1H, config.cal1_h()).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_WRITE_WOM_SETTING)
            .await
    }

    /// Applies accelerometer host delta offsets and issues the CTRL9 command.
    pub(crate) async fn apply_accel_host_delta_offset(
        &mut self,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.write_calibration_xyz(x, y, z).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_ACCEL_HOST_DELTA_OFFSET)
            .await
    }

    /// Applies accelerometer host delta offsets and waits for CTRL9 completion.
    pub(crate) async fn apply_accel_host_delta_offset_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.apply_accel_host_delta_offset(x, y, z).await?;
        self.wait_ctrl9_done(delay).await
    }

    /// Applies gyroscope host delta offsets and issues the CTRL9 command.
    pub(crate) async fn apply_gyro_host_delta_offset(
        &mut self,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.write_calibration_xyz(x, y, z).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_GYRO_HOST_DELTA_OFFSET)
            .await
    }

    /// Applies gyroscope host delta offsets and waits for CTRL9 completion.
    pub(crate) async fn apply_gyro_host_delta_offset_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.apply_gyro_host_delta_offset(x, y, z).await?;
        self.wait_ctrl9_done(delay).await
    }

    /// Copies gyro bias from CAL registers to FIFO and issues the CTRL9 command.
    ///
    /// Values are signed Q11.5 in CAL1/2/3 (GX/GY/GZ).
    pub(crate) async fn copy_gyro_bias_to_fifo(
        &mut self,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.write_calibration_xyz(x, y, z).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_GYRO_BIAS).await
    }

    /// Copies gyro bias from CAL registers to FIFO and waits for CTRL9 completion.
    pub(crate) async fn copy_gyro_bias_to_fifo_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        x: i16,
        y: i16,
        z: i16,
    ) -> Result<(), Error> {
        self.copy_gyro_bias_to_fifo(x, y, z).await?;
        self.wait_ctrl9_done(delay).await
    }

    /// Reads gyro bias values from FIFO (GX/GY/GZ order).
    ///
    /// Returned values are signed Q11.5.
    pub(crate) async fn read_gyro_bias_from_fifo<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<GyroRaw, Error> {
        self.request_fifo_read().await?;
        self.wait_ctrl9_done(delay).await?;
        self.enable_fifo_read_mode().await?;

        let status = self.fifo_status().await?;
        if status.sample_count_bytes < 6 {
            self.finish_fifo_read().await?;
            return Err(Error::NotReady);
        }

        let mut buffer = [0u8; 6];
        self.read_regs(Register::FifoData, &mut buffer).await?;
        self.finish_fifo_read().await?;
        Ok(GyroRaw::from_bytes(buffer, self.settings.big_endian))
    }

    /// Runs the on-demand calibration CTRL9 command and restores configuration.
    pub(crate) async fn run_on_demand_calibration<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error> {
        self.write_reg(Register::Ctrl7, 0).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_ON_DEMAND_CALIBRATION)
            .await?;
        self.wait_ctrl9_done_with_retries(delay, 2_000).await?;
        self.apply_config().await
    }

    /// Applies Wake on Motion configuration and waits for CTRL9 completion.
    pub(crate) async fn apply_wom_config_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        config: WomConfig,
    ) -> Result<(), Error> {
        self.apply_wom_config(config).await?;
        self.wait_ctrl9_done(delay).await
    }

    /// Enters Wake on Motion mode using the current accel configuration.
    pub(crate) async fn enable_wom<D: DelayNs>(
        &mut self,
        delay: &mut D,
        config: WomConfig,
    ) -> Result<(), Error> {
        if config.threshold_mg == 0 {
            return Err(Error::InvalidData);
        }
        let accel = self.config.accel.ok_or(Error::InvalidData)?;
        if self.config.gyro.is_some() || !accel.odr.is_low_power() {
            return Err(Error::InvalidData);
        }

        self.write_reg(Register::Ctrl7, 0).await?;
        self.write_reg(Register::Ctrl2, accel.ctrl2_value()).await?;
        self.write_reg(Register::Ctrl5, self.config.ctrl5_value())
            .await?;
        self.apply_wom_config_with_delay(delay, config).await?;

        let ctrl7_value = self.compose_ctrl7(ctrl7::A_EN);
        self.write_reg(Register::Ctrl7, ctrl7_value).await?;
        self.mode.set_mode(OperatingMode::WakeOnMotion);
        Ok(())
    }

    /// Exits Wake on Motion mode by disabling sensors and clearing the threshold.
    pub(crate) async fn disable_wom<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.write_reg(Register::Ctrl7, 0).await?;
        self.write_reg(Register::Cal1L, 0).await?;
        self.write_reg(Register::Cal1H, 0).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_WRITE_WOM_SETTING)
            .await?;
        self.wait_ctrl9_done(delay).await?;
        self.mode.set_mode(OperatingMode::PowerOnDefault);
        Ok(())
    }

    /// Enables/disables AHB clock gating via the CTRL9 command.
    pub(crate) async fn set_ahb_clock_gating(&mut self, enable: bool) -> Result<(), Error> {
        let cal1_value = if enable { 0x00 } else { 0x01 };
        self.write_reg(Register::Cal1L, cal1_value).await?;
        self.write_reg(Register::Ctrl9, ctrl9::CMD_AHB_CLOCK_GATING)
            .await
    }

    /// Enables/disables AHB clock gating and waits for CTRL9 completion.
    pub(crate) async fn set_ahb_clock_gating_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
        enable: bool,
    ) -> Result<(), Error> {
        self.set_ahb_clock_gating(enable).await?;
        self.wait_ctrl9_done(delay).await
    }

    /// Returns the current FIFO status and sample count.
    pub(crate) async fn fifo_status(&mut self) -> Result<FifoStatus, Error> {
        let count_lsb = self.read_reg(Register::FifoSmplCnt).await?;
        let status = self.read_reg(Register::FifoStatus).await?;
        Ok(FifoStatus::from_regs(status, count_lsb))
    }

    /// Issues the CTRL9 FIFO reset command.
    pub(crate) async fn reset_fifo(&mut self) -> Result<(), Error> {
        self.write_reg(Register::Ctrl9, ctrl9::CMD_RST_FIFO).await
    }

    /// Issues the CTRL9 FIFO reset command and waits for completion.
    pub(crate) async fn reset_fifo_with_delay<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error> {
        self.reset_fifo().await?;
        self.wait_ctrl9_done(delay).await
    }

    /// Requests access to FIFO data (sets FIFO_RD_MODE via CTRL9).
    pub(crate) async fn request_fifo_read(&mut self) -> Result<(), Error> {
        self.write_reg(Register::Ctrl9, ctrl9::CMD_REQ_FIFO).await
    }

    /// Ensures FIFO_RD_MODE is set in FIFO_CTRL.
    pub(crate) async fn enable_fifo_read_mode(&mut self) -> Result<(), Error> {
        let ctrl = self.read_reg(Register::FifoCtrl).await?;
        if (ctrl & fifo_ctrl::FIFO_RD_MODE) == 0 {
            let set = ctrl | fifo_ctrl::FIFO_RD_MODE;
            self.write_reg(Register::FifoCtrl, set).await?;
        }
        Ok(())
    }

    /// Clears FIFO_RD_MODE after FIFO readout completes.
    pub(crate) async fn finish_fifo_read(&mut self) -> Result<(), Error> {
        let ctrl = self.read_reg(Register::FifoCtrl).await?;
        let cleared = ctrl & !fifo_ctrl::FIFO_RD_MODE;
        self.write_reg(Register::FifoCtrl, cleared).await
    }

    /// Waits for CTRL9 command completion by polling the CmdDone bit.
    pub(crate) async fn wait_ctrl9_done<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.wait_ctrl9_done_with_retries(delay, 100).await
    }

    async fn wait_ctrl9_done_with_retries<D: DelayNs>(
        &mut self,
        delay: &mut D,
        poll_retries: u16,
    ) -> Result<(), Error> {
        const POLL_DELAY_NS: u32 = 1_000_000;

        for attempt in 0..poll_retries {
            let status = self.read_reg(Register::StatusInt).await?;
            if (status & status_int::CMD_DONE) != 0 {
                self.write_reg(Register::Ctrl9, ctrl9::CMD_ACK).await?;
                return Ok(());
            }
            if attempt + 1 < poll_retries {
                delay.delay_ns(POLL_DELAY_NS).await;
            }
        }

        Err(Error::NotReady)
    }

    /// Reads FIFO data bytes from the FIFO_DATA register and returns the sample timestamp.
    pub(crate) async fn read_fifo_data(&mut self, buffer: &mut [u8]) -> Result<Timestamp, Error> {
        self.read_regs(Register::FifoData, buffer).await?;
        self.read_timestamp().await
    }

    /// Performs a full FIFO read transaction (CTRL9 request + read + clear).
    pub(crate) async fn read_fifo_burst<D: DelayNs>(
        &mut self,
        delay: &mut D,
        buffer: &mut [u8],
    ) -> Result<FifoReadout, Error> {
        self.request_fifo_read().await?;
        self.wait_ctrl9_done(delay).await?;
        self.enable_fifo_read_mode().await?;

        let status = self.fifo_status().await?;
        let format = self.fifo_frame_format();
        let bytes_read = fifo_read_len(status, format, buffer.len());
        let timestamp = self.read_fifo_data(&mut buffer[..bytes_read]).await?;

        self.finish_fifo_read().await?;

        Ok(FifoReadout {
            status,
            bytes_read,
            timestamp,
        })
    }

    /// Returns the FIFO frame format based on enabled sensors.
    pub(crate) fn fifo_frame_format(&self) -> FifoFrameFormat {
        FifoFrameFormat::new(
            self.config.accel.is_some(),
            self.config.gyro.is_some(),
            false,
        )
        .with_big_endian(self.settings.big_endian)
    }

    /// Reads the sample timestamp.
    pub(crate) async fn read_timestamp(&mut self) -> Result<Timestamp, Error> {
        let mut buffer = [0u8; 3];
        self.read_regs(Register::TimestampLow, &mut buffer).await?;
        Ok(Timestamp::from_bytes(buffer))
    }

    /// Reads timestamp, temperature, accelerometer, and gyroscope in one burst.
    pub(crate) async fn read_raw_block(&mut self) -> Result<RawBlock, Error> {
        let mut buffer = [0u8; crate::data::RAW_BLOCK_LEN];
        self.read_regs(crate::data::RAW_BLOCK_START, &mut buffer)
            .await?;
        Ok(decode_raw_block(
            &buffer,
            self.settings.big_endian,
            self.config.accel.is_some(),
            self.config.gyro.is_some(),
        ))
    }

    /// Reads the self-test output registers (dVX/dVY/dVZ).
    pub(crate) async fn read_self_test_axes(&mut self) -> Result<SelfTestAxis, Error> {
        let mut buffer = [0u8; 6];
        self.read_regs(Register::DvXL, &mut buffer).await?;
        Ok(SelfTestAxis::from_bytes(buffer, self.settings.big_endian))
    }

    /// Waits for a new sync sample to be available in STATUSINT.
    pub(crate) async fn wait_for_sync_sample<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error> {
        if !self.sync_sample_enabled() {
            return Err(Error::InvalidData);
        }

        const MIN_POLL_DELAY_NS: u32 = 100_000;
        const MAX_POLL_DELAY_NS: u32 = 50_000_000;
        const DEFAULT_POLL_DELAY_NS: u32 = 1_000_000;
        const DEFAULT_RETRIES: u16 = 200;

        let (poll_delay_ns, retries) = match self.sample_period_ns() {
            Some(period_ns) if period_ns > 0 => {
                let poll_delay_ns = (period_ns / 4).clamp(MIN_POLL_DELAY_NS, MAX_POLL_DELAY_NS);
                let max_wait_ns = period_ns.saturating_mul(2);
                let retries = (max_wait_ns / poll_delay_ns).max(1);
                (poll_delay_ns, retries.min(u32::from(u16::MAX)) as u16)
            }
            _ => (DEFAULT_POLL_DELAY_NS, DEFAULT_RETRIES),
        };

        for _ in 0..retries {
            let status = self.read_reg(Register::StatusInt).await?;
            if (status & status_int::AVAIL) != 0 {
                return Ok(());
            }
            delay.delay_ns(poll_delay_ns).await;
        }

        Err(Error::NotReady)
    }

    /// Reads a sync-locked sensor sample using the data-lock flow.
    pub(crate) async fn read_sync_sample<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<RawBlock, Error> {
        if !self.sync_sample_enabled() {
            return Err(Error::InvalidData);
        }

        self.wait_for_sync_sample(delay).await?;

        if let Some(delay_ns) = self.data_read_delay_ns()
            && delay_ns > 0
        {
            delay.delay_ns(delay_ns).await;
        }

        let mut buffer = [0u8; crate::data::RAW_BLOCK_LEN];
        let read_len = if self.config.gyro.is_some() {
            crate::data::RAW_BLOCK_LEN
        } else {
            crate::data::RAW_BLOCK_GYRO_OFFSET
        };
        self.read_regs(crate::data::RAW_BLOCK_START, &mut buffer[..read_len])
            .await?;

        Ok(decode_raw_block(
            &buffer,
            self.settings.big_endian,
            self.config.accel.is_some(),
            self.config.gyro.is_some(),
        ))
    }

    /// Reads raw accelerometer data (X, Y, Z) with a timestamp.
    pub(crate) async fn read_accel_raw(&mut self) -> Result<Sample<AccelRaw>, Error> {
        let mut buffer = [0u8; 6];
        self.read_regs(Register::AxL, &mut buffer).await?;
        let timestamp = self.read_timestamp().await?;
        Ok(Sample {
            timestamp,
            data: AccelRaw::from_bytes(buffer, self.settings.big_endian),
        })
    }

    /// Reads raw gyroscope data (X, Y, Z) with a timestamp.
    pub(crate) async fn read_gyro_raw(&mut self) -> Result<Sample<GyroRaw>, Error> {
        let mut buffer = [0u8; 6];
        self.read_regs(Register::GxL, &mut buffer).await?;
        let timestamp = self.read_timestamp().await?;
        Ok(Sample {
            timestamp,
            data: GyroRaw::from_bytes(buffer, self.settings.big_endian),
        })
    }

    /// Reads raw temperature data with a timestamp.
    pub(crate) async fn read_temperature_raw(&mut self) -> Result<Sample<TemperatureRaw>, Error> {
        let mut buffer = [0u8; 2];
        self.read_regs(Register::TempL, &mut buffer).await?;
        let timestamp = self.read_timestamp().await?;
        Ok(Sample {
            timestamp,
            data: TemperatureRaw::from_bytes(buffer, self.settings.big_endian),
        })
    }

    fn ctrl7_for_mode(&self, target: OperatingMode) -> Result<u8, Error> {
        let base = match target {
            OperatingMode::PowerOnDefault => {
                if self.config.accel.is_none() && self.config.gyro.is_none() {
                    Ok(0)
                } else {
                    Err(Error::InvalidData)
                }
            }
            OperatingMode::AccelOnly => match (self.config.accel, self.config.gyro) {
                (Some(accel), None) if !accel.odr.is_low_power() => Ok(ctrl7::A_EN),
                _ => Err(Error::InvalidData),
            },
            OperatingMode::LowPowerAccel | OperatingMode::WakeOnMotion => {
                match (self.config.accel, self.config.gyro) {
                    (Some(accel), None) if accel.odr.is_low_power() => Ok(ctrl7::A_EN),
                    _ => Err(Error::InvalidData),
                }
            }
            OperatingMode::GyroOnly => match (self.config.accel, self.config.gyro) {
                (None, Some(_)) => Ok(ctrl7::G_EN),
                _ => Err(Error::InvalidData),
            },
            OperatingMode::AccelGyroOnly => match (self.config.accel, self.config.gyro) {
                (Some(_), Some(_)) => Ok(ctrl7::A_EN | ctrl7::G_EN),
                _ => Err(Error::InvalidData),
            },
            _ => Err(Error::Unsupported),
        }?;

        Ok(self.compose_ctrl7(base))
    }

    const fn compose_ctrl7(&self, base: u8) -> u8 {
        if base == 0 {
            return 0;
        }

        let mut value = base | self.ctrl7_flags;
        if !self.sync_sample_enabled() && !self.drdy_enabled {
            value |= ctrl7::DRDY_DIS;
        }
        value
    }

    pub(crate) fn sample_period_ns(&self) -> Option<u32> {
        let odr_milli = self.sample_odr_milli()?;
        if odr_milli == 0 {
            return None;
        }
        Some(Self::nanos_from_ratio(1_000_000_000_000u64, odr_milli))
    }

    pub(crate) fn release(self) -> I {
        self.interface
    }

    pub(crate) async fn read_reg(&mut self, reg: Register) -> Result<u8, Error> {
        self.interface.read_reg(reg.addr()).await
    }

    pub(crate) async fn read_regs(
        &mut self,
        reg: Register,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.interface.read_regs(reg.addr(), buffer).await
    }

    pub(crate) async fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Error> {
        self.interface.write_reg(reg.addr(), value).await
    }

    #[allow(dead_code)]
    pub(crate) async fn write_regs(&mut self, reg: Register, data: &[u8]) -> Result<(), Error> {
        self.interface.write_regs(reg.addr(), data).await
    }

    async fn write_calibration_xyz(&mut self, x: i16, y: i16, z: i16) -> Result<(), Error> {
        let [x_l, x_h] = x.to_le_bytes();
        let [y_l, y_h] = y.to_le_bytes();
        let [z_l, z_h] = z.to_le_bytes();

        self.write_reg(Register::Cal1L, x_l).await?;
        self.write_reg(Register::Cal1H, x_h).await?;
        self.write_reg(Register::Cal2L, y_l).await?;
        self.write_reg(Register::Cal2H, y_h).await?;
        self.write_reg(Register::Cal3L, z_l).await?;
        self.write_reg(Register::Cal3H, z_h).await?;
        Ok(())
    }

    fn accel_odr_milli(&self) -> Option<u32> {
        let gyro_enabled = self.config.gyro.is_some();
        self.config
            .accel
            .map(|accel| accel.odr.effective_hz_milli(gyro_enabled))
    }

    fn gyro_odr_milli(&self) -> Option<u32> {
        self.config.gyro.map(|gyro| gyro.odr.hz_milli())
    }

    fn sample_odr_milli(&self) -> Option<u32> {
        match (self.accel_odr_milli(), self.gyro_odr_milli()) {
            (Some(accel), Some(gyro)) => Some(accel.max(gyro)),
            (Some(accel), None) => Some(accel),
            (None, Some(gyro)) => Some(gyro),
            (None, None) => None,
        }
    }

    fn data_read_delay_ns(&self) -> Option<u32> {
        if let Some(gyro) = self.config.gyro {
            return Some(gyro.odr.data_read_delay_us().saturating_mul(1_000));
        }
        self.config
            .accel
            .map(|accel| accel.odr.data_read_delay_us().saturating_mul(1_000))
    }

    fn transition_delay_ns(&self, delay: TransitionDelay) -> Option<u32> {
        match delay {
            TransitionDelay::Immediate => Some(0),
            TransitionDelay::T0 => Some(150_000_000),
            TransitionDelay::T1PlusT5 => self.t1_plus_t5_ns(),
            TransitionDelay::T2PlusT5 => self.t2_plus_t5_ns(),
            TransitionDelay::T3PlusT5 => self.t3_plus_t5_ns(),
            TransitionDelay::T4PlusT5 => self.t4_plus_t5_ns(),
            TransitionDelay::T6 => self.t6_ns(),
            TransitionDelay::T7 => Some(100_000),
        }
    }

    #[inline]
    fn nanos_from_ratio(numerator: u64, denominator: u32) -> u32 {
        if denominator == 0 {
            return 0;
        }
        let value = numerator / u64::from(denominator);
        if value > u64::from(u32::MAX) {
            u32::MAX
        } else {
            value as u32
        }
    }

    #[inline]
    fn t5_ns_from_odr_milli(odr_milli: u32) -> u32 {
        Self::nanos_from_ratio(3_000_000_000_000u64, odr_milli)
    }

    #[inline]
    pub(crate) fn t6_ns_from_odr_milli(&self, odr_milli: u32) -> u32 {
        Self::nanos_from_ratio(2_000_000_000_000u64, odr_milli)
    }

    #[inline]
    fn t1_plus_t5_ns(&self) -> Option<u32> {
        let t1_ns = 60_000_000u32;
        let t5_ns = self
            .gyro_odr_milli()
            .map(Self::t5_ns_from_odr_milli)
            .unwrap_or(0);
        Some(t1_ns.saturating_add(t5_ns))
    }

    #[inline]
    fn t2_plus_t5_ns(&self) -> Option<u32> {
        let t2_ns = 3_000_000u32;
        let t5_ns = self
            .accel_odr_milli()
            .map(Self::t5_ns_from_odr_milli)
            .unwrap_or(0);
        Some(t2_ns.saturating_add(t5_ns))
    }

    #[inline]
    fn t3_plus_t5_ns(&self) -> Option<u32> {
        let t3_ns = 12_000_000u32;
        let t5_ns = self
            .accel_odr_milli()
            .map(Self::t5_ns_from_odr_milli)
            .unwrap_or(0);
        Some(t3_ns.saturating_add(t5_ns))
    }

    #[inline]
    fn t4_plus_t5_ns(&self) -> Option<u32> {
        let t4_ns = 60_000_000u32;
        let t5_ns = self
            .gyro_odr_milli()
            .map(Self::t5_ns_from_odr_milli)
            .unwrap_or(0);
        Some(t4_ns.saturating_add(t5_ns))
    }

    #[inline]
    fn t6_ns(&self) -> Option<u32> {
        let accel = self.accel_odr_milli();
        let gyro = self.gyro_odr_milli();
        let odr_milli = match (accel, gyro) {
            (Some(a), Some(g)) => a.max(g),
            (Some(a), None) => a,
            (None, Some(g)) => g,
            (None, None) => 0,
        };
        Some(self.t6_ns_from_odr_milli(odr_milli))
    }
}

impl<I2C> DeviceCore<I2cInterface<I2C>> {
    pub(crate) fn set_i2c_address(&mut self, address: u8) {
        self.interface.set_address(address);
    }
}
#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::config::{AccelConfig, AccelOutputDataRate, AccelRange, Config};
    use crate::interface::InterfaceSettings;
    use crate::register::{Register, ctrl7, ctrl9, reset, status_int};
    use crate::testing::{MockDelay, MockInterface};
    use crate::wom::WomConfig;
    use alloc::vec;
    use futures::executor::block_on;

    const fn default_settings() -> InterfaceSettings {
        InterfaceSettings::new(true, true, false, false, false, false)
    }

    #[test]
    fn apply_config_writes_ctrls_in_order() {
        let interface = MockInterface::default();
        let config = Config::new();
        let settings = default_settings();
        let mut core = DeviceCore::new(interface, config, settings);

        block_on(core.apply_config()).expect("apply config");

        let expected = [
            (Register::Ctrl7.addr(), 0),
            (Register::Ctrl1.addr(), settings.ctrl1_value()),
            (Register::Ctrl2.addr(), config.ctrl2_value()),
            (Register::Ctrl3.addr(), config.ctrl3_value()),
            (Register::Ctrl5.addr(), config.ctrl5_value()),
            (Register::Ctrl7.addr(), config.ctrl7_value()),
        ];

        assert_eq!(core.interface.writes(), expected);
    }

    #[test]
    fn wait_ctrl9_done_uses_statusint() {
        let interface =
            MockInterface::default().with_reg(Register::StatusInt.addr(), status_int::CMD_DONE);
        let config = Config::new();
        let settings = default_settings();
        let mut core = DeviceCore::new(interface, config, settings);

        let mut delay = MockDelay::default();
        block_on(core.wait_ctrl9_done(&mut delay)).expect("ctrl9 done");

        assert_eq!(delay.calls, 0);
        assert_eq!(core.interface.reads(), [Register::StatusInt.addr()]);
        assert_eq!(
            core.interface.writes(),
            [(Register::Ctrl9.addr(), ctrl9::CMD_ACK)]
        );
    }

    #[test]
    fn wait_ctrl9_done_times_out_without_ack() {
        let interface = MockInterface::default();
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        let mut delay = MockDelay::default();

        assert_eq!(
            block_on(core.wait_ctrl9_done(&mut delay)),
            Err(Error::NotReady)
        );
        assert_eq!(core.interface.reads().len(), 100);
        assert!(
            core.interface
                .reads()
                .iter()
                .all(|reg| *reg == Register::StatusInt.addr())
        );
        assert!(core.interface.writes().is_empty());
        assert_eq!(delay.calls, 99);
    }

    #[test]
    fn calibration_can_complete_after_normal_ctrl9_timeout() {
        let mut sequence = vec![Ok(0); 150];
        sequence.push(Ok(status_int::CMD_DONE));
        let interface = MockInterface::default()
            .with_read_sequence(Register::StatusInt.addr(), sequence.as_slice());
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        let mut delay = MockDelay::default();

        block_on(core.run_on_demand_calibration(&mut delay)).expect("calibration");

        assert_eq!(delay.calls, 150);
        assert_eq!(
            core.interface.writes()[..3],
            [
                (Register::Ctrl7.addr(), 0),
                (Register::Ctrl9.addr(), ctrl9::CMD_ON_DEMAND_CALIBRATION),
                (Register::Ctrl9.addr(), ctrl9::CMD_ACK),
            ]
        );
    }

    #[test]
    fn reset_can_complete_on_first_status_read() {
        let sequence = [Ok(reset::RESET_DONE)];
        let interface =
            MockInterface::default().with_read_sequence(reset::RESET_DONE_REG, sequence.as_slice());
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        let mut delay = MockDelay::default();

        block_on(core.soft_reset(&mut delay)).expect("reset");

        assert_eq!(delay.calls, 1);
        assert_eq!(delay.total_ns, 1_000_000);
    }

    #[test]
    fn reset_can_complete_after_not_ready_reads() {
        let sequence = [Ok(0), Ok(0), Ok(reset::RESET_DONE)];
        let interface =
            MockInterface::default().with_read_sequence(reset::RESET_DONE_REG, sequence.as_slice());
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        let mut delay = MockDelay::default();

        block_on(core.soft_reset(&mut delay)).expect("reset");

        assert_eq!(delay.calls, 3);
        assert_eq!(delay.total_ns, 3_000_000);
    }

    #[test]
    fn reset_accepts_exact_done_value_after_transient_failures() {
        let sequence = [
            Err(Error::Bus),
            Ok(0),
            Err(Error::Bus),
            Ok(reset::RESET_DONE),
        ];
        let interface =
            MockInterface::default().with_read_sequence(reset::RESET_DONE_REG, sequence.as_slice());
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        core.mode.set_mode(OperatingMode::AccelGyroOnly);
        let mut delay = MockDelay::default();

        block_on(core.soft_reset(&mut delay)).expect("reset");

        assert_eq!(core.operating_mode(), OperatingMode::PowerOnDefault);
        assert_eq!(delay.calls, 4);
        assert_eq!(delay.total_ns, 4_000_000);
    }

    #[test]
    fn reset_wrong_values_timeout_without_changing_mode() {
        let sequence = [Ok(0x81); 20];
        let interface =
            MockInterface::default().with_read_sequence(reset::RESET_DONE_REG, sequence.as_slice());
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        core.mode.set_mode(OperatingMode::AccelGyroOnly);
        let mut delay = MockDelay::default();

        assert_eq!(block_on(core.soft_reset(&mut delay)), Err(Error::NotReady));
        assert_eq!(core.operating_mode(), OperatingMode::AccelGyroOnly);
        assert_eq!(delay.calls, 20);
        assert_eq!(delay.total_ns, 20_000_000);
    }

    #[test]
    fn reset_returns_bus_when_every_status_read_fails() {
        let sequence = [Err(Error::Bus); 20];
        let interface =
            MockInterface::default().with_read_sequence(reset::RESET_DONE_REG, sequence.as_slice());
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());
        let mut delay = MockDelay::default();

        assert_eq!(block_on(core.soft_reset(&mut delay)), Err(Error::Bus));
        assert_eq!(delay.calls, 20);
    }

    #[test]
    fn drdy_policy_survives_config_and_sync_transitions() {
        let interface = MockInterface::default();
        let mut core = DeviceCore::new(interface, Config::new(), default_settings());

        block_on(core.apply_config()).expect("config");
        block_on(core.set_drdy_enabled(false)).expect("disable drdy");
        assert!(!core.drdy_enabled());
        assert_eq!(
            core.interface.writes().last(),
            Some(&(
                Register::Ctrl7.addr(),
                ctrl7::A_EN | ctrl7::G_EN | ctrl7::DRDY_DIS
            ))
        );

        block_on(core.set_sync_sample(true)).expect("sync on");
        assert_eq!(
            core.interface.writes().last(),
            Some(&(
                Register::Ctrl7.addr(),
                ctrl7::A_EN | ctrl7::G_EN | ctrl7::SYNC_SMPL
            ))
        );
        assert!(!core.drdy_enabled());

        block_on(core.set_sync_sample(false)).expect("sync off");
        assert_eq!(
            core.interface.writes().last(),
            Some(&(
                Register::Ctrl7.addr(),
                ctrl7::A_EN | ctrl7::G_EN | ctrl7::DRDY_DIS
            ))
        );

        block_on(core.apply_config()).expect("reapply config");
        assert_eq!(
            core.interface.writes().last(),
            Some(&(
                Register::Ctrl7.addr(),
                ctrl7::A_EN | ctrl7::G_EN | ctrl7::DRDY_DIS
            ))
        );
    }

    #[test]
    fn drdy_policy_is_used_by_set_mode() {
        let config = Config::new().without_gyro();
        let interface = MockInterface::default();
        let mut core = DeviceCore::new(interface, config, default_settings());

        block_on(core.set_drdy_enabled(false)).expect("disable drdy");
        block_on(core.set_mode(OperatingMode::AccelOnly)).expect("accel mode");

        assert_eq!(
            core.interface.writes().last(),
            Some(&(Register::Ctrl7.addr(), ctrl7::A_EN | ctrl7::DRDY_DIS))
        );
    }

    #[test]
    fn drdy_policy_survives_wom_entry_and_exit() {
        let accel = AccelConfig::new(AccelRange::G8, AccelOutputDataRate::LowPowerHz21);
        let config = Config::new().with_accel_config(accel).without_gyro();
        let interface =
            MockInterface::default().with_reg(Register::StatusInt.addr(), status_int::CMD_DONE);
        let mut core = DeviceCore::new(interface, config, default_settings());
        let mut delay = MockDelay::default();

        block_on(core.set_drdy_enabled(false)).expect("disable drdy");
        block_on(core.enable_wom(&mut delay, WomConfig::new(10))).expect("enable wom");
        assert_eq!(
            core.interface.writes().last(),
            Some(&(Register::Ctrl7.addr(), ctrl7::A_EN | ctrl7::DRDY_DIS))
        );

        block_on(core.disable_wom(&mut delay)).expect("disable wom");
        block_on(core.apply_config()).expect("restore config");
        assert_eq!(
            core.interface.writes().last(),
            Some(&(Register::Ctrl7.addr(), ctrl7::A_EN | ctrl7::DRDY_DIS))
        );
    }

    #[test]
    fn self_test_t6_uses_two_output_periods() {
        let core = DeviceCore::new(MockInterface::default(), Config::new(), default_settings());

        assert_eq!(core.t6_ns_from_odr_milli(1_000_000), 2_000_000);
        assert_eq!(core.t6_ns_from_odr_milli(500_000), 4_000_000);
    }

    #[test]
    fn read_gyro_bias_from_fifo_reads_expected_axes() {
        let mut interface = MockInterface::default();
        interface.set_reg(Register::StatusInt.addr(), status_int::CMD_DONE);
        interface.set_reg(Register::FifoSmplCnt.addr(), 6);
        interface.set_reg(Register::FifoStatus.addr(), 0);
        interface.set_reg(Register::FifoCtrl.addr(), 0);

        let fifo_base = Register::FifoData.addr();
        interface.set_reg(fifo_base, 0x01);
        interface.set_reg(fifo_base.wrapping_add(1), 0x00);
        interface.set_reg(fifo_base.wrapping_add(2), 0x02);
        interface.set_reg(fifo_base.wrapping_add(3), 0x00);
        interface.set_reg(fifo_base.wrapping_add(4), 0x03);
        interface.set_reg(fifo_base.wrapping_add(5), 0x00);

        let config = Config::new();
        let settings = InterfaceSettings::new(true, false, false, false, false, false);
        let mut core = DeviceCore::new(interface, config, settings);

        let mut delay = MockDelay::default();
        let gyro = block_on(core.read_gyro_bias_from_fifo(&mut delay)).expect("bias read");

        assert_eq!(gyro.x, 1);
        assert_eq!(gyro.y, 2);
        assert_eq!(gyro.z, 3);
    }
}
