//! Convenience macros for common driver sequences.

/// Initialize a QMI8658 instance with a common configuration sequence.
///
/// This macro runs the typical initialization flow:
/// 1. `init_with_addresses`
/// 2. `apply_interrupt_config`
/// 3. `set_config` + `apply_config`
/// 4. Optional FIFO configuration + reset
///
/// The macro expands to a `Result<u8, Error>` expression, returning the
/// detected I2C address on success. It must be invoked from an async context.
///
/// Example without FIFO:
/// ```rust,no_run
/// # use ph_qmi8658::{Config, InterruptConfig, Qmi8658Address, qmi8658_init_sequence};
/// # async fn example(imu: &mut ph_qmi8658::Qmi8658I2c<impl embedded_hal_async::i2c::I2c>, delay: &mut impl embedded_hal_async::delay::DelayNs)
/// # -> Result<(), ph_qmi8658::Error> {
/// let config = Config::new();
/// let irq = InterruptConfig::new().with_ctrl9_handshake_statusint(true);
/// let address = qmi8658_init_sequence!(
///     imu: imu,
///     delay: delay,
///     addresses: &[Qmi8658Address::Primary.addr(), Qmi8658Address::Secondary.addr()],
///     interrupt: irq,
///     config: config,
/// )?;
/// # Ok(())
/// # }
/// ```
///
/// Example with FIFO:
/// ```rust,no_run
/// # use ph_qmi8658::{Config, InterruptConfig, FifoConfig, FifoMode, FifoSize, Qmi8658Address, qmi8658_init_sequence};
/// # async fn example(imu: &mut ph_qmi8658::Qmi8658I2c<impl embedded_hal_async::i2c::I2c>, delay: &mut impl embedded_hal_async::delay::DelayNs)
/// # -> Result<(), ph_qmi8658::Error> {
/// let config = Config::new();
/// let irq = InterruptConfig::new().with_ctrl9_handshake_statusint(true);
/// let fifo = FifoConfig::new(FifoMode::Stream, FifoSize::Samples64, 8);
/// let address = qmi8658_init_sequence!(
///     imu: imu,
///     delay: delay,
///     addresses: &[Qmi8658Address::Primary.addr(), Qmi8658Address::Secondary.addr()],
///     interrupt: irq,
///     config: config,
///     fifo: fifo,
/// )?;
/// # Ok(())
/// # }
/// ```
#[macro_export]
macro_rules! qmi8658_init_sequence {
    (
        imu: $imu:expr,
        delay: $delay:expr,
        addresses: $addresses:expr,
        interrupt: $irq:expr,
        config: $config:expr,
        fifo: $fifo:expr $(,)?
    ) => {{
        let address = $imu.init_with_addresses($delay, $addresses).await?;
        $imu.apply_interrupt_config($irq).await?;
        $imu.set_config($config);
        $imu.apply_config().await?;
        $imu.apply_fifo_config($fifo).await?;
        $imu.reset_fifo_with_delay($delay).await?;
        Ok::<u8, $crate::Error>(address)
    }};
    (
        imu: $imu:expr,
        delay: $delay:expr,
        addresses: $addresses:expr,
        interrupt: $irq:expr,
        config: $config:expr $(,)?
    ) => {{
        let address = $imu.init_with_addresses($delay, $addresses).await?;
        $imu.apply_interrupt_config($irq).await?;
        $imu.set_config($config);
        $imu.apply_config().await?;
        Ok::<u8, $crate::Error>(address)
    }};
}
