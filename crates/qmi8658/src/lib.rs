//! Async `#![no_std]` driver for the
//! [QMI8658C](https://www.qstcorp.com/en_comp_prod/QMI8658C) 6-axis IMU
//! (accelerometer + gyroscope + temperature) from QST Corporation.
//!
//! This crate provides a lightweight, `embedded-hal-async` based driver for the
//! QMI8658 6-axis IMU. It intentionally avoids any core-ports dependencies so
//! it can be reused in adapter or BSP layers.
//!
//! # Quick start (I2C)
//!
//! ```rust,no_run
//! use ph_qmi8658::{Config, I2cConfig, Qmi8658Address, Qmi8658I2c};
//! # use embedded_hal_async::delay::DelayNs;
//! # use embedded_hal_async::i2c::I2c;
//! #
//! # async fn example<I2C: I2c, D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<(), ph_qmi8658::Error> {
//! let config = Config::new();
//! let i2c_config = I2cConfig::new(Qmi8658Address::Primary.addr());
//! let mut imu: Qmi8658I2c<I2C> =
//!     Qmi8658I2c::with_i2c_config(i2c, None, None, config, i2c_config);
//! imu.init(delay).await?;
//! # Ok(())
//! # }
//! ```
//!
//! # FIFO
//!
//! FIFO parsing helpers are available for burst-mode reads. See [`FifoFrameIterator`].
//!
//! # CTRL9 handshake
//!
//! By default CTRL8.bit7 = 0 and CmdDone is routed to INT1/STATUS1.bit0.
//! Set [`InterruptConfig::with_ctrl9_handshake_statusint(true)`](InterruptConfig::with_ctrl9_handshake_statusint)
//! to route CmdDone to STATUSINT.bit7. The driver polls the correct source automatically.
//!
//! # Sync sample locking
//!
//! Enable with `set_sync_sample(true)`; use `read_sync_sample(delay)` to follow
//! the STATUSINT data-lock flow and data-read delay.
//! For I2C/I3C, disable AHB clock gating via
//! `set_ahb_clock_gating_with_delay(delay, false)` while sync sample is active.
//!
//! # Not yet supported
//!
//! - External magnetometer integration (mag raw types and FIFO mag frame parsing
//!   are included for future support).
//! - AttitudeEngine configuration and Motion-on-Demand.
//! - Motion detection engines (tap/any/no/sig motion, pedometer) &mdash; these are
//!   QMI8658A-only and are not applicable to the QMI8658C.
//!
//! # Scaling helpers
//!
//! Use [`accel_lsb_per_g`], [`gyro_lsb_per_dps`], and [`temperature_lsb_per_celsius`]
//! (or the milli-unit ratios [`accel_mg_per_lsb`] / [`gyro_mdps_per_lsb`]) to
//! convert raw counts to physical units without floating-point math.
//!
//! # Fixed-point conversions
//!
//! Enable the `fixed` feature to access fixed-point helpers that convert raw
//! readings into g, dps, and degrees C using `I32F32` integer math.

#![no_std]
#![deny(missing_docs)]
#![allow(unsafe_code)]
#![deny(unsafe_op_in_unsafe_fn)]
// Clippy lint levels live here; thresholds and config are in clippy.toml.
#![deny(clippy::correctness)]
#![warn(
    clippy::suspicious,
    clippy::style,
    clippy::complexity,
    clippy::perf,
    clippy::cloned_instead_of_copied,
    clippy::explicit_iter_loop,
    clippy::implicit_clone,
    clippy::inconsistent_struct_constructor,
    clippy::manual_assert,
    clippy::manual_let_else,
    clippy::match_same_arms,
    clippy::needless_pass_by_value,
    clippy::semicolon_if_nothing_returned,
    clippy::uninlined_format_args,
    clippy::unnested_or_patterns,
    clippy::std_instead_of_core,
    clippy::std_instead_of_alloc,
    clippy::alloc_instead_of_core
)]
#![allow(
    clippy::mod_module_files,
    clippy::self_named_module_files,
    clippy::similar_names,
    clippy::too_many_arguments,
    clippy::struct_excessive_bools,
    clippy::fn_params_excessive_bools,
    clippy::type_complexity,
    clippy::must_use_candidate,
    clippy::assertions_on_constants,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::panic_in_result_fn,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::module_name_repetitions,
    clippy::wildcard_imports,
    clippy::items_after_statements,
    clippy::let_underscore_future
)]

#[cfg(feature = "fixed")]
extern crate fixed as fixed_crate;

mod config;
mod data;
mod device;
mod driver;
mod error;
mod interface;
mod interrupt;
mod macros;
mod register;
mod self_test;
mod wom;

#[cfg(test)]
mod testing;

// Interface layer
pub use interface::Qmi8658Address;
pub use interface::{I2cConfig, I2cInterface};
pub use interface::{SpiConfig, SpiInterface};

// Configuration
pub use config::{AccelConfig, AccelOutputDataRate, AccelRange, LowPassFilterMode};
pub use config::{Config, OperatingMode};
pub use config::{GyroConfig, GyroOutputDataRate, GyroRange};

// Driver
pub use driver::{Qmi8658, Qmi8658I2c, Qmi8658Spi};

// Data types
pub use data::{AccelRaw, GyroRaw, MagRaw, RawBlock, Sample, TemperatureRaw, Timestamp};
pub use data::{
    FifoConfig,
    FifoFrame,
    FifoFrameFormat,
    FifoFrameIterator,
    FifoMode,
    FifoReadout,
    FifoSize,
    FifoStatus,
    ScaleFactor,
    accel_lsb_per_g,
    accel_mg_per_lsb,
    gyro_lsb_per_dps,
    gyro_mdps_per_lsb,
    temperature_lsb_per_celsius,
    temperature_mdegc_per_lsb,
};

// Features
pub use error::Error;
pub use interrupt::{InterruptConfig, InterruptPin, InterruptStatus, InterruptWaitError};
pub use self_test::{SelfTestAxis, SelfTestError, SelfTestReport};
pub use wom::{WomConfig, WomInterruptLevel};

// Fixed-point conversions (feature-gated)
#[cfg(feature = "fixed")]
pub use data::fixed::{
    AccelFixed, Fixed, GyroFixed, TemperatureFixed, accel_sample_to_g, accel_to_g,
    gyro_sample_to_dps, gyro_to_dps, temperature_celsius, temperature_sample_celsius,
};
