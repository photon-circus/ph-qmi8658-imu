# QMI8658 IMU Driver

[![CI](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml/badge.svg)](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml) [![Docs.rs](https://docs.rs/ph-qmi8658/badge.svg)](https://docs.rs/ph-qmi8658) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Asynchronous Rust driver for the QMI8658A/C 6-axis IMU (accelerometer +
gyroscope) from QST Corporation, plus ESP32-S3 example applications. Built on
`embedded-hal-async` for use in `#![no_std]` environments. QMI8658C is the
default variant; QMI8658A-specific register behavior is feature-gated.

## Features

- Async I2C and SPI transports via `embedded-hal-async`
- Configurable accelerometer (up to 16 g) and gyroscope (up to 2048 dps)
- FIFO buffering with burst reads and frame-level parsing
- Interrupt routing, wake-on-motion, and sync-sample data locking
- Self-test, calibration, and on-demand calibration support
- Integer scaling helpers (no floats required); optional fixed-point conversions
- Targets ESP32 (Xtensa & RISC-V) and ARM Cortex-M

QMI8658B is not currently supported because an official vendor datasheet was
not available for review.

MSRV: **1.92.0** &mdash; [API docs on docs.rs](https://docs.rs/ph-qmi8658)

## Repository Layout

| Path | Description |
|------|-------------|
| [`crates/qmi8658`](crates/qmi8658/) | Driver crate (`ph-qmi8658`) |
| [`apps/qa-runner`](apps/qa-runner/) | Automated A/C, I2C/SPI, LE/BE hardware evidence runner |
| [`apps/imu-example`](apps/imu-example/) | FIFO-based example app for ESP32-S3 |
| [`docs/vendor`](docs/vendor/) | QST datasheets used for register-level review |

## Documentation

- [Driver README](crates/qmi8658/README.md) &mdash; usage flows and code examples
- [Architecture](crates/qmi8658/ARCHITECTURE.md) &mdash; module structure and sequence diagrams
- [IMU Example](apps/imu-example/README.md) &mdash; FIFO streaming example for ESP32-S3
- [Hardware Evidence Runner](apps/qa-runner/README.md) &mdash; BE-01 wiring, collection, and matrix validation
- [Changelog](CHANGELOG.md) &mdash; release history
- [Contributing](CONTRIBUTING.md) &mdash; development setup and PR guidelines
- [Release Checklist](crates/qmi8658/RELEASE_CHECKLIST.md) &mdash; publish procedures
- [v0.1.2 Remediation Tracker](V0.1.2_REMEDIATION_PLAN.md) &mdash; release decisions and gates
- [Security](SECURITY.md) &mdash; vulnerability reporting
- [Code of Conduct](CODE_OF_CONDUCT.md) &mdash; community standards

## Quick Start

Add the dependency:
```toml
[dependencies]
ph-qmi8658 = "0.1"
```

This selects QMI8658C. For QMI8658A, disable default features and select the
variant explicitly:

```toml
[dependencies]
ph-qmi8658 = { version = "0.1", default-features = false, features = ["qmi8658a"] }
```

Initialize the driver and read sensor data:
```rust
use ph_qmi8658::{Config, I2cConfig, Qmi8658Address, Qmi8658I2c};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

async fn example<I2C: I2c, D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<(), ph_qmi8658::Error> {
  let config = Config::new();
  let i2c_config = I2cConfig::new(Qmi8658Address::Primary.addr());
  let mut imu: Qmi8658I2c<I2C> = Qmi8658I2c::with_i2c_config(i2c, None, None, config, i2c_config);
  imu.init(delay).await?;

  // Read accelerometer, gyroscope, temperature, and timestamp in one burst.
  let block = imu.read_raw_block().await?;
  let _ = block;
  Ok(())
}
```

See the [driver README](crates/qmi8658/README.md) for configuration, FIFO, interrupt,
scaling, and advanced usage examples.

## Example Apps

Build targets from their directories:

QA runner:
```bash
cd apps/qa-runner
cargo +esp build
```

FIFO example:
```bash
cd apps/imu-example
cargo build
```

Flash:
```bash
espflash flash --monitor --log-format defmt
```

## License

MIT. See [`LICENSE`](LICENSE).
