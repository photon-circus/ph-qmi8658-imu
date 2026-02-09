# QMI8658 IMU Driver

[![CI](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml/badge.svg)](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Asynchronous driver for the QMI8658 6-axis IMU sensor, plus ESP32-S3 example applications.

## Repository Layout
- `crates/qmi8658`: driver crate (`ph-qmi8658`)
- `apps/qa-runner`: hardware test runner for the ESP32-S3 Matrix Board
- `apps/imu-example`: FIFO-based example app for the ESP32-S3 Matrix Board

## Project Docs
- Driver README: `crates/qmi8658/README.md`
- Driver Architecture: `crates/qmi8658/ARCHITECTURE.md`
- Release Checklist: `crates/qmi8658/RELEASE_CHECKLIST.md`
- Contributing: `CONTRIBUTING.md`
- Security: `SECURITY.md`
- Changelog: `CHANGELOG.md`
- Code of Conduct: `CODE_OF_CONDUCT.md`

## Quick Start (Driver)
```rust
use ph_qmi8658::{Config, I2cConfig, Qmi8658Address, Qmi8658I2c};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

async fn example<I2C: I2c, D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<(), ph_qmi8658::Error> {
  let config = Config::new();
  let i2c_config = I2cConfig::new(Qmi8658Address::Primary.addr());
  let mut imu: Qmi8658I2c<I2C> = Qmi8658I2c::with_i2c_config(i2c, None, None, config, i2c_config);
  imu.init(delay).await?;
  Ok(())
}
```

## Example Apps
Build targets from their directories:

QA runner:
```bash
cd apps/qa-runner
cargo build
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
MIT. See `LICENSE`.
