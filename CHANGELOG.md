# Changelog

All notable changes to this project will be documented in this file.

## Unreleased
- No entries yet.

## 0.1.1 - 2026-02-09
### Added
- Integer scaling helpers (`ScaleFactor`, `accel_mg_per_lsb`, `gyro_mdps_per_lsb`) for
  converting raw counts to physical units without floating-point math.
- FIFO-based ESP32-S3 example app (`apps/imu-example`) with motion feature output
  (RMS, peak, activity percentage).
- `qmi8658_init_sequence!` helper macro to reduce boilerplate in common init flows.

### Changed
- Fixed-point conversion helpers now share the same scale tables as the new integer helpers,
  ensuring consistent sensitivity values across both APIs.

### Docs
- Expanded driver README with flow examples, section headings, and scaling usage.
- Improved root README with chip description, feature list, and linked documentation.

## 0.1.0 - 2026-02-04
Initial release.

### Added
- Async I2C and SPI driver for the QMI8658C 6-axis IMU.
- Accelerometer and gyroscope configuration (ranges, ODRs, low-pass filters).
- FIFO buffering with burst and manual read paths, frame-level parsing.
- Interrupt routing to INT1/INT2 with CTRL9 handshake support.
- Sync-sample data locking for coherent multi-register reads.
- Wake-on-motion detection.
- Self-test, host-delta offset calibration, and on-demand calibration.
- Optional `defmt` formatting support.
- Optional fixed-point conversion helpers (`fixed` feature).
- `apps/qa-runner` hardware test runner for ESP32-S3.
