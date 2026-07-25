# Changelog

All notable changes to this project will be documented in this file.

## 0.1.2 - Unreleased

### Added

- Added mutually exclusive `qmi8658a` and `qmi8658c` device-variant features.
  QMI8658C remains the default and legacy no-feature behavior.
- Added QMI8658A-only INT1/INT2 push-pull output controls.
- Added common FIFO interrupt mapping controls, preserving INT2 as the default.
- Added runtime DRDY control that persists across configuration, mode,
  SyncSample, and Wake-on-Motion transitions.
- Added typed pull-up resistor configuration for the vendor-defined Aux, SDx,
  CS, and SCL/SDA groups.

### Changed

- Soft reset now waits 1 ms and polls register `0x4D` for exact value `0x80`
  for up to 20 ms, tolerating transient read failures.
- CTRL9 completion always polls STATUSINT.bit7 and acknowledges successful
  commands by writing `0x00` to CTRL9.
- Normal CTRL9 waits use a 100 ms limit; on-demand calibration allows up to
  2 seconds.
- Pull-up configuration writes only the documented CAL1_L payload.
- Self-test waits two output periods after disabling sensors.

### Fixed

- Restored the v0.1.1 `Config` and `Error` public shapes for patch-release
  source compatibility.
- Prevented QMI8658C initialization from writing its reserved CTRL1 bits 4–3.
- Prevented DRDY disable state from being lost during later CTRL7 writes.
- Removed the invalid STATUS1.bit0 CmdDone interpretation.
- Corrected FIFO routing defaults so unchanged applications retain main-branch
  register behavior.
- Aligned both ESP32-S3 applications with `defmt` 1.1.1 and refreshed their
  lockfiles so they build against the v0.1.2 driver.
- Expanded CI architecture coverage to explicitly include the ESP32-C2/C3,
  C5/C6/C61/H2, P4, RP2040, both RP2350/RP2354 CPU architectures, and common
  STM32, Nordic, Microchip SAM, and NXP Cortex-M families.
- Added automated BE hardware-evidence firmware and host tooling for all eight
  QMI8658A/C, I2C/SPI, and little-/big-endian combinations, including raw-byte
  cross-checks, physical plausibility checks, matrix gating, and collection
  instructions.

### Documentation

- Added variant selection, migration, hardware validation, and publishing
  guidance.
- Added the vendor datasheets used for the register review and a v0.1.2
  remediation/status tracker.
- QMI8658B remains unsupported pending an official vendor datasheet and
  hardware evidence.

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
