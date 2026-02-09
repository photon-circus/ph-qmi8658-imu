# IMU FIFO Example (ESP32-S3)

FIFO-based example application for the QMI8658 IMU driver. This app targets the
ESP32-S3 Matrix Board and streams FIFO data, then prints simple motion features
(RMS, peak, and activity percentage) to the console.

## Quick Start

Build the app:
```bash
cd apps/imu-example
cargo build
```

Flash and monitor:
```bash
espflash flash --monitor --log-format defmt
```

## What It Does

- Initializes the IMU with a retry loop for `Error::NotReady`.
- Configures accel/gyro at 250 Hz and enables FIFO stream mode.
- Reads FIFO bursts and computes motion features. Outputs include RMS magnitude
  (accel in mg, gyro in mdps), peak magnitude, and activity percentage above a
  threshold.

## Configuration Defaults

Defaults are defined in `apps/imu-example/src/main.rs`:
- Accel: `G4` @ `Hz250`
- Gyro: `Dps256` @ `Hz250`
- FIFO: stream mode, 64-sample depth, watermark = 8

Adjust these values if you need different ranges, ODRs, or FIFO sizing.

## Pinout (ESP32-S3 Matrix Board)

The example assumes the following pins:
- I2C SDA: GPIO11
- I2C SCL: GPIO12
- INT1: GPIO10
- INT2: GPIO13

If your wiring differs, update the pin assignments in
`apps/imu-example/src/main.rs`.

## Notes

- This app uses the `esp` toolchain defined in `apps/imu-example/rust-toolchain.toml`.
- The IMU initialization sequence uses the `qmi8658_init_sequence!` helper macro
  from the driver crate.
