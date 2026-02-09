# ph-qmi8658 Driver

[![Crates.io](https://img.shields.io/crates/v/ph-qmi8658.svg)](https://crates.io/crates/ph-qmi8658) [![Docs.rs](https://docs.rs/ph-qmi8658/badge.svg)](https://docs.rs/ph-qmi8658) [![CI](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml/badge.svg)](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](../../LICENSE)

Async `#![no_std]` driver for the [QMI8658C](https://www.qstcorp.com/en_comp_prod/QMI8658C)
6-axis IMU (3-axis accelerometer + 3-axis gyroscope + temperature sensor) from QST
Corporation. Built on `embedded-hal-async` with I2C and SPI transport support.

MSRV: **1.92.0**

This README covers the driver's usage flows with code examples. For module structure and
data-flow diagrams, see [ARCHITECTURE.md](ARCHITECTURE.md). If you change driver behavior
or flow sequencing, update both documents to keep them aligned with the code.

## Driver Capabilities

- Configurable accelerometer (2/4/8/16 g) and gyroscope (16 to 2048 dps) ranges and ODRs
- FIFO buffering (stream, FIFO, and watermark modes) with burst and manual read paths
- Interrupt routing to INT1/INT2 pins with CTRL9 handshake support
- Wake-on-motion detection for low-power applications
- Sync-sample data locking for coherent multi-register reads
- Self-test, host-delta offset calibration, and on-demand calibration
- Integer scaling helpers (`ScaleFactor` ratios, no floats); optional `fixed`-point conversions
- Init-sequence helper macro to reduce boilerplate

## Usage Examples

All examples below assume an async context, an `I2C` bus, and a `DelayNs`
implementation. Use the Common Setup snippet once, then follow the flow examples.

### Common Setup

```rust
use ph_qmi8658::{Config, I2cConfig, Qmi8658Address, Qmi8658I2c};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

async fn setup<I2C: I2c, D: DelayNs>(
    i2c: I2C,
    delay: &mut D,
) -> Result<Qmi8658I2c<I2C>, ph_qmi8658::Error> {
    let config = Config::new();
    let i2c_config = I2cConfig::new(Qmi8658Address::Primary.addr());
    let mut imu: Qmi8658I2c<I2C> = Qmi8658I2c::with_i2c_config(i2c, None, None, config, i2c_config);
    imu.init(delay).await?;
    Ok(imu)
}
```

### Initialization

Reset the sensor, verify the chip ID, and apply the default configuration.

```rust
// Known address
imu.init(delay).await?;

// Probe multiple addresses
let address = imu.init_with_addresses(
    delay,
    &[Qmi8658Address::Primary.addr(), Qmi8658Address::Secondary.addr()],
).await?;
let _ = address;
```

If `init` or `init_with_addresses` returns `Error::NotReady`, delay briefly and retry.

### Initialization Macro

The `qmi8658_init_sequence!` macro combines address probing, interrupt configuration,
and sensor configuration into a single call.

```rust
use ph_qmi8658::{
    Config,
    InterruptConfig,
    Qmi8658Address,
    qmi8658_init_sequence,
};

let config = Config::new();
let irq = InterruptConfig::new().with_ctrl9_handshake_statusint(true);
let address = qmi8658_init_sequence!(
    imu: imu,
    delay: delay,
    addresses: &[Qmi8658Address::Primary.addr(), Qmi8658Address::Secondary.addr()],
    interrupt: irq,
    config: config,
)?;
let _ = address;
```

### Configuration

Set accelerometer and gyroscope ranges, output data rates, and low-pass filter modes.

```rust
use ph_qmi8658::{AccelConfig, AccelOutputDataRate, AccelRange, Config};

let accel = AccelConfig::new(AccelRange::G4, AccelOutputDataRate::Hz250);
let config = Config::new().with_accel_config(accel).without_gyro();
imu.set_config(config);
imu.apply_config().await?;
```

### Interrupt Routing + Status

Route data-ready and motion events to physical INT pins and read back interrupt status.

```rust
use ph_qmi8658::{InterruptConfig, InterruptPin};

let irq = InterruptConfig::new()
    .with_ctrl9_handshake_statusint(true)
    .with_motion_pin(InterruptPin::Int1);
imu.apply_interrupt_config(irq).await?;

let status = imu.read_interrupt_status().await?;
let _ = status;
```

### Raw Data Reads

Read individual sensor outputs or a full block (timestamp + temperature + accel + gyro).

```rust
let block = imu.read_raw_block().await?;
let accel = imu.read_accel_raw().await?;
let gyro = imu.read_gyro_raw().await?;
let temp = imu.read_temperature_raw().await?;
let ts = imu.read_timestamp().await?;
let _ = (block, accel, gyro, temp, ts);
```

### Scaling Helpers (Integer, No Floats)

Convert raw counts to physical units using integer `ScaleFactor` ratios. Useful on
targets without an FPU.

```rust
use ph_qmi8658::{AccelRange, GyroRange, accel_mg_per_lsb, gyro_mdps_per_lsb};

let accel_scale = accel_mg_per_lsb(AccelRange::G4);
let gyro_scale = gyro_mdps_per_lsb(GyroRange::Dps256);

// Example conversion for a single axis (i16 raw -> i32 milli-units).
let ax_mg = (i32::from(accel.x) * accel_scale.numerator) / accel_scale.denominator;
let gx_mdps = (i32::from(gyro.x) * gyro_scale.numerator) / gyro_scale.denominator;
let _ = (ax_mg, gx_mdps);
```

### FIFO Burst Read + Decode

Stream sensor data through the hardware FIFO and iterate over decoded frames.

```rust
use ph_qmi8658::{FifoConfig, FifoFrameIterator, FifoMode, FifoSize};

let fifo = FifoConfig::new(FifoMode::Stream, FifoSize::Samples64, 8);
imu.apply_fifo_config(fifo).await?;
imu.reset_fifo_with_delay(delay).await?;

let mut buffer = [0u8; 192];
let readout = imu.read_fifo_burst(delay, &mut buffer).await?;
let format = imu.fifo_frame_format();
for frame in FifoFrameIterator::new(&buffer[..readout.bytes_read], format) {
    let _ = frame;
}
```

### FIFO Manual Read Sequence

Step through the FIFO read protocol manually for fine-grained control.

```rust
let mut buffer = [0u8; 96];
imu.request_fifo_read().await?;
imu.wait_ctrl9_done(delay).await?;
imu.enable_fifo_read_mode().await?;
let timestamp = imu.read_fifo_data(&mut buffer).await?;
imu.finish_fifo_read().await?;
let _ = timestamp;
```

### Sync Sample (Data-Lock)

Lock a coherent snapshot of all sensor registers and read them atomically.

```rust
// For I2C/I3C, disable AHB clock gating while sync sample is active.
imu.set_ahb_clock_gating_with_delay(delay, false).await?;
imu.set_sync_sample(true).await?;

let sample = imu.read_sync_sample(delay).await?;
let _ = sample;

imu.set_sync_sample(false).await?;
imu.set_ahb_clock_gating_with_delay(delay, true).await?;
```

### Wake on Motion (WoM)

Configure low-power wake-on-motion detection to trigger on acceleration exceeding a
threshold.

```rust
use ph_qmi8658::{AccelConfig, AccelOutputDataRate, AccelRange, Config, WomConfig};

let accel = AccelConfig::new(AccelRange::G4, AccelOutputDataRate::LowPowerHz21);
imu.set_config(Config::new().with_accel_config(accel).without_gyro());
imu.apply_config().await?;

let wom = WomConfig::new(50);
imu.enable_wom(delay, wom).await?;
// ... wait for motion / handle interrupt ...
imu.disable_wom(delay).await?;
```

### Self-Test & Calibration

Run the built-in self-test, apply host-delta offsets, or trigger on-demand calibration.

```rust
let accel_report = imu.run_accel_self_test(delay).await?;
let gyro_report = imu.run_gyro_self_test(delay).await?;
let axes = imu.read_self_test_axes().await?;

imu.apply_accel_host_delta_offset_with_delay(delay, 0, 0, 0).await?;
imu.apply_gyro_host_delta_offset_with_delay(delay, 0, 0, 0).await?;
let bias = imu.copy_gyro_bias_and_read(delay, 0, 0, 0).await?;
imu.run_on_demand_calibration(delay).await?;

let _ = (accel_report, gyro_report, axes, bias);
```

### Operating Modes

Switch between accelerometer-only, gyroscope-only, or dual-sensor modes. The driver
returns the required stabilization delay.

```rust
use ph_qmi8658::OperatingMode;

let delay_ns = imu.set_mode(OperatingMode::AccelOnly).await?;
if delay_ns > 0 {
    delay.delay_ns(delay_ns).await;
}

imu.set_mode_with_delay(delay, OperatingMode::GyroOnly).await?;
```

## Target Platforms

The driver is `#![no_std]` and builds for any target that supports `embedded-hal-async`.
Tested targets include:

**ESP32 (Xtensa)** &mdash; requires the Espressif `esp` toolchain:
- `xtensa-esp32-none-elf`
- `xtensa-esp32s2-none-elf`
- `xtensa-esp32s3-none-elf`

**ESP32 (RISC-V)**:
- `riscv32imc-unknown-none-elf`
- `riscv32imac-unknown-none-elf`

**ARM Cortex-M** &mdash; standard Rust toolchains:
- `thumbv6m-none-eabi`, `thumbv7m-none-eabi`, `thumbv7em-none-eabi`,
  `thumbv7em-none-eabihf`, `thumbv8m.base-none-eabi`, `thumbv8m.main-none-eabi`,
  `thumbv8m.main-none-eabihf`

## Cargo Features

| Feature | Description |
|---------|-------------|
| `defmt` | Enable `defmt::Format` derives on public types for structured logging |
| `fixed` | Enable fixed-point conversion helpers (`I32F32`) for raw-to-physical-unit math |

No features are enabled by default.

## Testing

The driver has unit tests covering configuration validation, data decoding, FIFO frame
parsing, and interrupt/status decoding. Run them with:

```bash
cargo test -p ph-qmi8658
```

End-to-end hardware validation uses the [`apps/qa-runner`](../../apps/qa-runner/) app on
ESP32-S3.

## Release Checklist

See [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).
