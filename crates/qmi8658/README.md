# ph-qmi8658 Driver

[![Crates.io](https://img.shields.io/crates/v/ph-qmi8658.svg)](https://crates.io/crates/ph-qmi8658) [![Docs.rs](https://docs.rs/ph-qmi8658/badge.svg)](https://docs.rs/ph-qmi8658) [![CI](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml/badge.svg)](https://github.com/photon-circus/ph-qmi8658-imu/actions/workflows/ci.yml) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](../../LICENSE)

Async `#![no_std]` driver for the QMI8658A/C 6-axis IMU (3-axis accelerometer,
3-axis gyroscope, and temperature sensor) from QST Corporation. Built on
`embedded-hal-async` with I2C and SPI transport support.

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

QMI8658C is selected by default. QMI8658A-specific behavior is available through
the `qmi8658a` feature. QMI8658B is not supported because an official vendor
datasheet was not available for review.

## Device Variant Selection

Use the default dependency for QMI8658C:

```toml
ph-qmi8658 = "0.1"
```

Select QMI8658A explicitly:

```toml
ph-qmi8658 = { version = "0.1", default-features = false, features = ["qmi8658a"] }
```

`qmi8658a` and `qmi8658c` are mutually exclusive. A no-default-features build
without either variant retains legacy QMI8658C behavior for v0.1.x source
compatibility.

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
Initialization writes the soft-reset command, waits 1 ms, and polls the reset
completion register for up to 20 ms. A complete window of bus failures returns
`Error::Bus`.

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

// DRDY defaults to enabled. This policy survives mode and WoM transitions.
imu.set_drdy_enabled(false).await?;
assert!(!imu.drdy_enabled());
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

`with_ctrl9_handshake_statusint(true)` suppresses the external INT1 command
handshake; it does not change the completion register. The driver always polls
STATUSINT.bit7 and acknowledges successful commands by writing `0x00` to CTRL9.

FIFO watermark interrupts map to INT2 by default. Opt into INT1 through the
transport configuration:

```rust
let i2c_config = I2cConfig::default().with_fifo_int_use_int1(true);
```

On QMI8658A only, the physical interrupt outputs can be changed from high-Z to
push-pull:

```rust
# #[cfg(feature = "qmi8658a")]
let i2c_config = I2cConfig::default()
    .with_enable_int1(true)
    .with_enable_int2(true);
```

Those builders are unavailable for QMI8658C because CTRL1 bits 4–3 are reserved.

### Internal Pull-Ups

Use `PullUpConfig` to change the vendor-defined IO pull-up groups. The completed
API performs the CAL1_L write, SET_RPU command, STATUSINT wait, and CTRL9 ACK.

```rust
use ph_qmi8658::{PullUpConfig, PullUpGroup};

let pull_ups = PullUpConfig::new()
    .with_group(PullUpGroup::Cs, true)
    .with_group(PullUpGroup::SclSda, true);
imu.apply_pull_up_config_with_delay(delay, pull_ups).await?;
```

The non-delay `apply_pull_up_config` method only starts the asynchronous command;
call `wait_ctrl9_done` before issuing another CTRL9 command.

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

Self-test waits two output periods after disabling sensors. Normal CTRL9 waits
time out after 100 ms; on-demand calibration allows up to 2 seconds.

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

**ESP32 (RISC-V)** &mdash; standard Rust toolchains:
- `riscv32imc-unknown-none-elf`: ESP32-C2 and ESP32-C3
- `riscv32imac-unknown-none-elf`: ESP32-C5, ESP32-C6, ESP32-C61, and ESP32-H2
- `riscv32imafc-unknown-none-elf`: ESP32-P4

**Raspberry Pi RP series**:
- `thumbv6m-none-eabi`: RP2040
- `thumbv8m.main-none-eabihf`: RP2350/RP2354 Arm Cortex-M33
- `riscv32imac-unknown-none-elf`: RP2350/RP2354 Hazard3 RISC-V

**Other common ARM Cortex-M families** &mdash; standard Rust toolchains:
- `thumbv6m-none-eabi`: STM32F0/G0/L0, nRF51, SAMD21
- `thumbv7m-none-eabi`: STM32F1/F2/L1, SAM3/SAM4
- `thumbv7em-none-eabi` and `thumbv7em-none-eabihf`: STM32F3/F4/F7/H7/L4,
  nRF52, SAMD51
- `thumbv8m.base-none-eabi`: Cortex-M23 devices such as SAM L10/L11
- `thumbv8m.main-none-eabi` and `thumbv8m.main-none-eabihf`: STM32H5/L5/U5,
  nRF53/nRF54, LPC55, and other Cortex-M33 devices

These are CPU/ABI compile checks for the HAL-independent driver. Only the
ESP32-S3 applications in this repository provide a board-level integration
build.

## Cargo Features

| Feature | Description |
|---------|-------------|
| `qmi8658c` | Select QMI8658C register behavior; enabled by default |
| `qmi8658a` | Select QMI8658A behavior, including CTRL1 INT output-enable bits |
| `defmt` | Enable `defmt::Format` derives on public types for structured logging |
| `fixed` | Enable fixed-point conversion helpers (`I32F32`) for raw-to-physical-unit math |

Do not enable both device-variant features.

## Testing

The driver has unit tests covering configuration validation, data decoding, FIFO frame
parsing, and interrupt/status decoding. Run them with:

```bash
cargo test -p ph-qmi8658
cargo test -p ph-qmi8658 --no-default-features
cargo test -p ph-qmi8658 --no-default-features --features qmi8658a
```

End-to-end hardware validation uses the [`apps/qa-runner`](../../apps/qa-runner/) app on
ESP32-S3.

The v0.1.2 candidate still requires hardware byte-order verification for
QMI8658A/C over I2C/SPI with CTRL1.BE set and cleared. No transport-specific
workaround is applied without reproducible evidence.

The QA runner includes automated firmware selection, serial-log capture,
raw-byte validation, physical plausibility checks, and a complete eight-row
matrix gate. See its [hardware evidence instructions](../../apps/qa-runner/README.md).

For qualification and vendor-register inspection, `read_register` and
`read_registers` expose read-only access by raw register address. These methods
do not bypass the driver's typed write paths.

## v0.1.1 Compatibility

v0.1.1 `Config` struct literals and exhaustive `Error` matches remain valid.
DRDY control is an additive driver method rather than a new public `Config`
field, and reset timeout uses the existing `Error::NotReady` variant.

## Release Checklist

See [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).
