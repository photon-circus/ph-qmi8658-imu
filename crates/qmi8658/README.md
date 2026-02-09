# ph-qmi8658 Driver

Async driver for the QMI8658 6-axis IMU sensor built on `embedded-hal-async`.

This README describes the flows that are implemented in the current code. For
architecture and data flow details, see `ARCHITECTURE.md` in this directory. If
you change driver behavior or flow sequencing, update both documents to keep
them aligned with the code.

All examples below assume an async context, an `I2C` bus, and a `DelayNs`
implementation. Use the Common Setup snippet once, then follow the flow
examples.

**Common Setup**
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

**Initialization**
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

**Initialization Macro**
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

**Configuration**
```rust
use ph_qmi8658::{AccelConfig, AccelOutputDataRate, AccelRange, Config};

let accel = AccelConfig::new(AccelRange::G4, AccelOutputDataRate::Hz250);
let config = Config::new().with_accel_config(accel).without_gyro();
imu.set_config(config);
imu.apply_config().await?;
```

**Interrupt Routing + Status**
```rust
use ph_qmi8658::{InterruptConfig, InterruptPin};

let irq = InterruptConfig::new()
    .with_ctrl9_handshake_statusint(true)
    .with_motion_pin(InterruptPin::Int1);
imu.apply_interrupt_config(irq).await?;

let status = imu.read_interrupt_status().await?;
let _ = status;
```

**Raw Data Reads**
```rust
let block = imu.read_raw_block().await?;
let accel = imu.read_accel_raw().await?;
let gyro = imu.read_gyro_raw().await?;
let temp = imu.read_temperature_raw().await?;
let ts = imu.read_timestamp().await?;
let _ = (block, accel, gyro, temp, ts);
```

**FIFO Burst Read + Decode**
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

**FIFO Manual Read Sequence**
```rust
let mut buffer = [0u8; 96];
imu.request_fifo_read().await?;
imu.wait_ctrl9_done(delay).await?;
imu.enable_fifo_read_mode().await?;
let timestamp = imu.read_fifo_data(&mut buffer).await?;
imu.finish_fifo_read().await?;
let _ = timestamp;
```

**Sync Sample (Data-Lock)**
```rust
// For I2C/I3C, disable AHB clock gating while sync sample is active.
imu.set_ahb_clock_gating_with_delay(delay, false).await?;
imu.set_sync_sample(true).await?;

let sample = imu.read_sync_sample(delay).await?;
let _ = sample;

imu.set_sync_sample(false).await?;
imu.set_ahb_clock_gating_with_delay(delay, true).await?;
```

**Wake on Motion (WoM)**
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

**Self-Test & Calibration**
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

**Operating Modes**
```rust
use ph_qmi8658::OperatingMode;

let delay_ns = imu.set_mode(OperatingMode::AccelOnly).await?;
if delay_ns > 0 {
    delay.delay_ns(delay_ns).await;
}

imu.set_mode_with_delay(delay, OperatingMode::GyroOnly).await?;
```

**Target Matrix (CLI Builds)**
- ESP32 (xtensa): `xtensa-esp32-none-elf`
- ESP32-S2 (xtensa): `xtensa-esp32s2-none-elf`
- ESP32-S3 (xtensa): `xtensa-esp32s3-none-elf`
- ESP32-C2/C3/C6/H2 (riscv32): `riscv32imc-unknown-none-elf`, `riscv32imac-unknown-none-elf`
- ARM Cortex-M (common): `thumbv6m-none-eabi`, `thumbv7m-none-eabi`, `thumbv7em-none-eabi`,
  `thumbv7em-none-eabihf`, `thumbv8m.base-none-eabi`, `thumbv8m.main-none-eabi`,
  `thumbv8m.main-none-eabihf`

Notes:
- Xtensa targets require the Espressif `esp` toolchain.
- ARM targets use the standard Rust toolchains.

**Features**
- `defmt`: defmt formatting support
- `fixed`: fixed-point conversions for raw data
