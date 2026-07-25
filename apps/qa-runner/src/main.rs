#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

#[cfg(all(feature = "qmi8658a", feature = "qmi8658c"))]
compile_error!("features `qmi8658a` and `qmi8658c` are mutually exclusive");
#[cfg(not(any(feature = "qmi8658a", feature = "qmi8658c")))]
compile_error!("select exactly one sensor variant: `qmi8658a` or `qmi8658c`");
#[cfg(all(feature = "transport-i2c", feature = "transport-spi"))]
compile_error!("features `transport-i2c` and `transport-spi` are mutually exclusive");
#[cfg(not(any(feature = "transport-i2c", feature = "transport-spi")))]
compile_error!("select exactly one transport: `transport-i2c` or `transport-spi`");

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
#[cfg(feature = "transport-spi")]
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Pull};
#[cfg(feature = "transport-spi")]
use esp_hal::gpio::{Level, Output, OutputConfig};
#[cfg(feature = "transport-i2c")]
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
#[cfg(feature = "transport-spi")]
use esp_hal::spi::{
    Mode,
    master::{Config as SpiBusConfig, Spi},
};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
#[cfg(feature = "transport-spi")]
use ph_qmi8658::Qmi8658Spi;
use ph_qmi8658::{
    AccelConfig, AccelOutputDataRate, AccelRange, Config, Error as ImuError, FifoConfig, FifoMode,
    FifoSize, GyroConfig, GyroOutputDataRate, GyroRange, InterruptConfig, InterruptPin,
    OperatingMode, WomConfig, WomInterruptLevel,
};
#[cfg(feature = "transport-i2c")]
use ph_qmi8658::{Qmi8658Address, Qmi8658I2c};
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

type ImuPin = Input<'static>;
#[cfg(feature = "transport-i2c")]
type ImuI2c = I2c<'static, esp_hal::Async>;
#[cfg(feature = "transport-i2c")]
type ImuDriver = Qmi8658I2c<ImuI2c, ImuPin, ImuPin>;
#[cfg(feature = "transport-spi")]
type ImuSpiBus = Spi<'static, esp_hal::Async>;
#[cfg(feature = "transport-spi")]
type ImuSpiDevice = ExclusiveDevice<ImuSpiBus, Output<'static>, Delay>;
#[cfg(feature = "transport-spi")]
type ImuDriver = Qmi8658Spi<ImuSpiDevice, ImuPin, ImuPin>;
type SelfTestErr = ph_qmi8658::SelfTestError<<ImuPin as embedded_hal::digital::ErrorType>::Error>;

defmt::timestamp!("{=u64:ms}", 0u64);

#[used]
static APP_DESC_REF: &esp_bootloader_esp_idf::EspAppDesc = &ESP_APP_DESC;

const BOARD_NAME: &str = "ESP32-S3 Matrix Board";
#[cfg(feature = "qmi8658a")]
const SENSOR_VARIANT: &str = "qmi8658a";
#[cfg(feature = "qmi8658c")]
const SENSOR_VARIANT: &str = "qmi8658c";
#[cfg(feature = "transport-i2c")]
const TRANSPORT: &str = "i2c";
#[cfg(feature = "transport-spi")]
const TRANSPORT: &str = "spi";
const BIG_ENDIAN: bool = cfg!(feature = "big-endian");

const RETRY_LIMIT: u8 = 5;
const RETRY_DELAY_MS: u64 = 20;
const DATA_READY_RETRY_LIMIT: u16 = 20;
const DATA_READY_DELAY_MS: u64 = 5;
const FIFO_READY_RETRY_LIMIT: u16 = 20;
const FIFO_READY_DELAY_MS: u64 = 10;
const STREAM_SAMPLE_COUNT: usize = 5;
const STREAM_SAMPLE_DELAY_MS: u64 = 5;
const EVIDENCE_DIRECT_SAMPLE_COUNT: usize = 16;
const EVIDENCE_FIFO_BUFFER_LEN: usize = 192;

async fn reset_fifo_retry(imu: &mut ImuDriver, delay: &mut Delay) -> Result<(), ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.reset_fifo_with_delay(delay).await {
            Ok(()) => return Ok(()),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

async fn read_fifo_burst_retry(
    imu: &mut ImuDriver,
    delay: &mut Delay,
    buffer: &mut [u8],
) -> Result<(), ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.read_fifo_burst(delay, buffer).await {
            Ok(_) => return Ok(()),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

async fn set_ahb_clock_gating_retry(
    imu: &mut ImuDriver,
    delay: &mut Delay,
    enable: bool,
) -> Result<(), ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.set_ahb_clock_gating_with_delay(delay, enable).await {
            Ok(()) => return Ok(()),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

async fn read_sync_sample_retry(imu: &mut ImuDriver, delay: &mut Delay) -> Result<(), ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.read_sync_sample(delay).await {
            Ok(_) => return Ok(()),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

async fn read_sync_sample_case(imu: &mut ImuDriver, delay: &mut Delay) -> Result<(), ImuError> {
    imu.set_sync_sample(true).await?;
    Timer::after(Duration::from_millis(5)).await;
    let result = read_sync_sample_retry(imu, delay).await;
    if matches!(result, Err(ImuError::NotReady)) {
        if let Ok(status) = imu.read_interrupt_status().await {
            warn!(
                "sync status: avail={} locked={} accel_ready={} gyro_ready={} cmd_done={}",
                status.data_available,
                status.data_locked,
                status.accel_ready,
                status.gyro_ready,
                status.cmd_done
            );
        } else {
            warn!("sync status: failed to read interrupt status");
        }
        warn!("sync sample period ns={:?}", imu.sample_period_ns());
    }
    let _ = imu.set_sync_sample(false).await;
    result
}

async fn enable_wom_retry(
    imu: &mut ImuDriver,
    delay: &mut Delay,
    config: WomConfig,
) -> Result<(), ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.enable_wom(delay, config).await {
            Ok(()) => return Ok(()),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

async fn disable_wom_retry(imu: &mut ImuDriver, delay: &mut Delay) -> Result<(), ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.disable_wom(delay).await {
            Ok(()) => return Ok(()),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

async fn wait_for_data_ready(imu: &mut ImuDriver, accel: bool, gyro: bool) -> Result<(), ImuError> {
    if !accel && !gyro {
        return Ok(());
    }

    let (retries, delay_ms) = if let Some(period_ns) = imu.sample_period_ns() {
        let period_ms = u64::from(period_ns).div_ceil(1_000_000);
        let window_ms = (period_ms.saturating_mul(5)).clamp(20, 2_000);
        let delay_ms = DATA_READY_DELAY_MS.max(1);
        let retries = (window_ms / delay_ms).max(1);
        (retries.min(u64::from(u16::MAX)) as u16, delay_ms)
    } else {
        (DATA_READY_RETRY_LIMIT, DATA_READY_DELAY_MS)
    };

    let mut last_status = None;
    for _ in 0..retries {
        let status = imu.read_interrupt_status().await?;
        last_status = Some(status);
        let accel_ok = !accel || status.accel_ready;
        let gyro_ok = !gyro || status.gyro_ready;
        if status.data_available || (accel_ok && gyro_ok) {
            return Ok(());
        }
        Timer::after(Duration::from_millis(delay_ms)).await;
    }

    let ts0 = imu.read_timestamp().await?;
    Timer::after(Duration::from_millis(delay_ms.max(1))).await;
    let ts1 = imu.read_timestamp().await?;
    if ts0 != ts1 {
        warn!(
            "data-ready status missing; timestamp advanced {:?} -> {:?}",
            ts0, ts1
        );
        return Ok(());
    }

    if let Some(status) = last_status {
        warn!(
            "data-ready status timeout: avail={} locked={} accel_ready={} gyro_ready={}",
            status.data_available, status.data_locked, status.accel_ready, status.gyro_ready
        );
    }

    Err(ImuError::NotReady)
}

async fn wait_for_fifo_ready(imu: &mut ImuDriver) -> Result<(), ImuError> {
    let frame_bytes = imu.fifo_frame_format().bytes_per_frame();
    if frame_bytes == 0 {
        return Err(ImuError::InvalidData);
    }

    for _ in 0..FIFO_READY_RETRY_LIMIT {
        let status = imu.fifo_status().await?;
        if status.sample_count_bytes as usize >= frame_bytes {
            return Ok(());
        }
        Timer::after(Duration::from_millis(FIFO_READY_DELAY_MS)).await;
    }
    Err(ImuError::NotReady)
}

async fn read_fifo_manual(imu: &mut ImuDriver, delay: &mut Delay) -> Result<(), ImuError> {
    imu.request_fifo_read().await?;
    imu.wait_ctrl9_done(delay).await?;
    imu.enable_fifo_read_mode().await?;

    let status = imu.fifo_status().await?;
    let format = imu.fifo_frame_format();
    let frame_bytes = format.bytes_per_frame();
    if frame_bytes == 0 {
        let _ = imu.finish_fifo_read().await;
        return Err(ImuError::InvalidData);
    }

    let mut buffer = [0u8; 96];
    let mut read_len = status.sample_count_bytes as usize;
    if read_len > buffer.len() {
        read_len = buffer.len();
    }
    read_len -= read_len % frame_bytes;

    let result = if read_len > 0 {
        imu.read_fifo_data(&mut buffer[..read_len])
            .await
            .map(|_| ())
    } else {
        Ok(())
    };

    let _ = imu.finish_fifo_read().await;
    result
}

async fn stream_raw_blocks(imu: &mut ImuDriver) -> Result<(), ImuError> {
    for _ in 0..STREAM_SAMPLE_COUNT {
        let _ = imu.read_raw_block().await?;
        Timer::after(Duration::from_millis(STREAM_SAMPLE_DELAY_MS)).await;
    }
    Ok(())
}

fn decode_i16(bytes: [u8; 2]) -> i16 {
    if BIG_ENDIAN {
        i16::from_be_bytes(bytes)
    } else {
        i16::from_le_bytes(bytes)
    }
}

fn print_direct_record(index: usize, bytes: &[u8; 17]) {
    let timestamp = u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16);
    let temperature = decode_i16([bytes[3], bytes[4]]);
    let accel_x = decode_i16([bytes[5], bytes[6]]);
    let accel_y = decode_i16([bytes[7], bytes[8]]);
    let accel_z = decode_i16([bytes[9], bytes[10]]);
    let gyro_x = decode_i16([bytes[11], bytes[12]]);
    let gyro_y = decode_i16([bytes[13], bytes[14]]);
    let gyro_z = decode_i16([bytes[15], bytes[16]]);

    esp_println::println!(
        "QMI_EVIDENCE_DIRECT index={} raw={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} timestamp={} temp={} ax={} ay={} az={} gx={} gy={} gz={}",
        index,
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
        bytes[4],
        bytes[5],
        bytes[6],
        bytes[7],
        bytes[8],
        bytes[9],
        bytes[10],
        bytes[11],
        bytes[12],
        bytes[13],
        bytes[14],
        bytes[15],
        bytes[16],
        timestamp,
        temperature,
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z
    );
}

fn print_fifo_record(index: usize, bytes: &[u8]) {
    if bytes.len() < 12 {
        return;
    }
    let accel_x = decode_i16([bytes[0], bytes[1]]);
    let accel_y = decode_i16([bytes[2], bytes[3]]);
    let accel_z = decode_i16([bytes[4], bytes[5]]);
    let gyro_x = decode_i16([bytes[6], bytes[7]]);
    let gyro_y = decode_i16([bytes[8], bytes[9]]);
    let gyro_z = decode_i16([bytes[10], bytes[11]]);

    esp_println::println!(
        "QMI_EVIDENCE_FIFO index={} raw={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} ax={} ay={} az={} gx={} gy={} gz={}",
        index,
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
        bytes[4],
        bytes[5],
        bytes[6],
        bytes[7],
        bytes[8],
        bytes[9],
        bytes[10],
        bytes[11],
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z
    );
}

async fn run_be_evidence(imu: &mut ImuDriver, delay: &mut Delay) -> Result<(), ImuError> {
    let who_am_i = imu.read_register(0x00).await?;
    let revision_id = imu.read_register(0x01).await?;
    let ctrl1 = imu.read_register(0x02).await?;
    esp_println::println!(
        "QMI_EVIDENCE_META who_am_i={:02x} revision_id={:02x} ctrl1={:02x}",
        who_am_i,
        revision_id,
        ctrl1
    );

    wait_for_data_ready(imu, true, true).await?;
    for index in 0..EVIDENCE_DIRECT_SAMPLE_COUNT {
        let mut bytes = [0u8; 17];
        imu.read_registers(0x30, &mut bytes).await?;
        print_direct_record(index, &bytes);
        Timer::after(Duration::from_millis(STREAM_SAMPLE_DELAY_MS)).await;
    }

    let fifo = FifoConfig::new(FifoMode::Stream, FifoSize::Samples32, 8);
    imu.apply_fifo_config(fifo).await?;
    reset_fifo_retry(imu, delay).await?;
    wait_for_fifo_ready(imu).await?;

    let mut fifo_bytes = [0u8; EVIDENCE_FIFO_BUFFER_LEN];
    let mut readout = None;
    for _ in 0..RETRY_LIMIT {
        match imu.read_fifo_burst(delay, &mut fifo_bytes).await {
            Ok(value) => {
                readout = Some(value);
                break;
            }
            Err(ImuError::NotReady) => {
                Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await;
            }
            Err(err) => return Err(err),
        }
    }
    let readout = readout.ok_or(ImuError::NotReady)?;
    esp_println::println!(
        "QMI_EVIDENCE_FIFO_META bytes_read={} sample_count_bytes={} overflow={}",
        readout.bytes_read,
        readout.status.sample_count_bytes,
        u8::from(readout.status.overflow)
    );
    for (index, frame) in fifo_bytes[..readout.bytes_read]
        .chunks_exact(12)
        .enumerate()
    {
        print_fifo_record(index, frame);
    }

    imu.apply_fifo_config(FifoConfig::default()).await?;
    Ok(())
}

async fn copy_gyro_bias_and_read_retry(
    imu: &mut ImuDriver,
    delay: &mut Delay,
) -> Result<ph_qmi8658::GyroRaw, ImuError> {
    for _ in 0..RETRY_LIMIT {
        match imu.copy_gyro_bias_and_read(delay, 0, 0, 0).await {
            Ok(bias) => return Ok(bias),
            Err(ImuError::NotReady) => Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await,
            Err(err) => return Err(err),
        }
    }
    Err(ImuError::NotReady)
}

#[derive(Default)]
struct GroupReport {
    name: &'static str,
    passed: u32,
    failed: u32,
}

impl GroupReport {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            passed: 0,
            failed: 0,
        }
    }

    fn record(&mut self, ok: bool) {
        if ok {
            self.passed += 1;
        } else {
            self.failed += 1;
        }
    }

    fn log(&self) {
        if self.failed == 0 {
            info!("group {}: {} passed", self.name, self.passed);
        } else {
            warn!(
                "group {}: {} passed, {} failed",
                self.name, self.passed, self.failed
            );
        }
    }
}

async fn run_case<E: defmt::Format>(
    name: &'static str,
    fut: impl core::future::Future<Output = Result<(), E>>,
) -> bool {
    match fut.await {
        Ok(()) => {
            info!("PASS: {}", name);
            true
        }
        Err(err) => {
            warn!("FAIL: {}: {}", name, err);
            false
        }
    }
}

async fn apply_default_config(imu: &mut ImuDriver) -> Result<(), ImuError> {
    let config = Config::new();
    imu.set_config(config);
    imu.apply_config().await
}

async fn run_reset(imu: &mut ImuDriver, delay: &mut Delay) -> GroupReport {
    let mut report = GroupReport::new("reset");
    report.record(run_case("soft_reset", async { imu.soft_reset(delay).await }).await);
    report.record(run_case("verify_device", async { imu.verify_device().await }).await);
    report.record(
        run_case("apply_interrupt_config", async {
            let irq = InterruptConfig::new().with_ctrl9_handshake_statusint(true);
            imu.apply_interrupt_config(irq).await
        })
        .await,
    );
    report.record(run_case("apply_config", async { apply_default_config(imu).await }).await);
    report
}

async fn run_basic(imu: &mut ImuDriver) -> GroupReport {
    let mut report = GroupReport::new("basic");
    report.record(run_case("verify_device", async { imu.verify_device().await }).await);
    report.record(
        run_case("wait_for_data_ready", async {
            let config = imu.config();
            wait_for_data_ready(imu, config.accel.is_some(), config.gyro.is_some()).await
        })
        .await,
    );
    report.record(
        run_case("read_timestamp", async {
            let _ = imu.read_timestamp().await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report.record(
        run_case("read_raw_block", async {
            let _ = imu.read_raw_block().await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report.record(
        run_case("read_temperature_raw", async {
            let _ = imu.read_temperature_raw().await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report.record(
        run_case("read_accel_raw", async {
            let _ = imu.read_accel_raw().await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report.record(
        run_case("read_gyro_raw", async {
            let _ = imu.read_gyro_raw().await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report
}

async fn run_workflows(imu: &mut ImuDriver) -> GroupReport {
    let mut report = GroupReport::new("workflows");
    report.record(
        run_case("standby_resume", async {
            let original = imu.config();
            let result = (async {
                imu.set_config(Config::new().without_accel().without_gyro());
                imu.apply_config().await?;
                if imu.operating_mode() != OperatingMode::PowerOnDefault {
                    return Err(ImuError::InvalidData);
                }

                imu.set_config(original);
                imu.apply_config().await?;
                wait_for_data_ready(imu, original.accel.is_some(), original.gyro.is_some()).await?;
                let _ = imu.read_raw_block().await?;
                Ok::<(), ImuError>(())
            })
            .await;
            imu.set_config(original);
            let _ = imu.apply_config().await;
            result
        })
        .await,
    );
    report.record(
        run_case("accel_only_cycle", async {
            let original = imu.config();
            let result = (async {
                let accel = AccelConfig::new(AccelRange::G4, AccelOutputDataRate::Hz125);
                imu.set_config(Config::new().with_accel_config(accel).without_gyro());
                imu.apply_config().await?;
                wait_for_data_ready(imu, true, false).await?;
                let _ = imu.read_accel_raw().await?;
                Ok::<(), ImuError>(())
            })
            .await;
            imu.set_config(original);
            let _ = imu.apply_config().await;
            result
        })
        .await,
    );
    report.record(
        run_case("gyro_only_cycle", async {
            let original = imu.config();
            let result = (async {
                let gyro = GyroConfig::new(GyroRange::Dps256, GyroOutputDataRate::Hz125);
                imu.set_config(Config::new().with_gyro_config(gyro).without_accel());
                imu.apply_config().await?;
                wait_for_data_ready(imu, false, true).await?;
                let _ = imu.read_gyro_raw().await?;
                Ok::<(), ImuError>(())
            })
            .await;
            imu.set_config(original);
            let _ = imu.apply_config().await;
            result
        })
        .await,
    );
    report.record(
        run_case("stream_raw_blocks", async {
            let config = imu.config();
            wait_for_data_ready(imu, config.accel.is_some(), config.gyro.is_some()).await?;
            stream_raw_blocks(imu).await
        })
        .await,
    );
    report
}

async fn run_fifo(imu: &mut ImuDriver, delay: &mut Delay) -> GroupReport {
    let mut report = GroupReport::new("fifo");
    let fifo = FifoConfig::new(FifoMode::Stream, FifoSize::Samples32, 8);
    report.record(
        run_case("apply_fifo_config", async {
            imu.apply_fifo_config(fifo).await
        })
        .await,
    );
    report.record(run_case("reset_fifo", async { reset_fifo_retry(imu, delay).await }).await);
    report.record(run_case("wait_fifo_ready", async { wait_for_fifo_ready(imu).await }).await);
    report.record(
        run_case("read_fifo_manual", async {
            read_fifo_manual(imu, delay).await
        })
        .await,
    );
    report.record(
        run_case("wait_fifo_ready_after_manual", async {
            wait_for_fifo_ready(imu).await
        })
        .await,
    );
    report.record(
        run_case("read_fifo_burst", async {
            let mut buffer = [0u8; 96];
            read_fifo_burst_retry(imu, delay, &mut buffer).await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report
}

async fn run_sync_sample(imu: &mut ImuDriver, delay: &mut Delay) -> GroupReport {
    let mut report = GroupReport::new("sync-sample");
    report.record(
        run_case("disable_fifo", async {
            imu.apply_fifo_config(FifoConfig::default()).await
        })
        .await,
    );
    report.record(run_case("apply_config", async { apply_default_config(imu).await }).await);
    report.record(
        run_case("disable_ahb_clock_gating", async {
            set_ahb_clock_gating_retry(imu, delay, false).await
        })
        .await,
    );
    report.record(
        run_case("read_sync_sample", async {
            read_sync_sample_case(imu, delay).await
        })
        .await,
    );
    report.record(
        run_case("restore_ahb_clock_gating", async {
            set_ahb_clock_gating_retry(imu, delay, true).await
        })
        .await,
    );
    report
}

async fn run_wom(imu: &mut ImuDriver, delay: &mut Delay) -> GroupReport {
    let mut report = GroupReport::new("wom");
    report.record(
        run_case("enable_disable_wom", async {
            let original = imu.config();
            let accel = AccelConfig::new(AccelRange::G4, AccelOutputDataRate::LowPowerHz21);
            imu.set_config(Config::new().with_accel_config(accel).without_gyro());
            imu.apply_config().await?;

            let wom = WomConfig::new(50)
                .with_blanking_samples(8)
                .with_interrupt(InterruptPin::Int1, WomInterruptLevel::Low);
            enable_wom_retry(imu, delay, wom).await?;
            Timer::after(Duration::from_millis(10)).await;
            disable_wom_retry(imu, delay).await?;

            imu.set_config(original);
            imu.apply_config().await?;
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report
}

async fn run_calibration(imu: &mut ImuDriver, delay: &mut Delay) -> GroupReport {
    let mut report = GroupReport::new("calibration");
    report.record(
        run_case("accel_host_delta_offset", async {
            imu.apply_accel_host_delta_offset_with_delay(delay, 0, 0, 0)
                .await
        })
        .await,
    );
    report.record(
        run_case("gyro_host_delta_offset", async {
            imu.apply_gyro_host_delta_offset_with_delay(delay, 0, 0, 0)
                .await
        })
        .await,
    );
    report.record(
        run_case("gyro_bias_copy", async {
            let bias = copy_gyro_bias_and_read_retry(imu, delay).await?;
            info!("gyro bias fifo={:?}", bias);
            Ok::<(), ImuError>(())
        })
        .await,
    );
    report.record(
        run_case("on_demand_calibration", async {
            imu.run_on_demand_calibration(delay).await
        })
        .await,
    );
    report.record(run_case("apply_config", async { apply_default_config(imu).await }).await);
    report
}

async fn run_self_test(imu: &mut ImuDriver, delay: &mut Delay) -> GroupReport {
    let mut report = GroupReport::new("self-test");
    report.record(
        run_case::<SelfTestErr>("accel_self_test", async {
            let _ = imu.run_accel_self_test(delay).await?;
            Ok(())
        })
        .await,
    );
    report.record(
        run_case::<SelfTestErr>("gyro_self_test", async {
            let _ = imu.run_gyro_self_test(delay).await?;
            Ok(())
        })
        .await,
    );
    report
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("IMU hardware test runner starting ({})", BOARD_NAME);
    esp_println::println!(
        "QMI_EVIDENCE_BEGIN schema=1 variant={} transport={} be={} orientation=z-up direct_samples={} fifo_buffer_bytes={}",
        SENSOR_VARIANT,
        TRANSPORT,
        u8::from(BIG_ENDIAN),
        EVIDENCE_DIRECT_SAMPLE_COUNT,
        EVIDENCE_FIFO_BUFFER_LEN
    );

    let int1 = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    let int2 = Input::new(
        peripherals.GPIO13,
        InputConfig::default().with_pull(Pull::Up),
    );

    #[cfg(feature = "transport-i2c")]
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    #[cfg(feature = "transport-i2c")]
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO11)
        .with_scl(peripherals.GPIO12)
        .into_async();

    let mut delay = Delay;
    let config = Config::new();

    #[cfg(feature = "transport-i2c")]
    let mut imu = {
        let interface_config =
            ph_qmi8658::I2cConfig::new(Qmi8658Address::Primary.addr()).with_big_endian(BIG_ENDIAN);
        Qmi8658I2c::with_i2c_config(i2c, Some(int1), Some(int2), config, interface_config)
    };

    #[cfg(feature = "transport-spi")]
    let mut imu = {
        let spi_bus_config = SpiBusConfig::default()
            .with_frequency(Rate::from_khz(1_000))
            .with_mode(Mode::_0);
        let spi = Spi::new(peripherals.SPI2, spi_bus_config)
            .unwrap()
            .with_sck(peripherals.GPIO12)
            .with_mosi(peripherals.GPIO11)
            .with_miso(peripherals.GPIO14)
            .into_async();
        let cs = Output::new(peripherals.GPIO9, Level::High, OutputConfig::default());
        let device = ExclusiveDevice::new(spi, cs, Delay).unwrap();
        let interface_config = ph_qmi8658::SpiConfig::new().with_big_endian(BIG_ENDIAN);
        Qmi8658Spi::with_spi_config(device, Some(int1), Some(int2), config, interface_config)
    };

    #[cfg(feature = "transport-i2c")]
    let init_result = imu
        .init_with_addresses(
            &mut delay,
            &[
                Qmi8658Address::Primary.addr(),
                Qmi8658Address::Secondary.addr(),
            ],
        )
        .await
        .map(|address| {
            info!("IMU init ok @0x{:02x}", address);
        });

    #[cfg(feature = "transport-spi")]
    let init_result = imu.init(&mut delay).await.map(|()| {
        info!("IMU SPI init ok");
    });

    match init_result {
        Ok(()) => {}
        Err(err) => {
            error!("IMU init failed: {}", err);
            esp_println::println!("QMI_EVIDENCE_ERROR stage=init error={:?}", err);
            esp_println::println!("QMI_EVIDENCE_END result=fail");
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    let irq = InterruptConfig::new().with_ctrl9_handshake_statusint(true);
    if let Err(err) = imu.apply_interrupt_config(irq).await {
        warn!("interrupt config failed: {}", err);
    }

    if let Err(err) = apply_default_config(&mut imu).await {
        error!("apply_config failed: {}", err);
    }

    match run_be_evidence(&mut imu, &mut delay).await {
        Ok(()) => {
            esp_println::println!("QMI_EVIDENCE_END result=pass");
        }
        Err(err) => {
            warn!("BE evidence collection failed: {}", err);
            esp_println::println!("QMI_EVIDENCE_ERROR stage=collection error={:?}", err);
            esp_println::println!("QMI_EVIDENCE_END result=fail");
        }
    }

    let mut total_passed = 0u32;
    let mut total_failed = 0u32;

    {
        let report = run_reset(&mut imu, &mut delay).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_basic(&mut imu).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_workflows(&mut imu).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_fifo(&mut imu, &mut delay).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_sync_sample(&mut imu, &mut delay).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_wom(&mut imu, &mut delay).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_self_test(&mut imu, &mut delay).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    {
        let report = run_calibration(&mut imu, &mut delay).await;
        total_passed += report.passed;
        total_failed += report.failed;
        report.log();
    }

    info!(
        "hardware tests complete: {} passed, {} failed",
        total_passed, total_failed
    );

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
