#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
//! FIFO-based motion feature example for the QMI8658 IMU.
//!
//! This app targets the ESP32-S3 Matrix Board and demonstrates:
//! - Robust IMU initialization with retry on `Error::NotReady`.
//! - FIFO burst reads for efficient sensor capture.
//! - Lightweight motion features (RMS, peak, and activity percentage).
//!
//! The flow is intentionally linear and verbose to serve as a reference
//! for integrating the driver in other embedded apps.

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Rate;
use ph_qmi8658::{
    AccelConfig,
    AccelOutputDataRate,
    AccelRange,
    Config,
    Error as ImuError,
    FifoConfig,
    FifoFrameIterator,
    FifoMode,
    FifoSize,
    GyroConfig,
    GyroOutputDataRate,
    GyroRange,
    InterruptConfig,
    Qmi8658Address,
    Qmi8658I2c,
    qmi8658_init_sequence,
};
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

type ImuI2c = I2c<'static, esp_hal::Async>;
type ImuPin = Input<'static>;
type ImuDriver = Qmi8658I2c<ImuI2c, ImuPin, ImuPin>;

defmt::timestamp!("{=u64:ms}", 0u64);

#[used]
static APP_DESC_REF: &esp_bootloader_esp_idf::EspAppDesc = &ESP_APP_DESC;

/// Friendly hardware identifier for logs.
const BOARD_NAME: &str = "ESP32-S3 Matrix Board";

/// FIFO buffer sized for 16 frames: 12 bytes per frame (accel + gyro).
const FIFO_BUFFER_LEN: usize = 192;
/// FIFO watermark for stream reads.
const FIFO_WATERMARK: u8 = 8;
/// Delay between FIFO polling attempts when no data is ready.
const FIFO_POLL_DELAY_MS: u64 = 10;
/// Delay between init retries when the IMU reports `NotReady`.
const INIT_RETRY_DELAY_MS: u64 = 100;
/// Small settle time after switching modes or applying config.
const MODE_SETTLE_DELAY_MS: u64 = 20;

/// Retry helper for operations that can return `Error::NotReady`.
///
/// Any other error is surfaced immediately to the caller.
macro_rules! retry_not_ready {
    ($label:literal, $delay_ms:expr, $body:expr) => {{
        loop {
            match $body {
                Ok(value) => break Ok(value),
                Err(ImuError::NotReady) => {
                    warn!("{} not ready, retrying...", $label);
                    Timer::after(Duration::from_millis($delay_ms)).await;
                }
                Err(err) => break Err(err),
            }
        }
    }};
}

/// Activity thresholds used to estimate motion percentage.
const ACCEL_MOTION_THRESHOLD_MG: i64 = 300;
const GYRO_MOTION_THRESHOLD_MDPS: i64 = 50_000;

#[derive(Default)]
struct MotionStats {
    accel_samples: u32,
    gyro_samples: u32,
    accel_mag_sq_sum: u64,
    accel_mag_sq_peak: u64,
    gyro_mag_sq_sum: u64,
    gyro_mag_sq_peak: u64,
    accel_motion_samples: u32,
    gyro_motion_samples: u32,
}

impl MotionStats {
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// Accumulate raw FIFO frames into statistics using configured ranges.
    fn push(
        &mut self,
        frame: ph_qmi8658::FifoFrame,
        accel_range: AccelRange,
        gyro_range: GyroRange,
    ) {
        if let Some(accel) = frame.accel {
            let ax = accel_raw_to_mg(accel.x, accel_range);
            let ay = accel_raw_to_mg(accel.y, accel_range);
            let az = accel_raw_to_mg(accel.z, accel_range);
            let mag_sq = mag_sq_i64(ax, ay, az);
            self.accel_samples += 1;
            self.accel_mag_sq_sum = self.accel_mag_sq_sum.saturating_add(mag_sq);
            if mag_sq > self.accel_mag_sq_peak {
                self.accel_mag_sq_peak = mag_sq;
            }
            if mag_sq > (ACCEL_MOTION_THRESHOLD_MG as u64)
                * (ACCEL_MOTION_THRESHOLD_MG as u64)
            {
                self.accel_motion_samples += 1;
            }
        }

        if let Some(gyro) = frame.gyro {
            let gx = gyro_raw_to_mdps(gyro.x, gyro_range);
            let gy = gyro_raw_to_mdps(gyro.y, gyro_range);
            let gz = gyro_raw_to_mdps(gyro.z, gyro_range);
            let mag_sq = mag_sq_i64(gx, gy, gz);
            self.gyro_samples += 1;
            self.gyro_mag_sq_sum = self.gyro_mag_sq_sum.saturating_add(mag_sq);
            if mag_sq > self.gyro_mag_sq_peak {
                self.gyro_mag_sq_peak = mag_sq;
            }
            if mag_sq > (GYRO_MOTION_THRESHOLD_MDPS as u64)
                * (GYRO_MOTION_THRESHOLD_MDPS as u64)
            {
                self.gyro_motion_samples += 1;
            }
        }
    }
}

/// Convert raw accelerometer reading to milli-g (mg).
fn accel_raw_to_mg(raw: i16, range: AccelRange) -> i64 {
    let range_g = i64::from(range.g());
    (i64::from(raw) * range_g * 1000) / 32768
}

/// Convert raw gyro reading to milli-degrees per second (mdps).
fn gyro_raw_to_mdps(raw: i16, range: GyroRange) -> i64 {
    let range_dps = i64::from(range.dps());
    (i64::from(raw) * range_dps * 1000) / 32768
}

/// Return squared magnitude for motion thresholding and RMS calculations.
fn mag_sq_i64(x: i64, y: i64, z: i64) -> u64 {
    let x2 = x.saturating_mul(x) as u64;
    let y2 = y.saturating_mul(y) as u64;
    let z2 = z.saturating_mul(z) as u64;
    x2.saturating_add(y2).saturating_add(z2)
}

/// Integer square root for RMS and peak magnitude computation.
fn isqrt_u64(value: u64) -> u64 {
    if value == 0 {
        return 0;
    }
    let mut x = value;
    let mut y = (x + 1) / 2;
    while y < x {
        x = y;
        y = (x + value / x) / 2;
    }
    x
}

/// Compute RMS magnitude from a sum of squared magnitudes.
fn rms_from_sum(sum: u64, samples: u32) -> u64 {
    if samples == 0 {
        0
    } else {
        isqrt_u64(sum / u64::from(samples))
    }
}

async fn configure_imu(
    imu: &mut ImuDriver,
    delay: &mut Delay,
    desired_config: Config,
) -> Result<u8, ImuError> {
    // Apply the standard init sequence with interrupt config and desired modes.
    let irq = InterruptConfig::new().with_ctrl9_handshake_statusint(true);
    let address = retry_not_ready!(
        "IMU init",
        INIT_RETRY_DELAY_MS,
        qmi8658_init_sequence!(
            imu: imu,
            delay: delay,
            addresses: &[
                Qmi8658Address::Primary.addr(),
                Qmi8658Address::Secondary.addr()
            ],
            interrupt: irq,
            config: desired_config,
        )
    )?;

    // Allow time for mode/config to settle before FIFO operations.
    Timer::after(Duration::from_millis(MODE_SETTLE_DELAY_MS)).await;

    // Configure FIFO streaming and reset to a clean state.
    let fifo = FifoConfig::new(FifoMode::Stream, FifoSize::Samples64, FIFO_WATERMARK);
    imu.apply_fifo_config(fifo).await?;
    retry_not_ready!(
        "FIFO reset",
        INIT_RETRY_DELAY_MS,
        imu.reset_fifo_with_delay(delay).await
    )?;

    Ok(address)
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // Bring up clocks and timers early so delays are reliable.
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Configure interrupt pins from the IMU.
    let int1 = Input::new(peripherals.GPIO10, InputConfig::default().with_pull(Pull::Up));
    let int2 = Input::new(peripherals.GPIO13, InputConfig::default().with_pull(Pull::Up));

    // I2C at 400 kHz for fast FIFO reads.
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO11)
        .with_scl(peripherals.GPIO12)
        .into_async();

    // Target operating configuration for the example.
    let desired_accel = AccelConfig::new(AccelRange::G4, AccelOutputDataRate::Hz250);
    let desired_gyro = GyroConfig::new(GyroRange::Dps256, GyroOutputDataRate::Hz250);
    let desired_config = Config::new()
        .with_accel_config(desired_accel)
        .with_gyro_config(desired_gyro);

    // Instantiate the driver with the selected I2C address and interrupt pins.
    let i2c_config = ph_qmi8658::I2cConfig::new(Qmi8658Address::Primary.addr());
    let mut imu =
        Qmi8658I2c::with_i2c_config(i2c, Some(int1), Some(int2), Config::new(), i2c_config);

    let mut delay = Delay;
    // Retry init until the device is ready. Other errors are treated as fatal.
    let address = loop {
        match configure_imu(&mut imu, &mut delay, desired_config).await {
            Ok(address) => break address,
            Err(ImuError::NotReady) => {
                warn!("IMU configure not ready, retrying...");
                Timer::after(Duration::from_millis(INIT_RETRY_DELAY_MS)).await;
            }
            Err(err) => {
                error!("IMU init failed: {}", err);
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }
    };
    info!("IMU init ok @0x{:02x}", address);

    let accel_range = AccelRange::G4;
    let gyro_range = GyroRange::Dps256;

    // FIFO buffer and stats accumulator for motion features.
    let mut buffer = [0u8; FIFO_BUFFER_LEN];
    let mut stats = MotionStats::default();

    info!("FIFO motion example running ({})", BOARD_NAME);

    loop {
        // Read a FIFO burst. NotReady is a normal condition when no samples are queued.
        let readout = match imu.read_fifo_burst(&mut delay, &mut buffer).await {
            Ok(readout) => readout,
            Err(ImuError::NotReady) => {
                Timer::after(Duration::from_millis(FIFO_POLL_DELAY_MS)).await;
                continue;
            }
            Err(err) => {
                warn!("FIFO read failed: {}", err);
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
        };

        // No bytes means FIFO watermark not reached.
        if readout.bytes_read == 0 {
            Timer::after(Duration::from_millis(FIFO_POLL_DELAY_MS)).await;
            continue;
        }

        if readout.status.overflow {
            warn!("FIFO overflow");
        }

        let format = imu.fifo_frame_format();
        let data = &buffer[..readout.bytes_read];
        for frame in FifoFrameIterator::new(data, format) {
            stats.push(frame, accel_range, gyro_range);
        }

        // Report aggregate stats and reset per-burst.
        if stats.accel_samples > 0 || stats.gyro_samples > 0 {
            let accel_rms = rms_from_sum(stats.accel_mag_sq_sum, stats.accel_samples);
            let accel_peak = isqrt_u64(stats.accel_mag_sq_peak);
            let gyro_rms = rms_from_sum(stats.gyro_mag_sq_sum, stats.gyro_samples);
            let gyro_peak = isqrt_u64(stats.gyro_mag_sq_peak);

            let accel_active_pct = if stats.accel_samples == 0 {
                0
            } else {
                (stats.accel_motion_samples * 100) / stats.accel_samples
            };
            let gyro_active_pct = if stats.gyro_samples == 0 {
                0
            } else {
                (stats.gyro_motion_samples * 100) / stats.gyro_samples
            };

            info!(
                "fifo: accel_rms={}mg accel_peak={}mg accel_active={}pct gyro_rms={}mdps gyro_peak={}mdps gyro_active={}pct",
                accel_rms,
                accel_peak,
                accel_active_pct,
                gyro_rms,
                gyro_peak,
                gyro_active_pct
            );
        }

        stats.reset();
    }
}
