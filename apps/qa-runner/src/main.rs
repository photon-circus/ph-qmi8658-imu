#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

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
    FifoMode,
    FifoSize,
    GyroConfig,
    GyroOutputDataRate,
    GyroRange,
    InterruptConfig,
    InterruptPin,
    OperatingMode,
    Qmi8658Address,
    Qmi8658I2c,
    WomConfig,
    WomInterruptLevel,
};
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

type ImuI2c = I2c<'static, esp_hal::Async>;
type ImuPin = Input<'static>;
type ImuDriver = Qmi8658I2c<ImuI2c, ImuPin, ImuPin>;
type SelfTestErr =
    ph_qmi8658::SelfTestError<<ImuPin as embedded_hal::digital::ErrorType>::Error>;

defmt::timestamp!("{=u64:ms}", 0u64);

#[used]
static APP_DESC_REF: &esp_bootloader_esp_idf::EspAppDesc = &ESP_APP_DESC;

const BOARD_NAME: &str = "ESP32-S3 Matrix Board";

const RETRY_LIMIT: u8 = 5;
const RETRY_DELAY_MS: u64 = 20;
const DATA_READY_RETRY_LIMIT: u16 = 20;
const DATA_READY_DELAY_MS: u64 = 5;
const FIFO_READY_RETRY_LIMIT: u16 = 20;
const FIFO_READY_DELAY_MS: u64 = 10;
const STREAM_SAMPLE_COUNT: usize = 5;
const STREAM_SAMPLE_DELAY_MS: u64 = 5;

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

async fn wait_for_data_ready(
    imu: &mut ImuDriver,
    accel: bool,
    gyro: bool,
) -> Result<(), ImuError> {
    if !accel && !gyro {
        return Ok(());
    }

    let (retries, delay_ms) = if let Some(period_ns) = imu.sample_period_ns() {
        let period_ms = (u64::from(period_ns) + 999_999) / 1_000_000;
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
            status.data_available,
            status.data_locked,
            status.accel_ready,
            status.gyro_ready
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
                self.name,
                self.passed,
                self.failed
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
    report.record(run_case("apply_fifo_config", async { imu.apply_fifo_config(fifo).await }).await);
    report.record(
        run_case("reset_fifo", async { reset_fifo_retry(imu, delay).await }).await,
    );
    report.record(
        run_case("wait_fifo_ready", async { wait_for_fifo_ready(imu).await }).await,
    );
    report.record(
        run_case("read_fifo_manual", async { read_fifo_manual(imu, delay).await }).await,
    );
    report.record(
        run_case("wait_fifo_ready_after_manual", async { wait_for_fifo_ready(imu).await }).await,
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

    let int1 = Input::new(peripherals.GPIO10, InputConfig::default().with_pull(Pull::Up));
    let int2 = Input::new(peripherals.GPIO13, InputConfig::default().with_pull(Pull::Up));

    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO11)
        .with_scl(peripherals.GPIO12)
        .into_async();

    let mut delay = Delay;
    let config = Config::new();
    let i2c_config = ph_qmi8658::I2cConfig::new(Qmi8658Address::Primary.addr());
    let mut imu = Qmi8658I2c::with_i2c_config(i2c, Some(int1), Some(int2), config, i2c_config);
    match imu
        .init_with_addresses(
            &mut delay,
            &[Qmi8658Address::Primary.addr(), Qmi8658Address::Secondary.addr()],
        )
        .await
    {
        Ok(address) => info!("IMU init ok @0x{:02x}", address),
        Err(err) => {
            error!("IMU init failed: {}", err);
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
        total_passed,
        total_failed
    );

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
