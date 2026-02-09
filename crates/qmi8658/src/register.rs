//! QMI8658 register definitions.
//!
//! This module contains the full UI register map from the datasheet, plus
//! the bit masks used by the driver.

#![allow(dead_code)] // Full register map is intentional; many entries are not wired yet.

/// QMI8658 register addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    /// Device identifier register.
    WhoAmI = 0x00,
    /// Device revision register.
    RevisionId = 0x01,
    /// Control register 1.
    Ctrl1 = 0x02,
    /// Control register 2 (accelerometer settings).
    Ctrl2 = 0x03,
    /// Control register 3 (gyroscope settings).
    Ctrl3 = 0x04,
    /// Control register 4 (reserved / special settings).
    Ctrl4 = 0x05,
    /// Control register 5 (filter settings).
    Ctrl5 = 0x06,
    /// Control register 6 (AttitudeEngine settings).
    Ctrl6 = 0x07,
    /// Control register 7 (sensor enable).
    Ctrl7 = 0x08,
    /// Control register 8 (motion detection control).
    Ctrl8 = 0x09,
    /// Control register 9 (host commands).
    Ctrl9 = 0x0A,
    /// Calibration register 1 low byte.
    Cal1L = 0x0B,
    /// Calibration register 1 high byte.
    Cal1H = 0x0C,
    /// Calibration register 2 low byte.
    Cal2L = 0x0D,
    /// Calibration register 2 high byte.
    Cal2H = 0x0E,
    /// Calibration register 3 low byte.
    Cal3L = 0x0F,
    /// Calibration register 3 high byte.
    Cal3H = 0x10,
    /// Calibration register 4 low byte.
    Cal4L = 0x11,
    /// Calibration register 4 high byte.
    Cal4H = 0x12,
    /// FIFO watermark threshold.
    FifoWtmTh = 0x13,
    /// FIFO control.
    FifoCtrl = 0x14,
    /// FIFO sample count.
    FifoSmplCnt = 0x15,
    /// FIFO status.
    FifoStatus = 0x16,
    /// FIFO data.
    FifoData = 0x17,
    /// Status register with lock/CmdDone.
    StatusInt = 0x2D,
    /// Status register 0 (data ready).
    Status0 = 0x2E,
    /// Status register 1 (misc status / wake on motion).
    Status1 = 0x2F,
    /// Timestamp low byte.
    TimestampLow = 0x30,
    /// Timestamp mid byte.
    TimestampMid = 0x31,
    /// Timestamp high byte.
    TimestampHigh = 0x32,
    /// Temperature low byte.
    TempL = 0x33,
    /// Temperature high byte.
    TempH = 0x34,
    /// Accelerometer X-axis low byte.
    AxL = 0x35,
    /// Accelerometer X-axis high byte.
    AxH = 0x36,
    /// Accelerometer Y-axis low byte.
    AyL = 0x37,
    /// Accelerometer Y-axis high byte.
    AyH = 0x38,
    /// Accelerometer Z-axis low byte.
    AzL = 0x39,
    /// Accelerometer Z-axis high byte.
    AzH = 0x3A,
    /// Gyroscope X-axis low byte.
    GxL = 0x3B,
    /// Gyroscope X-axis high byte.
    GxH = 0x3C,
    /// Gyroscope Y-axis low byte.
    GyL = 0x3D,
    /// Gyroscope Y-axis high byte.
    GyH = 0x3E,
    /// Gyroscope Z-axis low byte.
    GzL = 0x3F,
    /// Gyroscope Z-axis high byte.
    GzH = 0x40,
    /// Quaternion increment W low byte.
    DqWL = 0x49,
    /// Quaternion increment W high byte.
    DqWH = 0x4A,
    /// Quaternion increment X low byte.
    DqXL = 0x4B,
    /// Quaternion increment X high byte.
    DqXH = 0x4C,
    /// Quaternion increment Y low byte.
    DqYL = 0x4D,
    /// Quaternion increment Y high byte.
    DqYH = 0x4E,
    /// Quaternion increment Z low byte.
    DqZL = 0x4F,
    /// Quaternion increment Z high byte.
    DqZH = 0x50,
    /// Delta velocity X low byte.
    DvXL = 0x51,
    /// Delta velocity X high byte.
    DvXH = 0x52,
    /// Delta velocity Y low byte.
    DvYL = 0x53,
    /// Delta velocity Y high byte.
    DvYH = 0x54,
    /// Delta velocity Z low byte.
    DvZL = 0x55,
    /// Delta velocity Z high byte.
    DvZH = 0x56,
    /// AttitudeEngine register 1.
    AeReg1 = 0x57,
    /// AttitudeEngine register 2.
    AeReg2 = 0x58,
    /// Soft reset register.
    Reset = 0x60,
}

impl Register {
    /// Returns the register address.
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

/// Expected values for WHO_AM_I.
pub mod who_am_i {
    /// Expected WHO_AM_I register value.
    pub const EXPECTED: u8 = 0b0000_0101;
}

/// Revision ID values.
pub mod revision_id {
    /// Revision ID for QMI8658C.
    pub const QMI8658C: u8 = 0b0110_1000;
}

/// CTRL1 register bits.
pub mod ctrl1 {
    /// SPI interface mode select (1 = 3-wire, 0 = 4-wire).
    pub const SIM: u8 = 0b1000_0000;
    /// Auto-increment register address on reads.
    pub const ADDR_AI: u8 = 0b0100_0000;
    /// Big-endian serial interface read data.
    pub const BE: u8 = 0b0010_0000;
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b0001_1110;
    /// Disable the internal 2 MHz oscillator (power-down).
    pub const SENSOR_DISABLE: u8 = 0b0000_0001;
}

/// CTRL2 register bits.
pub mod ctrl2 {
    /// Accelerometer self-test enable.
    pub const A_ST: u8 = 0b1000_0000;
    /// Accelerometer full-scale selection mask.
    pub const A_FS_MASK: u8 = 0b0111_0000;
    /// Accelerometer full-scale selection shift.
    pub const A_FS_SHIFT: u8 = 4;
    /// Accelerometer output data rate selection mask.
    pub const A_ODR_MASK: u8 = 0b0000_1111;
}

/// CTRL3 register bits.
pub mod ctrl3 {
    /// Gyroscope self-test enable.
    pub const G_ST: u8 = 0b1000_0000;
    /// Gyroscope full-scale selection mask.
    pub const G_FS_MASK: u8 = 0b0111_0000;
    /// Gyroscope full-scale selection shift.
    pub const G_FS_SHIFT: u8 = 4;
    /// Gyroscope output data rate selection mask.
    pub const G_ODR_MASK: u8 = 0b0000_1111;
}

/// CTRL4 register bits.
pub mod ctrl4 {
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b1111_1111;
}

/// CTRL5 register bits.
pub mod ctrl5 {
    /// Reserved bit.
    pub const RESERVED_7: u8 = 0b1000_0000;
    /// Gyroscope low-pass filter mode mask.
    pub const G_LPF_MODE_MASK: u8 = 0b0110_0000;
    /// Gyroscope low-pass filter mode shift.
    pub const G_LPF_MODE_SHIFT: u8 = 5;
    /// Gyroscope low-pass filter enable.
    pub const G_LPF_EN: u8 = 0b0001_0000;
    /// Reserved bit.
    pub const RESERVED_3: u8 = 0b0000_1000;
    /// Accelerometer low-pass filter mode mask.
    pub const A_LPF_MODE_MASK: u8 = 0b0000_0110;
    /// Accelerometer low-pass filter mode shift.
    pub const A_LPF_MODE_SHIFT: u8 = 1;
    /// Accelerometer low-pass filter enable.
    pub const A_LPF_EN: u8 = 0b0000_0001;
}

/// CTRL6 register bits.
pub mod ctrl6 {
    /// Motion on Demand enable.
    pub const S_MOD: u8 = 0b1000_0000;
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b0111_1000;
    /// AttitudeEngine output data rate selection mask.
    pub const S_ODR_MASK: u8 = 0b0000_0111;
    /// AttitudeEngine output data rate selection shift.
    pub const S_ODR_SHIFT: u8 = 0;
}

/// CTRL7 register bits.
pub mod ctrl7 {
    /// Enable sync sample mode.
    pub const SYNC_SMPL: u8 = 0b1000_0000;
    /// High-speed internal clock.
    pub const SYS_HS: u8 = 0b0100_0000;
    /// Reserved bit.
    pub const RESERVED_5: u8 = 0b0010_0000;
    /// Gyroscope snooze mode.
    pub const G_SN: u8 = 0b0001_0000;
    /// Enable AttitudeEngine.
    pub const S_EN: u8 = 0b0000_1000;
    /// Reserved bit.
    pub const RESERVED_2: u8 = 0b0000_0100;
    /// Enable gyroscope.
    pub const G_EN: u8 = 0b0000_0010;
    /// Enable accelerometer.
    pub const A_EN: u8 = 0b0000_0001;
    /// Enable gyroscope (alias).
    pub const GYR_ENABLE: u8 = G_EN;
    /// Enable accelerometer (alias).
    pub const ACC_ENABLE: u8 = A_EN;
}

/// CTRL8 register bits.
pub mod ctrl8 {
    /// Use STATUSINT.bit7 for CTRL9 handshake when set.
    pub const CTRL9_HANDSHAKE_STATUSINT: u8 = 0b1000_0000;
    /// Motion detection interrupt select (set = INT1, clear = INT2).
    pub const MOTION_INT_INT1: u8 = 0b0100_0000;
    /// Reserved bit.
    pub const RESERVED_5: u8 = 0b0010_0000;
    /// Pedometer enable.
    pub const PEDOMETER_EN: u8 = 0b0001_0000;
    /// Significant motion enable.
    pub const SIGNIFICANT_MOTION_EN: u8 = 0b0000_1000;
    /// No motion enable.
    pub const NO_MOTION_EN: u8 = 0b0000_0100;
    /// Any motion enable.
    pub const ANY_MOTION_EN: u8 = 0b0000_0010;
    /// Tap enable.
    pub const TAP_EN: u8 = 0b0000_0001;
}

/// CTRL9 register command values.
pub mod ctrl9 {
    /// Command field mask.
    pub const CMD_MASK: u8 = 0b1111_1111;
    /// No operation.
    pub const CMD_NOP: u8 = 0b0000_0000;
    /// Copy gyro bias from CAL registers.
    pub const CMD_GYRO_BIAS: u8 = 0b0000_0001;
    /// Request MoD (SDI) data.
    pub const CMD_REQ_SDI: u8 = 0b0000_0011;
    /// Reset FIFO.
    pub const CMD_RST_FIFO: u8 = 0b0000_0100;
    /// Request FIFO data.
    pub const CMD_REQ_FIFO: u8 = 0b0000_0101;
    /// Write Wake on Motion settings.
    pub const CMD_WRITE_WOM_SETTING: u8 = 0b0000_1000;
    /// Change accelerometer offset.
    pub const CMD_ACCEL_HOST_DELTA_OFFSET: u8 = 0b0000_1001;
    /// Change gyroscope offset.
    pub const CMD_GYRO_HOST_DELTA_OFFSET: u8 = 0b0000_1010;
    /// Copy USID and FW version bytes.
    pub const CMD_COPY_USID: u8 = 0b0001_0000;
    /// Configure IO pull-ups.
    pub const CMD_SET_RPU: u8 = 0b0001_0001;
    /// Internal AHB clock gating switch.
    pub const CMD_AHB_CLOCK_GATING: u8 = 0b0001_0010;
    /// On-demand gyro calibration.
    pub const CMD_ON_DEMAND_CALIBRATION: u8 = 0b1010_0010;
}

/// FIFO watermark threshold register bits.
pub mod fifo_wtm_th {
    /// FIFO watermark threshold mask.
    pub const WTM_MASK: u8 = 0b1111_1111;
}

/// FIFO control register bits.
pub mod fifo_ctrl {
    /// FIFO read mode enable.
    pub const FIFO_RD_MODE: u8 = 0b1000_0000;
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b0111_0000;
    /// FIFO size selection mask.
    pub const FIFO_SIZE_MASK: u8 = 0b0000_1100;
    /// FIFO size selection shift.
    pub const FIFO_SIZE_SHIFT: u8 = 2;
    /// FIFO mode selection mask.
    pub const FIFO_MODE_MASK: u8 = 0b0000_0011;
    /// FIFO mode selection shift.
    pub const FIFO_MODE_SHIFT: u8 = 0;
}

/// FIFO sample count register bits.
pub mod fifo_smpl_cnt {
    /// FIFO sample count LSB mask.
    pub const SMPL_CNT_LSB_MASK: u8 = 0b1111_1111;
}

/// FIFO status register bits.
pub mod fifo_status {
    /// FIFO full flag.
    pub const FIFO_FULL: u8 = 0b1000_0000;
    /// FIFO watermark flag.
    pub const FIFO_WTM: u8 = 0b0100_0000;
    /// FIFO overflow flag.
    pub const FIFO_OVFLOW: u8 = 0b0010_0000;
    /// FIFO not empty flag.
    pub const FIFO_NOT_EMPTY: u8 = 0b0001_0000;
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b0000_1100;
    /// FIFO sample count MSB mask.
    pub const SMPL_CNT_MSB_MASK: u8 = 0b0000_0011;
}

/// FIFO data register bits.
pub mod fifo_data {
    /// FIFO data mask.
    pub const DATA_MASK: u8 = 0b1111_1111;
}

/// STATUSINT register bits.
pub mod status_int {
    /// CTRL9 command done.
    pub const CMD_DONE: u8 = 0b1000_0000;
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b0111_1100;
    /// Data locked flag.
    pub const LOCKED: u8 = 0b0000_0010;
    /// Data available flag.
    pub const AVAIL: u8 = 0b0000_0001;
}

/// STATUS0 register bits.
pub mod status0 {
    /// AttitudeEngine data available.
    pub const AE_AVAIL: u8 = 0b0000_1000;
    /// Reserved bit.
    pub const RESERVED_2: u8 = 0b0000_0100;
    /// Accelerometer data available.
    pub const ACCEL_AVAIL: u8 = 0b0000_0001;
    /// Gyroscope data available.
    pub const GYRO_AVAIL: u8 = 0b0000_0010;
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b1111_0000;
}

/// STATUS1 register bits.
pub mod status1 {
    /// Significant motion detected.
    pub const SIGNIFICANT_MOTION: u8 = 0b1000_0000;
    /// No motion detected.
    pub const NO_MOTION: u8 = 0b0100_0000;
    /// Any motion detected.
    pub const ANY_MOTION: u8 = 0b0010_0000;
    /// Pedometer detected.
    pub const PEDOMETER: u8 = 0b0001_0000;
    /// Reserved bit.
    pub const RESERVED_3: u8 = 0b0000_1000;
    /// Wake on Motion detected.
    pub const WOM: u8 = 0b0000_0100;
    /// Tap detected.
    pub const TAP: u8 = 0b0000_0010;
    /// Ctrl9 command done (CTRL9 handshake when routed to STATUS1).
    pub const CMD_DONE: u8 = 0b0000_0001;
}

/// AttitudeEngine register 1 bits.
pub mod ae_reg1 {
    /// Reserved bit.
    pub const RESERVED_7: u8 = 0b1000_0000;
    /// Gyro bias updated during this period.
    pub const GYRO_BIAS_ACK: u8 = 0b0100_0000;
    /// Gyro Z clip flag.
    pub const WZ_CLIP: u8 = 0b0010_0000;
    /// Gyro Y clip flag.
    pub const WY_CLIP: u8 = 0b0001_0000;
    /// Gyro X clip flag.
    pub const WX_CLIP: u8 = 0b0000_1000;
    /// Accel Z clip flag.
    pub const AZ_CLIP: u8 = 0b0000_0100;
    /// Accel Y clip flag.
    pub const AY_CLIP: u8 = 0b0000_0010;
    /// Accel X clip flag.
    pub const AX_CLIP: u8 = 0b0000_0001;
}

/// AttitudeEngine register 2 bits.
pub mod ae_reg2 {
    /// Reserved bits.
    pub const RESERVED_MASK: u8 = 0b1111_1000;
    /// Delta velocity Z overflow.
    pub const DVZ_OVERFLOW: u8 = 0b0000_0100;
    /// Delta velocity Y overflow.
    pub const DVY_OVERFLOW: u8 = 0b0000_0010;
    /// Delta velocity X overflow.
    pub const DVX_OVERFLOW: u8 = 0b0000_0001;
}

/// RESET register values.
pub mod reset {
    /// Soft reset command value.
    pub const SOFT_RESET: u8 = 0b1011_0000;
}
