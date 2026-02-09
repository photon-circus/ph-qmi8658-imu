//! Sensor data readout helpers.

pub(crate) mod fifo;
#[cfg(feature = "fixed")]
pub(crate) mod fixed;

pub(crate) use fifo::fifo_read_len;
pub use fifo::{
    FifoConfig, FifoFrame, FifoFrameFormat, FifoFrameIterator, FifoMode, FifoReadout, FifoSize,
    FifoStatus,
};

use crate::register::Register;

pub(crate) const RAW_BLOCK_START: Register = Register::TimestampLow;
pub(crate) const RAW_BLOCK_LEN: usize = 17;
pub(crate) const RAW_BLOCK_TIMESTAMP_OFFSET: usize = 0;
pub(crate) const RAW_BLOCK_TEMPERATURE_OFFSET: usize = 3;
pub(crate) const RAW_BLOCK_ACCEL_OFFSET: usize = 5;
pub(crate) const RAW_BLOCK_GYRO_OFFSET: usize = 11;

pub(crate) const FIFO_AXIS_BYTES: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct FifoFrameLayout {
    pub accel: bool,
    pub gyro: bool,
    pub mag: bool,
    pub bytes_per_frame: usize,
}

pub(crate) const FIFO_FRAME_LAYOUTS: [FifoFrameLayout; 8] = [
    FifoFrameLayout {
        accel: false,
        gyro: false,
        mag: false,
        bytes_per_frame: 0,
    },
    FifoFrameLayout {
        accel: true,
        gyro: false,
        mag: false,
        bytes_per_frame: FIFO_AXIS_BYTES,
    },
    FifoFrameLayout {
        accel: false,
        gyro: true,
        mag: false,
        bytes_per_frame: FIFO_AXIS_BYTES,
    },
    FifoFrameLayout {
        accel: true,
        gyro: true,
        mag: false,
        bytes_per_frame: FIFO_AXIS_BYTES * 2,
    },
    FifoFrameLayout {
        accel: false,
        gyro: false,
        mag: true,
        bytes_per_frame: FIFO_AXIS_BYTES,
    },
    FifoFrameLayout {
        accel: true,
        gyro: false,
        mag: true,
        bytes_per_frame: FIFO_AXIS_BYTES * 2,
    },
    FifoFrameLayout {
        accel: false,
        gyro: true,
        mag: true,
        bytes_per_frame: FIFO_AXIS_BYTES * 2,
    },
    FifoFrameLayout {
        accel: true,
        gyro: true,
        mag: true,
        bytes_per_frame: FIFO_AXIS_BYTES * 3,
    },
];

pub(crate) const fn fifo_frame_bytes(accel: bool, gyro: bool, mag: bool) -> usize {
    let idx = (accel as usize) | ((gyro as usize) << 1) | ((mag as usize) << 2);
    FIFO_FRAME_LAYOUTS[idx].bytes_per_frame
}

/// Sample timestamp (24-bit counter).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timestamp {
    /// Raw 24-bit counter value.
    pub ticks: u32,
}

impl Timestamp {
    pub(crate) const fn from_bytes(bytes: [u8; 3]) -> Self {
        let ticks = ((bytes[2] as u32) << 16) | ((bytes[1] as u32) << 8) | (bytes[0] as u32);
        Self { ticks }
    }
}

/// Timestamped sample wrapper.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sample<T> {
    /// Sample timestamp.
    pub timestamp: Timestamp,
    /// Sample payload.
    pub data: T,
}

/// Raw block of sensor data sampled at a single timestamp.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawBlock {
    /// Sample timestamp.
    pub timestamp: Timestamp,
    /// Raw temperature reading.
    pub temperature: TemperatureRaw,
    /// Raw accelerometer reading, if enabled.
    pub accel: Option<AccelRaw>,
    /// Raw gyroscope reading, if enabled.
    pub gyro: Option<GyroRaw>,
}

/// Decodes a raw block buffer into a structured sample.
pub(crate) fn decode_raw_block(
    buffer: &[u8; RAW_BLOCK_LEN],
    big_endian: bool,
    accel_enabled: bool,
    gyro_enabled: bool,
) -> RawBlock {
    let timestamp = Timestamp::from_bytes([
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET],
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET + 1],
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET + 2],
    ]);
    let temperature = TemperatureRaw::from_bytes(
        [
            buffer[RAW_BLOCK_TEMPERATURE_OFFSET],
            buffer[RAW_BLOCK_TEMPERATURE_OFFSET + 1],
        ],
        big_endian,
    );

    let accel = if accel_enabled {
        Some(AccelRaw::from_bytes(
            [
                buffer[RAW_BLOCK_ACCEL_OFFSET],
                buffer[RAW_BLOCK_ACCEL_OFFSET + 1],
                buffer[RAW_BLOCK_ACCEL_OFFSET + 2],
                buffer[RAW_BLOCK_ACCEL_OFFSET + 3],
                buffer[RAW_BLOCK_ACCEL_OFFSET + 4],
                buffer[RAW_BLOCK_ACCEL_OFFSET + 5],
            ],
            big_endian,
        ))
    } else {
        None
    };

    let gyro = if gyro_enabled {
        Some(GyroRaw::from_bytes(
            [
                buffer[RAW_BLOCK_GYRO_OFFSET],
                buffer[RAW_BLOCK_GYRO_OFFSET + 1],
                buffer[RAW_BLOCK_GYRO_OFFSET + 2],
                buffer[RAW_BLOCK_GYRO_OFFSET + 3],
                buffer[RAW_BLOCK_GYRO_OFFSET + 4],
                buffer[RAW_BLOCK_GYRO_OFFSET + 5],
            ],
            big_endian,
        ))
    } else {
        None
    };

    RawBlock {
        timestamp,
        temperature,
        accel,
        gyro,
    }
}

/// Raw accelerometer sample with timestamp, flattened for stable layout.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub struct AccelSampleRaw {
    /// Raw timestamp counter.
    pub timestamp_ticks: u32,
    /// Accelerometer X-axis.
    pub accel_x: i16,
    /// Accelerometer Y-axis.
    pub accel_y: i16,
    /// Accelerometer Z-axis.
    pub accel_z: i16,
}

/// Raw accelerometer + gyroscope sample with timestamp, flattened for stable layout.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub struct AccelGyroSampleRaw {
    /// Raw timestamp counter.
    pub timestamp_ticks: u32,
    /// Accelerometer X-axis.
    pub accel_x: i16,
    /// Accelerometer Y-axis.
    pub accel_y: i16,
    /// Accelerometer Z-axis.
    pub accel_z: i16,
    /// Gyroscope X-axis.
    pub gyro_x: i16,
    /// Gyroscope Y-axis.
    pub gyro_y: i16,
    /// Gyroscope Z-axis.
    pub gyro_z: i16,
}

/// Raw accelerometer + gyroscope + magnetometer sample with timestamp, flattened for stable layout.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub struct AccelGyroMagSampleRaw {
    /// Raw timestamp counter.
    pub timestamp_ticks: u32,
    /// Accelerometer X-axis.
    pub accel_x: i16,
    /// Accelerometer Y-axis.
    pub accel_y: i16,
    /// Accelerometer Z-axis.
    pub accel_z: i16,
    /// Gyroscope X-axis.
    pub gyro_x: i16,
    /// Gyroscope Y-axis.
    pub gyro_y: i16,
    /// Gyroscope Z-axis.
    pub gyro_z: i16,
    /// Magnetometer X-axis.
    pub mag_x: i16,
    /// Magnetometer Y-axis.
    pub mag_y: i16,
    /// Magnetometer Z-axis.
    pub mag_z: i16,
}

/// Raw accelerometer sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelRaw {
    /// X-axis raw count.
    pub x: i16,
    /// Y-axis raw count.
    pub y: i16,
    /// Z-axis raw count.
    pub z: i16,
}

impl AccelRaw {
    pub(crate) fn from_bytes(bytes: [u8; 6], big_endian: bool) -> Self {
        let (x, y, z) = if big_endian {
            (
                i16::from_be_bytes([bytes[0], bytes[1]]),
                i16::from_be_bytes([bytes[2], bytes[3]]),
                i16::from_be_bytes([bytes[4], bytes[5]]),
            )
        } else {
            (
                i16::from_le_bytes([bytes[0], bytes[1]]),
                i16::from_le_bytes([bytes[2], bytes[3]]),
                i16::from_le_bytes([bytes[4], bytes[5]]),
            )
        };
        Self { x, y, z }
    }

    #[allow(dead_code)]
    pub(crate) fn from_le_bytes(bytes: [u8; 6]) -> Self {
        Self::from_bytes(bytes, false)
    }
}

/// Raw gyroscope sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroRaw {
    /// X-axis raw count.
    pub x: i16,
    /// Y-axis raw count.
    pub y: i16,
    /// Z-axis raw count.
    pub z: i16,
}

impl GyroRaw {
    pub(crate) fn from_bytes(bytes: [u8; 6], big_endian: bool) -> Self {
        let (x, y, z) = if big_endian {
            (
                i16::from_be_bytes([bytes[0], bytes[1]]),
                i16::from_be_bytes([bytes[2], bytes[3]]),
                i16::from_be_bytes([bytes[4], bytes[5]]),
            )
        } else {
            (
                i16::from_le_bytes([bytes[0], bytes[1]]),
                i16::from_le_bytes([bytes[2], bytes[3]]),
                i16::from_le_bytes([bytes[4], bytes[5]]),
            )
        };
        Self { x, y, z }
    }

    #[allow(dead_code)]
    pub(crate) fn from_le_bytes(bytes: [u8; 6]) -> Self {
        Self::from_bytes(bytes, false)
    }
}

/// Raw temperature sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TemperatureRaw {
    /// Raw temperature count.
    pub value: i16,
}

impl TemperatureRaw {
    pub(crate) fn from_bytes(bytes: [u8; 2], big_endian: bool) -> Self {
        let value = if big_endian {
            i16::from_be_bytes(bytes)
        } else {
            i16::from_le_bytes(bytes)
        };
        Self { value }
    }

    #[allow(dead_code)]
    pub(crate) fn from_le_bytes(bytes: [u8; 2]) -> Self {
        Self::from_bytes(bytes, false)
    }
}

/// Raw magnetometer sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub struct MagRaw {
    /// X-axis raw count.
    pub x: i16,
    /// Y-axis raw count.
    pub y: i16,
    /// Z-axis raw count.
    pub z: i16,
}

#[allow(dead_code)]
impl MagRaw {
    pub(crate) fn from_bytes(bytes: [u8; 6], big_endian: bool) -> Self {
        let (x, y, z) = if big_endian {
            (
                i16::from_be_bytes([bytes[0], bytes[1]]),
                i16::from_be_bytes([bytes[2], bytes[3]]),
                i16::from_be_bytes([bytes[4], bytes[5]]),
            )
        } else {
            (
                i16::from_le_bytes([bytes[0], bytes[1]]),
                i16::from_le_bytes([bytes[2], bytes[3]]),
                i16::from_le_bytes([bytes[4], bytes[5]]),
            )
        };
        Self { x, y, z }
    }

    pub(crate) fn from_le_bytes(bytes: [u8; 6]) -> Self {
        Self::from_bytes(bytes, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_raw_block_little_endian() {
        let mut buffer = [0u8; RAW_BLOCK_LEN];
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET] = 0x01;
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET + 1] = 0x02;
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET + 2] = 0x03;
        buffer[RAW_BLOCK_TEMPERATURE_OFFSET] = 0x10;
        buffer[RAW_BLOCK_TEMPERATURE_OFFSET + 1] = 0x00;

        buffer[RAW_BLOCK_ACCEL_OFFSET] = 0x01;
        buffer[RAW_BLOCK_ACCEL_OFFSET + 2] = 0x02;
        buffer[RAW_BLOCK_ACCEL_OFFSET + 4] = 0x03;

        buffer[RAW_BLOCK_GYRO_OFFSET] = 0x04;
        buffer[RAW_BLOCK_GYRO_OFFSET + 2] = 0x05;
        buffer[RAW_BLOCK_GYRO_OFFSET + 4] = 0x06;

        let block = decode_raw_block(&buffer, false, true, true);
        assert_eq!(block.timestamp.ticks, 0x030201);
        assert_eq!(block.temperature.value, 0x0010);

        let accel = block.accel.expect("accel");
        let gyro = block.gyro.expect("gyro");
        assert_eq!(accel.x, 1);
        assert_eq!(accel.y, 2);
        assert_eq!(accel.z, 3);
        assert_eq!(gyro.x, 4);
        assert_eq!(gyro.y, 5);
        assert_eq!(gyro.z, 6);
    }

    #[test]
    fn decode_raw_block_big_endian_no_sensors() {
        let mut buffer = [0u8; RAW_BLOCK_LEN];
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET] = 0xAA;
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET + 1] = 0xBB;
        buffer[RAW_BLOCK_TIMESTAMP_OFFSET + 2] = 0xCC;
        buffer[RAW_BLOCK_TEMPERATURE_OFFSET] = 0x00;
        buffer[RAW_BLOCK_TEMPERATURE_OFFSET + 1] = 0x10;

        let block = decode_raw_block(&buffer, true, false, false);
        assert_eq!(block.timestamp.ticks, 0xCCBBAA);
        assert_eq!(block.temperature.value, 0x0010);
        assert!(block.accel.is_none());
        assert!(block.gyro.is_none());
    }
}
