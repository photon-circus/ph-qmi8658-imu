//! FIFO configuration and status helpers.

use super::{AccelRaw, GyroRaw, MagRaw, Timestamp};
use crate::register::{fifo_ctrl, fifo_status};

/// FIFO operating mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// FIFO disabled (bypass).
    Bypass,
    /// FIFO mode (stop collecting when full).
    Fifo,
    /// Streaming mode (overwrite oldest when full).
    Stream,
    /// Stream-to-FIFO mode (reserved on some revisions).
    StreamToFifo,
}

impl FifoMode {
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Bypass => 0b00,
            Self::Fifo => 0b01,
            Self::Stream => 0b10,
            Self::StreamToFifo => 0b11,
        }
    }
}

/// FIFO size selection (samples per enabled sensor).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoSize {
    /// 16 samples per enabled sensor.
    Samples16,
    /// 32 samples per enabled sensor.
    Samples32,
    /// 64 samples per enabled sensor.
    Samples64,
    /// 128 samples per enabled sensor.
    Samples128,
}

impl FifoSize {
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Samples16 => 0b00,
            Self::Samples32 => 0b01,
            Self::Samples64 => 0b10,
            Self::Samples128 => 0b11,
        }
    }

    /// Returns the configured samples per enabled sensor.
    pub const fn samples(self) -> u16 {
        match self {
            Self::Samples16 => 16,
            Self::Samples32 => 32,
            Self::Samples64 => 64,
            Self::Samples128 => 128,
        }
    }
}

/// FIFO configuration (watermark + mode + size).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoConfig {
    /// FIFO watermark level (in ODRs).
    pub watermark: u8,
    /// FIFO mode.
    pub mode: FifoMode,
    /// FIFO size setting.
    pub size: FifoSize,
}

impl FifoConfig {
    /// Default FIFO configuration (bypass).
    pub const DEFAULT: Self = Self {
        watermark: 0,
        mode: FifoMode::Bypass,
        size: FifoSize::Samples16,
    };

    /// Creates a new FIFO configuration.
    pub const fn new(mode: FifoMode, size: FifoSize, watermark: u8) -> Self {
        Self {
            watermark,
            mode,
            size,
        }
    }

    /// Sets the FIFO mode.
    #[must_use]
    pub const fn with_mode(self, mode: FifoMode) -> Self {
        Self { mode, ..self }
    }

    /// Sets the FIFO size.
    #[must_use]
    pub const fn with_size(self, size: FifoSize) -> Self {
        Self { size, ..self }
    }

    /// Sets the FIFO watermark level (in ODRs).
    #[must_use]
    pub const fn with_watermark(self, watermark: u8) -> Self {
        Self { watermark, ..self }
    }

    pub(crate) const fn ctrl_value(self) -> u8 {
        ((self.size.bits() << fifo_ctrl::FIFO_SIZE_SHIFT) & fifo_ctrl::FIFO_SIZE_MASK)
            | ((self.mode.bits() << fifo_ctrl::FIFO_MODE_SHIFT) & fifo_ctrl::FIFO_MODE_MASK)
    }
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// FIFO status decoded from FIFO_STATUS and FIFO_SMPL_CNT.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoStatus {
    /// FIFO is full.
    pub full: bool,
    /// FIFO watermark reached.
    pub watermark: bool,
    /// FIFO overflow has occurred.
    pub overflow: bool,
    /// FIFO is not empty.
    pub not_empty: bool,
    /// FIFO sample count in bytes.
    pub sample_count_bytes: u16,
}

/// Result of a FIFO readout.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoReadout {
    /// Status snapshot captured before reading.
    pub status: FifoStatus,
    /// Number of bytes read into the caller buffer.
    pub bytes_read: usize,
    /// Timestamp captured after FIFO readout.
    pub timestamp: Timestamp,
}

/// FIFO frame format derived from enabled sensors.
///
/// Datasheet order:
/// - One sensor: that sensor's X/Y/Z sample order.
/// - Two sensors: accel samples then gyro samples, repeated.
/// - Three sensors: accel, gyro, then magnetometer samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoFrameFormat {
    /// Whether accelerometer data is present in each frame.
    pub accel: bool,
    /// Whether gyroscope data is present in each frame.
    pub gyro: bool,
    /// Whether magnetometer data is present in each frame.
    pub mag: bool,
    /// Interpret 16-bit samples as big-endian (CTRL1.BE = 1).
    pub big_endian: bool,
}

impl FifoFrameFormat {
    /// Creates a new FIFO frame format.
    pub const fn new(accel: bool, gyro: bool, mag: bool) -> Self {
        Self {
            accel,
            gyro,
            mag,
            big_endian: false,
        }
    }

    /// Sets the endianness for frame decoding.
    #[must_use]
    pub const fn with_big_endian(mut self, big_endian: bool) -> Self {
        self.big_endian = big_endian;
        self
    }

    /// Returns the bytes per FIFO frame.
    pub const fn bytes_per_frame(self) -> usize {
        super::fifo_frame_bytes(self.accel, self.gyro, self.mag)
    }

    /// Returns the maximum number of complete frames in the provided byte count.
    pub const fn frame_count(self, bytes: usize) -> usize {
        let frame = self.bytes_per_frame();
        if frame == 0 { 0 } else { bytes / frame }
    }
}

pub(crate) fn fifo_read_len(
    status: FifoStatus,
    format: FifoFrameFormat,
    buffer_len: usize,
) -> usize {
    if buffer_len == 0 {
        return 0;
    }
    let mut available = status.sample_count_bytes as usize;
    if available == 0 {
        return 0;
    }
    let frame = format.bytes_per_frame();
    if frame == 0 {
        return 0;
    }
    if available > buffer_len {
        available = buffer_len;
    }
    available - (available % frame)
}

/// One decoded FIFO frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub struct FifoFrame {
    /// Accelerometer data, if present in this frame.
    pub accel: Option<AccelRaw>,
    /// Gyroscope data, if present in this frame.
    pub gyro: Option<GyroRaw>,
    /// Magnetometer data, if present in this frame.
    pub mag: Option<MagRaw>,
}

/// Iterator over FIFO frames.
#[allow(dead_code)]
pub struct FifoFrameIterator<'a> {
    data: &'a [u8],
    format: FifoFrameFormat,
    offset: usize,
}

#[allow(dead_code)]
impl<'a> FifoFrameIterator<'a> {
    /// Creates an iterator over FIFO frames.
    pub const fn new(data: &'a [u8], format: FifoFrameFormat) -> Self {
        Self {
            data,
            format,
            offset: 0,
        }
    }

    fn take6(&self, offset: usize) -> [u8; 6] {
        [
            self.data[offset],
            self.data[offset + 1],
            self.data[offset + 2],
            self.data[offset + 3],
            self.data[offset + 4],
            self.data[offset + 5],
        ]
    }

    /// Returns the remaining unparsed bytes.
    pub const fn remaining(&self) -> usize {
        if self.offset >= self.data.len() {
            0
        } else {
            self.data.len() - self.offset
        }
    }
}

impl<'a> Iterator for FifoFrameIterator<'a> {
    type Item = FifoFrame;

    fn next(&mut self) -> Option<Self::Item> {
        let frame_len = self.format.bytes_per_frame();
        if frame_len == 0 || self.offset + frame_len > self.data.len() {
            return None;
        }

        let mut offset = self.offset;
        let accel = if self.format.accel {
            let bytes = self.take6(offset);
            offset += 6;
            Some(AccelRaw::from_bytes(bytes, self.format.big_endian))
        } else {
            None
        };

        let gyro = if self.format.gyro {
            let bytes = self.take6(offset);
            offset += 6;
            Some(GyroRaw::from_bytes(bytes, self.format.big_endian))
        } else {
            None
        };

        let mag = if self.format.mag {
            let bytes = self.take6(offset);
            offset += 6;
            Some(MagRaw::from_bytes(bytes, self.format.big_endian))
        } else {
            None
        };

        self.offset = offset;
        Some(FifoFrame { accel, gyro, mag })
    }
}

#[allow(dead_code)]
impl FifoFrame {
    /// Returns the expected FIFO format for accel-only data.
    pub const fn accel_only() -> FifoFrameFormat {
        FifoFrameFormat::new(true, false, false)
    }

    /// Returns the expected FIFO format for gyro-only data.
    pub const fn gyro_only() -> FifoFrameFormat {
        FifoFrameFormat::new(false, true, false)
    }

    /// Returns the expected FIFO format for accel + gyro data.
    pub const fn accel_gyro() -> FifoFrameFormat {
        FifoFrameFormat::new(true, true, false)
    }
}

impl FifoStatus {
    pub(crate) const fn from_regs(status: u8, smpl_cnt_lsb: u8) -> Self {
        let msb = (status & fifo_status::SMPL_CNT_MSB_MASK) as u16;
        let lsb = smpl_cnt_lsb as u16;
        Self {
            full: (status & fifo_status::FIFO_FULL) != 0,
            watermark: (status & fifo_status::FIFO_WTM) != 0,
            overflow: (status & fifo_status::FIFO_OVFLOW) != 0,
            not_empty: (status & fifo_status::FIFO_NOT_EMPTY) != 0,
            sample_count_bytes: (msb << 8) | lsb,
        }
    }

    /// Returns true if any FIFO status flag is set.
    pub const fn any(self) -> bool {
        self.full || self.watermark || self.overflow || self.not_empty
    }

    /// Converts the byte count to samples given a frame size.
    pub const fn samples(self, bytes_per_sample: u16) -> u16 {
        if bytes_per_sample == 0 {
            0
        } else {
            self.sample_count_bytes / bytes_per_sample
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_format_sizes() {
        let none = FifoFrameFormat::new(false, false, false);
        assert_eq!(none.bytes_per_frame(), 0);

        let accel = FifoFrameFormat::new(true, false, false);
        assert_eq!(accel.bytes_per_frame(), 6);
        assert_eq!(accel.frame_count(12), 2);
        assert_eq!(accel.frame_count(11), 1);

        let accel_gyro = FifoFrameFormat::new(true, true, false);
        assert_eq!(accel_gyro.bytes_per_frame(), 12);

        let all = FifoFrameFormat::new(true, true, true);
        assert_eq!(all.bytes_per_frame(), 18);
    }

    #[test]
    fn parse_accel_gyro_little_endian() {
        let format = FifoFrameFormat::new(true, true, false).with_big_endian(false);
        let buffer = [
            0x01, 0x00, 0x02, 0x00, 0x03, 0x00, // accel: 1,2,3
            0x04, 0x00, 0x05, 0x00, 0x06, 0x00, // gyro: 4,5,6
        ];

        let mut iter = FifoFrameIterator::new(&buffer, format);
        let frame = iter.next().expect("frame");
        let accel = frame.accel.expect("accel");
        let gyro = frame.gyro.expect("gyro");

        assert_eq!(accel.x, 1);
        assert_eq!(accel.y, 2);
        assert_eq!(accel.z, 3);
        assert_eq!(gyro.x, 4);
        assert_eq!(gyro.y, 5);
        assert_eq!(gyro.z, 6);
        assert!(iter.next().is_none());
    }

    #[test]
    fn parse_accel_big_endian() {
        let format = FifoFrameFormat::new(true, false, false).with_big_endian(true);
        let buffer = [0x00, 0x01, 0x00, 0x02, 0x00, 0x03];

        let mut iter = FifoFrameIterator::new(&buffer, format);
        let frame = iter.next().expect("frame");
        let accel = frame.accel.expect("accel");

        assert_eq!(accel.x, 1);
        assert_eq!(accel.y, 2);
        assert_eq!(accel.z, 3);
    }

    #[test]
    fn fifo_read_len_aligns_to_frames() {
        let status = FifoStatus {
            sample_count_bytes: 25,
            ..FifoStatus::default()
        };
        let format = FifoFrameFormat::new(true, true, false); // 12 bytes/frame
        assert_eq!(fifo_read_len(status, format, 64), 24);
        assert_eq!(fifo_read_len(status, format, 12), 12);
        assert_eq!(fifo_read_len(status, format, 8), 0);
    }
}
