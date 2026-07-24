//! GET WINDOW(10) and SET WINDOW(10), SCSI-2 scanner devices, 15.2.2 / 15.2.9.

use crate::scsi::{Cdb, Command, CommandData, Error};
use tracing::*;

#[derive(Debug, Copy, Clone)]
pub struct GetWindow {
    /// Logical unit number (3 bits)
    lun: u8,
    /// "Single": specifies that a single window descriptor shall be returned for the specified window identifier
    single: bool,
    /// Window identifier
    window_identifier: u8,
    /// Transfer length (24-bits)
    transfer_length: u32,
    /// Control,
    control: u8,
}

impl GetWindow {
    /// `transfer_length` is how many bytes we're willing to receive - the
    /// caller has to size it (header + however many descriptors of however
    /// many bytes are expected back), since neither the descriptor count nor
    /// the vendor-specific tail length is something this generic command can
    /// know ahead of time; that's device-specific.
    pub fn new(
        lun: u8,
        single: bool,
        window_identifier: u8,
        transfer_length: u32,
        control: u8,
    ) -> Self {
        Self {
            lun,
            single,
            window_identifier,
            transfer_length,
            control,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ImageCompositionCode {
    BiLevelBlackAndWhite,
    DitheredHalftoneBlackAndWhite,
    Greyscale,
    BiLevelRgb,
    DitheredHalftoneRgb,
    Rgb,
    Reserved(u8),
}

impl ImageCompositionCode {
    fn to_byte(self) -> u8 {
        match self {
            ImageCompositionCode::BiLevelBlackAndWhite => 0x00,
            ImageCompositionCode::DitheredHalftoneBlackAndWhite => 0x01,
            ImageCompositionCode::Greyscale => 0x02,
            ImageCompositionCode::BiLevelRgb => 0x03,
            ImageCompositionCode::DitheredHalftoneRgb => 0x04,
            ImageCompositionCode::Rgb => 0x05,
            ImageCompositionCode::Reserved(x) => x,
        }
    }

    fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => ImageCompositionCode::BiLevelBlackAndWhite,
            0x01 => ImageCompositionCode::DitheredHalftoneBlackAndWhite,
            0x02 => ImageCompositionCode::Greyscale,
            0x03 => ImageCompositionCode::BiLevelRgb,
            0x04 => ImageCompositionCode::DitheredHalftoneRgb,
            0x05 => ImageCompositionCode::Rgb,
            other => ImageCompositionCode::Reserved(other),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PaddingType {
    NoPadding,
    PadWithZeros,
    PadWithOnes,
    Truncate,
    Reserved(u8),
}

impl PaddingType {
    fn to_byte(self) -> u8 {
        match self {
            PaddingType::NoPadding => 0x00,
            PaddingType::PadWithZeros => 0x01,
            PaddingType::PadWithOnes => 0x02,
            PaddingType::Truncate => 0x03,
            PaddingType::Reserved(x) => x,
        }
    }

    /// Padding type only occupies the low 3 bits of its byte (the rest is
    /// shared with the RIF bit and reserved bits), so only 0x00-0x07 are
    /// actually representable - anything above 0x03 is reserved.
    fn from_byte(byte: u8) -> Self {
        match byte & 0x07 {
            0x00 => PaddingType::NoPadding,
            0x01 => PaddingType::PadWithZeros,
            0x02 => PaddingType::PadWithOnes,
            0x03 => PaddingType::Truncate,
            other => PaddingType::Reserved(other),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CompressionType {
    NoCompression,
    CcittGroupIii1dimensional,
    CcittGroupIii2dimensional,
    CcittGroupIv2dimensional,
    Reserved(u8),
    Ocr,
    Vendor(u8),
}

impl CompressionType {
    fn to_byte(self) -> u8 {
        match self {
            CompressionType::NoCompression => 0x00,
            CompressionType::CcittGroupIii1dimensional => 0x01,
            CompressionType::CcittGroupIii2dimensional => 0x02,
            CompressionType::CcittGroupIv2dimensional => 0x03,
            CompressionType::Reserved(x) => x,
            CompressionType::Ocr => 0x10,
            CompressionType::Vendor(x) => x,
        }
    }

    /// 04h-0Fh and 11h-7Fh are reserved; 80h-FFh is the vendor-specific range
    fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => CompressionType::NoCompression,
            0x01 => CompressionType::CcittGroupIii1dimensional,
            0x02 => CompressionType::CcittGroupIii2dimensional,
            0x03 => CompressionType::CcittGroupIv2dimensional,
            0x10 => CompressionType::Ocr,
            0x80..=0xFF => CompressionType::Vendor(byte),
            other => CompressionType::Reserved(other),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// GET WINDOW 4-byte data header, precedes one or more window descriptors
pub struct GetWindowHeader {
    /// Length in bytes of all the data that follows this field, not including itself
    pub data_length: u16,
    /// Length in bytes of each window descriptor
    pub descriptor_length: u16,
}

#[derive(Debug, Clone)]
/// The 40-byte standard descriptor
pub struct WindowDescriptor {
    /// Window identifier
    id: u8,
    auto: bool,
    x_resolution: u16,
    y_resolution: u16,
    x_upper_left: u32,
    y_upper_left: u32,
    width: u32,
    length: u32,
    brightness: u8,
    threshold: u8,
    contrast: u8,
    composition: ImageCompositionCode,
    bits_per_pixel: u8,
    halftone_pattern: u16,
    rif: bool,
    padding: PaddingType,
    bit_ordering: u16,
    compression: CompressionType,
    compression_arg: u8,
    /// Vendor-specific descriptor tail (bytes 40..). Nikon stores the
    /// firmware-measured per-channel exposure here.
    pub vendor: Vec<u8>,
}

impl WindowDescriptor {
    fn from_bytes(bytes: &[u8]) -> Self {
        debug!("{}", bytes.len());
        Self {
            id: bytes[0],
            auto: bytes[1] & 1 == 1,
            x_resolution: u16::from_be_bytes([bytes[2], bytes[3]]),
            y_resolution: u16::from_be_bytes([bytes[4], bytes[5]]),
            x_upper_left: u32::from_be_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]),
            y_upper_left: u32::from_be_bytes([bytes[10], bytes[11], bytes[12], bytes[13]]),
            width: u32::from_be_bytes([bytes[14], bytes[15], bytes[16], bytes[17]]),
            length: u32::from_be_bytes([bytes[18], bytes[19], bytes[20], bytes[21]]),
            brightness: bytes[22],
            threshold: bytes[23],
            contrast: bytes[24],
            composition: ImageCompositionCode::from_byte(bytes[25]),
            bits_per_pixel: bytes[26],
            halftone_pattern: u16::from_be_bytes([bytes[27], bytes[28]]),
            rif: (bytes[29] & 0b10000000 >> 7) == 1,
            padding: PaddingType::from_byte(bytes[29] & 0b111),
            bit_ordering: u16::from_be_bytes([bytes[30], bytes[31]]),
            compression: CompressionType::from_byte(bytes[32]),
            compression_arg: bytes[33],
            vendor: bytes[40..].to_vec(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct GetWindowResponse {
    pub header: GetWindowHeader,
    pub descriptors: Vec<WindowDescriptor>,
}

impl Command for GetWindow {
    type Response = Vec<WindowDescriptor>;
    type Cdb = Cdb<10>;

    fn cdb(&self) -> Self::Cdb {
        Cdb([
            0x25, // opcode
            ((self.lun & 0b111) << 5) | (self.single as u8),
            0x00, // reserved
            0x00, // reserved
            0x00, // reserved
            self.window_identifier,
            ((self.transfer_length & 0xFF0000) >> 16) as u8,
            ((self.transfer_length & 0x00FF00) >> 8) as u8,
            (self.transfer_length & 0x0000FF) as u8,
            self.control,
        ])
    }

    fn data(&self) -> CommandData<'_> {
        CommandData::Read(self.transfer_length as usize)
    }

    fn decode(&self, data: &[u8]) -> Result<Self::Response, Error> {
        if data.len() < 8 {
            return Err(Error::InvalidResponse(
                "GET WINDOW response shorter than the 8-byte header",
            ));
        }

        let header = GetWindowHeader {
            data_length: u16::from_be_bytes([data[0], data[1]]),
            descriptor_length: u16::from_be_bytes([data[6], data[7]]),
        };

        debug!("{:#?}", header);

        let descriptor_len = header.descriptor_length as usize;
        if descriptor_len < 40 {
            return Err(Error::InvalidResponse(
                "window descriptor shorter than the standardized 40 bytes",
            ));
        }

        // `header.data_length` is the device's advertised total across every
        // window it has defined, not the size of `data` - per spec it's
        // "not adjusted to reflect truncation," so it can't be used to size
        // anything here. Only decode as many whole descriptors as actually
        // fit in what we received; a short trailing remainder (data.len()
        // not an exact multiple of descriptor_len) is silently dropped
        // rather than indexed into.
        let descriptors: Vec<_> = data[8..]
            .chunks_exact(descriptor_len)
            .map(WindowDescriptor::from_bytes)
            .collect();

        Ok(descriptors)
    }
}
