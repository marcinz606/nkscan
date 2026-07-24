//! WRITE(10) for SCSI-2 scanner devices, the data-out counterpart of READ(10).

use super::DataTypeCode;
use crate::scsi::{Cdb, Command, CommandData, Error};

/// SCSI-2 scanner WRITE(10)
///
/// Mirrors [`Read`](super::Read) but sends data host->scanner: the same DTC/DTQ
/// select what is written (gamma LUT, calibration, ...), and the transfer length
/// is the payload size. Control byte is caller-supplied; unlike READ (which uses
/// the `0x80` vendor flag) WRITE uses `0x00` on this hardware.
pub struct Write {
    /// Logical unit number (3 bits)
    lun: u8,
    /// Data-type code
    dtc: DataTypeCode,
    /// Data-type qualifier
    dtq: u16,
    /// Data-out payload; its length is the CDB transfer length.
    payload: Vec<u8>,
    /// Control byte.
    control: u8,
}

impl Write {
    pub fn new(lun: u8, dtc: DataTypeCode, dtq: u16, payload: Vec<u8>, control: u8) -> Self {
        Write {
            lun,
            dtc,
            dtq,
            payload,
            control,
        }
    }
}

impl Command for Write {
    type Response = ();
    type Cdb = Cdb<10>;

    fn cdb(&self) -> Self::Cdb {
        let len = self.payload.len() as u32;
        Cdb([
            0x2A, // opcode
            self.lun << 5,
            self.dtc.to_byte(),
            0x00, // reserved
            ((self.dtq & 0xFF00) >> 8) as u8,
            (self.dtq & 0x00FF) as u8,
            ((len & 0xFF0000) >> 16) as u8,
            ((len & 0x00FF00) >> 8) as u8,
            (len & 0x0000FF) as u8,
            self.control,
        ])
    }

    fn data(&self) -> CommandData<'_> {
        CommandData::Write(&self.payload)
    }

    fn decode(&self, _data: &[u8]) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cdb_encodes_opcode_lun_and_dtc() {
        let write = Write::new(2, DataTypeCode::HalftoneMask, 0, vec![], 0x00);
        let cdb = write.cdb().0;
        assert_eq!(cdb[0], 0x2A);
        assert_eq!(cdb[1], 2 << 5);
        assert_eq!(cdb[2], 0x02);
        assert_eq!(cdb[3], 0x00);
    }

    #[test]
    fn cdb_encodes_dtq_big_endian() {
        let cdb = Write::new(0, DataTypeCode::GammaFunction, 0x1234, vec![], 0x00)
            .cdb()
            .0;
        assert_eq!(cdb[4], 0x12);
        assert_eq!(cdb[5], 0x34);
    }

    #[test]
    fn cdb_transfer_length_is_payload_len_big_endian_u24() {
        let cdb = Write::new(
            0,
            DataTypeCode::GammaFunction,
            0,
            vec![0u8; 0x01_2345],
            0x00,
        )
        .cdb()
        .0;
        assert_eq!(cdb[6], 0x01);
        assert_eq!(cdb[7], 0x23);
        assert_eq!(cdb[8], 0x45);
    }

    #[test]
    fn cdb_encodes_control_byte_verbatim() {
        let cdb = Write::new(0, DataTypeCode::GammaFunction, 0, vec![], 0x80)
            .cdb()
            .0;
        assert_eq!(cdb[9], 0x80);
    }

    #[test]
    fn cdb_matches_gamma_lut_capture() {
        // RE 002-preview exchange #15: 32 KB gamma LUT for channel R, DTQ 0x0101
        // (CDB[4]=channel 01, CDB[5]=0x01 constant), control 0x00.
        let cdb = Write::new(
            0,
            DataTypeCode::GammaFunction,
            0x0101,
            vec![0u8; 32768],
            0x00,
        )
        .cdb()
        .0;
        assert_eq!(
            cdb,
            [0x2A, 0x00, 0x03, 0x00, 0x01, 0x01, 0x00, 0x80, 0x00, 0x00]
        );
    }

    #[test]
    fn data_is_write_of_the_payload() {
        let payload = vec![0xAA, 0xBB, 0xCC];
        let w = Write::new(0, DataTypeCode::GammaFunction, 0, payload.clone(), 0x00);
        assert!(matches!(w.data(), CommandData::Write(p) if p == payload));
    }
}
