//! SET WINDOW(0x24), from SCSI-2 15.2.5

use crate::scsi::{Cdb, Command, CommandData, Error};

/// SET WINDOW - configures scan geometry, resolution, bit depth and mode.
///
/// Required before every SCAN. The data-out payload is an 8-byte window
/// parameter header followed by one or more window descriptors; this type is a
/// thin CDB wrapper over that already-built payload. Constructing a valid
/// descriptor is the caller's job — its field layout is Nikon-specific and only
/// partly verified, so it lives with the scanner, not here.
///
/// Control byte is `0x80`, the Nikon vendor flag (not the SCSI-standard `0x00`).
pub struct SetWindow {
    /// Window parameter header + descriptor(s), sent verbatim.
    payload: Vec<u8>,
}

impl SetWindow {
    pub fn new(payload: Vec<u8>) -> Self {
        Self { payload }
    }
}

impl Command for SetWindow {
    type Response = ();
    type Cdb = Cdb<10>;

    fn cdb(&self) -> Self::Cdb {
        let len = self.payload.len() as u32;
        Cdb([
            0x24, // opcode
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            ((len & 0xFF0000) >> 16) as u8,
            ((len & 0x00FF00) >> 8) as u8,
            (len & 0x0000FF) as u8,
            0x80, // Nikon vendor control flag
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
    fn cdb_encodes_transfer_length_as_big_endian_u24() {
        let cdb = SetWindow::new(vec![0u8; 0x01_2345]).cdb().0;
        assert_eq!(cdb[6], 0x01);
        assert_eq!(cdb[7], 0x23);
        assert_eq!(cdb[8], 0x45);
    }

    #[test]
    fn cdb_matches_real_preview_capture() {
        // A preview SET WINDOW carries a 58-byte payload (0x3A); control 0x80.
        let cdb = SetWindow::new(vec![0u8; 58]).cdb().0;
        assert_eq!(
            cdb,
            [0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x80]
        );
    }

    #[test]
    fn data_is_write_of_the_payload() {
        let payload = vec![0xAA, 0xBB, 0xCC];
        let sw = SetWindow::new(payload.clone());
        assert!(matches!(sw.data(), CommandData::Write(p) if p == payload));
    }
}
