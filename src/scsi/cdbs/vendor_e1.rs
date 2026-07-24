//! VENDOR 0xE1 — Nikon sensor/status read (RE `scsi-commands/vendor-e1.md`).

use crate::scsi::{Cdb, Command, CommandData, Error};

/// VENDOR E1 — reads sensor/status registers (motor position, exposure, focus,
/// lamp, CCD setup). Data-in counterpart to [`VendorE0`](super::VendorE0) and
/// the result side of the `E0 -> C1 -> E1` cycle. Read-only: it moves nothing.
/// The firmware rejects a requested length above 13 bytes (sense 0x50).
pub struct VendorE1 {
    sub: u8,
    allocation_length: u32,
}

impl VendorE1 {
    pub fn new(sub: u8, allocation_length: u32) -> Self {
        Self {
            sub,
            allocation_length,
        }
    }
}

impl Command for VendorE1 {
    type Response = Vec<u8>;
    type Cdb = Cdb<10>;

    fn cdb(&self) -> Self::Cdb {
        let len = self.allocation_length;
        Cdb([
            0xE1, // opcode
            0x00,
            self.sub, // sub-command / register id
            0x00,
            0x00,
            0x00,
            ((len & 0xFF0000) >> 16) as u8,
            ((len & 0x00FF00) >> 8) as u8,
            (len & 0x0000FF) as u8,
            0x00, // control
        ])
    }

    fn data(&self) -> CommandData<'_> {
        CommandData::Read(self.allocation_length as usize)
    }

    fn decode(&self, data: &[u8]) -> Result<Self::Response, Error> {
        Ok(data.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cdb_sets_sub_command_and_length() {
        // Exposure readback: sub 0x45, 13-byte allocation (the firmware max).
        let cdb = VendorE1::new(0x45, 13).cdb().0;
        assert_eq!(
            cdb,
            [0xE1, 0x00, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x00]
        );
    }

    #[test]
    fn data_is_read_of_allocation_length() {
        assert!(matches!(
            VendorE1::new(0xA0, 9).data(),
            CommandData::Read(9)
        ));
    }
}
