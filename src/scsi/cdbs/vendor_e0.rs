//! VENDOR 0xE0 — Nikon control write (RE `scsi-commands/vendor-e0.md`).

use crate::scsi::{Cdb, Command, CommandData, Error};

/// VENDOR E0 — writes control parameters to the scanner (focus, exposure, lamp,
/// CCD setup, ...). The sub-command byte selects the register; C1 then triggers
/// the operation (the `E0 -> C1 -> E1` cycle). The data-out payload holds the
/// register value (empty for trigger-only sub-commands like 0x80 lamp on/off).
pub struct VendorE0 {
    sub: u8,
    payload: Vec<u8>,
}

impl VendorE0 {
    pub fn new(sub: u8, payload: Vec<u8>) -> Self {
        Self { sub, payload }
    }
}

impl Command for VendorE0 {
    type Response = ();
    type Cdb = Cdb<10>;

    fn cdb(&self) -> Self::Cdb {
        let len = self.payload.len() as u32;
        Cdb([
            0xE0, // opcode
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
    fn cdb_sets_sub_command_and_length() {
        // CCD-setup preheat: sub 0xA0, 9-byte payload.
        let cdb = VendorE0::new(0xA0, vec![0u8; 9]).cdb().0;
        assert_eq!(
            cdb,
            [0xE0, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00]
        );
    }

    #[test]
    fn trigger_sub_command_has_zero_length() {
        // Lamp on/off (sub 0x80) carries no payload.
        let cdb = VendorE0::new(0x80, vec![]).cdb().0;
        assert_eq!(cdb[2], 0x80);
        assert_eq!([cdb[6], cdb[7], cdb[8]], [0, 0, 0]);
    }
}
