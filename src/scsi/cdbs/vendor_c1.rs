//! VENDOR 0xC1 — Nikon control trigger (RE `scsi-commands/vendor-c1.md`).

use crate::scsi::{Cdb, Command, CommandData, Error};

/// VENDOR C1 — fires the operation whose parameters a preceding VENDOR E0 wrote
/// (the `E0 -> C1 -> E1` cycle). Opcode-only, no data phase.
#[derive(Debug, Default)]
pub struct VendorC1;

impl VendorC1 {
    pub fn new() -> Self {
        Self
    }
}

impl Command for VendorC1 {
    type Response = ();
    type Cdb = Cdb<6>;

    fn cdb(&self) -> Self::Cdb {
        Cdb([0xC1, 0x00, 0x00, 0x00, 0x00, 0x00])
    }

    fn data(&self) -> CommandData<'_> {
        CommandData::None
    }

    fn decode(&self, _data: &[u8]) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cdb_is_opcode_only() {
        assert_eq!(
            VendorC1::new().cdb().0,
            [0xC1, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }
}
