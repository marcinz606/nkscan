//! SEND DIAGNOSTIC(0x1D), from SCSI-2 8.2.1

use crate::scsi::{Cdb, Command, CommandData, Error};

/// SEND DIAGNOSTIC with the SelfTest bit set.
///
/// NikonScan sends the same CDB (`1D 04 00 00 00 00`) at several points in a
/// scan; the firmware picks the action from its own state (self-test on init,
/// pre-scan calibration, lamp-off after a scan), so the host just issues it.
/// No parameter list, so no data phase.
#[derive(Debug, Default)]
pub struct SendDiagnostic;

impl SendDiagnostic {
    pub fn new() -> Self {
        Self
    }
}

impl Command for SendDiagnostic {
    type Response = ();
    type Cdb = Cdb<6>;

    fn cdb(&self) -> Self::Cdb {
        Cdb([
            0x1D, // opcode
            0x04, // SelfTest=1 (bit 2)
            0x00, // reserved
            0x00, // parameter list length (MSB)
            0x00, // parameter list length (LSB)
            0x00, // control
        ])
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
    fn cdb_sets_selftest_bit() {
        assert_eq!(
            SendDiagnostic::new().cdb().0,
            [0x1D, 0x04, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn has_no_data_phase() {
        assert!(matches!(SendDiagnostic::new().data(), CommandData::None));
    }
}
