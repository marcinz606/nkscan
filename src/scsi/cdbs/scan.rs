//! SCAN(0x1B), from SCSI-2 15.2.2

use crate::scsi::{Cdb, Command, CommandData, Error};

/// SCAN - starts acquisition for a set of channels.
///
/// Must follow a [`SetWindow`](super::SetWindow). Byte 4 is the transfer length,
/// which for the Coolscan equals the number of channel-id bytes in the data-out
/// payload; the payload lists the channels to acquire — `01 02 03` for an RGB
/// pass, `09 01 02 03` to prepend the IR channel on a calibration pass. After
/// SCAN returns GOOD the image is retrieved with READ.
pub struct Scan {
    /// Channel ids to scan, sent as the data-out payload.
    channels: Vec<u8>,
}

impl Scan {
    pub fn new(channels: Vec<u8>) -> Self {
        Self { channels }
    }
}

impl Command for Scan {
    type Response = ();
    type Cdb = Cdb<6>;

    fn cdb(&self) -> Self::Cdb {
        Cdb([
            0x1B, // opcode
            0x00,
            0x00,
            0x00,
            self.channels.len() as u8, // transfer length = channel count
            0x00,                      // control
        ])
    }

    fn data(&self) -> CommandData<'_> {
        CommandData::Write(&self.channels)
    }

    fn decode(&self, _data: &[u8]) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cdb_transfer_length_is_channel_count() {
        // RGB preview pass: three channels.
        let cdb = Scan::new(vec![0x01, 0x02, 0x03]).cdb().0;
        assert_eq!(cdb, [0x1B, 0x00, 0x00, 0x00, 0x03, 0x00]);
    }

    #[test]
    fn cdb_matches_calibration_pass_capture() {
        // Cal pass prepends the IR channel: 09 01 02 03 -> length 4.
        let cdb = Scan::new(vec![0x09, 0x01, 0x02, 0x03]).cdb().0;
        assert_eq!(cdb, [0x1B, 0x00, 0x00, 0x00, 0x04, 0x00]);
    }

    #[test]
    fn data_is_write_of_channels() {
        let scan = Scan::new(vec![0x01, 0x02, 0x03]);
        assert!(matches!(
            scan.data(),
            CommandData::Write(&[0x01, 0x02, 0x03])
        ));
    }
}
