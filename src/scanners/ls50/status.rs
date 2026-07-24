//! TEST UNIT READY sense → readiness state. Only key/ASC/ASCQ triples seen in normal
//! operation map to a state; anything else stays a real error.

use crate::scsi::SenseData;

/// Scanner state, as reported by TEST UNIT READY.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Ready,
    /// 0x02 04/01 — warming up / initializing. Retry.
    Initializing,
    /// 0x02 04/02 — waiting for the init self-test. Won't clear on retry; caller must
    /// run init.
    NeedsInit,
    /// 0x02 3A/00 — no film loaded (adapter still reported by `holder()`).
    NoFilm,
    /// 0x02 05/00 — ejecting film.
    Ejecting,
    /// 0x06 29/00 — power-on / bus reset. Clear and retry.
    Reset,
    /// 0x06 28/00 — not-ready-to-ready change (medium).
    MediumChanged,
    /// 0x06 3F/03 — inquiry data changed; adapter swapped. Re-read holder.
    HolderChanged,
}

impl Status {
    /// Classify sense data as a known readiness state; `None` = treat as a real error.
    pub(crate) fn from_sense(sense: &SenseData) -> Option<Self> {
        match (sense.key, sense.asc, sense.ascq) {
            (0x02, 0x04, 0x01) => Some(Self::Initializing),
            (0x02, 0x04, 0x02) => Some(Self::NeedsInit),
            (0x02, 0x3A, 0x00) => Some(Self::NoFilm),
            (0x02, 0x05, 0x00) => Some(Self::Ejecting),
            (0x06, 0x29, 0x00) => Some(Self::Reset),
            (0x06, 0x28, 0x00) => Some(Self::MediumChanged),
            (0x06, 0x3F, 0x03) => Some(Self::HolderChanged),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sense(key: u8, asc: u8, ascq: u8) -> SenseData {
        SenseData {
            key,
            asc,
            ascq,
            ili: false,
            deferred: false,
        }
    }

    #[test]
    fn recognizes_initializing() {
        assert_eq!(
            Status::from_sense(&sense(0x02, 0x04, 0x01)),
            Some(Status::Initializing)
        );
    }

    #[test]
    fn recognizes_needs_init() {
        assert_eq!(
            Status::from_sense(&sense(0x02, 0x04, 0x02)),
            Some(Status::NeedsInit)
        );
    }

    #[test]
    fn recognizes_no_film() {
        assert_eq!(
            Status::from_sense(&sense(0x02, 0x3A, 0x00)),
            Some(Status::NoFilm)
        );
    }

    #[test]
    fn recognizes_ejecting() {
        assert_eq!(
            Status::from_sense(&sense(0x02, 0x05, 0x00)),
            Some(Status::Ejecting)
        );
    }

    #[test]
    fn recognizes_cold_start_unit_attentions() {
        // Real cold start drains these in order before GOOD.
        assert_eq!(
            Status::from_sense(&sense(0x06, 0x29, 0x00)),
            Some(Status::Reset)
        );
        assert_eq!(
            Status::from_sense(&sense(0x06, 0x28, 0x00)),
            Some(Status::MediumChanged)
        );
        assert_eq!(
            Status::from_sense(&sense(0x06, 0x3F, 0x03)),
            Some(Status::HolderChanged)
        );
    }

    #[test]
    fn ls9000_reset_code_is_not_a_state_here() {
        // 06/3F/04 is the LS-9000's reset code, not the LS-50's.
        assert_eq!(Status::from_sense(&sense(0x06, 0x3F, 0x04)), None);
    }

    #[test]
    fn unrecognized_sense_is_not_a_state() {
        assert_eq!(Status::from_sense(&sense(0x05, 0x24, 0x00)), None);
    }
}
