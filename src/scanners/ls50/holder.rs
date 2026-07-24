//! Film-adapter detection. The LS-50 has no "which adapter?" command; infer it from
//! the VPD page 0x00 supported-pages list. Keyed off firmware page codes, not Nikon's
//! labels (which confuse MA-21 vs SA-21).

use crate::scsi::cdbs::VpdPage;

/// Film adapter currently loaded, inferred from the supported-VPD-pages list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Holder {
    /// No adapter-specific page. Base pages 0xF8/0xFB/0xFC are advertised even with
    /// an adapter loaded, so this is the no-adapter fallback, not a positive match.
    None,
    /// Mounted-slide adapter — page 0x46 without 0xE2.
    Mount,
    /// Strip-film feeder (SA-21) or FH-3 strip holder — pages 0x43/0x44.
    Strip,
    /// Strip feeder in 6-frame mode — page 0x47.
    SixStrip,
    /// APS / IX240 adapter — pages 0x45/0xF1.
    Format240,
    /// Motorized auto-feeder — page 0x46 with 0xE2. Confirmed SA-21 strip feeder.
    Feeder,
    /// Recognized pages absent (e.g. 36-strip mode, whose page 0x10 collides with the
    /// standard device-id page and can't be distinguished here).
    Unknown,
}

impl Holder {
    /// EVPD page code for the supported-pages list.
    pub const SUPPORTED_PAGES_CODE: u8 = 0x00;
    /// Allocation length NikonScan uses for the supported-pages read (`0xFF`).
    pub const ALLOCATION_LENGTH: u8 = 0xFF;

    /// Classify the adapter from a VPD page 0x00 response (`page.data` = supported
    /// page-code bytes).
    pub fn from_supported_pages(page: &VpdPage) -> Holder {
        if page.page_code != Self::SUPPORTED_PAGES_CODE {
            return Holder::Unknown;
        }
        let has = |c: u8| page.data.contains(&c);
        if has(0x43) || has(0x44) {
            Holder::Strip
        } else if has(0x47) {
            Holder::SixStrip
        } else if has(0x45) || has(0xF1) {
            Holder::Format240
        } else if has(0x46) {
            if has(0xE2) {
                Holder::Feeder
            } else {
                Holder::Mount
            }
        } else if has(0xF8) || has(0xFA) || has(0xFB) || has(0xFC) {
            Holder::None
        } else {
            Holder::Unknown
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn page(data: Vec<u8>) -> VpdPage {
        VpdPage {
            page_code: Holder::SUPPORTED_PAGES_CODE,
            data,
        }
    }

    /// Standard pages always present.
    const STD: [u8; 8] = [0x00, 0x01, 0x10, 0x40, 0x41, 0x50, 0x51, 0x52];

    fn list(extra: &[u8]) -> VpdPage {
        let mut d = STD.to_vec();
        d.extend_from_slice(extra);
        page(d)
    }

    #[test]
    fn bare_mount_is_none() {
        assert_eq!(
            Holder::from_supported_pages(&list(&[0xF8, 0xFA, 0xFB, 0xFC])),
            Holder::None
        );
    }

    #[test]
    fn strip() {
        assert_eq!(
            Holder::from_supported_pages(&list(&[0x43, 0x44, 0xE2])),
            Holder::Strip
        );
    }

    #[test]
    fn mount_without_e2() {
        assert_eq!(Holder::from_supported_pages(&list(&[0x46])), Holder::Mount);
    }

    #[test]
    fn feeder_is_mount_page_plus_e2() {
        assert_eq!(
            Holder::from_supported_pages(&list(&[0x46, 0xE2])),
            Holder::Feeder
        );
    }

    #[test]
    fn six_strip() {
        assert_eq!(
            Holder::from_supported_pages(&list(&[0x47, 0xE2])),
            Holder::SixStrip
        );
    }

    #[test]
    fn format_240() {
        assert_eq!(
            Holder::from_supported_pages(&list(&[0x45, 0xF1])),
            Holder::Format240
        );
    }

    #[test]
    fn sa21_strip_feeder_real_capture() {
        // Real page-0x00 list, LS-50 + SA-21 feeder (captured 2026-07-22): 0x46 + 0xE2
        // → Feeder, base pages 0xF8/0xFB/0xFC present alongside.
        let real = page(vec![
            0x00, 0x01, 0x40, 0x41, 0x46, 0x50, 0x51, 0x60, 0x61, 0xC1, 0xD1, 0xE1, 0xF0, 0xF8,
            0xE2, 0xFB, 0xFC,
        ]);
        assert_eq!(Holder::from_supported_pages(&real), Holder::Feeder);
    }

    #[test]
    fn wrong_page_code_is_unknown() {
        let mut p = list(&[0x43]);
        p.page_code = 0x01;
        assert_eq!(Holder::from_supported_pages(&p), Holder::Unknown);
    }
}
