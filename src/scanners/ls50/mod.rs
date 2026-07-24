use crate::scsi::{Error as ScsiError, Transport, cdbs::*};
use holder::Holder;
use status::Status;
use tracing::debug;

pub mod decode;
pub mod holder;
mod scan;
pub mod status;

pub use scan::ScanError;

/// Base resolution (`0x0FA0` = 4000 DPI) + native scan area; [`AdapterCaps::default`]
/// fallback when page 0xc1 is unreadable.
const BASE_RES: u32 = 4000;
const NATIVE_XMAX: u32 = 3945;
const NATIVE_YMAX: u32 = 5958;

/// Adapter geometry + frame count from INQUIRY page 0xc1 (SANE `cs3_full_inquiry`);
/// SA-21 constants when absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdapterCaps {
    /// Frames sensed on the loaded strip (0 = none).
    pub n_frames: u32,
    /// Base (max) optical resolution, DPI.
    pub base_res: u32,
    /// Native scan area in max-resolution pixels.
    pub native_x: u32,
    pub native_y: u32,
}

impl Default for AdapterCaps {
    fn default() -> Self {
        Self {
            n_frames: 0,
            base_res: BASE_RES,
            native_x: NATIVE_XMAX,
            native_y: NATIVE_YMAX,
        }
    }
}

/// Parse page 0xc1 into [`AdapterCaps`]. SANE offsets index the raw response incl. its
/// 4-byte EVPD header; `VpdPage` strips it, so use `offset - 4`. `native_{x,y} =
/// boundary{x,y} - 1` (boundary is a count, geometry uses the max index). Fields fall
/// back to default on a short/zero slice.
fn parse_caps(page: &VpdPage) -> AdapterCaps {
    let d = &page.data;
    let be16 = |i: usize| {
        d.get(i..i + 2)
            .map(|b| u16::from_be_bytes([b[0], b[1]]) as u32)
    };
    let be32 = |i: usize| {
        d.get(i..i + 4)
            .map(|b| u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    };
    let def = AdapterCaps::default();
    AdapterCaps {
        n_frames: d.get(71).map(|&n| n as u32).unwrap_or(0),
        base_res: be16(38).filter(|&v| v != 0).unwrap_or(def.base_res), // resy_max
        native_x: be32(32).filter(|&v| v != 0).map_or(def.native_x, |v| v - 1), // boundaryx
        native_y: be32(54).filter(|&v| v != 0).map_or(def.native_y, |v| v - 1), // boundaryy
    }
}

/// 16-bit linear RGB (+ IR) over the full frame. The LS-50 scans only at
/// `base_res / pitch`, so `dpi` snaps to the grid (300 -> pitch 13 -> 307 DPI).
#[derive(Debug, Clone, Copy)]
pub struct ScanSettings {
    /// Optical resolution in DPI (snapped to the grid).
    pub dpi: u16,
    /// Capture the infrared plane (channel `0x09`) as a 4th planar channel.
    pub infrared: bool,
    /// Multi-sample count (`d[42]`). Only `1` works: a higher literal count arms a
    /// multi-pass scan that never streams (unresolved).
    pub samples: u8,
    /// Autoexposure pre-pass (`d[42]=0x20` + GET WINDOW read-back). Off = fixed default.
    pub autoexposure: bool,
    /// Firmware autofocus at the frame centre (`E0 0xA0`). Off = focus parked at 0 (soft).
    pub autofocus: bool,
    /// Adapter geometry driving window/read dims — [`Ls50::caps`] or `AdapterCaps::default`.
    pub caps: AdapterCaps,
}

impl ScanSettings {
    /// Native dots per output pixel: `base_res / dpi`, rounded, at least 1.
    fn pitch(&self) -> u32 {
        (self.caps.base_res as f64 / self.dpi as f64)
            .round()
            .max(1.0) as u32
    }

    /// Colour planes on the wire: 3 (RGB), or 4 with the infrared plane.
    pub fn n_colors(&self) -> usize {
        3 + usize::from(self.infrared)
    }

    /// Grid-snapped resolution (`base_res / pitch`) for the SET WINDOW descriptor.
    pub fn res(&self) -> u16 {
        (self.caps.base_res / self.pitch()) as u16
    }

    /// Output `(width, height)` px — drives the READ count and decode.
    pub fn output_dims(&self) -> (u32, u32) {
        let pitch = self.pitch();
        (self.caps.native_x / pitch, self.caps.native_y / pitch)
    }

    /// Native (max-res) window `(width, height)` for the SET WINDOW descriptor.
    pub fn native_dims(&self) -> (u32, u32) {
        let pitch = self.pitch();
        let (w, h) = self.output_dims();
        (w * pitch, h * pitch)
    }

    /// On-wire bytes/line: `n_colors` planes, each `width` samples padded even at
    /// 2 bytes/sample, line block-padded to a 512 multiple.
    pub fn bytes_per_line(&self) -> usize {
        let (w, _) = self.output_dims();
        let even_w = w as usize + (w as usize & 1);
        (self.n_colors() * even_w * 2).div_ceil(512) * 512
    }

    /// Total image bytes the scanner returns for this frame.
    pub fn expected_bytes(&self) -> u64 {
        let (_, height) = self.output_dims();
        self.bytes_per_line() as u64 * height as u64
    }
}

/// Nikon LS-50 ED (Super Coolscan V ED), 35mm USB film scanner. Generic over the
/// transport; [`UsbTransport`](crate::scsi::usb::UsbTransport) in practice.
pub struct Ls50<T> {
    transport: T,
}

impl<T> Ls50<T>
where
    T: Transport,
{
    pub fn new(transport: T) -> Self {
        Ls50 { transport }
    }

    #[cfg(test)]
    pub(super) fn transport(&self) -> &T {
        &self.transport
    }

    /// INQUIRY identity (`"Nikon"` + `"LS-50"` in vendor/product). Revision is
    /// unreliable (`2.03` on hardware vs `1.02` in the firmware template) — don't gate on it.
    pub fn inquiry(&mut self) -> Result<InquiryResponse, ScsiError> {
        self.transport.send(&Inquiry::new())
    }

    /// Adapter geometry + frame count from page 0xc1; [`AdapterCaps::default`] if
    /// absent. Read-only.
    pub fn caps(&mut self) -> AdapterCaps {
        match self.transport.send(&VpdInquiry::new(0xC1, 87)) {
            Ok(page) => {
                let caps = parse_caps(&page);
                // Log both n_frames candidates: data[71] sensed, data[70] capacity.
                debug!(
                    ?caps,
                    raw70 = page.data.get(70),
                    raw71 = page.data.get(71),
                    "adapter caps (page 0xc1)"
                );
                if caps.base_res != BASE_RES {
                    debug!(
                        caps.base_res,
                        "page 0xc1 base_res differs from the 4000-dpi default"
                    );
                }
                caps
            }
            Err(e) => {
                debug!(?e, "page 0xc1 unavailable; using default adapter caps");
                AdapterCaps::default()
            }
        }
    }

    /// Current readiness state, classified from TEST UNIT READY sense.
    pub fn status(&mut self) -> Result<Status, ScsiError> {
        match self.transport.send(&TestUnitReady::new()) {
            Ok(()) => Ok(Status::Ready),
            Err(err) => {
                if let ScsiError::Status {
                    sense: Some(sense), ..
                } = &err
                    && let Some(state) = Status::from_sense(sense)
                {
                    return Ok(state);
                }
                Err(err)
            }
        }
    }

    /// Which film adapter is loaded, inferred from the EVPD supported-pages list.
    pub fn holder(&mut self) -> Result<Holder, ScsiError> {
        let page = self.transport.send(&VpdInquiry::new(
            Holder::SUPPORTED_PAGES_CODE,
            Holder::ALLOCATION_LENGTH,
        ))?;
        let holder = Holder::from_supported_pages(&page);
        // Log the raw page list to diagnose an unrecognized adapter.
        debug!(supported_pages = ?page.data, ?holder, "decoded holder from VPD page 0x00");
        Ok(holder)
    }

    /// Claim exclusive access to the scanner (RESERVE UNIT).
    pub fn reserve(&mut self) -> Result<(), ScsiError> {
        self.transport.send(&ReserveUnit::default())
    }

    /// Release exclusive access to the scanner (RELEASE UNIT).
    pub fn release(&mut self) -> Result<(), ScsiError> {
        self.transport.send(&ReleaseUnit::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scsi::{DataDirection, Error, SenseData};

    /// In-test transport: TUR from `tur_sense` (`None` = GOOD), INQUIRY/EVPD from
    /// `vpd` raw bytes.
    struct MockTransport {
        tur_sense: Option<SenseData>,
        vpd: Vec<u8>,
    }

    impl Transport for MockTransport {
        fn execute(
            &mut self,
            cdb: &[u8],
            _direction: DataDirection,
            data: &mut [u8],
            _sense: &mut [u8],
        ) -> Result<(), Error> {
            match cdb[0] {
                0x00 => match self.tur_sense {
                    None => Ok(()),
                    Some(sense) => Err(Error::Status {
                        status: 0x02,
                        sense: Some(sense),
                    }),
                },
                0x12 => {
                    let n = self.vpd.len().min(data.len());
                    data[..n].copy_from_slice(&self.vpd[..n]);
                    Ok(())
                }
                other => panic!("unexpected opcode {other:#04x}"),
            }
        }
    }

    /// Build a VPD page 0x00 response: 4-byte header + supported-page codes.
    fn vpd_page0(codes: &[u8]) -> Vec<u8> {
        let mut v = vec![0x06, 0x00, 0x00, codes.len() as u8];
        v.extend_from_slice(codes);
        v
    }

    #[test]
    fn geometry_snaps_dpi_to_grid() {
        let s = ScanSettings {
            dpi: 300,
            infrared: false,
            samples: 1,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        assert_eq!(s.res(), 307);
        assert_eq!(s.output_dims(), (303, 458));
        assert_eq!(s.native_dims(), (3939, 5954));
        assert_eq!(s.n_colors(), 3);
        // 3 planes * even(303)=304 * 2 bytes/sample = 1824 -> block-padded to 2048.
        assert_eq!(s.bytes_per_line(), 2048);
        assert_eq!(s.expected_bytes(), 2048 * 458);
    }

    #[test]
    fn infrared_adds_a_fourth_plane() {
        let s = ScanSettings {
            dpi: 300,
            infrared: true,
            samples: 1,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        // 4 planes: 4 * 304 * 2 = 2432 -> block-padded to 2560.
        assert_eq!(s.output_dims(), (303, 458));
        assert_eq!(s.n_colors(), 4);
        assert_eq!(s.bytes_per_line(), 2560);
        assert_eq!(s.expected_bytes(), 2560 * 458);
    }

    #[test]
    fn status_ready_on_good() {
        let mut s = Ls50::new(MockTransport {
            tur_sense: None,
            vpd: vec![],
        });
        assert_eq!(s.status().unwrap(), Status::Ready);
    }

    #[test]
    fn status_classifies_no_film() {
        let mut s = Ls50::new(MockTransport {
            tur_sense: Some(SenseData {
                key: 0x02,
                asc: 0x3A,
                ascq: 0x00,
                ili: false,
                deferred: false,
            }),
            vpd: vec![],
        });
        assert_eq!(s.status().unwrap(), Status::NoFilm);
    }

    #[test]
    fn status_propagates_real_errors() {
        // An unmapped sense triple must surface as an error, not a state.
        let mut s = Ls50::new(MockTransport {
            tur_sense: Some(SenseData {
                key: 0x05,
                asc: 0x24,
                ascq: 0x00,
                ili: false,
                deferred: false,
            }),
            vpd: vec![],
        });
        assert!(s.status().is_err());
    }

    #[test]
    fn holder_decodes_from_vpd() {
        let mut s = Ls50::new(MockTransport {
            tur_sense: None,
            vpd: vpd_page0(&[0x00, 0x01, 0x43, 0x44, 0xE2]),
        });
        assert_eq!(s.holder().unwrap(), Holder::Strip);
    }

    #[test]
    fn parse_caps_reads_geometry_and_frames() {
        // Header-stripped page 0xc1 (SANE offset - 4): boundaryx BE32 @32, resy_max
        // BE16 @38, boundaryy BE32 @54, n_frames @71. native = boundary - 1.
        let mut data = vec![0u8; 83];
        data[32..36].copy_from_slice(&3946u32.to_be_bytes());
        data[38..40].copy_from_slice(&4000u16.to_be_bytes());
        data[54..58].copy_from_slice(&5959u32.to_be_bytes());
        data[71] = 6;
        let page = VpdPage {
            page_code: 0xC1,
            data,
        };
        assert_eq!(
            parse_caps(&page),
            AdapterCaps {
                n_frames: 6,
                base_res: 4000,
                native_x: 3945,
                native_y: 5958,
            }
        );
    }

    #[test]
    fn parse_caps_falls_back_when_short_or_zero() {
        // Too short and all-zero pages both yield the defaults (zeros are filtered).
        let short = VpdPage {
            page_code: 0xC1,
            data: vec![0u8; 10],
        };
        let zeros = VpdPage {
            page_code: 0xC1,
            data: vec![0u8; 83],
        };
        assert_eq!(parse_caps(&short), AdapterCaps::default());
        assert_eq!(parse_caps(&zeros), AdapterCaps::default());
    }

    #[test]
    fn geometry_follows_caps() {
        // Non-default native area flows through pitch/output/native dims.
        let s = ScanSettings {
            dpi: 4000,
            infrared: false,
            samples: 1,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps {
                n_frames: 0,
                base_res: 4000,
                native_x: 2000,
                native_y: 4000,
            },
        };
        assert_eq!(s.pitch(), 1); // 4000 / 4000
        assert_eq!(s.output_dims(), (2000, 4000)); // native / pitch
        assert_eq!(s.native_dims(), (2000, 4000));
    }
}
