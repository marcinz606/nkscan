//! Device selection and the per-device dispatch seam.
//!
//! One [`Backend`] variant per supported scanner; every per-device `match` lives here so
//! the command handlers in [`super`] stay device-agnostic. Adding a device: add a
//! [`Device`] variant, a [`Backend`] variant, an [`open`] arm, then fill the `Backend`
//! method arms the compiler now flags. `Backend` is an enum, not `Box<dyn _>`, because
//! `Transport::send` is generic (not object-safe) so drivers keep a concrete transport.

use super::ScanCommon;
use super::output::{GrayImage, Image};
use nkscan::scanners::ls50::{AdapterCaps, Ls50, ScanSettings};
use nkscan::scsi::usb::UsbTransport;
use std::{error::Error, path::Path, thread::sleep, time::Duration};

/// Nikon LS-50 USB identity.
const LS50_VID: u16 = 0x04B0;
const LS50_PID: u16 = 0x4001;

/// Poll interval for [`Backend::watch`].
const WATCH_POLL: Duration = Duration::from_millis(250);

/// Frames a scan produced, as raw image buffers (device-neutral; see [`super::output`]).
type Frames = Vec<(Image, Option<GrayImage>)>;

#[derive(Copy, Clone, clap::ValueEnum)]
pub enum Device {
    /// Nikon LS-50 ED / Super Coolscan V ED (USB).
    Ls50,
    // Ls9000,  // Nikon LS-9000 ED (SCSI) — enable once ls9000ed has a scan.rs.
}

/// An opened scanner.
pub enum Backend {
    Ls50(Ls50<UsbTransport>),
    // Ls9k(Ls9k<SgDevice>),
}

/// Open the selected device. `_path` is the SCSI node for SCSI-only devices (USB ignores it).
pub fn open(device: Device, _path: Option<&Path>) -> Result<Backend, Box<dyn Error>> {
    match device {
        Device::Ls50 => Ok(Backend::Ls50(Ls50::new(UsbTransport::open(
            LS50_VID, LS50_PID,
        )?))),
        // Device::Ls9000 => {
        //     let path = _path.ok_or("ls9000 is SCSI-only: pass --path /dev/sgN")?;
        //     Ok(Backend::Ls9k(Ls9k::new(SgDevice::open(path)?)))
        // }
    }
}

impl Backend {
    pub fn status(&mut self) -> Result<String, Box<dyn Error>> {
        match self {
            Backend::Ls50(s) => Ok(format!("{:?}", s.status()?)),
        }
    }

    pub fn info(&mut self) -> Result<String, Box<dyn Error>> {
        match self {
            Backend::Ls50(s) => {
                let inq = s.inquiry()?;
                let holder = s.holder()?;
                let caps = s.caps();
                Ok(format!(
                    "{} {} rev {}\nholder: {holder:?}\ncaps: {caps:?}",
                    inq.vendor, inq.product, inq.revision
                ))
            }
        }
    }

    pub fn eject(&mut self) -> Result<(), Box<dyn Error>> {
        match self {
            Backend::Ls50(s) => Ok(s.eject()?),
        }
    }

    /// Poll readiness and print each change until interrupted (Ctrl-C).
    pub fn watch(&mut self) -> Result<(), Box<dyn Error>> {
        let mut last = String::new();
        loop {
            let now = self.status()?;
            if now != last {
                println!("{now}");
                last = now;
            }
            sleep(WATCH_POLL);
        }
    }

    pub fn scan(&mut self, common: &ScanCommon) -> Result<Frames, Box<dyn Error>> {
        match self {
            Backend::Ls50(s) => {
                let settings = ls50_settings(common, s.caps());
                let f = s.scan_at(&settings, common.offset)?;
                Ok(vec![(f.rgb, f.ir)])
            }
        }
    }

    /// Scan `count` frames, or the adapter's sensed count when `None`.
    pub fn scan_strip(
        &mut self,
        common: &ScanCommon,
        count: Option<u32>,
    ) -> Result<Frames, Box<dyn Error>> {
        match self {
            Backend::Ls50(s) => {
                let caps = s.caps();
                let n = count.unwrap_or(caps.n_frames);
                if n == 0 {
                    return Err(
                        "no frame count and the adapter reports 0 frames (load a strip, or pass --count N)".into(),
                    );
                }
                let settings = ls50_settings(common, caps);
                let frames = s.scan_strip(&settings, n, common.offset)?;
                Ok(frames.into_iter().map(|f| (f.rgb, f.ir)).collect())
            }
        }
    }
}

/// Build LS-50 scan settings from the shared CLI flags + probed caps. Pure — unit-tested.
fn ls50_settings(common: &ScanCommon, caps: AdapterCaps) -> ScanSettings {
    ScanSettings {
        dpi: common.dpi,
        infrared: common.ir,
        samples: common.samples,
        autoexposure: common.ae,
        autofocus: common.af,
        caps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn common() -> ScanCommon {
        ScanCommon {
            output: "scan.tiff".into(),
            ir: true,
            dpi: 2000,
            samples: 1,
            ae: true,
            af: true,
            offset: 1.5,
        }
    }

    #[test]
    fn ls50_settings_maps_flags() {
        let caps = AdapterCaps::default();
        let s = ls50_settings(&common(), caps);
        assert_eq!(s.dpi, 2000);
        assert!(s.infrared);
        assert_eq!(s.samples, 1);
        assert!(s.autoexposure);
        assert!(s.autofocus);
        assert_eq!(s.caps, caps);
    }
}
