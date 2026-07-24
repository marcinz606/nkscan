//! The LS-50 scan drive: cold-start init, arming, and draining the image.
//!
//! RGB or RGB+IR ([`ScanSettings::infrared`]), one frame
//! ([`scan`](Ls50::scan)/[`scan_at`](Ls50::scan_at)) or a whole strip
//! ([`scan_strip`](Ls50::scan_strip)).

use super::decode::{Frame, FrameDecoder, PlanarDecoder};
use super::status::Status;
use super::{Holder, Ls50, ScanSettings};
use crate::scsi::{Command, Error as ScsiError, Transport, cdbs::*};
use std::{thread::sleep, time::Duration};
use tracing::{debug, warn};

/// R, G, B channel ids (SCAN payload / SET WINDOW window-id).
const CHANNELS_RGB: [u8; 3] = [0x01, 0x02, 0x03];

/// Infrared channel id, appended after RGB when [`ScanSettings::infrared`] is set.
/// Its window exposure is left at 0.
const IR_CHANNEL: u8 = 0x09;

/// Channel list for this scan: `[R, G, B]`, or `[R, G, B, IR]` with infrared.
fn channels(settings: &ScanSettings) -> Vec<u8> {
    let mut c = CHANNELS_RGB.to_vec();
    if settings.infrared {
        c.push(IR_CHANNEL);
    }
    c
}

/// Fixed per-channel exposure (R, G, B) in 10 ns units — AE seed, and used when AE off.
// ponytail: tune per film/holder.
const EXPOSURE_10NS: [u32; 3] = [120_000, 120_000, 100_000];

/// SCAN returns a busy CHECK CONDITION during lamp/carriage warm-up; retry until GOOD.
const SCAN_ATTEMPTS: u32 = 30;
const SCAN_RETRY_PAUSE: Duration = Duration::from_millis(500);

/// Poll TUR long enough to drain cold-start unit attentions + the ~12 s warm-up.
const READY_ATTEMPTS: u32 = 100;
const READY_PAUSE: Duration = Duration::from_millis(250);

/// Mid-drain not-ready sense = next line not produced yet, not end-of-data; retry cap.
const IMAGE_IDLE_ATTEMPTS: u32 = 600;
const IMAGE_IDLE_PAUSE: Duration = Duration::from_millis(200);

#[derive(Debug, thiserror::Error)]
pub enum ScanError {
    #[error(transparent)]
    Scsi(#[from] ScsiError),
    #[error(transparent)]
    Decode(#[from] super::decode::Error),
    #[error("SCAN never reached GOOD after {0} attempts")]
    ScanNotReady(u32),
    #[error("scanner not ready to scan: {0:?}")]
    NotReady(Status),
    #[error("scanner produced no image data (pre-scan calibration/preheat not done?)")]
    NoData,
}

impl<T> Ls50<T>
where
    T: Transport,
{
    /// Scan frame 0 and return the decoded frame (no feed offset).
    pub fn scan(&mut self, settings: &ScanSettings) -> Result<Frame, ScanError> {
        self.scan_at(settings, 0.0)
    }

    /// Scan one frame at feed-axis offset `subframe_mm`: prepare, arm, SCAN, drain, decode.
    pub fn scan_at(
        &mut self,
        settings: &ScanSettings,
        subframe_mm: f32,
    ) -> Result<Frame, ScanError> {
        self.prepare()?;
        let sub = subframe_native(subframe_mm);
        let mut dec = PlanarDecoder::new(settings)?;
        // One frame: declare one, select frame 0.
        let result = match self.exposure_for(settings, 1, 0, sub) {
            Ok(exp) => self.drive_one(settings, 1, 0, sub, &exp, &mut dec),
            Err(e) => Err(e),
        };
        let _ = self.release();
        result?;
        let frame = dec.finish()?.to_owned();
        debug!(
            width = frame.rgb.width(),
            height = frame.rgb.height(),
            "image drained"
        );
        if frame.rgb.height() == 0 {
            return Err(ScanError::NoData);
        }
        Ok(frame)
    }

    /// Scan `frames` frames of a loaded strip. `prepare` once; each frame declares the
    /// whole strip in the boundary (so the feed advances) and selects frame `f` by
    /// window Y-offset `f*pitch + subframe`. No host feed command — the feed motor moves.
    /// Strip must be sensed multi-frame (freshly loaded) or later frames come back black.
    pub fn scan_strip(
        &mut self,
        settings: &ScanSettings,
        frames: u32,
        subframe_mm: f32,
    ) -> Result<Vec<Frame>, ScanError> {
        self.prepare()?;
        let result = self.drive_strip(settings, frames, subframe_mm);
        let _ = self.release();
        result
    }

    /// Eject the loaded film/strip: VENDOR E0 sub `0xD0` + 13 zero bytes, then C1.
    /// Reserves, ejects, releases. Load/advance sub `0xD1` is rejected `05/24` here —
    /// frames advance by window Y-offset, no load counterpart.
    pub fn eject(&mut self) -> Result<(), ScanError> {
        self.reserve()?;
        self.trigger(&VendorE0::new(0xD0, vec![0u8; 13]))?;
        self.trigger(&VendorC1::new())?;
        // Eject motor runs several s (TUR = Ejecting); drain transients before release.
        let _ = self.wait_settled();
        let _ = self.release();
        Ok(())
    }

    /// Per-frame loop for [`scan_strip`](Self::scan_strip); caller releases the unit.
    fn drive_strip(
        &mut self,
        settings: &ScanSettings,
        frames: u32,
        subframe_mm: f32,
    ) -> Result<Vec<Frame>, ScanError> {
        let sub = subframe_native(subframe_mm);
        // Measure exposure once on frame 0 and reuse it for the whole strip.
        let exposure = self.exposure_for(settings, frames, 0, sub)?;
        let mut dec = PlanarDecoder::new(settings)?;
        let mut out = Vec::with_capacity(frames as usize);
        for f in 0..frames {
            dec.reset();
            // Boundary declares all frames so the feed reaches `f`; window selects it.
            self.drive_one(settings, frames, f, sub, &exposure, &mut dec)?;
            let frame = dec.finish()?.to_owned();
            debug!(
                frame = f,
                width = frame.rgb.width(),
                height = frame.rgb.height(),
                "strip frame drained"
            );
            if frame.rgb.height() == 0 {
                return Err(ScanError::NoData);
            }
            out.push(frame);
        }
        Ok(out)
    }

    /// Arm frame `frame`, issue SCAN, and drain its lines into `dec`.
    fn drive_one(
        &mut self,
        settings: &ScanSettings,
        n_frames: u32,
        frame: u32,
        subframe: u32,
        exposure: &[u32; 3],
        dec: &mut PlanarDecoder,
    ) -> Result<(), ScanError> {
        let chans = channels(settings);
        self.arm_pass(settings, n_frames, frame, subframe, exposure, false)?;
        self.start_scan(&chans)?;
        self.drain_image(settings, dec)
    }

    /// Per-channel exposure for the upcoming pass: firmware-measured (via an AE
    /// pre-pass on `frame`) when [`ScanSettings::autoexposure`] is set, else the
    /// fixed [`EXPOSURE_10NS`].
    fn exposure_for(
        &mut self,
        settings: &ScanSettings,
        n_frames: u32,
        frame: u32,
        subframe: u32,
    ) -> Result<[u32; 3], ScanError> {
        if settings.autoexposure {
            // Measure RGB only; IR keeps a fixed exposure.
            let ae = ScanSettings {
                infrared: false,
                autoexposure: false,
                ..*settings
            };
            self.measure_exposure(&ae, n_frames, frame, subframe)
        } else {
            Ok(EXPOSURE_10NS)
        }
    }

    /// Autoexposure pre-pass: arm `frame` in AE mode (`d[42]=0x20`), SCAN RGB, read the
    /// measured exposure via GET WINDOW. AE pass streams no image — nothing to drain.
    /// Firmware re-measures only when calibration is stale (cold-start/idle); otherwise
    /// it returns the seed — detect that and fall back to fixed exposure.
    fn measure_exposure(
        &mut self,
        settings: &ScanSettings,
        n_frames: u32,
        frame: u32,
        subframe: u32,
    ) -> Result<[u32; 3], ScanError> {
        self.arm_pass(settings, n_frames, frame, subframe, &EXPOSURE_10NS, true)?;
        self.start_scan(&CHANNELS_RGB)?;
        let _ = self.wait_ready();
        let measured = self.read_exposure_rgb()?;
        if measured == EXPOSURE_10NS {
            warn!(
                "autoexposure: firmware skipped measurement (calibration cached); using fixed exposure"
            );
            return Ok(EXPOSURE_10NS);
        }
        debug!(?measured, "autoexposure: measured");
        Ok(measured)
    }

    /// Read the per-channel RGB exposure (RAM 0x400FAE) back from the window
    /// descriptors: GET WINDOW bytes 54..57 per channel.
    fn read_exposure_rgb(&mut self) -> Result<[u32; 3], ScanError> {
        Ok([
            self.get_window_exposure(CHANNELS_RGB[0])?,
            self.get_window_exposure(CHANNELS_RGB[1])?,
            self.get_window_exposure(CHANNELS_RGB[2])?,
        ])
    }

    /// Firmware autofocus at frame centre (SANE `cs3_autofocus`): read focus, run AF at
    /// the centre (`E0 0xA0` + focusX/focusY, then C1), read result. Centre = `width/2`,
    /// `y_offset + height/2` native (xoffset 0). `E0 0xA0` with a focus X/Y payload does
    /// not eject here (a different payload did); before/after read confirms motor moved.
    fn autofocus(&mut self, settings: &ScanSettings, y_offset: u32) -> Result<(), ScanError> {
        let (native_w, native_h) = settings.native_dims();
        let focus_x = native_w / 2;
        let focus_y = y_offset + native_h / 2;
        let before = self.read_focus().unwrap_or(0);
        self.trigger(&VendorE0::new(0xA0, autofocus_payload(focus_x, focus_y)))?;
        self.trigger(&VendorC1::new())?;
        let _ = self.wait_ready();
        let after = self.read_focus().unwrap_or(0);
        debug!(focus_x, focus_y, before, after, "autofocus done");
        Ok(())
    }

    /// Read the current focus position (SANE `cs3_read_focus`): `E1 0xC1`, 13 bytes,
    /// focus = response bytes 1..4 u32 BE.
    fn read_focus(&mut self) -> Result<u32, ScanError> {
        let resp = self.transport.send(&VendorE1::new(0xC1, 13))?;
        Ok(resp
            .get(1..5)
            .map(|b| u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
            .unwrap_or(0))
    }

    /// Read one channel's firmware-measured exposure from its window descriptor:
    /// GET WINDOW (Single, window-id = channel), exposure = descriptor `vendor[6..10]`
    /// u32 BE (descriptor offset 46..50, the same offset [`build_window`] writes).
    fn get_window_exposure(&mut self, channel: u8) -> Result<u32, ScanError> {
        let descriptors = self.transport.send(&GetWindow::new(0, true, channel, 58, 0x00))?;
        let exp = descriptors
            .first()
            .and_then(|d| d.vendor.get(6..10))
            .map(|b| u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
            .unwrap_or(0);
        Ok(exp)
    }

    /// Arm one image pass: frame boundary, focus, per-channel identity gamma LUT,
    /// per-channel SET WINDOW (per-channel `exposure`; IR gets none), GET WINDOW
    /// read-back. `ae` sets the AE scan-mode byte for a measurement pass. The caller
    /// issues SCAN afterwards. The boundary declares all `n_frames`; the window
    /// selects `frame` (0-based) at `frame*pitch + subframe`.
    fn arm_pass(
        &mut self,
        settings: &ScanSettings,
        n_frames: u32,
        frame: u32,
        subframe: u32,
        exposure: &[u32; 3],
        ae: bool,
    ) -> Result<(), ScanError> {
        let y_offset = frame * pitch_native() + subframe;
        // Boundary: declare whole strip so the feed reaches `frame`.
        match self
            .transport
            .send(&self.boundary_cmd(settings, n_frames, subframe))
        {
            Ok(_) => debug!(n_frames, "arm: boundary ok"),
            Err(e) => debug!(?e, "arm: boundary rejected"),
        }
        // Autofocus at the frame centre (real pass, when enabled); else park focus at 0.
        // The motor runs ~1 s afterward — wait it out or SCAN fires mid-motion.
        if settings.autofocus && !ae {
            self.autofocus(settings, y_offset)?;
        } else {
            self.trigger(&VendorE0::new(0xC1, vec![0u8; 9]))?;
            self.trigger(&VendorC1::new())?;
            let _ = self.wait_ready();
        }
        debug!("arm: focus done");
        let chans = channels(settings);
        // Identity gamma LUT — normal scans only (SANE `cs3_scan` skips it for AE).
        if !ae {
            let lut = build_gamma_lut();
            for &channel in &chans {
                self.transport.send(&Write::new(
                    0,
                    DataTypeCode::GammaFunction,
                    ((channel as u16) << 8) | 0x01,
                    lut.clone(),
                    0x00,
                ))?;
            }
            debug!("arm: lut done");
        }
        // Per-channel window: given exposure, or 0 for the IR channel (index >= 3).
        for (k, &channel) in chans.iter().enumerate() {
            let exp = exposure.get(k).copied();
            let payload = build_window(settings, channel, exp, y_offset, ae);
            match self.transport.send(&SetWindow::new(payload)) {
                Ok(()) => debug!(channel, "arm: set window ok"),
                // key 0x01 RECOVERED (e.g. 01/37): window set, value snapped to grid.
                Err(ScsiError::Status { sense: Some(s), .. }) if s.key == 0x01 => {
                    debug!(channel, ?s, "arm: set window accepted (recovered)")
                }
                Err(e) => {
                    debug!(channel, ?e, "arm: set window rejected");
                    return Err(e.into());
                }
            }
        }
        // GET WINDOW read-back required or the scan never reaches read-ready.
        for _ in &chans {
            let _ = self.transport.send(&GetWindow::new(0, false, 0, 58, 0x00));
        }
        let _ = self.wait_ready();
        debug!("arm: windows read back");
        Ok(())
    }

    /// SET BOUNDARY (WRITE DTC 0x88): declare all `n_frames` frames in one command. The
    /// scanner needs the full layout to advance the feed; a single-frame boundary leaves
    /// later frames black. Payload: `[len_hi len_lo n n]` (`len = 4 + 16*n`) then per
    /// frame `[Ystart Xstart Yend Xend]` u32 BE: `Ystart = i*pitch + subframe`,
    /// `Yend = Ystart + pitch - 1`, `Xend = native_x - 1` (adapter caps).
    fn boundary_cmd(&self, settings: &ScanSettings, n_frames: u32, subframe: u32) -> Write {
        let len = 4 + 16 * n_frames;
        let mut p = vec![(len >> 8) as u8, len as u8, n_frames as u8, n_frames as u8];
        let pitch = pitch_native();
        for i in 0..n_frames {
            let ystart = i * pitch + subframe;
            p.extend_from_slice(&ystart.to_be_bytes()); // Ystart
            p.extend_from_slice(&0u32.to_be_bytes()); // Xstart
            p.extend_from_slice(&(ystart + pitch - 1).to_be_bytes()); // Yend
            p.extend_from_slice(&(settings.caps.native_x - 1).to_be_bytes()); // Xend
        }
        Write::new(0, DataTypeCode::Vendor(0x88), 0x03, p, 0x00)
    }

    /// Cold-start init, once before configuring: reserve, adapter probe, mode select,
    /// and — non-feeder holder only — self-test + lamp warm-up. Triggers report benign
    /// "busy"; the trailing `wait_ready` waits out the warm-up.
    fn prepare(&mut self) -> Result<(), ScanError> {
        // Bail on NoFilm before motion — self-test/lamp ejects a strip not at the scan
        // position. A feeder's loaded strip reads Ready, not NoFilm.
        if self.wait_settled()? == Status::NoFilm {
            return Err(ScanError::NotReady(Status::NoFilm));
        }
        let feeder = matches!(self.holder(), Ok(Holder::Feeder));
        self.trigger(&ReserveUnit::new(0, None, 0))?;
        self.detect_adapter();
        self.configure_mode()?;
        if !feeder {
            // Self-test homes carriage; lamp/C1 eject a positioned strip — non-feeder only.
            self.trigger(&SendDiagnostic::new())?; // self-test: clears NeedsInit
            self.trigger(&VendorE0::new(0x80, vec![]))?; // lamp on
            self.trigger(&VendorC1::new())?;
        }
        self.wait_ready()
    }

    /// Probe the INQUIRY EVPD adapter-config pages, best-effort — pages absent on
    /// this unit come back `05/24` and are skipped. Read-only, no film motion.
    fn detect_adapter(&mut self) {
        for (page, alloc) in [
            (0x00u8, 23u8),
            (0xd1, 28),
            (0xc1, 87),
            (0xe1, 39),
            (0xf0, 53),
            (0xf8, 17),
        ] {
            let _ = self.transport.send(&VpdInquiry::new(page, alloc));
        }
    }

    /// MODE SELECT page 0x03 — base resolution (`0x0FA0` = 4000 DPI) and scan area.
    /// Without it the scan mode is unset and SCAN rejects its channel list
    /// (`05/26/02`).
    fn configure_mode(&mut self) -> Result<(), ScanError> {
        const MODE_PAGE_3: [u8; 20] = [
            0x00, 0x00, 0x00, 0x08, // header: block-descriptor length 8
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // block descriptor
            0x03, 0x06, 0x00, 0x00, 0x0F, 0xA0, 0x00, 0x00, // page 0x03, res 0x0FA0
        ];
        self.trigger(&ModeSelect::new(0, true, false, MODE_PAGE_3.to_vec(), 0x00))
    }

    /// Fire-and-forget control command, tolerating the "busy" CHECK CONDITION a
    /// triggered operation returns while it runs.
    fn trigger<C: Command>(&mut self, command: &C) -> Result<(), ScanError> {
        match self.transport.send(command) {
            Ok(_) | Err(ScsiError::Status { .. }) => Ok(()),
            Err(err) => Err(err.into()),
        }
    }

    /// Poll TUR until decisive — `Ready`/`NoFilm`/`NeedsInit` — draining transient
    /// becoming-ready / unit-attention states. `NeedsInit` ends the poll: scanner wants
    /// the self-test next.
    fn wait_settled(&mut self) -> Result<Status, ScanError> {
        for _ in 0..READY_ATTEMPTS {
            let status = self.status()?;
            match status {
                Status::Ready | Status::NoFilm | Status::NeedsInit => return Ok(status),
                _ => sleep(READY_PAUSE),
            }
        }
        Err(ScanError::NotReady(Status::Initializing))
    }

    /// Poll TUR until `Ready`, draining cold-start unit attentions (reset/medium/holder
    /// changed). Without it the first SET WINDOW is rejected by a pending unit attention.
    fn wait_ready(&mut self) -> Result<(), ScanError> {
        let mut last = Status::Initializing;
        for _ in 0..READY_ATTEMPTS {
            match self.status()? {
                Status::Ready => return Ok(()),
                Status::NoFilm => return Err(ScanError::NotReady(Status::NoFilm)),
                other => last = other,
            }
            sleep(READY_PAUSE);
        }
        Err(ScanError::NotReady(last))
    }

    fn start_scan(&mut self, channels: &[u8]) -> Result<(), ScanError> {
        for attempt in 0..SCAN_ATTEMPTS {
            match self.transport.send(&Scan::new(channels.to_vec())) {
                Ok(()) => return Ok(()),
                // CHECK CONDITION here is the warm-up "busy" state, not a failure.
                Err(ScsiError::Status { sense, .. }) => {
                    debug!(attempt, ?sense, "SCAN busy, retrying");
                    sleep(SCAN_RETRY_PAUSE);
                }
                Err(err) => return Err(err.into()),
            }
        }
        Err(ScanError::ScanNotReady(SCAN_ATTEMPTS))
    }

    /// Drain the image into `dec`, one padded line per READ (each plane padded to even
    /// bytes, planes concatenated, line block-padded to 512). Lines arrive as the
    /// carriage moves: a not-ready sense (key 0x02, or 05/2C) means retry the same line;
    /// any other CHECK CONDITION is end-of-data.
    fn drain_image(
        &mut self,
        settings: &ScanSettings,
        dec: &mut PlanarDecoder,
    ) -> Result<(), ScanError> {
        let (_, max_lines) = settings.output_dims();
        let stride = settings.bytes_per_line();
        'lines: for _ in 0..max_lines {
            let mut idle = 0u32;
            loop {
                // Wait Ready before each READ — reading mid-positioning (05/2C) aborts the scan.
                if !matches!(self.status(), Ok(Status::Ready)) {
                    idle += 1;
                    if idle >= IMAGE_IDLE_ATTEMPTS {
                        return Err(ScanError::ScanNotReady(idle));
                    }
                    sleep(IMAGE_IDLE_PAUSE);
                    continue;
                }
                match self.transport.send(&Read::new(
                    0,
                    DataTypeCode::Image,
                    0,
                    stride as u32,
                    0x00,
                )) {
                    Ok(line) => {
                        dec.push(&line)?;
                        break;
                    }
                    // Flipped back to not-ready between poll and read: retry the line.
                    Err(ScsiError::Status { sense: Some(s), .. })
                        if s.key == 0x02 || (s.key == 0x05 && s.asc == 0x2C) =>
                    {
                        idle += 1;
                        if idle >= IMAGE_IDLE_ATTEMPTS {
                            return Err(ScanError::ScanNotReady(idle));
                        }
                        sleep(IMAGE_IDLE_PAUSE);
                    }
                    // any other sense ends the transfer early.
                    Err(ScsiError::Status { sense, .. }) => {
                        debug!(?sense, "image: end of data");
                        break 'lines;
                    }
                    Err(err) => return Err(err.into()),
                }
            }
        }
        Ok(())
    }
}

/// Bits per sample requested in the window descriptor. 14-bit ADC range, delivered
/// as 2 bytes/sample big-endian in a 16-bit container.
// ponytail: if the scanner rejects 0x0E or returns mis-sized lines, try 0x10 (16).
const DEPTH_14BIT: u8 = 0x0E;

/// Build the SET WINDOW data-out payload for one channel — 50-byte descriptor packed
/// from zero (fresh forces `d[1]=0`; a stale read-back there once no-op'd the scan).
/// Body `d` (after 8-byte header): `d[0]` channel, `d[2..6]` resolution, `d[10..14]` Y
/// offset, `d[14..22]` native w/h, `d[25]` composition, `d[26]` depth, `d[42]`
/// scan-mode, `d[46..50]` exposure. `exposure` `None` (→ 0) for IR.
fn build_window(
    settings: &ScanSettings,
    channel: u8,
    exposure: Option<u32>,
    y_offset: u32,
    ae: bool,
) -> Vec<u8> {
    let res = settings.res();
    let (native_w, native_h) = settings.native_dims();
    let mut d = vec![0u8; 50];
    d[0] = channel;
    // d[1] = 0; x offset d[6..10] = 0 (full-frame origin); reserved fields stay 0.
    d[2..4].copy_from_slice(&res.to_be_bytes()); // X resolution (grid-snapped)
    d[4..6].copy_from_slice(&res.to_be_bytes()); // Y resolution
    d[10..14].copy_from_slice(&y_offset.to_be_bytes()); // Y offset (feed axis)
    // Width/height are in native (max-resolution) units, not output pixels.
    d[14..18].copy_from_slice(&native_w.to_be_bytes());
    d[18..22].copy_from_slice(&native_h.to_be_bytes());
    d[25] = 0x05; // composition: RGB
    d[26] = DEPTH_14BIT; // 14-bit, delivered in a 16-bit container
    d[40] = 0x00;
    d[41] = 0x81; // 0x80 | positive
    // Scan-mode byte: 0x01 normal, 0x20 AE (SANE), 0x22=4x multisample. Literal
    // `samples` valid only at 1; the example guards >1 (invalid code stalls). THIRD_LIGHT §2.
    d[42] = if ae { 0x20 } else { settings.samples.max(1) };
    d[43] = 0x02; // compression: none
    d[44] = 0x02; // colour interleaving
    d[45] = 0xff; // AE
    d[46..50].copy_from_slice(&exposure.unwrap_or(0).to_be_bytes()); // exposure (0 = IR)

    let mut payload = vec![0u8; 8];
    payload[6..8].copy_from_slice(&(d.len() as u16).to_be_bytes());
    payload.extend_from_slice(&d);
    payload
}

/// The `E0 0xA0` autofocus data-out payload: `00` + focusX (u32 BE) + focusY (u32 BE),
/// 9 bytes (SANE `cs3_autofocus`).
fn autofocus_payload(focus_x: u32, focus_y: u32) -> Vec<u8> {
    let mut p = vec![0u8; 9];
    p[1..5].copy_from_slice(&focus_x.to_be_bytes());
    p[5..9].copy_from_slice(&focus_y.to_be_bytes());
    p
}

/// 135 frame pitch (mm): feed advance between frames.
const FRAME_PITCH_MM: f32 = 38.0;

/// Frame pitch in native pixels: `pitch_mm * BASE_RES / 25.4` = 5984.
// ponytail: calibration knob — residual drift persists at 38.0 mm; null per-scan with
// the subframe offset, or retune FRAME_PITCH_MM per holder.
fn pitch_native() -> u32 {
    (FRAME_PITCH_MM * super::BASE_RES as f32 / 25.4).round() as u32
}

/// Feed-axis offset in mm converted to native pixels.
fn subframe_native(subframe_mm: f32) -> u32 {
    (subframe_mm * super::BASE_RES as f32 / 25.4)
        .round()
        .max(0.0) as u32
}

/// Identity gamma LUT: 16384 BE words `lut[i] = i` (32768 bytes). Hardware-applied;
/// identity = linear pass-through.
fn build_gamma_lut() -> Vec<u8> {
    (0u16..16384).flat_map(|i| i.to_be_bytes()).collect()
}

#[cfg(test)]
mod tests {
    use super::super::AdapterCaps;
    use super::*;
    use crate::scsi::{CommandData, DataDirection, Error, SenseData};

    /// Samples to their big-endian wire bytes (2 bytes each), as the scanner sends.
    fn be_line(samples: &[u16]) -> Vec<u8> {
        samples.iter().flat_map(|s| s.to_be_bytes()).collect()
    }

    /// Scripts the scan exchange: SET WINDOW asserts the scan-mode byte (0x01/0x20) and
    /// per-pass exposure, SCAN returns GOOD, READ returns one padded line until exhausted
    /// (then end-of-data). A channel-selected GET WINDOW serves `measured_exposure`.
    struct ScanMock {
        image: Vec<u8>,
        cursor: usize,
        /// Logical bytes per line; the mock returns one line per READ, padded to
        /// the requested block length, as the scanner does.
        line_len: usize,
        wrote_lut: bool,
        set_windows: usize,
        scans: u32,
        /// Per-channel exposure the real (non-AE) pass must carry in SET WINDOW.
        expect_exposure: [u32; 3],
        /// Firmware-measured exposure served by a channel-selected GET WINDOW;
        /// `None` = no autoexposure in play.
        measured_exposure: Option<[u32; 3]>,
        /// The last `E0 0xA0` autofocus data-out payload seen, if any.
        af_payload: Option<Vec<u8>>,
    }

    impl Transport for ScanMock {
        fn execute(
            &mut self,
            cdb: &[u8],
            _direction: DataDirection,
            data: &mut [u8],
            _sense: &mut [u8],
        ) -> Result<(), Error> {
            match cdb[0] {
                // TUR / RESERVE / RELEASE / SEND DIAG / MODE SELECT / E1 / C1
                0x00 | 0x16 | 0x17 | 0x1D | 0x15 | 0xE1 | 0xC1 => Ok(()),
                // VENDOR E0: capture the autofocus (sub 0xA0) data-out payload.
                0xE0 => {
                    if cdb[2] == 0xA0 {
                        self.af_payload = Some(data.to_vec());
                    }
                    Ok(())
                }
                // INQUIRY / EVPD adapter-detection: any header-sized response.
                0x12 => {
                    data.fill(0);
                    Ok(())
                }
                // WRITE(10) gamma LUT — uploaded before SET WINDOW.
                0x2A => {
                    self.wrote_lut = true;
                    Ok(())
                }
                // SET WINDOW: scan-mode byte 0x01 (normal) or 0x20 (AE pass). Only the
                // real pass (0x01) carries the exposure under test; the AE pass isn't
                // asserted (it just seeds the fixed default).
                0x24 => {
                    let d = &data[8..];
                    assert!(
                        matches!(d[42], 0x01 | 0x20),
                        "SET WINDOW #{} scan-mode byte {:#04x}",
                        self.set_windows,
                        d[42]
                    );
                    if d[42] == 0x01 {
                        let exp = u32::from_be_bytes(d[46..50].try_into().unwrap());
                        let want = self
                            .expect_exposure
                            .get(self.set_windows)
                            .copied()
                            .unwrap_or(0);
                        assert_eq!(exp, want, "SET WINDOW #{} exposure", self.set_windows);
                    }
                    self.set_windows += 1;
                    Ok(())
                }
                // SCAN — the gamma LUT must have been uploaded first. Each SCAN is a
                // fresh pass: rewind the image and the window counter so a strip
                // drains one image per frame.
                0x1B => {
                    assert!(self.wrote_lut, "SCAN issued before the gamma LUT");
                    self.scans += 1;
                    self.cursor = 0;
                    self.set_windows = 0;
                    Ok(())
                }
                // GET WINDOW: 8-byte header (descriptor_length at bytes 6..7) + one
                // 50-byte descriptor. A channel-selected read (Single bit, byte1=0x01)
                // serves the measured exposure at descriptor offset 46..50 (raw 54..58);
                // the generic read-back is 0.
                0x25 => {
                    data.fill(0);
                    if data.len() >= 58 {
                        data[6..8].copy_from_slice(&50u16.to_be_bytes());
                        if cdb[1] == 0x01
                            && let Some(measured) = self.measured_exposure
                        {
                            let v = measured
                                .get((cdb[5] as usize).wrapping_sub(1))
                                .copied()
                                .unwrap_or(0);
                            data[54..58].copy_from_slice(&v.to_be_bytes());
                        }
                    }
                    Ok(())
                }
                0x28 => match cdb[2] {
                    0x00 => {
                        if self.cursor >= self.image.len() {
                            return Err(Error::Status {
                                status: 0x02,
                                sense: Some(SenseData {
                                    key: 0x0b,
                                    asc: 0x3e,
                                    ascq: 0x00,
                                    ili: false,
                                    deferred: false,
                                }),
                            });
                        }
                        // One padded line per read: `line_len` real bytes into the
                        // (larger) block-padded request buffer.
                        let end = (self.cursor + self.line_len).min(self.image.len());
                        let n = end - self.cursor;
                        data[..n].copy_from_slice(&self.image[self.cursor..end]);
                        self.cursor = end;
                        Ok(())
                    }
                    other => panic!("unexpected DTC {other:#04x}"),
                },
                other => panic!("unexpected opcode {other:#04x}"),
            }
        }
    }

    #[test]
    fn scan_decodes_rgb_frame() {
        // dpi 2 -> logical 1x2: two lines of one RGB pixel each. Planar, width 1 ->
        // plane padded to 2 samples: R@0 G@2 B@4 per line, 2 wire bytes/sample.
        let image = [be_line(&[1, 0, 1, 0, 1]), be_line(&[2, 0, 2, 0, 2])].concat();
        let mut s = Ls50::new(ScanMock {
            image,
            cursor: 0,
            line_len: 10, // 5 samples x 2 bytes (last pad sample omitted)
            wrote_lut: false,
            set_windows: 0,
            scans: 0,
            expect_exposure: EXPOSURE_10NS,
            measured_exposure: None,
            af_payload: None,
        });
        let frame = s
            .scan(&ScanSettings {
                dpi: 2,
                infrared: false,
                samples: 1,
                autoexposure: false,
                autofocus: false,
                caps: AdapterCaps::default(),
            })
            .unwrap();
        assert_eq!(frame.rgb.dimensions(), (1, 2));
        assert!(frame.ir.is_none());
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [1u16, 1, 1]);
        assert_eq!(frame.rgb.get_pixel(0, 1).0, [2u16, 2, 2]);
    }

    #[test]
    fn scan_decodes_rgbi_frame() {
        // dpi 2 -> logical 1x2; infrared -> four planes per line: R G B I, each
        // width 1 padded to 2 samples. Last pad sample omitted -> 7 samples/line.
        let image = [
            be_line(&[1, 0, 1, 0, 1, 0, 90]), // line 0 -> rgb (1,1,1), ir 90
            be_line(&[2, 0, 2, 0, 2, 0, 91]), // line 1 -> rgb (2,2,2), ir 91
        ]
        .concat();
        let mut s = Ls50::new(ScanMock {
            image,
            cursor: 0,
            line_len: 14,
            wrote_lut: false,
            set_windows: 0,
            scans: 0,
            expect_exposure: EXPOSURE_10NS,
            measured_exposure: None,
            af_payload: None,
        });
        let frame = s
            .scan(&ScanSettings {
                dpi: 2,
                infrared: true,
                samples: 1,
                autoexposure: false,
                autofocus: false,
                caps: AdapterCaps::default(),
            })
            .unwrap();
        assert_eq!(frame.rgb.dimensions(), (1, 2));
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [1u16, 1, 1]);
        assert_eq!(frame.rgb.get_pixel(0, 1).0, [2u16, 2, 2]);
        let ir = frame.ir.expect("IR plane captured");
        assert_eq!(ir.dimensions(), (1, 2));
        assert_eq!(ir.get_pixel(0, 0).0, [90u16]);
        assert_eq!(ir.get_pixel(0, 1).0, [91u16]);
    }

    #[test]
    fn scan_measures_and_applies_exposure() {
        // AE pre-pass (d[42]=0x20) measures, read back via channel-selected GET WINDOW;
        // the real pass carries the measured values (mock asserts them).
        let measured = [1111u32, 2222, 3333];
        let image = [be_line(&[1, 0, 1, 0, 1]), be_line(&[2, 0, 2, 0, 2])].concat();
        let mut s = Ls50::new(ScanMock {
            image,
            cursor: 0,
            line_len: 10,
            wrote_lut: false,
            set_windows: 0,
            scans: 0,
            expect_exposure: measured,
            measured_exposure: Some(measured),
            af_payload: None,
        });
        let frame = s
            .scan(&ScanSettings {
                dpi: 2,
                infrared: false,
                samples: 1,
                autoexposure: true,
                autofocus: false,
                caps: AdapterCaps::default(),
            })
            .unwrap();
        assert_eq!(frame.rgb.dimensions(), (1, 2));
    }

    #[test]
    fn build_window_descriptor_layout() {
        let settings = ScanSettings {
            dpi: 300,
            infrared: false,
            samples: 1,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        let payload = build_window(&settings, 0x01, Some(120_000), 0, false);
        assert_eq!(payload.len(), 58); // 8-byte header + 50-byte descriptor
        assert_eq!(
            &payload[..8],
            &[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32]
        );
        let d = &payload[8..];
        let (native_w, native_h) = settings.native_dims();
        assert_eq!(d[0], 0x01); // channel
        assert_eq!(d[1], 0x00);
        assert_eq!(u16::from_be_bytes([d[2], d[3]]), settings.res());
        assert_eq!(u16::from_be_bytes([d[4], d[5]]), settings.res());
        assert_eq!(&d[6..14], &[0u8; 8]); // x/y offset = full-frame origin
        assert_eq!(u32::from_be_bytes(d[14..18].try_into().unwrap()), native_w);
        assert_eq!(u32::from_be_bytes(d[18..22].try_into().unwrap()), native_h);
        assert_eq!(d[25], 0x05); // RGB
        assert_eq!(d[26], 0x0E); // 14-bit
        assert_eq!(d[40], 0x00);
        assert_eq!(d[41], 0x81); // 0x80 | positive
        assert_eq!(d[42], 0x01); // 1 sample
        assert_eq!(&d[43..46], &[0x02, 0x02, 0xff]);
        assert_eq!(u32::from_be_bytes(d[46..50].try_into().unwrap()), 120_000);

        // IR / no exposure -> zeroed exposure field.
        let ir = build_window(&settings, IR_CHANNEL, None, 0, false);
        assert_eq!(&ir[8..][46..50], &[0u8; 4]);
    }

    #[test]
    fn build_window_ae_mode() {
        // AE pass sets the scan-mode byte to 0x20; normal is 0x01.
        let settings = ScanSettings {
            dpi: 300,
            infrared: false,
            samples: 1,
            autoexposure: true,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        let d = &build_window(&settings, 0x01, Some(120_000), 0, true)[8..];
        assert_eq!(d[42], 0x20);
        let normal = &build_window(&settings, 0x01, Some(120_000), 0, false)[8..];
        assert_eq!(normal[42], 0x01);
    }

    #[test]
    fn build_window_encodes_multisample() {
        // Multi-sample count is a literal N at d[42]; the single-sample bytes around
        // it stay put.
        let settings = ScanSettings {
            dpi: 300,
            infrared: false,
            samples: 4,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        let d = &build_window(&settings, 0x01, Some(120_000), 0, false)[8..];
        assert_eq!(d[42], 0x04); // literal 4x
        assert_eq!(d[40], 0x00);
        assert_eq!(d[43], 0x02);
    }

    #[test]
    fn pitch_and_subframe_native() {
        // 38.0 mm at 4000 native dpi: 38.0 * 4000 / 25.4 = 5984.25 -> 5984.
        assert_eq!(pitch_native(), 5984);
        assert_eq!(subframe_native(0.0), 0);
        assert_eq!(subframe_native(1.0), 157); // ~1 mm -> 157 native px
    }

    #[test]
    fn build_window_writes_y_offset() {
        let settings = ScanSettings {
            dpi: 300,
            infrared: false,
            samples: 1,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        let payload = build_window(&settings, 0x01, Some(120_000), 5984, false);
        let d = &payload[8..];
        assert_eq!(&d[6..10], &[0u8; 4]); // x offset stays at the origin
        assert_eq!(u32::from_be_bytes(d[10..14].try_into().unwrap()), 5984); // y offset
    }

    #[test]
    fn boundary_cmd_declares_all_frames() {
        struct NullTransport;
        impl Transport for NullTransport {
            fn execute(
                &mut self,
                _: &[u8],
                _: DataDirection,
                _: &mut [u8],
                _: &mut [u8],
            ) -> Result<(), Error> {
                unreachable!("boundary_cmd builds a CDB without touching the transport")
            }
        }
        let s = Ls50::new(NullTransport);
        let settings = ScanSettings {
            dpi: 4000,
            infrared: false,
            samples: 1,
            autoexposure: false,
            autofocus: false,
            caps: AdapterCaps::default(),
        };
        let pitch = pitch_native();

        let descriptor = |i: u32, subframe: u32| {
            let ystart = i * pitch + subframe;
            let mut d = Vec::new();
            d.extend_from_slice(&ystart.to_be_bytes()); // Ystart
            d.extend_from_slice(&0u32.to_be_bytes()); // Xstart
            d.extend_from_slice(&(ystart + pitch - 1).to_be_bytes()); // Yend
            d.extend_from_slice(&3944u32.to_be_bytes()); // Xend = native_x - 1 (3945 - 1)
            d
        };

        // Single frame: header len = 4 + 16 = 20 (0x14), count 1, one descriptor.
        let mut one = vec![0x00, 0x14, 0x01, 0x01];
        one.extend(descriptor(0, 0));
        assert!(
            matches!(s.boundary_cmd(&settings, 1, 0).data(), CommandData::Write(p) if p == one)
        );

        // Six frames: len = 4 + 96 = 100 (0x64), count 6, six descriptors one pitch
        // apart, all shifted by the subframe offset.
        let mut six = vec![0x00, 0x64, 0x06, 0x06];
        for i in 0..6 {
            six.extend(descriptor(i, 7));
        }
        assert!(
            matches!(s.boundary_cmd(&settings, 6, 7).data(), CommandData::Write(p) if p == six)
        );
    }

    #[test]
    fn eject_uses_e0_d0_command() {
        // E0 sub 0xD0 + 13 zero bytes (then C1). The load sub 0xD1 is rejected
        // 05/24 on this unit, so only eject has a working E0 form.
        let cmd = VendorE0::new(0xD0, vec![0u8; 13]);
        assert_eq!(
            cmd.cdb().0,
            [0xE0, 0x00, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x00]
        );
        assert!(matches!(cmd.data(), CommandData::Write(p) if p == [0u8; 13]));
    }

    #[test]
    fn scan_strip_yields_a_frame_per_request() {
        // Each SCAN rewinds the mock image, so a 3-frame strip decodes the same
        // 1x2 frame three times.
        let image = [be_line(&[1, 0, 1, 0, 1]), be_line(&[2, 0, 2, 0, 2])].concat();
        let mut s = Ls50::new(ScanMock {
            image,
            cursor: 0,
            line_len: 10,
            wrote_lut: false,
            set_windows: 0,
            scans: 0,
            expect_exposure: EXPOSURE_10NS,
            measured_exposure: None,
            af_payload: None,
        });
        let frames = s
            .scan_strip(
                &ScanSettings {
                    dpi: 2,
                    infrared: false,
                    samples: 1,
                    autoexposure: false,
                    autofocus: false,
                    caps: AdapterCaps::default(),
                },
                3,
                0.0,
            )
            .unwrap();
        assert_eq!(frames.len(), 3);
        for f in &frames {
            assert_eq!(f.rgb.dimensions(), (1, 2));
            assert_eq!(f.rgb.get_pixel(0, 0).0, [1u16, 1, 1]);
            assert_eq!(f.rgb.get_pixel(0, 1).0, [2u16, 2, 2]);
        }
    }

    #[test]
    fn autofocus_payload_layout() {
        // 00 + focusX u32 BE + focusY u32 BE, 9 bytes (SANE cs3_autofocus).
        let p = autofocus_payload(0x0000_07B4, 0x0000_0BA3);
        assert_eq!(p, [0x00, 0x00, 0x00, 0x07, 0xB4, 0x00, 0x00, 0x0B, 0xA3]);
    }

    #[test]
    fn scan_autofocus_targets_frame_centre() {
        // dpi 2 -> native (2000, 4000); centre = (width/2, yoffset + height/2)
        // = (1000, 2000). Autofocus issues E0 0xA0 with that centre on the real pass.
        let image = [be_line(&[1, 0, 1, 0, 1]), be_line(&[2, 0, 2, 0, 2])].concat();
        let mut s = Ls50::new(ScanMock {
            image,
            cursor: 0,
            line_len: 10,
            wrote_lut: false,
            set_windows: 0,
            scans: 0,
            expect_exposure: EXPOSURE_10NS,
            measured_exposure: None,
            af_payload: None,
        });
        s.scan(&ScanSettings {
            dpi: 2,
            infrared: false,
            samples: 1,
            autoexposure: false,
            autofocus: true,
            caps: AdapterCaps::default(),
        })
        .unwrap();
        assert_eq!(
            s.transport().af_payload,
            Some(autofocus_payload(1000, 2000))
        );
    }
}
