//! Decoding the LS-9000's raw scan stream into an image.
//!
//! Interleave-block layout. The shared frame types and the streaming interface
//! live in [`crate::scanners::decode`].

use super::ScanSettings;
use crate::scanners::decode::sample_at;
use image::ImageBuffer;

pub use crate::scanners::decode::{Error, Frame, FrameDecoder, FrameView, Image, IrMask};

/// Sensor pixels processed per inner tile, chosen so both the input runs and the output tile stay in L2 during the transpose
const CHUNK: usize = 256;

/// Streaming decoder.
/// Feed `READ(10)` payloads to [`push`](FrameDecoder::push) in arrival order, then call [`finish`](FrameDecoder::finish)
///
/// A *sample* is one 16-bit value: one channel, at one sensor pixel, from one CCD line, in one readout
pub struct InterleaveDecoder {
    // --- output geometry ---
    /// Output columns (stage positions x CCD lines)
    width: usize,
    /// Output rows (active sensor pixels)
    height: usize,

    // --- resolved acquisition parameters ---
    /// CCD lines per readout: 3, or 1 in single-line mode
    lines: usize,
    /// Stage positions per interleave block; equivalently the CCD line spacing in output columns (`N = 12/k`). 1 in single-line mode
    block: usize,
    /// Multi-sample repeats per stage position
    multisample: usize,
    /// Whether an infrared readout is present
    ir: bool,
    /// Samples in one readout: one sweep of the sensor bar across all lines
    readout_samples: usize,
    /// Samples per stage position (`readouts * readout_samples`)
    stage_stride: usize,
    /// Total stream length in bytes; the transfer is complete at this count
    expected: u64,

    // --- unscrambled output ---
    rgb: Vec<u16>,
    ir_plane: Vec<u16>,

    // --- streaming state ---
    /// One interleave block of raw bytes, refilled as the stream arrives.
    staging: Vec<u8>,
    /// Bytes currently in `staging`.
    filled: usize,
    /// Blocks emitted so far; the left edge of the current output strip.
    block_index: usize,
    /// Bytes received across all `push` calls.
    received: u64,
}

impl InterleaveDecoder {
    pub fn new(settings: &ScanSettings) -> Result<Self, Error> {
        let (width, height) = settings.output_dims().ok_or(Error::IndivisibleWindow)?;
        let (stages, block) = (
            settings.stages().ok_or(Error::IndivisibleWindow)?,
            settings.ccd_block(),
        );
        if stages % block != 0 {
            return Err(Error::UnalignedStageExtent { stages, block });
        }

        let (width, height) = (width as usize, height as usize);
        let lines = settings.lines() as usize;
        let readout_samples = height * lines;
        let stage_stride = settings.readouts() as usize * readout_samples;
        let px = width * height;

        Ok(Self {
            width,
            height,
            lines,
            block: block as usize,
            multisample: settings.multisample.count() as usize,
            ir: settings.ir,
            readout_samples,
            stage_stride,
            expected: settings.expected_bytes().ok_or(Error::IndivisibleWindow)?,
            rgb: vec![0; px * 3],
            ir_plane: if settings.ir { vec![0; px] } else { Vec::new() },
            staging: vec![0; block as usize * stage_stride * 2],
            filled: 0,
            block_index: 0,
            received: 0,
        })
    }

    /// Which readout slot holds channel `c` on repeat `s`.
    ///
    /// Channels are `0=R, 1=G, 2=B, 3=IR`. Infrared exists only on repeat 0.
    #[inline]
    fn readout_of(&self, c: usize, s: usize) -> usize {
        if c >= 3 {
            3
        } else if s == 0 {
            c
        } else {
            3 + usize::from(self.ir) + (s - 1) * 3 + c
        }
    }

    /// Transpose the freshly filled block into its output strip.
    ///
    /// One block covers `self.block` stage positions, which in three-line mode
    /// map to a contiguous run of `self.block * lines` output columns.
    /// Iterating column-outer, sensor-inner keeps a chunk of the output column
    /// in cache while the input is read sequentially down the bar.
    fn emit(&mut self) {
        let first_col = self.block_index * self.block * self.lines;
        let strip_cols = self.block * self.lines;
        let rsamp = self.readout_samples;

        let mut p0 = 0;
        while p0 < self.height {
            let p_end = (p0 + CHUNK).min(self.height);

            for col in 0..strip_cols {
                // A block's columns run [line 0 x N][line 1 x N][line 2 x N],
                // so the strip column splits into a stage position and a line.
                let (stage, line) = if self.lines == 3 {
                    (col % self.block, col / self.block)
                } else {
                    (col, 0)
                };
                let x = first_col + col;
                // Invariant across the whole sensor sweep for this column.
                let col_base = stage * self.stage_stride + line;

                for p in p0..p_end {
                    // The sensor bar reads out opposite to increasing y.
                    let y = self.height - 1 - p;
                    let out3 = (y * self.width + x) * 3;
                    // Readout 0, channel 0 (= red) of this pixel; other
                    // readouts follow at multiples of `rsamp`.
                    let base = col_base + p * self.lines;

                    // Gather the pixel into a stack triple, then write it in one shot
                    // RGB is interleaved in the output, so the triple is contiguous and this is a single bounds check.
                    let rgb = if self.multisample == 1 {
                        // Readout slot for channel c is just c.
                        [
                            sample_at(&self.staging, base),
                            sample_at(&self.staging, base + rsamp),
                            sample_at(&self.staging, base + 2 * rsamp),
                        ]
                    } else {
                        let m = self.multisample as u32;
                        let mut t = [0u16; 3];
                        for (channel, out) in t.iter_mut().enumerate() {
                            let mut acc = 0u32;
                            for rep in 0..self.multisample {
                                let idx = base + self.readout_of(channel, rep) * rsamp;
                                acc += u32::from(sample_at(&self.staging, idx));
                            }
                            *out = (acc / m) as u16;
                        }
                        t
                    };
                    self.rgb[out3..out3 + 3].copy_from_slice(&rgb);

                    if self.ir {
                        // IR is readout slot 3, present only on repeat 0.
                        self.ir_plane[y * self.width + x] =
                            sample_at(&self.staging, base + 3 * rsamp);
                    }
                }
            }
            p0 = p_end;
        }
    }
}

impl FrameDecoder for InterleaveDecoder {
    fn push(&mut self, mut bytes: &[u8]) -> Result<(), Error> {
        self.received += bytes.len() as u64;
        if self.received > self.expected {
            return Err(Error::LengthMismatch {
                got: self.received,
                expected: self.expected,
            });
        }
        while !bytes.is_empty() {
            let take = (self.staging.len() - self.filled).min(bytes.len());
            self.staging[self.filled..self.filled + take].copy_from_slice(&bytes[..take]);
            self.filled += take;
            bytes = &bytes[take..];
            if self.filled == self.staging.len() {
                self.emit();
                self.filled = 0;
                self.block_index += 1;
            }
        }
        Ok(())
    }

    /// `new()` guarantees the stage count is a whole number of blocks, so there
    /// is never a trailing partial block to flush here.
    fn finish(&mut self) -> Result<FrameView<'_>, Error> {
        if self.received != self.expected {
            return Err(Error::LengthMismatch {
                got: self.received,
                expected: self.expected,
            });
        }
        let (w, h) = (self.width as u32, self.height as u32);
        Ok(FrameView {
            rgb: ImageBuffer::from_raw(w, h, self.rgb.as_slice()).expect("buffer sized in new"),
            ir: self.ir.then(|| {
                ImageBuffer::from_raw(w, h, self.ir_plane.as_slice()).expect("sized in new")
            }),
        })
    }

    fn reset(&mut self) {
        self.filled = 0;
        self.block_index = 0;
        self.received = 0;
    }
}
