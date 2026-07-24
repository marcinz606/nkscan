//! Decode the LS-50 scan stream into an image.
//!
//! **Planar**, line-by-line: each line is one plane per channel back to back —
//! `R[plane] G[plane] B[plane]` (`… I` with infrared), top line first. Samples are
//! big-endian 16-bit; each plane is `width` samples padded even; the line is
//! block-padded to a 512 multiple. Decode = per-line de-interleave.
//!
//! [`PlanarDecoder`] streams it: feed each `READ(10)` payload to
//! [`push`](FrameDecoder::push) in arrival order, then [`finish`](FrameDecoder::finish).
//! Shared frame types and the interface live in [`crate::scanners::decode`].

use super::ScanSettings;
use crate::scanners::decode::sample_at;
use image::ImageBuffer;

pub use crate::scanners::decode::{Error, Frame, FrameDecoder, FrameView, Image, IrMask};

/// Streaming decoder.
///
/// Feed `READ(10)` payloads to [`push`](FrameDecoder::push) in arrival order, then
/// call [`finish`](FrameDecoder::finish).
pub struct PlanarDecoder {
    /// Output columns (image width in pixels).
    width: usize,
    /// Colour planes per line: 3 (RGB) or 4 (RGB + infrared).
    n_colors: usize,
    /// One colour plane's width in samples: `width` padded to an even count.
    plane: usize,
    /// On-wire bytes per line: the planes then block padding to a 512 multiple.
    stride: usize,
    /// Total frame bytes (`stride * height`); transfer completes at or before this.
    expected: u64,

    /// De-interleaved RGB output, `width * height * 3` samples, top row first.
    rgb: Vec<u16>,
    /// De-interleaved IR output, `width * height` samples; empty when RGB-only.
    ir_plane: Vec<u16>,

    /// One line of raw bytes, refilled as the stream arrives.
    staging: Vec<u8>,
    /// Bytes currently in `staging`.
    filled: usize,
    /// Complete lines emitted so far; the current output row.
    line_index: usize,
    /// Bytes received across all `push` calls.
    received: u64,
}

impl PlanarDecoder {
    /// Decoder sized for `settings`' output geometry.
    pub fn new(settings: &ScanSettings) -> Result<Self, Error> {
        let (width, height) = settings.output_dims();
        Ok(Self::from_dims(
            width as usize,
            height as usize,
            settings.n_colors(),
            settings.bytes_per_line(),
        ))
    }

    /// Decoder from explicit dims/channels/stride; `new` derives these from geometry,
    /// tests call directly.
    fn from_dims(width: usize, height: usize, n_colors: usize, bytes_per_line: usize) -> Self {
        Self {
            width,
            n_colors,
            plane: width + (width & 1),
            stride: bytes_per_line,
            expected: bytes_per_line as u64 * height as u64,
            rgb: vec![0; width * height * 3],
            ir_plane: if n_colors == 4 {
                vec![0; width * height]
            } else {
                Vec::new()
            },
            staging: vec![0; bytes_per_line],
            filled: 0,
            line_index: 0,
            received: 0,
        }
    }

    /// De-interleave the staged line's planes into the current output row:
    /// planes 0/1/2 into RGB, plane 3 into the IR mask when present.
    fn emit(&mut self) {
        let rgb_base = self.line_index * self.width * 3;
        let (r, g, b) = (0, self.plane, 2 * self.plane);
        for x in 0..self.width {
            self.rgb[rgb_base + x * 3] = sample_at(&self.staging, r + x);
            self.rgb[rgb_base + x * 3 + 1] = sample_at(&self.staging, g + x);
            self.rgb[rgb_base + x * 3 + 2] = sample_at(&self.staging, b + x);
        }
        if self.n_colors == 4 {
            let ir_base = self.line_index * self.width;
            let i = 3 * self.plane;
            for x in 0..self.width {
                self.ir_plane[ir_base + x] = sample_at(&self.staging, i + x);
            }
        }
    }
}

impl FrameDecoder for PlanarDecoder {
    fn push(&mut self, mut bytes: &[u8]) -> Result<(), Error> {
        self.received += bytes.len() as u64;
        if self.received > self.expected {
            return Err(Error::LengthMismatch {
                got: self.received,
                expected: self.expected,
            });
        }
        while !bytes.is_empty() {
            let take = (self.stride - self.filled).min(bytes.len());
            self.staging[self.filled..self.filled + take].copy_from_slice(&bytes[..take]);
            self.filled += take;
            bytes = &bytes[take..];
            if self.filled == self.stride {
                self.emit();
                self.filled = 0;
                self.line_index += 1;
            }
        }
        Ok(())
    }

    /// Lenient on a short stream: the read loop stops on end-of-data, so the frame is
    /// the lines that arrived (a trailing partial line in `staging` is dropped).
    fn finish(&mut self) -> Result<FrameView<'_>, Error> {
        let rows = self.line_index as u32;
        let rgb = ImageBuffer::from_raw(
            self.width as u32,
            rows,
            &self.rgb[..self.line_index * self.width * 3],
        )
        .expect("buffer sized in new");
        let ir = (self.n_colors == 4).then(|| {
            ImageBuffer::from_raw(
                self.width as u32,
                rows,
                &self.ir_plane[..self.line_index * self.width],
            )
            .expect("buffer sized in new")
        });
        Ok(FrameView { rgb, ir })
    }

    fn reset(&mut self) {
        self.filled = 0;
        self.line_index = 0;
        self.received = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Samples to their big-endian wire bytes (2 bytes each), as the scanner sends.
    fn be(samples: &[u16]) -> Vec<u8> {
        samples.iter().flat_map(|s| s.to_be_bytes()).collect()
    }

    /// Push `data` as one whole stream, then finish.
    fn decode(data: &[u8], width: usize, height: usize, n_colors: usize, stride: usize) -> Frame {
        let mut dec = PlanarDecoder::from_dims(width, height, n_colors, stride);
        dec.push(data).unwrap();
        dec.finish().unwrap().to_owned()
    }

    #[test]
    fn deinterleaves_planar_line() {
        // 2x2, three planes back to back per line: R[..] G[..] B[..] (width even).
        // Six samples/line at 2 bytes each -> stride 12.
        let data = be(&[
            10, 11, 20, 21, 30, 31, // line 0: R[10,11] G[20,21] B[30,31]
            40, 41, 50, 51, 60, 61, // line 1
        ]);
        let frame = decode(&data, 2, 2, 3, 12);
        assert!(frame.ir.is_none());
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [10u16, 20, 30]);
        assert_eq!(frame.rgb.get_pixel(1, 0).0, [11u16, 21, 31]);
        assert_eq!(frame.rgb.get_pixel(0, 1).0, [40u16, 50, 60]);
        assert_eq!(frame.rgb.get_pixel(1, 1).0, [41u16, 51, 61]);
    }

    #[test]
    fn reads_samples_big_endian() {
        // A sample above 255 must decode from both wire bytes, not just the low one.
        // width 1 -> planes padded to 2 samples: R@0 G@2 B@4.
        let data = be(&[0x0102, 0, 0x0304, 0, 0x0506, 0]);
        let frame = decode(&data, 1, 1, 3, 12);
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [0x0102u16, 0x0304, 0x0506]);
    }

    #[test]
    fn splits_rgb_and_ir_planes() {
        // 2x2, four planes per line: R G B I. The 4th plane is the IR mask.
        // Eight samples/line -> stride 16.
        let data = be(&[
            10, 11, 20, 21, 30, 31, 90, 91, // line 0: R G B I
            40, 41, 50, 51, 60, 61, 92, 93, // line 1
        ]);
        let frame = decode(&data, 2, 2, 4, 16);
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [10u16, 20, 30]);
        assert_eq!(frame.rgb.get_pixel(1, 1).0, [41u16, 51, 61]);
        let ir = frame.ir.expect("IR plane present");
        assert_eq!(ir.dimensions(), (2, 2));
        assert_eq!(ir.get_pixel(0, 0).0, [90u16]);
        assert_eq!(ir.get_pixel(1, 0).0, [91u16]);
        assert_eq!(ir.get_pixel(0, 1).0, [92u16]);
        assert_eq!(ir.get_pixel(1, 1).0, [93u16]);
    }

    #[test]
    fn pads_odd_width_planes_to_even() {
        // width 1 -> each plane is 2 samples (1 sample + 1 pad); the pad is skipped.
        // Six samples/line -> stride 12.
        let data = be(&[
            9, 0xAAAA, 8, 0xBBBB, 7, 0xCCCC, // line 0: R[9,_] G[8,_] B[7,_]
            1, 0xA0A0, 2, 0xB0B0, 3, 0xC0C0, // line 1
        ]);
        let frame = decode(&data, 1, 2, 3, 12);
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [9u16, 8, 7]);
        assert_eq!(frame.rgb.get_pixel(0, 1).0, [1u16, 2, 3]);
    }

    #[test]
    fn drops_block_padding() {
        // 2x1: planes need 6 samples (12 bytes), stride 16 -> 2 trailing pad samples.
        let data = be(&[5, 6, 15, 16, 25, 26, 0xEEEE, 0xFFFF]);
        let frame = decode(&data, 2, 1, 3, 16);
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [5u16, 15, 25]);
        assert_eq!(frame.rgb.get_pixel(1, 0).0, [6u16, 16, 26]);
    }

    #[test]
    fn reassembles_across_split_pushes() {
        // A line split mid-sample over two pushes still emits once the stride fills.
        let mut dec = PlanarDecoder::from_dims(2, 1, 3, 12);
        let line = be(&[5, 6, 15, 16, 25, 26]);
        dec.push(&line[..5]).unwrap(); // split on an odd byte (mid-sample)
        dec.push(&line[5..]).unwrap();
        let frame = dec.finish().unwrap().to_owned();
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [5u16, 15, 25]);
        assert_eq!(frame.rgb.get_pixel(1, 0).0, [6u16, 16, 26]);
    }

    #[test]
    fn short_stream_yields_only_the_lines_received() {
        // Height 2 declared, only line 0 pushed -> a 1-row frame, not an error.
        let mut dec = PlanarDecoder::from_dims(2, 2, 3, 12);
        dec.push(&be(&[5, 6, 15, 16, 25, 26])).unwrap();
        let frame = dec.finish().unwrap().to_owned();
        assert_eq!(frame.rgb.dimensions(), (2, 1));
        assert_eq!(frame.rgb.get_pixel(0, 0).0, [5u16, 15, 25]);
    }

    #[test]
    fn rejects_overlong_stream() {
        let mut dec = PlanarDecoder::from_dims(2, 1, 3, 12);
        assert!(matches!(
            dec.push(&[0; 13]),
            Err(Error::LengthMismatch { .. })
        ));
    }
}
