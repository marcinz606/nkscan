//! Frame types and the streaming-decode interface shared by both scanners.
//!
//! Wire layout differs per model (LS-50 planar, LS-9000 interleave-block), but
//! both stream `READ(10)` payloads the same way — push in order, then finish.
//! Concrete decoders: `PlanarDecoder`, `InterleaveDecoder`.

use image::{ImageBuffer, Luma, Rgb};

// Output image types. Both scanners send BE u16 over the wire.
pub type Image = ImageBuffer<Rgb<u16>, Vec<u16>>;
pub type IrMask = ImageBuffer<Luma<u16>, Vec<u16>>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("scan window does not divide evenly at this resolution")]
    IndivisibleWindow,

    #[error(
        "stage extent gives {stages} positions, not a multiple of the {block}-position CCD block"
    )]
    UnalignedStageExtent { stages: u32, block: u32 },

    #[error("received {got} bytes, expected {expected}")]
    LengthMismatch { got: u64, expected: u64 },
}

/// A decoded frame that borrows the decoder's buffers.
pub struct FrameView<'a> {
    /// The image data read out from the scanner.
    pub rgb: ImageBuffer<Rgb<u16>, &'a [u16]>,
    /// The optional IR mask for dust removal.
    pub ir: Option<ImageBuffer<Luma<u16>, &'a [u16]>>,
}

impl FrameView<'_> {
    /// Copy into owned buffers, so the frame outlives the decoder's reuse.
    pub fn to_owned(&self) -> Frame {
        Frame {
            rgb: Image::from_raw(self.rgb.width(), self.rgb.height(), self.rgb.to_vec())
                .expect("view is well formed"),
            ir: self.ir.as_ref().map(|ir| {
                IrMask::from_raw(ir.width(), ir.height(), ir.to_vec()).expect("view is well formed")
            }),
        }
    }
}

/// An owned decoded frame.
pub struct Frame {
    /// The image data read out from the scanner.
    pub rgb: Image,
    /// The optional IR mask for dust removal.
    pub ir: Option<IrMask>,
}

/// Feed `READ(10)` payloads to [`push`](Self::push) in order, then [`finish`](Self::finish).
pub trait FrameDecoder {
    fn push(&mut self, bytes: &[u8]) -> Result<(), Error>;

    /// Borrows the decoder's buffers — only valid until the next push/reset.
    /// [`FrameView::to_owned`] to keep it.
    fn finish(&mut self) -> Result<FrameView<'_>, Error>;

    /// Reuse the buffers for the next frame.
    fn reset(&mut self);
}

/// Read one big-endian sample (`i` in samples; 2 wire bytes each).
#[inline(always)]
pub fn sample_at(buf: &[u8], i: usize) -> u16 {
    u16::from_be_bytes([buf[2 * i], buf[2 * i + 1]])
}
