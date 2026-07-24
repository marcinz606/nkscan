//! Decode a captured raw scan stream into a TIFF, to check the decoder against known-good captures without hardware.

use clap::Parser;
use image::ImageFormat;
use nkscan::scanners::ls9000ed::{
    CcdMode, Dpi, Multisample, ScanSettings, Window,
    decode::{FrameDecoder, FrameView, InterleaveDecoder},
};
use std::{
    fs::File,
    io::{BufRead, BufReader, BufWriter},
    path::{Path, PathBuf},
};

#[derive(Parser, Debug)]
struct Args {
    /// Input raw file of all the bytes from the scanner, in order as read
    raw_file: PathBuf,
    /// Output image path
    out_file: PathBuf,
    /// Whether this file has IR data
    #[arg(long)]
    ir: bool,
    /// Mutlisample factor
    #[arg(long)]
    multisample: u8,
    /// DPI
    #[arg(long)]
    dpi: u16,
    /// Single-line mode
    #[arg(long)]
    single_line: bool,
    /// x-size in pixels (along sensor plane)
    #[arg(long)]
    x: u32,
    /// y-size in pixels (along motor plane)
    #[arg(long)]
    y: u32,
}

fn main() {
    // Build the settings block
    let cli = Args::parse();

    let dpi = match cli.dpi {
        4000 => Dpi::_4000,
        2000 => Dpi::_2000,
        1333 => Dpi::_1333,
        333 => Dpi::_333,
        _ => panic!("not a valid DPI"),
    };

    let settings = ScanSettings {
        ccd_mode: if cli.single_line {
            CcdMode::SingleLine
        } else {
            CcdMode::ThreeLine
        },
        ir: cli.ir,
        dpi,
        multisample: match cli.multisample {
            1 => Multisample::X1,
            2 => Multisample::X2,
            4 => Multisample::X4,
            8 => Multisample::X8,
            16 => Multisample::X16,
            _ => panic!("not a valid multisample factor"),
        },
        // The position of the window here doesn't really matter, just the size
        window: Window::centred(0, cli.x * dpi.divisor(), cli.y * dpi.divisor()),
    };

    // Setup the decoder state
    let mut dec = InterleaveDecoder::new(&settings).expect("layout");
    let (w, h) = settings.output_dims().unwrap();
    println!(
        "{}x{}  stages={:?} readouts={} block={}  expect {:?} bytes",
        w,
        h,
        settings.stages(),
        settings.readouts(),
        settings.ccd_block(),
        settings.expected_bytes()
    );

    // Stream the raw file through the decoder
    let file = File::open(&cli.raw_file).expect("open raw");
    // Chunk by 1MB. NOTE: The scanner will be in different chunk sizes
    let mut reader = BufReader::with_capacity(1 << 20, file);
    loop {
        let chunk = reader.fill_buf().expect("read raw");
        if chunk.is_empty() {
            break;
        }
        let n = chunk.len();
        dec.push(chunk).expect("push");
        reader.consume(n);
    }

    let frame = dec.finish().expect("finish");
    write_tiff(&frame, &cli.out_file);
    println!("wrote {}", cli.out_file.display());
}

/// Encode a frame through a `BufWriter` so the TIFF encoder's many small writes
/// don't each become a syscall. The IR mask, if present, goes beside the RGB
/// output with an `_ir` suffix.
fn write_tiff(frame: &FrameView, out: &Path) {
    let mut w = BufWriter::new(File::create(out).expect("create output"));
    frame
        .rgb
        .write_to(&mut w, ImageFormat::Tiff)
        .expect("encode rgb");

    if let Some(ir) = &frame.ir {
        let ir_path = out.with_extension("");
        let ir_path = PathBuf::from(format!("{}_ir.tiff", ir_path.display()));
        let mut w = BufWriter::new(File::create(ir_path).expect("create ir output"));
        ir.write_to(&mut w, ImageFormat::Tiff).expect("encode ir");
    }
}
