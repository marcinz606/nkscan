//! TIFF output for scanned frames. Device-neutral: both drivers alias their image
//! buffers to the same concrete types, so this takes the buffers, not a named `Frame`.

use image::{ImageBuffer, ImageFormat, Luma, Rgb};
use std::{
    error::Error,
    fs::File,
    io::BufWriter,
    path::{Path, PathBuf},
};

pub type Image = ImageBuffer<Rgb<u16>, Vec<u16>>;
pub type GrayImage = ImageBuffer<Luma<u16>, Vec<u16>>;

/// Write each frame's RGB TIFF. With `numbered`, one file per frame as `OUT_00.tiff`,
/// `OUT_01.tiff`, …; otherwise all to `output` (single-frame use).
pub fn write_frames(
    frames: &[(Image, Option<GrayImage>)],
    output: &Path,
    numbered: bool,
) -> Result<(), Box<dyn Error>> {
    for (k, (rgb, ir)) in frames.iter().enumerate() {
        let path = if numbered {
            suffix(output, &format!("_{k:02}"))
        } else {
            output.to_path_buf()
        };
        write_frame(rgb, ir.as_ref(), &path)?;
    }
    Ok(())
}

/// Write `rgb` as a TIFF at `path`; when `ir` is present, write it beside as `*_ir.<ext>`.
fn write_frame(rgb: &Image, ir: Option<&GrayImage>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut w = BufWriter::new(File::create(path)?);
    rgb.write_to(&mut w, ImageFormat::Tiff)?;
    println!(
        "wrote {} ({}x{})",
        path.display(),
        rgb.width(),
        rgb.height()
    );

    if let Some(ir) = ir {
        let ir_path = suffix(path, "_ir");
        let mut w = BufWriter::new(File::create(&ir_path)?);
        ir.write_to(&mut w, ImageFormat::Tiff)?;
        println!("wrote {}", ir_path.display());
    }
    Ok(())
}

/// Insert `s` before the file extension: `scan.tiff` + `_ir` -> `scan_ir.tiff`. No
/// extension appends: `scan` + `_ir` -> `scan_ir`. Directory components are preserved.
fn suffix(path: &Path, s: &str) -> PathBuf {
    match path.extension() {
        Some(ext) => {
            let mut name = path.file_stem().unwrap_or_default().to_os_string();
            name.push(s);
            name.push(".");
            name.push(ext);
            path.with_file_name(name)
        }
        None => {
            let mut name = path.as_os_str().to_os_string();
            name.push(s);
            PathBuf::from(name)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::suffix;
    use std::path::Path;

    #[test]
    fn suffix_goes_before_extension() {
        assert_eq!(
            suffix(Path::new("scan.tiff"), "_ir"),
            Path::new("scan_ir.tiff")
        );
        assert_eq!(
            suffix(Path::new("scan.tiff"), "_00"),
            Path::new("scan_00.tiff")
        );
    }

    #[test]
    fn suffix_preserves_directory() {
        assert_eq!(
            suffix(Path::new("out/scan.tiff"), "_00"),
            Path::new("out/scan_00.tiff")
        );
    }

    #[test]
    fn suffix_without_extension_appends() {
        assert_eq!(suffix(Path::new("scan"), "_ir"), Path::new("scan_ir"));
    }
}
