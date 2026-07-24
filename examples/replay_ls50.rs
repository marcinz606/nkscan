//! Replay the coolscanpy LS-5000 arming plan against a real LS-50, to prove the
//! sequence ports across the shared LS5000.md3 driver family.
//!
//! `cargo run --example replay_ls50 -- <plan.jsonl> [max_seq] [out.tiff]`
//!
//! Sends each plan CDB verbatim (data-out / data-in per `expected_phase`),
//! tolerating sense diffs (the LS-50 differs from the LS-5000 on identity and
//! timing), and logs actual sense vs the plan's `expected_sense`. Image data
//! (READ DTC 0x00) is collected; if any arrived and `out.tiff` is given, it is
//! written as a raw-stride TIFF for a first look.
//!
//! Defaults to seq ≤ 98 (through the first SCAN → GOOD) — the arming proof.

use image::{ImageBuffer, ImageFormat, Rgb};
use nkscan::scsi::{DataDirection, Error, Transport, usb::UsbTransport};
use std::{env, fs, io::BufWriter};

const LS50_VID: u16 = 0x04B0;
const LS50_PID: u16 = 0x4001;

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex"))
        .collect()
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn")),
        )
        .init();

    let mut args = env::args().skip(1);
    let plan_path = args
        .next()
        .expect("usage: replay_ls50 <plan.jsonl> [max_seq] [out.tiff]");
    let max_seq: u64 = args
        .next()
        .map(|s| s.parse().expect("max_seq"))
        .unwrap_or(98);
    let out_tiff = args.next();

    let plan = fs::read_to_string(&plan_path).expect("read plan");
    let mut transport = UsbTransport::open(LS50_VID, LS50_PID).expect("open LS-50");

    let mut image: Vec<u8> = Vec::new();
    let mut stride = 0usize; // bytes-per-line from the last DTC 0x87, if seen

    for line in plan.lines().filter(|l| !l.trim().is_empty()) {
        let o: serde_json::Value = serde_json::from_str(line).expect("json");
        let seq = o["seq"].as_u64().unwrap_or(0);
        if seq > max_seq {
            break;
        }
        let name = o["name"].as_str().unwrap_or("?");
        let cdb = hex(o["cdb"].as_str().expect("cdb"));
        let expected = o["expected_sense"].as_str().unwrap_or("");

        let (dir, mut data) = match o["expected_phase"].as_u64().unwrap_or(1) {
            2 => (
                DataDirection::Write,
                hex(o["data_out"].as_str().unwrap_or("")),
            ),
            3 => (
                DataDirection::Read,
                vec![0u8; o["request_len"].as_u64().unwrap_or(0) as usize],
            ),
            _ => (DataDirection::None, Vec::new()),
        };
        let mut sense = [0u8; 96];

        let actual = match transport.execute(&cdb, dir, &mut data, &mut sense) {
            Ok(()) => "000000".to_string(),
            Err(Error::Status { sense: Some(s), .. }) => {
                format!("{:02x}{:02x}{:02x}", s.key, s.asc, s.ascq)
            }
            Err(e) => format!("ERR {e}"),
        };
        let flag = if actual == expected { "ok " } else { "DIFF" };
        println!("seq {seq:>3} {flag} {name:<20} exp={expected} got={actual}");

        // DTC 0x87 scan params: bytes-per-line at [11..13] (big-endian).
        if cdb.first() == Some(&0x28) && cdb.get(2) == Some(&0x87) && data.len() >= 13 {
            stride = u16::from_be_bytes([data[11], data[12]]) as usize;
        }
        // DTC 0x00 image chunk: collect whatever came back.
        if cdb.first() == Some(&0x28) && cdb.get(2) == Some(&0x00) && actual == "000000" {
            image.extend_from_slice(&data);
        }
    }

    println!(
        "\nreplayed to seq {max_seq}: {} image bytes collected, stride {stride}",
        image.len()
    );
    if let (Some(path), true) = (out_tiff, stride >= 3 && !image.is_empty()) {
        let width = (stride / 3) as u32;
        let height = (image.len() / stride) as u32;
        let mut pixels = Vec::with_capacity(width as usize * 3 * height as usize);
        for y in 0..height as usize {
            pixels.extend_from_slice(&image[y * stride..y * stride + width as usize * 3]);
        }
        let img: ImageBuffer<Rgb<u8>, _> =
            ImageBuffer::from_raw(width, height, pixels).expect("sized");
        let mut w = BufWriter::new(fs::File::create(&path).expect("create tiff"));
        img.write_to(&mut w, ImageFormat::Tiff)
            .expect("encode tiff");
        println!("wrote {path} ({width}x{height})");
    }
}
