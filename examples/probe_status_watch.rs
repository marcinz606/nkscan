//! Watch TEST UNIT READY status live for ~40s, read-only (moves nothing).
//! Used to see whether the SA-21 auto-grips a strip on insertion (status flips
//! NoFilm -> Ready) without any feed command. Throwaway diagnostic.

use nkscan::{
    scanners::ls50::{Ls50, status::Status},
    scsi::usb::UsbTransport,
};
use std::{thread::sleep, time::Duration};

fn main() {
    let transport = UsbTransport::open(0x04B0, 0x4001).expect("open LS-50");
    let mut s = Ls50::new(transport);

    let mut last = String::new();
    for i in 0..80 {
        let now = match s.status() {
            Ok(Status::Ready) => "Ready".to_string(),
            Ok(state) => format!("{state:?}"),
            Err(e) => format!("err: {e}"),
        };
        // Only print on change, plus a heartbeat every ~5s.
        if now != last || i % 10 == 0 {
            println!("[{:>4.1}s] {now}", i as f32 * 0.5);
            last = now;
        }
        sleep(Duration::from_millis(500));
    }
}
