//! Dump the LS-50's INQUIRY EVPD adapter-config pages (0xC1/0xD1/0xE1/0xF0/0xF8)
//! raw, to see the firmware's adapter identification. Read-only, no motion.
//! `cargo run --example probe_vpd_ls50`

use nkscan::scsi::{Transport, cdbs::VpdInquiry, usb::UsbTransport};

fn main() {
    let mut t = UsbTransport::open(0x04B0, 0x4001).expect("open LS-50");
    for (page, alloc) in [
        (0x00u8, 32u8),
        (0xC1, 87),
        (0xD1, 28),
        (0xE1, 39),
        (0xF0, 53),
        (0xF8, 17),
    ] {
        match t.send(&VpdInquiry::new(page, alloc)) {
            Ok(p) => println!("page {page:#04x} ({:>3} B): {:02x?}", p.data.len(), p.data),
            Err(e) => println!("page {page:#04x}: ERR {e}"),
        }
    }
}
