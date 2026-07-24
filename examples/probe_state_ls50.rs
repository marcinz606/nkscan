//! Dump the LS-50's sensor/status registers read-only, to understand scanner
//! state during scan bring-up WITHOUT moving film. Sends only data-in reads
//! (VENDOR E1, READ DTC 0x88 calibration) — no E0 writes, no C1 triggers, no
//! motor commands. Safe to run with or without film loaded.

use nkscan::scsi::{
    Transport,
    cdbs::{DataTypeCode, Inquiry, Read, TestUnitReady, VendorE1},
    usb::UsbTransport,
};
use std::{thread::sleep, time::Duration};

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn main() {
    let mut t = UsbTransport::open(0x04B0, 0x4001).expect("open LS-50");

    for _ in 0..40 {
        if t.send(&TestUnitReady::new()).is_ok() {
            break;
        }
        sleep(Duration::from_millis(250));
    }

    match t.send(&Inquiry::new()) {
        Ok(inq) => println!(
            "INQUIRY: {:?} {:?} {:?}",
            inq.vendor, inq.product, inq.revision
        ),
        Err(e) => println!("INQUIRY: {e}"),
    }

    // VENDOR E1 sensor registers (read-only). Max 13 bytes per the firmware.
    for (sub, name) in [
        (0x44u8, "motor position"),
        (0x45, "exposure time"),
        (0x46, "focus position"),
        (0x47, "lamp settings"),
        (0xA0, "CCD setup"),
    ] {
        match t.send(&VendorE1::new(sub, 13)) {
            Ok(d) => println!("E1 {sub:#04x} ({name:<14}): {}", hex(&d)),
            Err(e) => println!("E1 {sub:#04x} ({name:<14}): {e}"),
        }
    }

    // READ DTC 0x88 calibration boundary data (read-only), per channel.
    for qual in [0u16, 1, 2, 3] {
        match t.send(&Read::new(0, DataTypeCode::Vendor(0x88), qual, 64, 0x80)) {
            Ok(d) => println!("DTC 0x88 qual {qual}: {}", hex(&d)),
            Err(e) => println!("DTC 0x88 qual {qual}: {e}"),
        }
    }
}
