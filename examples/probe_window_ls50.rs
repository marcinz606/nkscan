//! Dump the LS-50's current window descriptor (GET WINDOW), to learn the exact
//! bytes this hardware accepts before crafting SET WINDOW. Throwaway diagnostic.

use nkscan::scsi::{
    Transport,
    cdbs::{GetWindow, TestUnitReady},
    usb::UsbTransport,
};
use std::{thread::sleep, time::Duration};

fn main() {
    let mut t = UsbTransport::open(0x04B0, 0x4001).expect("open LS-50");

    // Drain cold-start unit attentions.
    for _ in 0..40 {
        if t.send(&TestUnitReady::new()).is_ok() {
            break;
        }
        sleep(Duration::from_millis(250));
    }

    match t.send(&GetWindow::new(0, false, 0, 256, 0x00)) {
        Ok(descriptors) => {
            println!("GET WINDOW: {} descriptor(s)", descriptors.len());
            for d in &descriptors {
                println!("{d:#?}");
            }
        }
        Err(e) => println!("GET WINDOW failed: {e}"),
    }
}
