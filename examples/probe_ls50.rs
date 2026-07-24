//! One-shot hardware bring-up probe for the LS-50: open over USB, identify,
//! watch the status settle out of cold-start, and read the loaded adapter.
//!
//! Run with `RUST_LOG=debug cargo run --example probe_ls50` to also see each
//! SCSI command's CDB, phase byte, and raw sense from the transport.

use nkscan::{
    scanners::ls50::{Ls50, status::Status},
    scsi::usb::UsbTransport,
};
use std::{thread::sleep, time::Duration};

const LS50_VID: u16 = 0x04B0;
const LS50_PID: u16 = 0x4001;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("debug")),
        )
        .init();

    let transport = UsbTransport::open(LS50_VID, LS50_PID).expect("open LS-50 over USB");
    let mut scanner = Ls50::new(transport);

    match scanner.inquiry() {
        Ok(inq) => println!(
            "INQUIRY: vendor={:?} product={:?} revision={:?}",
            inq.vendor, inq.product, inq.revision
        ),
        Err(err) => println!("INQUIRY failed: {err}"),
    }

    // Poll status until it settles to Ready (or we give up), letting the
    // cold-start UNIT ATTENTIONs drain and the lamp warm up.
    for i in 0..20 {
        match scanner.status() {
            Ok(Status::Ready) => {
                println!("STATUS[{i}]: Ready");
                break;
            }
            Ok(state) => println!("STATUS[{i}]: {state:?}"),
            Err(err) => println!("STATUS[{i}] error: {err}"),
        }
        sleep(Duration::from_millis(500));
    }

    match scanner.holder() {
        Ok(holder) => println!("HOLDER: {holder:?}"),
        Err(err) => println!("HOLDER failed: {err}"),
    }
}
