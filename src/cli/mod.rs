//! The `nkscan` command-line interface: parse args, open the selected device, dispatch.

mod device;
mod output;

use clap::{ArgAction, Parser, Subcommand};
use device::Device;
use std::{error::Error, path::PathBuf};
use tracing_subscriber::EnvFilter;

#[derive(Parser)]
#[command(
    name = "nkscan",
    version,
    about = "Nikon Coolscan film scanner control"
)]
struct Cli {
    /// Which scanner to talk to.
    #[arg(long, value_enum, default_value_t = Device::Ls50, global = true)]
    device: Device,
    /// SCSI device node (e.g. /dev/sg2) for SCSI-only scanners.
    #[arg(long, global = true)]
    path: Option<PathBuf>,
    /// Increase log verbosity (-v info, -vv debug, -vvv trace). RUST_LOG overrides.
    #[arg(short, long, action = ArgAction::Count, global = true)]
    verbose: u8,
    #[command(subcommand)]
    command: Commands,
}

/// Flags shared by `scan` and `strip`.
#[derive(clap::Args)]
pub struct ScanCommon {
    /// Output TIFF path (IR written beside it as `*_ir.tiff`).
    #[arg(default_value = "scan.tiff")]
    pub output: PathBuf,
    /// Capture the infrared cleaning plane.
    #[arg(long)]
    pub ir: bool,
    /// Optical resolution in DPI (snapped to the sensor grid).
    #[arg(long, default_value_t = 300)]
    pub dpi: u16,
    /// Multi-sample count. Only 1 works (a multi-pass scan never streams).
    #[arg(long, default_value_t = 1, value_parser = parse_samples)]
    pub samples: u8,
    /// Autoexposure pre-pass.
    #[arg(long)]
    pub ae: bool,
    /// Firmware autofocus at the frame centre.
    #[arg(long)]
    pub af: bool,
    /// Feed-axis offset in mm applied to each frame.
    #[arg(long, default_value_t = 0.0)]
    pub offset: f32,
}

#[derive(Subcommand)]
enum Commands {
    /// Scan a single frame to a TIFF.
    Scan {
        #[command(flatten)]
        common: ScanCommon,
    },
    /// Scan a strip to numbered TIFFs (`OUT_00.tiff`, …), then eject.
    Strip {
        #[command(flatten)]
        common: ScanCommon,
        /// Frames to scan; omit to auto-detect from the adapter.
        #[arg(short = 'n', long)]
        count: Option<u32>,
        /// Keep the strip loaded instead of ejecting after the batch.
        #[arg(long)]
        no_eject: bool,
    },
    /// Eject the loaded film and exit.
    Eject,
    /// Print the scanner's current readiness state.
    Status,
    /// Print inquiry identity, holder, and adapter caps.
    Info,
    /// Watch the scanner and print every state change.
    Watch,
}

/// Reject `--samples > 1` at parse time (documents the multi-pass hardware limit).
fn parse_samples(s: &str) -> Result<u8, String> {
    match s.parse::<u8>() {
        Ok(1) => Ok(1),
        Ok(_) => Err("only --samples 1 works: the multi-pass scan never streams".into()),
        Err(_) => Err("expected a number".into()),
    }
}

pub fn run() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();
    init_tracing(cli.verbose);

    let mut backend = device::open(cli.device, cli.path.as_deref())?;

    match cli.command {
        Commands::Scan { common } => {
            let frames = backend.scan(&common)?;
            output::write_frames(&frames, &common.output, false)?;
        }
        Commands::Strip {
            common,
            count,
            no_eject,
        } => {
            let frames = backend.scan_strip(&common, count)?;
            output::write_frames(&frames, &common.output, true)?;
            if !no_eject {
                backend.eject()?;
                println!("ejected");
            }
        }
        Commands::Eject => {
            backend.eject()?;
            println!("ejected");
        }
        Commands::Status => println!("{}", backend.status()?),
        Commands::Info => println!("{}", backend.info()?),
        Commands::Watch => backend.watch()?,
    }
    Ok(())
}

fn init_tracing(verbose: u8) {
    let default = match verbose {
        0 => "warn",
        1 => "info",
        2 => "debug",
        _ => "trace",
    };
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(default)),
        )
        .init();
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn cli_is_well_formed() {
        Cli::command().debug_assert();
    }

    #[test]
    fn defaults_resolve() {
        let cli = Cli::try_parse_from(["nkscan", "scan"]).unwrap();
        assert!(matches!(cli.device, Device::Ls50));
        match cli.command {
            Commands::Scan { common } => {
                assert_eq!(common.output, PathBuf::from("scan.tiff"));
                assert_eq!(common.dpi, 300);
                assert_eq!(common.samples, 1);
            }
            _ => panic!("expected scan"),
        }
    }

    #[test]
    fn strip_count_optional() {
        let bare = Cli::try_parse_from(["nkscan", "strip"]).unwrap();
        let forced = Cli::try_parse_from(["nkscan", "strip", "--count", "6"]).unwrap();
        assert!(matches!(bare.command, Commands::Strip { count: None, .. }));
        assert!(matches!(
            forced.command,
            Commands::Strip { count: Some(6), .. }
        ));
    }

    #[test]
    fn samples_over_one_rejected() {
        assert!(parse_samples("1").is_ok());
        assert!(parse_samples("2").is_err());
        assert!(parse_samples("x").is_err());
        assert!(Cli::try_parse_from(["nkscan", "scan", "--samples", "4"]).is_err());
    }
}
