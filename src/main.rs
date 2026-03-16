//! jewellery_azure_cutter — Automatic azure (à jour) cuts on jewellery STL meshes.
//!
//! Pipeline:
//! 1. Load config (TOML) + STL mesh
//! 2. Detect stone seats (boundary loops → circle fitting → filtering)
//! 3. Generate azure cutters (chimney shape: cylinder + rectangular pyramid)
//! 4. Boolean subtract cutters from ring mesh
//! 5. Write output STL

mod azure;
mod boolean;
mod config;
mod detect;
mod mesh;

use anyhow::{Context, Result};
use clap::Parser;
use std::path::PathBuf;

/// Automatic azure cutter for jewellery STL meshes.
#[derive(Parser, Debug)]
#[command(name = "jewellery-azure-cutter")]
#[command(about = "Generate and apply azure (à jour) cuts to reduce metal weight in jewellery STL files")]
struct Cli {
    /// Path to the TOML configuration file.
    #[arg(short, long)]
    config: PathBuf,

    /// Override: input STL file (takes precedence over config).
    #[arg(short, long)]
    input: Option<PathBuf>,

    /// Override: output STL file (takes precedence over config).
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Verbose logging.
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Initialize logging
    let log_level = if cli.verbose { "debug" } else { "info" };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(log_level))
        .init();

    log::info!("jewellery-azure-cutter v{}", env!("CARGO_PKG_VERSION"));

    // 1. Load configuration
    let cfg = config::Config::from_file(&cli.config)
        .with_context(|| "Failed to load configuration")?;

    let input_path = cli
        .input
        .unwrap_or_else(|| PathBuf::from(&cfg.input.stl_file));
    let output_path = cli
        .output
        .unwrap_or_else(|| PathBuf::from(cfg.output_path()));

    log::info!("Input:  {}", input_path.display());
    log::info!("Output: {}", output_path.display());

    // 2. Load STL mesh
    let ring_mesh = mesh::TriMesh::from_stl(&input_path)
        .with_context(|| "Failed to load input STL")?;

    log::info!(
        "Ring mesh: {} vertices, {} faces",
        ring_mesh.vertices.len(),
        ring_mesh.faces.len()
    );

    // 3. Detect stone seats
    let seats = detect::detect_stone_seats(&ring_mesh, &cfg.stone)
        .with_context(|| "Stone seat detection failed")?;

    if seats.is_empty() {
        log::warn!("No stone seats detected — check your STL or diameter range settings");
        // Write unmodified mesh
        ring_mesh.write_stl(&output_path)?;
        return Ok(());
    }

    log::info!("Detected {} stone seats", seats.len());

    // 4. Generate azure cutters (chimney shape: cylinder top + rect pyramid bottom)
    let cutters = azure::generate_azure_cutters(&seats, &ring_mesh, &cfg.azure);
    log::info!("Generated {} cutters", cutters.len());

    // 5. Boolean subtract
    let result_mesh = boolean::subtract_cutters(&ring_mesh, &cutters)
        .with_context(|| "Boolean subtraction failed")?;

    // 6. Write output
    result_mesh.write_stl(&output_path)?;
    log::info!("Done. Output written to {}", output_path.display());

    Ok(())
}
