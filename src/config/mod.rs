//! Configuration module — loads and validates user parameters from TOML.

use serde::Deserialize;
use anyhow::{Context, Result};
use std::path::Path;

/// Top-level configuration for the azure cutter tool.
#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub input: InputConfig,
    pub stone: StoneConfig,
    pub azure: AzureConfig,
}

/// Input file paths.
#[derive(Debug, Clone, Deserialize)]
pub struct InputConfig {
    /// Path to the input STL file (must have pre-drilled stone holes).
    pub stl_file: String,
    /// Path for the output STL file. Defaults to "<input>_azure.stl".
    pub output_file: Option<String>,
}

/// Stone detection parameters.
#[derive(Debug, Clone, Deserialize)]
pub struct StoneConfig {
    /// Stone type — only "round" supported for now.
    #[serde(default = "default_stone_type")]
    pub stone_type: String,
    /// Minimum hole diameter (mm) to consider as a stone seat.
    #[serde(default = "default_diameter_min")]
    pub diameter_min: f64,
    /// Maximum hole diameter (mm) to consider as a stone seat.
    #[serde(default = "default_diameter_max")]
    pub diameter_max: f64,
}

/// Azure cut generation parameters.
#[derive(Debug, Clone, Deserialize)]
pub struct AzureConfig {
    /// Minimum metal thickness remaining at bottom of cut (mm).
    #[serde(default = "default_min_wall_thickness")]
    pub min_wall_thickness: f64,
    /// Minimum metal width between adjacent azure cuts (mm).
    #[serde(default = "default_min_rib_width")]
    pub min_rib_width: f64,
    /// Taper angle of azure sidewalls (degrees from vertical).
    #[serde(default = "default_taper_angle")]
    pub taper_angle: f64,
    /// Radial safety margin around stone hole edge (mm).
    #[serde(default = "default_seat_margin")]
    pub seat_margin: f64,
}

// --- Defaults ---

fn default_stone_type() -> String { "round".to_string() }
fn default_diameter_min() -> f64 { 0.8 }
fn default_diameter_max() -> f64 { 10.0 }
fn default_min_wall_thickness() -> f64 { 0.5 }
fn default_min_rib_width() -> f64 { 0.4 }
fn default_taper_angle() -> f64 { 30.0 }
fn default_seat_margin() -> f64 { 0.2 }

impl Config {
    /// Load configuration from a TOML file.
    pub fn from_file(path: &Path) -> Result<Self> {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read config file: {}", path.display()))?;
        let config: Config = toml::from_str(&content)
            .with_context(|| "Failed to parse config TOML")?;
        config.validate()?;
        Ok(config)
    }

    /// Validate configuration values are within sane ranges.
    pub fn validate(&self) -> Result<()> {
        anyhow::ensure!(
            self.stone.stone_type == "round",
            "Only 'round' stone type is supported currently"
        );
        anyhow::ensure!(
            self.stone.diameter_min > 0.0 && self.stone.diameter_min < self.stone.diameter_max,
            "diameter_min must be > 0 and < diameter_max"
        );
        anyhow::ensure!(
            self.azure.min_wall_thickness > 0.0,
            "min_wall_thickness must be > 0"
        );
        anyhow::ensure!(
            self.azure.min_rib_width > 0.0,
            "min_rib_width must be > 0"
        );
        anyhow::ensure!(
            self.azure.taper_angle > 0.0 && self.azure.taper_angle < 90.0,
            "taper_angle must be between 0 and 90 degrees"
        );
        anyhow::ensure!(
            self.azure.seat_margin >= 0.0,
            "seat_margin must be >= 0"
        );
        Ok(())
    }

    /// Derive the output file path.
    pub fn output_path(&self) -> String {
        self.input.output_file.clone().unwrap_or_else(|| {
            let stem = Path::new(&self.input.stl_file)
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("output");
            format!("{}_azure.stl", stem)
        })
    }
}
