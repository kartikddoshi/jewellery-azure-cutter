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
///
/// The azure cutter is shaped like a chimney:
///   - TOP: A cylinder matching the stone hole, extending inward
///   - BOTTOM: A tapered rectangular pyramid widening toward the inner surface
///
/// ```text
///        ┌──────┐         ← Stone hole (circle)
///        │      │
///        │ CYLNDR│  ← girdle_distance deep
///        │      │
///        ├──┐┌──┤         ← step (flat ledge at transition)
///       /   ││   \
///      / PYRAMID  \  ← taper controls the widening angle
///     /     ││     \
///    └──────┘└──────┘     ← bottom rectangle on inner surface
/// ```
#[derive(Debug, Clone, Deserialize)]
pub struct AzureConfig {
    // --- Cylinder (top) zone ---

    /// Distance the cylinder extends below the stone girdle (mm).
    /// This is how deep the straight bore goes before the pyramid begins.
    #[serde(default = "default_girdle_distance")]
    pub girdle_distance: f64,

    // --- Pyramid (bottom) zone ---

    /// Taper angle of the pyramid walls from vertical (degrees).
    /// Controls how quickly the rectangle widens from the cylinder base.
    #[serde(default = "default_taper_angle")]
    pub taper_angle: f64,

    /// Height of the flat step/ledge at the cylinder-to-pyramid transition (mm).
    /// 0.0 means no step — direct transition.
    #[serde(default = "default_step_height")]
    pub step_height: f64,

    /// How far the cut extends below/past the inner surface (mm).
    /// Ensures the boolean fully pierces through.
    #[serde(default = "default_overhang")]
    pub overhang: f64,

    // --- Wall constraints ---

    /// Minimum metal thickness at the inner surface / bottom of cut (mm).
    #[serde(default = "default_min_wall_thickness")]
    pub min_wall_thickness: f64,

    /// Minimum metal rib width between adjacent azure cuts (mm).
    /// This constrains the bottom rectangle dimensions.
    #[serde(default = "default_min_rib_width")]
    pub min_rib_width: f64,

    /// Radial safety margin around stone hole edge in the cylinder zone (mm).
    #[serde(default = "default_seat_margin")]
    pub seat_margin: f64,

    // --- Resolution ---

    /// Number of segments for the cylinder cross-section (polygon approximation).
    #[serde(default = "default_cylinder_segments")]
    pub cylinder_segments: usize,
}

// --- Defaults ---

fn default_stone_type() -> String { "round".to_string() }
fn default_diameter_min() -> f64 { 0.8 }
fn default_diameter_max() -> f64 { 10.0 }
fn default_girdle_distance() -> f64 { 0.5 }
fn default_taper_angle() -> f64 { 15.0 }
fn default_step_height() -> f64 { 0.0 }
fn default_overhang() -> f64 { 0.1 }
fn default_min_wall_thickness() -> f64 { 0.5 }
fn default_min_rib_width() -> f64 { 0.4 }
fn default_seat_margin() -> f64 { 0.2 }
fn default_cylinder_segments() -> usize { 32 }

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
            self.azure.girdle_distance >= 0.0,
            "girdle_distance must be >= 0"
        );
        anyhow::ensure!(
            self.azure.taper_angle >= 0.0 && self.azure.taper_angle < 90.0,
            "taper_angle must be between 0 and 90 degrees"
        );
        anyhow::ensure!(
            self.azure.step_height >= 0.0,
            "step_height must be >= 0"
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
            self.azure.seat_margin >= 0.0,
            "seat_margin must be >= 0"
        );
        anyhow::ensure!(
            self.azure.cylinder_segments >= 8,
            "cylinder_segments must be >= 8"
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
