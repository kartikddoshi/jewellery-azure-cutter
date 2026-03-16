//! Azure cutter generation module.
//!
//! For each detected stone seat, generates a tapered cutter solid that:
//! - Top circle: matches the stone hole (radius + seat_margin)
//! - Bottom circle: expanded by taper, clipped so that adjacent cutters
//!   maintain min_rib_width between them
//! - Height: local_thickness - min_wall_thickness
//!
//! The cutter is a conical frustum (truncated cone) represented as a triangle mesh.

use crate::config::AzureConfig;
use crate::detect::StoneSeat;
use crate::mesh::TriMesh;
use nalgebra::{Point3, Vector3};
use std::f64::consts::PI;

/// A generated azure cutter solid, ready for boolean subtraction.
#[derive(Debug, Clone)]
pub struct AzureCutter {
    /// The stone seat this cutter belongs to.
    pub seat_id: usize,
    /// Triangle mesh of the cutter solid.
    pub mesh: TriMesh,
    /// Top radius used (at stone side).
    pub top_radius: f64,
    /// Bottom radius used (at inner surface side), after neighbor adjustment.
    pub bottom_radius: f64,
    /// Height of the cutter.
    pub height: f64,
}

/// Generate azure cutters for all detected stone seats.
///
/// This is the core algorithm:
/// 1. Compute raw cutter dimensions per seat
/// 2. Build a neighbor graph and resolve rib-width conflicts
/// 3. Generate the frustum meshes
pub fn generate_azure_cutters(
    seats: &[StoneSeat],
    config: &AzureConfig,
) -> Vec<AzureCutter> {
    // Step 1: Compute raw (unconstrained) cutter dimensions for each seat.
    let mut raw_cutters: Vec<RawCutterParams> = seats
        .iter()
        .map(|seat| compute_raw_params(seat, config))
        .collect();

    // Step 2: Resolve neighbor conflicts — ensure min_rib_width between adjacent cutters.
    resolve_neighbor_conflicts(&mut raw_cutters, seats, config.min_rib_width);

    // Step 3: Generate frustum meshes.
    let mut cutters = Vec::new();
    for (i, (params, seat)) in raw_cutters.iter().zip(seats.iter()).enumerate() {
        if params.height <= 0.0 {
            log::warn!("Seat {}: azure height <= 0, skipping (metal too thin)", seat.id);
            continue;
        }

        let mesh = generate_frustum(
            &seat.center,
            &seat.inward,
            params.top_radius,
            params.bottom_radius,
            params.height,
            32, // segments for circular cross-section
        );

        cutters.push(AzureCutter {
            seat_id: seat.id,
            mesh,
            top_radius: params.top_radius,
            bottom_radius: params.bottom_radius,
            height: params.height,
        });
    }

    log::info!("Generated {} azure cutters", cutters.len());
    cutters
}

/// Intermediate raw parameters before neighbor resolution.
#[derive(Debug, Clone)]
struct RawCutterParams {
    top_radius: f64,
    bottom_radius: f64,
    height: f64,
}

/// Compute raw (unconstrained) cutter parameters for a single seat.
fn compute_raw_params(seat: &StoneSeat, config: &AzureConfig) -> RawCutterParams {
    // Top radius: stone hole radius + seat margin
    let top_radius = seat.radius + config.seat_margin;

    // Height: available metal minus the wall we must preserve
    let height = (seat.local_thickness - config.min_wall_thickness).max(0.0);

    // Bottom radius: top radius expanded by taper
    // tan(angle) = (bottom_radius - top_radius) / height
    let taper_rad = config.taper_angle.to_radians();
    let bottom_radius = top_radius + height * taper_rad.tan();

    RawCutterParams {
        top_radius,
        bottom_radius,
        height,
    }
}

/// For each pair of adjacent seats, if their bottom circles (plus rib margin)
/// overlap, shrink both bottom radii proportionally until the constraint is met.
fn resolve_neighbor_conflicts(
    params: &mut [RawCutterParams],
    seats: &[StoneSeat],
    min_rib_width: f64,
) {
    let n = seats.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let dist = nalgebra::distance(&seats[i].center, &seats[j].center);
            let required = params[i].bottom_radius + params[j].bottom_radius + min_rib_width;

            if required > dist {
                // Need to shrink. Distribute proportionally.
                let available = (dist - min_rib_width).max(0.0);
                let total_radii = params[i].bottom_radius + params[j].bottom_radius;

                if total_radii > 0.0 {
                    let ratio_i = params[i].bottom_radius / total_radii;
                    let ratio_j = params[j].bottom_radius / total_radii;
                    let new_ri = available * ratio_i;
                    let new_rj = available * ratio_j;

                    // Only shrink, never grow
                    params[i].bottom_radius = params[i].bottom_radius.min(new_ri);
                    params[j].bottom_radius = params[j].bottom_radius.min(new_rj);

                    // Ensure bottom_radius >= top_radius (no inverted taper)
                    params[i].bottom_radius = params[i].bottom_radius.max(params[i].top_radius);
                    params[j].bottom_radius = params[j].bottom_radius.max(params[j].top_radius);

                    log::debug!(
                        "Adjusted neighbors {} & {}: dist={:.2}, new_r=({:.2},{:.2})",
                        i, j, dist, params[i].bottom_radius, params[j].bottom_radius
                    );
                }
            }
        }
    }
}

/// Generate a frustum (truncated cone) mesh centered at `origin`,
/// oriented along `axis` (from top to bottom), with given radii and height.
///
/// The frustum is a closed solid (top cap + bottom cap + side walls).
fn generate_frustum(
    origin: &Point3<f64>,
    axis: &Vector3<f64>,
    top_radius: f64,
    bottom_radius: f64,
    height: f64,
    segments: usize,
) -> TriMesh {
    let axis_norm = axis.normalize();

    // Build a local coordinate frame: axis = Z-like, with U and V as the radial directions
    let arbitrary = if axis_norm.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = axis_norm.cross(&arbitrary).normalize();
    let v = axis_norm.cross(&u).normalize();

    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // Generate ring of vertices at top (at `origin`, radius = top_radius)
    let top_center_idx = vertices.len();
    vertices.push(*origin);
    let top_ring_start = vertices.len();
    for i in 0..segments {
        let angle = 2.0 * PI * (i as f64) / (segments as f64);
        let offset = u * angle.cos() * top_radius + v * angle.sin() * top_radius;
        vertices.push(origin + offset);
    }

    // Generate ring of vertices at bottom (at `origin + axis * height`, radius = bottom_radius)
    let bottom_center = origin + axis_norm * height;
    let bottom_center_idx = vertices.len();
    vertices.push(bottom_center);
    let bottom_ring_start = vertices.len();
    for i in 0..segments {
        let angle = 2.0 * PI * (i as f64) / (segments as f64);
        let offset = u * angle.cos() * bottom_radius + v * angle.sin() * bottom_radius;
        vertices.push(bottom_center + offset);
    }

    // Top cap (fan from center) — normals point opposite to axis (outward from cutter)
    for i in 0..segments {
        let next = (i + 1) % segments;
        faces.push([top_center_idx, top_ring_start + next, top_ring_start + i]);
    }

    // Bottom cap (fan from center) — normals point along axis
    for i in 0..segments {
        let next = (i + 1) % segments;
        faces.push([bottom_center_idx, bottom_ring_start + i, bottom_ring_start + next]);
    }

    // Side walls — quad strips as two triangles each
    for i in 0..segments {
        let next = (i + 1) % segments;
        let t0 = top_ring_start + i;
        let t1 = top_ring_start + next;
        let b0 = bottom_ring_start + i;
        let b1 = bottom_ring_start + next;

        faces.push([t0, b0, t1]);
        faces.push([t1, b0, b1]);
    }

    // Compute normals
    let normals = faces
        .iter()
        .map(|f| {
            let e1 = vertices[f[1]] - vertices[f[0]];
            let e2 = vertices[f[2]] - vertices[f[0]];
            e1.cross(&e2).normalize()
        })
        .collect();

    TriMesh {
        vertices,
        faces,
        normals,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frustum_generation() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(0.0, 0.0, 1.0);
        let mesh = generate_frustum(&origin, &axis, 1.0, 2.0, 3.0, 16);

        // 16 segments -> top center(1) + top ring(16) + bottom center(1) + bottom ring(16) = 34
        assert_eq!(mesh.vertices.len(), 34);
        // top cap(16) + bottom cap(16) + sides(32) = 64 faces
        assert_eq!(mesh.faces.len(), 64);
    }

    #[test]
    fn test_neighbor_conflict_resolution() {
        let config_rib = 0.4;
        let seats = vec![
            StoneSeat {
                id: 0,
                center: Point3::new(0.0, 0.0, 0.0),
                radius: 0.5,
                normal: Vector3::z(),
                inward: -Vector3::z(),
                local_thickness: 2.0,
                boundary_vertices: vec![],
            },
            StoneSeat {
                id: 1,
                center: Point3::new(2.0, 0.0, 0.0), // 2mm apart
                radius: 0.5,
                normal: Vector3::z(),
                inward: -Vector3::z(),
                local_thickness: 2.0,
                boundary_vertices: vec![],
            },
        ];

        let mut params = vec![
            RawCutterParams { top_radius: 0.7, bottom_radius: 1.5, height: 1.5 },
            RawCutterParams { top_radius: 0.7, bottom_radius: 1.5, height: 1.5 },
        ];

        // Without resolution: 1.5 + 1.5 + 0.4 = 3.4 > 2.0 (distance)
        resolve_neighbor_conflicts(&mut params, &seats, config_rib);

        // After resolution: r_i + r_j + 0.4 <= 2.0 → r_i + r_j <= 1.6
        assert!(params[0].bottom_radius + params[1].bottom_radius + config_rib <= 2.0 + 1e-6);
    }
}
