//! Azure cutter generation module.
//!
//! GEOMETRY PHILOSOPHY:
//! An azure cutter is NOT a simple cone or frustum. It is a **ruled surface**
//! connecting two boundaries:
//!   - TOP boundary: a circle on the outer surface (defined by the stone hole)
//!   - BOTTOM boundary: the projection of that circle onto the inner surface,
//!     pulled back by min_wall_thickness
//!
//! The bottom shape varies depending on the inner surface geometry:
//!   - Flat surface     → circle (trivial case)
//!   - Cylindrical band → ellipse (stretched along band direction)
//!   - S-curve/comfort  → asymmetric freeform curve
//!   - Dome/sphere      → warped circle (non-uniform scaling)
//!
//! We handle ALL cases by ray-projecting each point on the top circle
//! down to the inner surface. This produces the correct bottom boundary
//! regardless of the inner surface shape.

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
    /// Per-sample projected depths (for diagnostics).
    pub sample_depths: Vec<f64>,
}

/// Generate azure cutters for all detected stone seats.
///
/// This is the core algorithm:
/// 1. For each seat, sample the top circle boundary
/// 2. Raycast each sample point to the inner surface to find the bottom boundary
/// 3. Pull back by min_wall_thickness
/// 4. Resolve neighbor conflicts on the projected bottom points
/// 5. Build the ruled-surface cutter mesh connecting top and bottom rings
pub fn generate_azure_cutters(
    seats: &[StoneSeat],
    ring_mesh: &TriMesh,
    config: &AzureConfig,
    segments: usize,
) -> Vec<AzureCutter> {
    // Step 1 & 2: Project each seat's top circle to find bottom boundaries
    let mut projected: Vec<ProjectedCutter> = seats
        .iter()
        .map(|seat| project_cutter_boundary(seat, ring_mesh, config, segments))
        .collect();

    // Step 3: Resolve neighbor conflicts on the bottom boundaries
    resolve_neighbor_conflicts_projected(&mut projected, seats, config.min_rib_width);

    // Step 4: Build cutter meshes
    let mut cutters = Vec::new();
    for (proj, seat) in projected.iter().zip(seats.iter()) {
        if proj.bottom_points.is_empty() {
            log::warn!("Seat {}: no valid bottom projection, skipping", seat.id);
            continue;
        }

        let mesh = build_ruled_surface_cutter(
            &proj.top_points,
            &proj.bottom_points,
            &seat.center,
            &proj.bottom_center,
        );

        cutters.push(AzureCutter {
            seat_id: seat.id,
            mesh,
            top_radius: proj.top_radius,
            sample_depths: proj.depths.clone(),
        });
    }

    log::info!("Generated {} azure cutters", cutters.len());
    cutters
}

/// Intermediate representation of a projected cutter before mesh generation.
#[derive(Debug, Clone)]
struct ProjectedCutter {
    /// Points on the top circle (on the outer surface, stone side).
    top_points: Vec<Point3<f64>>,
    /// Points on the bottom boundary (projected onto inner surface - wall_thickness).
    bottom_points: Vec<Point3<f64>>,
    /// Center of the top circle.
    top_center: Point3<f64>,
    /// Centroid of the bottom boundary points.
    bottom_center: Point3<f64>,
    /// Per-sample ray depths (distance from top to inner surface hit).
    depths: Vec<f64>,
    /// Top radius for reference.
    top_radius: f64,
}

/// Project a single seat's top circle onto the inner surface.
///
/// For each of N sample points on the top circle:
///   1. Compute point on the circle: center + r*(cos(θ)*u + sin(θ)*v)
///   2. Cast ray from that point along the inward normal
///   3. Find where it hits the inner surface (closest triangle)
///   4. Pull back by min_wall_thickness along the ray direction
///   5. Apply taper: expand radially by tan(taper_angle) * depth
///
/// The result is a set of bottom boundary points whose shape naturally
/// conforms to whatever the inner surface geometry is.
fn project_cutter_boundary(
    seat: &StoneSeat,
    ring_mesh: &TriMesh,
    config: &AzureConfig,
    segments: usize,
) -> ProjectedCutter {
    let top_radius = seat.radius + config.seat_margin;
    let axis = seat.inward.normalize();
    let taper_rad = config.taper_angle.to_radians();

    // Build local coordinate frame on the outer surface at the stone seat
    let arbitrary = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u_axis = axis.cross(&arbitrary).normalize();
    let v_axis = axis.cross(&u_axis).normalize();

    let mut top_points = Vec::with_capacity(segments);
    let mut bottom_points = Vec::with_capacity(segments);
    let mut depths = Vec::with_capacity(segments);

    for i in 0..segments {
        let angle = 2.0 * PI * (i as f64) / (segments as f64);
        let radial_dir = u_axis * angle.cos() + v_axis * angle.sin();

        // Point on the top circle
        let top_pt = seat.center + radial_dir * top_radius;
        top_points.push(top_pt);

        // Cast ray inward from this point
        let ray_origin = top_pt;
        let ray_dir = axis;

        match raycast_to_surface(ring_mesh, &ray_origin, &ray_dir) {
            Some(hit_distance) => {
                // The depth available for the azure cut
                let usable_depth = (hit_distance - config.min_wall_thickness).max(0.0);
                depths.push(usable_depth);

                // Bottom point: travel along ray by usable_depth,
                // then expand radially by taper
                let depth_point = ray_origin + ray_dir * usable_depth;

                // Taper expansion: at this depth, expand outward from the seat axis
                let taper_expansion = usable_depth * taper_rad.tan();
                let bottom_pt = depth_point + radial_dir * taper_expansion;

                bottom_points.push(bottom_pt);
            }
            None => {
                // No hit — this sample can't project. Use a zero-depth fallback.
                log::debug!(
                    "Seat {}: ray from sample {} found no inner surface hit",
                    seat.id, i
                );
                depths.push(0.0);
                bottom_points.push(top_pt); // degenerate: bottom = top
            }
        }
    }

    let bottom_center = if !bottom_points.is_empty() {
        let sum: Vector3<f64> = bottom_points.iter().map(|p| p.coords).sum();
        Point3::from(sum / bottom_points.len() as f64)
    } else {
        seat.center
    };

    ProjectedCutter {
        top_points,
        bottom_points,
        top_center: seat.center,
        bottom_center,
        depths,
        top_radius,
    }
}

/// Raycast from `origin` in direction `dir` against the mesh.
/// Returns the distance to the first hit (beyond a small epsilon to skip self-hits).
///
/// This is the same Möller–Trumbore approach but we only care about the closest hit.
fn raycast_to_surface(
    mesh: &TriMesh,
    origin: &Point3<f64>,
    dir: &Vector3<f64>,
) -> Option<f64> {
    let dir_norm = dir.normalize();
    let mut min_t = f64::MAX;

    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0]];
        let v1 = mesh.vertices[face[1]];
        let v2 = mesh.vertices[face[2]];

        if let Some(t) = ray_triangle_intersect(origin, &dir_norm, &v0, &v1, &v2) {
            // Skip self-intersection at the hole boundary (small epsilon)
            if t > 0.05 && t < min_t {
                min_t = t;
            }
        }
    }

    if min_t < f64::MAX {
        Some(min_t)
    } else {
        None
    }
}

/// Möller–Trumbore ray-triangle intersection.
fn ray_triangle_intersect(
    origin: &Point3<f64>,
    dir: &Vector3<f64>,
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
) -> Option<f64> {
    let epsilon = 1e-8;
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);

    if a.abs() < epsilon {
        return None;
    }

    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(&q);
    if t > epsilon { Some(t) } else { None }
}

/// Resolve neighbor conflicts on projected bottom boundaries.
///
/// For each pair of neighboring seats, check if any bottom boundary points
/// from the two cutters are too close (less than min_rib_width apart).
/// If so, shrink the offending bottom points inward toward the seat axis
/// until the constraint is satisfied.
///
/// This is more sophisticated than the old radius-based approach because
/// the bottom boundaries are now freeform curves, not circles.
fn resolve_neighbor_conflicts_projected(
    cutters: &mut [ProjectedCutter],
    seats: &[StoneSeat],
    min_rib_width: f64,
) {
    let n = cutters.len();

    for i in 0..n {
        for j in (i + 1)..n {
            // Quick check: are these seats close enough to potentially conflict?
            let seat_dist = nalgebra::distance(&seats[i].center, &seats[j].center);
            let max_possible_reach = cutters[i].top_radius * 3.0 + cutters[j].top_radius * 3.0;
            if seat_dist > max_possible_reach {
                continue;
            }

            // For each bottom point in cutter i, check distance to the
            // nearest bottom point in cutter j. If too close, pull both inward.
            let segments = cutters[i].bottom_points.len();
            for si in 0..segments {
                for sj in 0..cutters[j].bottom_points.len() {
                    let dist = nalgebra::distance(
                        &cutters[i].bottom_points[si],
                        &cutters[j].bottom_points[sj],
                    );

                    if dist < min_rib_width && dist > 1e-10 {
                        // Pull both points toward their respective seat centers
                        let deficit = min_rib_width - dist;
                        let pull_each = deficit / 2.0 + 0.01; // slight overshoot for safety

                        // Pull point i toward seat i center
                        let dir_i = (cutters[i].bottom_center - cutters[i].bottom_points[si])
                            .try_normalize(1e-10);
                        if let Some(d) = dir_i {
                            cutters[i].bottom_points[si] += d * pull_each;
                        }

                        // Pull point j toward seat j center
                        let dir_j = (cutters[j].bottom_center - cutters[j].bottom_points[sj])
                            .try_normalize(1e-10);
                        if let Some(d) = dir_j {
                            cutters[j].bottom_points[sj] += d * pull_each;
                        }

                        log::debug!(
                            "Neighbor conflict seats {} & {}: samples ({},{}) dist={:.3}mm, pulled by {:.3}mm each",
                            i, j, si, sj, dist, pull_each
                        );
                    }
                }
            }
        }
    }
}

/// Build a ruled-surface cutter mesh connecting the top ring to the bottom ring.
///
/// This is a generalized frustum where:
/// - The top ring is a circle (all points at same radius from top_center)
/// - The bottom ring is a freeform curve (projected onto the inner surface)
/// - Side walls are triangle strips connecting corresponding top/bottom samples
/// - Top and bottom caps close the solid for boolean operations
///
/// The resulting solid correctly handles:
/// - Flat surfaces (bottom ring = circle → standard frustum)
/// - Cylindrical surfaces (bottom ring = ellipse → tapered elliptical cone)
/// - S-curves (bottom ring = asymmetric curve → warped cutter)
/// - Domes (bottom ring = warped circle → dome-following cutter)
fn build_ruled_surface_cutter(
    top_ring: &[Point3<f64>],
    bottom_ring: &[Point3<f64>],
    top_center: &Point3<f64>,
    bottom_center: &Point3<f64>,
) -> TriMesh {
    assert_eq!(top_ring.len(), bottom_ring.len(), "Ring sample counts must match");
    let segments = top_ring.len();

    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // Vertex layout:
    // [0]                    = top center
    // [1..=segments]         = top ring
    // [segments+1]           = bottom center
    // [segments+2..=2*seg+1] = bottom ring

    let top_center_idx = 0;
    vertices.push(*top_center);

    let top_ring_start = vertices.len();
    for pt in top_ring {
        vertices.push(*pt);
    }

    let bottom_center_idx = vertices.len();
    vertices.push(*bottom_center);

    let bottom_ring_start = vertices.len();
    for pt in bottom_ring {
        vertices.push(*pt);
    }

    // Top cap: fan from top_center to top ring (winding for outward normal)
    for i in 0..segments {
        let next = (i + 1) % segments;
        faces.push([top_center_idx, top_ring_start + next, top_ring_start + i]);
    }

    // Bottom cap: fan from bottom_center to bottom ring
    for i in 0..segments {
        let next = (i + 1) % segments;
        faces.push([bottom_center_idx, bottom_ring_start + i, bottom_ring_start + next]);
    }

    // Side walls: triangle strip connecting top ring to bottom ring
    for i in 0..segments {
        let next = (i + 1) % segments;
        let t0 = top_ring_start + i;
        let t1 = top_ring_start + next;
        let b0 = bottom_ring_start + i;
        let b1 = bottom_ring_start + next;

        faces.push([t0, b0, t1]);
        faces.push([t1, b0, b1]);
    }

    // Compute per-face normals
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
    fn test_ruled_surface_flat() {
        // Flat case: top and bottom are both circles → should behave like old frustum
        let segments = 16;
        let mut top_ring = Vec::new();
        let mut bottom_ring = Vec::new();

        for i in 0..segments {
            let angle = 2.0 * PI * (i as f64) / (segments as f64);
            top_ring.push(Point3::new(angle.cos() * 1.0, angle.sin() * 1.0, 0.0));
            bottom_ring.push(Point3::new(angle.cos() * 2.0, angle.sin() * 2.0, 3.0));
        }

        let top_center = Point3::new(0.0, 0.0, 0.0);
        let bottom_center = Point3::new(0.0, 0.0, 3.0);

        let mesh = build_ruled_surface_cutter(&top_ring, &bottom_ring, &top_center, &bottom_center);

        // top_center(1) + top_ring(16) + bottom_center(1) + bottom_ring(16) = 34
        assert_eq!(mesh.vertices.len(), 34);
        // top_cap(16) + bottom_cap(16) + sides(32) = 64
        assert_eq!(mesh.faces.len(), 64);
    }

    #[test]
    fn test_ruled_surface_elliptical() {
        // Cylindrical projection: bottom is an ellipse
        let segments = 16;
        let mut top_ring = Vec::new();
        let mut bottom_ring = Vec::new();

        for i in 0..segments {
            let angle = 2.0 * PI * (i as f64) / (segments as f64);
            // Top: circle r=1
            top_ring.push(Point3::new(angle.cos(), angle.sin(), 0.0));
            // Bottom: ellipse a=2.0, b=1.5 (cylinder stretch in x)
            bottom_ring.push(Point3::new(angle.cos() * 2.0, angle.sin() * 1.5, 3.0));
        }

        let top_center = Point3::origin();
        let bottom_center = Point3::new(0.0, 0.0, 3.0);

        let mesh = build_ruled_surface_cutter(&top_ring, &bottom_ring, &top_center, &bottom_center);

        assert_eq!(mesh.vertices.len(), 34);
        assert_eq!(mesh.faces.len(), 64);

        // Verify bottom ring has elliptical shape (x extent > y extent)
        let max_x: f64 = bottom_ring.iter().map(|p| p.x.abs()).fold(0.0, f64::max);
        let max_y: f64 = bottom_ring.iter().map(|p| p.y.abs()).fold(0.0, f64::max);
        assert!(max_x > max_y, "Bottom should be elliptical: x={:.2} > y={:.2}", max_x, max_y);
    }

    #[test]
    fn test_neighbor_conflict_projected() {
        // Two adjacent cutters with bottom points that are too close
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
                center: Point3::new(2.0, 0.0, 0.0),
                radius: 0.5,
                normal: Vector3::z(),
                inward: -Vector3::z(),
                local_thickness: 2.0,
                boundary_vertices: vec![],
            },
        ];

        // Manually place bottom points that nearly touch
        let mut cutters = vec![
            ProjectedCutter {
                top_points: vec![Point3::new(0.5, 0.0, 0.0)],
                bottom_points: vec![Point3::new(0.95, 0.0, -1.5)], // close to seat 1
                top_center: Point3::new(0.0, 0.0, 0.0),
                bottom_center: Point3::new(0.0, 0.0, -1.5),
                depths: vec![1.5],
                top_radius: 0.7,
            },
            ProjectedCutter {
                top_points: vec![Point3::new(1.5, 0.0, 0.0)],
                bottom_points: vec![Point3::new(1.05, 0.0, -1.5)], // close to seat 0
                top_center: Point3::new(2.0, 0.0, 0.0),
                bottom_center: Point3::new(2.0, 0.0, -1.5),
                depths: vec![1.5],
                top_radius: 0.7,
            },
        ];

        let min_rib = 0.4;
        // Before: distance between bottom points = 0.1mm (less than 0.4)
        let dist_before = nalgebra::distance(
            &cutters[0].bottom_points[0],
            &cutters[1].bottom_points[0],
        );
        assert!(dist_before < min_rib);

        resolve_neighbor_conflicts_projected(&mut cutters, &seats, min_rib);

        // After: distance should be >= min_rib_width
        let dist_after = nalgebra::distance(
            &cutters[0].bottom_points[0],
            &cutters[1].bottom_points[0],
        );
        assert!(
            dist_after >= min_rib - 0.05, // small tolerance for floating point
            "After resolution, dist={:.3} should be >= {:.3}",
            dist_after,
            min_rib
        );
    }

    #[test]
    fn test_raycast_hit() {
        let origin = Point3::new(0.0, 0.0, -1.0);
        let dir = Vector3::new(0.0, 0.0, 1.0);
        let v0 = Point3::new(-5.0, -5.0, 2.0);
        let v1 = Point3::new(5.0, -5.0, 2.0);
        let v2 = Point3::new(0.0, 5.0, 2.0);

        let t = ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2);
        assert!(t.is_some());
        assert!((t.unwrap() - 3.0).abs() < 1e-6); // from z=-1 to z=2 = distance 3
    }
}
