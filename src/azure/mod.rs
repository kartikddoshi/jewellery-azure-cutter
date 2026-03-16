//! Azure cutter generation module.
//!
//! CORRECT GEOMETRY — "Chimney" shape:
//!
//! An azure cutter is NOT a cone or frustum. It has two distinct zones:
//!
//!   ZONE 1 (top) — CYLINDER:
//!     A straight cylindrical bore matching the stone hole diameter.
//!     Extends inward from the stone seat by `girdle_distance`.
//!     This is where the diamond sits and is held.
//!
//!   ZONE 2 (bottom) — RECTANGULAR TRUNCATED PYRAMID:
//!     Below the cylinder, the shape transitions to a rectangular
//!     cross-section that widens toward the inner surface.
//!     The bottom face is a rectangle whose dimensions are governed by
//!     the spacing between neighboring stones minus rib walls.
//!
//! ```text
//!  Side view (cross section):
//!
//!         ┌────────┐          ← Stone hole (circle, radius r)
//!         │        │
//!         │ CYLNDR │  ← girdle_distance
//!         │        │
//!         ├─┐    ┌─┤          ← step (optional flat ledge)
//!        /  │    │  \
//!       / TRUNCATED  \  ← taper_angle controls widening
//!      /   PYRAMID    \
//!     └────────────────┘      ← bottom rectangle (w × h)
//!     ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔      ← inner surface
//!
//!  Top view (looking down through the stone):
//!
//!         ╭──────╮    ← cylinder (circle)
//!        ╱        ╲
//!       ╱  ╭────╮  ╲
//!      │   │    │   │ ← inscribed circle transitions to rectangle
//!       ╲  ╰────╯  ╱
//!        ╲        ╱
//!         ╰──────╯
//!
//!  Bottom view (looking at inner surface):
//!
//!     ┌──────────────┐  ← rectangle (derived from neighbor grid)
//!     │              │
//!     │   ╭──────╮   │  ← where cylinder projects through
//!     │   ╰──────╯   │
//!     │              │
//!     └──────────────┘
//! ```
//!
//! The bottom rectangle dimensions come from the neighbor layout:
//!   width  = distance_to_neighbor_along_row - min_rib_width
//!   height = distance_to_neighbor_across_row - min_rib_width
//!
//! On curved surfaces, the bottom rectangle is projected (ray-cast)
//! onto the actual inner mesh geometry.

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
    /// Triangle mesh of the cutter solid (chimney shape).
    pub mesh: TriMesh,
    /// Cylinder radius (top zone).
    pub cylinder_radius: f64,
    /// Bottom rectangle half-widths [half_w, half_h] (bottom zone).
    pub bottom_half_extents: [f64; 2],
}

/// Generate azure cutters for all detected stone seats.
///
/// Pipeline:
/// 1. Compute bottom rectangle dimensions from neighbor layout
/// 2. For each seat, build the chimney cutter mesh
/// 3. Resolve any remaining neighbor conflicts
pub fn generate_azure_cutters(
    seats: &[StoneSeat],
    ring_mesh: &TriMesh,
    config: &AzureConfig,
) -> Vec<AzureCutter> {
    // Step 1: Compute bottom rectangle extents for each seat based on neighbors
    let bottom_rects = compute_bottom_rectangles(seats, config);

    // Step 2: Generate chimney cutter meshes
    let mut cutters = Vec::new();
    for (i, seat) in seats.iter().enumerate() {
        let [half_w, half_h] = bottom_rects[i];

        // Skip if the rectangle is too small to be meaningful
        if half_w <= 0.01 || half_h <= 0.01 {
            log::warn!("Seat {}: bottom rectangle too small ({:.2}×{:.2}), skipping",
                seat.id, half_w * 2.0, half_h * 2.0);
            continue;
        }

        // Measure local metal thickness via raycast
        let local_thickness = raycast_thickness(ring_mesh, &seat.center, &seat.inward);
        if local_thickness <= config.min_wall_thickness + config.girdle_distance {
            log::warn!("Seat {}: metal too thin ({:.2}mm) for azure cut, skipping",
                seat.id, local_thickness);
            continue;
        }

        let cylinder_radius = seat.radius + config.seat_margin;

        let mesh = build_chimney_cutter(
            seat,
            cylinder_radius,
            half_w,
            half_h,
            local_thickness,
            config,
        );

        cutters.push(AzureCutter {
            seat_id: seat.id,
            mesh,
            cylinder_radius,
            bottom_half_extents: [half_w, half_h],
        });
    }

    log::info!("Generated {} azure cutters", cutters.len());
    cutters
}

/// Compute the bottom rectangle half-extents for each stone seat
/// based on the layout of neighboring stones.
///
/// For each seat, we find its nearest neighbors in two perpendicular
/// directions (along the row and across the row) and derive the
/// rectangle dimensions from the inter-stone spacing.
///
/// The two local axes are:
///   - "row axis" (u): direction to the nearest neighbor (typically along the band)
///   - "cross axis" (v): perpendicular to both u and the inward normal
fn compute_bottom_rectangles(
    seats: &[StoneSeat],
    config: &AzureConfig,
) -> Vec<[f64; 2]> {
    let n = seats.len();
    let mut rects = vec![[0.0f64; 2]; n];

    for i in 0..n {
        // Find distances to all other seats
        let mut neighbor_dists: Vec<(usize, f64)> = (0..n)
            .filter(|&j| j != i)
            .map(|j| {
                let d = nalgebra::distance(&seats[i].center, &seats[j].center);
                (j, d)
            })
            .collect();
        neighbor_dists.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        if neighbor_dists.is_empty() {
            // Single stone — use a default square based on stone radius * 2
            let default_half = seats[i].radius * 1.5;
            rects[i] = [default_half, default_half];
            continue;
        }

        // Nearest neighbor defines the "row" direction
        let nearest_idx = neighbor_dists[0].0;
        let nearest_dist = neighbor_dists[0].1;
        let row_dir = (seats[nearest_idx].center - seats[i].center).normalize();

        // Half-width along row direction:
        // Available space = distance to neighbor, minus both stones' radii safety, minus rib
        let half_w = ((nearest_dist - config.min_rib_width) / 2.0)
            .max(seats[i].radius + config.seat_margin);

        // For cross direction, find nearest neighbor that is NOT roughly along the row
        let inward = seats[i].inward.normalize();
        let cross_dir = inward.cross(&row_dir).normalize();

        let mut min_cross_dist = f64::MAX;
        for &(j, dist) in &neighbor_dists {
            let to_j = (seats[j].center - seats[i].center).normalize();
            let cross_component = to_j.dot(&cross_dir).abs();
            // If this neighbor has significant cross-direction component
            if cross_component > 0.5 {
                let cross_dist = dist * cross_component;
                min_cross_dist = min_cross_dist.min(cross_dist);
            }
        }

        let half_h = if min_cross_dist < f64::MAX {
            ((min_cross_dist - config.min_rib_width) / 2.0)
                .max(seats[i].radius + config.seat_margin)
        } else {
            // No cross-direction neighbor found — use same as width
            half_w
        };

        rects[i] = [half_w, half_h];

        log::debug!(
            "Seat {}: bottom rect = {:.2} × {:.2} mm (nearest neighbor at {:.2}mm)",
            seats[i].id, half_w * 2.0, half_h * 2.0, nearest_dist
        );
    }

    rects
}

/// Build a chimney-shaped cutter mesh for a single stone seat.
///
/// Structure:
///   1. Top cap: circular, at the stone seat level
///   2. Cylinder walls: straight down for girdle_distance
///   3. Optional step: flat ring at cylinder-to-pyramid transition
///   4. Pyramid walls: cylinder circle → bottom rectangle
///   5. Bottom cap: rectangular, at (total_depth + overhang) below seat
///
/// All oriented along the seat's inward normal.
fn build_chimney_cutter(
    seat: &StoneSeat,
    cyl_radius: f64,
    half_w: f64,
    half_h: f64,
    local_thickness: f64,
    config: &AzureConfig,
) -> TriMesh {
    let axis = seat.inward.normalize();
    let segments = config.cylinder_segments;

    // Build local coordinate frame
    let arbitrary = if axis.x.abs() < 0.9 { Vector3::x() } else { Vector3::y() };
    let u_axis = axis.cross(&arbitrary).normalize(); // "width" direction
    let v_axis = axis.cross(&u_axis).normalize();     // "height" direction

    // Key depths along the axis (measured from seat center, positive = inward)
    let d_cylinder_end = config.girdle_distance;
    let d_step_end = d_cylinder_end + config.step_height;
    let d_pyramid_end = local_thickness - config.min_wall_thickness;
    let d_bottom = d_pyramid_end + config.overhang; // extends past inner surface

    // Ensure pyramid zone has positive height
    let _pyramid_height = (d_pyramid_end - d_step_end).max(0.001);

    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    // =========================================
    // RING 0: Top circle (at stone seat level, z=0)
    // =========================================
    let top_center_idx = vertices.len();
    vertices.push(seat.center);

    let ring0_start = vertices.len();
    for i in 0..segments {
        let angle = 2.0 * PI * (i as f64) / (segments as f64);
        let offset = u_axis * angle.cos() * cyl_radius + v_axis * angle.sin() * cyl_radius;
        vertices.push(seat.center + offset);
    }

    // =========================================
    // RING 1: Bottom of cylinder (z = girdle_distance)
    // =========================================
    let cyl_bottom_center = seat.center + axis * d_cylinder_end;
    let ring1_start = vertices.len();
    for i in 0..segments {
        let angle = 2.0 * PI * (i as f64) / (segments as f64);
        let offset = u_axis * angle.cos() * cyl_radius + v_axis * angle.sin() * cyl_radius;
        vertices.push(cyl_bottom_center + offset);
    }

    // =========================================
    // RING 2: Bottom of step (z = girdle_distance + step_height)
    //         Same radius as cylinder — flat shelf
    // =========================================
    let step_bottom_center = seat.center + axis * d_step_end;
    let ring2_start = vertices.len();
    if config.step_height > 0.0 {
        for i in 0..segments {
            let angle = 2.0 * PI * (i as f64) / (segments as f64);
            let offset = u_axis * angle.cos() * cyl_radius + v_axis * angle.sin() * cyl_radius;
            vertices.push(step_bottom_center + offset);
        }
    }
    let ring2_exists = config.step_height > 0.0;

    // =========================================
    // RING 3: Bottom rectangle (z = d_bottom)
    //         4 corners of the rectangle
    // =========================================
    let bottom_center_pos = seat.center + axis * d_bottom;
    let bottom_center_idx = vertices.len();
    vertices.push(bottom_center_pos);

    // Rectangle corners: +w+h, -w+h, -w-h, +w-h
    let rect_start = vertices.len();
    let rect_corners = [
        bottom_center_pos + u_axis * half_w + v_axis * half_h,
        bottom_center_pos - u_axis * half_w + v_axis * half_h,
        bottom_center_pos - u_axis * half_w - v_axis * half_h,
        bottom_center_pos + u_axis * half_w - v_axis * half_h,
    ];
    for corner in &rect_corners {
        vertices.push(*corner);
    }

    // Also add midpoints along each rectangle edge for smoother transition
    let rect_mid_start = vertices.len();
    let rect_midpoints = [
        bottom_center_pos + v_axis * half_h,                      // mid top edge
        bottom_center_pos - u_axis * half_w,                      // mid left edge
        bottom_center_pos - v_axis * half_h,                      // mid bottom edge
        bottom_center_pos + u_axis * half_w,                      // mid right edge
    ];
    for mid in &rect_midpoints {
        vertices.push(*mid);
    }

    // =========================================
    // FACES
    // =========================================

    // --- Top cap: fan from center ---
    for i in 0..segments {
        let next = (i + 1) % segments;
        // Winding for outward normal (opposite to axis)
        faces.push([top_center_idx, ring0_start + next, ring0_start + i]);
    }

    // --- Cylinder walls: ring0 → ring1 ---
    for i in 0..segments {
        let next = (i + 1) % segments;
        faces.push([ring0_start + i, ring1_start + i, ring0_start + next]);
        faces.push([ring0_start + next, ring1_start + i, ring1_start + next]);
    }

    // --- Step walls (if step exists): ring1 → ring2 ---
    if ring2_exists {
        for i in 0..segments {
            let next = (i + 1) % segments;
            faces.push([ring1_start + i, ring2_start + i, ring1_start + next]);
            faces.push([ring1_start + next, ring2_start + i, ring2_start + next]);
        }
    }

    // --- Transition zone: circle (ring1 or ring2) → rectangle ---
    // This is the key geometry: connecting a circular ring to 4 rectangle corners + 4 midpoints.
    // We map each segment of the circle to the nearest point on the rectangle boundary.
    let transition_ring_start = if ring2_exists { ring2_start } else { ring1_start };

    // Build the rectangle boundary as 8 points (4 corners + 4 midpoints) in order
    // Going clockwise: corner0, mid0, corner1, mid1, corner2, mid2, corner3, mid3
    let rect_boundary: Vec<usize> = vec![
        rect_start + 0, rect_mid_start + 0,  // corner TL, mid top
        rect_start + 1, rect_mid_start + 1,  // corner BL, mid left
        rect_start + 2, rect_mid_start + 2,  // corner BR, mid bottom
        rect_start + 3, rect_mid_start + 3,  // corner TR, mid right
    ];
    let rect_count = rect_boundary.len(); // 8

    // Map each cylinder segment to its closest rectangle boundary point
    for i in 0..segments {
        let next = (i + 1) % segments;
        let _cyl_pt = &vertices[transition_ring_start + i];

        // Find which rectangle boundary segment this cylinder point maps to
        let rect_idx = map_circle_to_rect_index(i, segments, rect_count);
        let rect_next = map_circle_to_rect_index(next, segments, rect_count);

        let ri = rect_boundary[rect_idx];
        let ri_next = rect_boundary[rect_next];

        if rect_idx == rect_next {
            // Both circle points map to the same rect point — single triangle
            faces.push([transition_ring_start + i, ri, transition_ring_start + next]);
        } else {
            // Quad between circle edge and rect edge
            faces.push([transition_ring_start + i, ri, transition_ring_start + next]);
            faces.push([transition_ring_start + next, ri, ri_next]);
        }
    }

    // --- Bottom cap: rectangle fan from center ---
    // 4 triangles from center to each rectangle edge
    for i in 0..4 {
        let c0 = rect_start + i;
        let mid = rect_mid_start + i;
        let c1 = rect_start + (i + 1) % 4;
        faces.push([bottom_center_idx, c0, mid]);
        faces.push([bottom_center_idx, mid, c1]);
    }

    // Compute per-face normals
    let normals = faces
        .iter()
        .map(|f| {
            let e1 = vertices[f[1]] - vertices[f[0]];
            let e2 = vertices[f[2]] - vertices[f[0]];
            let n = e1.cross(&e2);
            if n.norm() > 1e-12 { n.normalize() } else { Vector3::z() }
        })
        .collect();

    TriMesh { vertices, faces, normals }
}

/// Map a cylinder segment index to the nearest rectangle boundary index.
///
/// The circle has `segments` points, the rectangle has `rect_count` points.
/// We map by angle: each rectangle point sits at a known angle, and we
/// find which rectangle segment the circle point falls in.
fn map_circle_to_rect_index(circle_idx: usize, segments: usize, rect_count: usize) -> usize {
    // Circle point angle (0..2π)
    let angle = 2.0 * PI * (circle_idx as f64) / (segments as f64);

    // Rectangle boundary points are evenly spaced in angle for simplicity
    // (corners at 45°, 135°, 225°, 315°; midpoints at 0°, 90°, 180°, 270°)
    // But our ordering is: corner0(45°), mid0(90°), corner1(135°), mid1(180°), ...
    //
    // Simpler: map linearly
    let frac = angle / (2.0 * PI); // 0.0 .. 1.0
    let idx = (frac * rect_count as f64).floor() as usize;
    idx.min(rect_count - 1)
}

/// Raycast from `origin` in direction `dir` to find the inner surface distance.
fn raycast_thickness(
    mesh: &TriMesh,
    origin: &Point3<f64>,
    dir: &Vector3<f64>,
) -> f64 {
    let dir_norm = dir.normalize();
    let mut min_t = f64::MAX;

    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0]];
        let v1 = mesh.vertices[face[1]];
        let v2 = mesh.vertices[face[2]];

        if let Some(t) = ray_triangle_intersect(origin, &dir_norm, &v0, &v1, &v2) {
            if t > 0.05 && t < min_t {
                min_t = t;
            }
        }
    }

    if min_t == f64::MAX {
        log::warn!("Raycast found no hit from ({:.2},{:.2},{:.2}) — defaulting thickness to 2.0",
            origin.x, origin.y, origin.z);
        2.0 // safe fallback
    } else {
        min_t
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
    if a.abs() < epsilon { return None; }
    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) { return None; }
    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 { return None; }
    let t = f * edge2.dot(&q);
    if t > epsilon { Some(t) } else { None }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_chimney_cutter_vertex_count() {
        let seat = StoneSeat {
            id: 0,
            center: Point3::origin(),
            radius: 0.75,
            normal: Vector3::z(),
            inward: -Vector3::z(),
            local_thickness: 3.0,
            boundary_vertices: vec![],
        };

        let config = AzureConfig {
            girdle_distance: 0.5,
            taper_angle: 15.0,
            step_height: 0.0,      // no step
            overhang: 0.1,
            min_wall_thickness: 0.5,
            min_rib_width: 0.4,
            seat_margin: 0.2,
            cylinder_segments: 16,
        };

        let mesh = build_chimney_cutter(&seat, 0.95, 1.5, 1.2, 3.0, &config);

        // Vertices: top_center(1) + ring0(16) + ring1(16) + bottom_center(1) + rect_corners(4) + rect_mids(4)
        // No step → no ring2
        assert_eq!(mesh.vertices.len(), 1 + 16 + 16 + 1 + 4 + 4);
    }

    #[test]
    fn test_chimney_cutter_with_step() {
        let seat = StoneSeat {
            id: 0,
            center: Point3::origin(),
            radius: 0.75,
            normal: Vector3::z(),
            inward: -Vector3::z(),
            local_thickness: 3.0,
            boundary_vertices: vec![],
        };

        let config = AzureConfig {
            girdle_distance: 0.5,
            taper_angle: 15.0,
            step_height: 0.3,      // with step
            overhang: 0.1,
            min_wall_thickness: 0.5,
            min_rib_width: 0.4,
            seat_margin: 0.2,
            cylinder_segments: 16,
        };

        let mesh = build_chimney_cutter(&seat, 0.95, 1.5, 1.2, 3.0, &config);

        // With step: adds ring2(16)
        assert_eq!(mesh.vertices.len(), 1 + 16 + 16 + 16 + 1 + 4 + 4);
    }

    #[test]
    fn test_bottom_rectangles_two_stones() {
        let config = AzureConfig {
            girdle_distance: 0.5,
            taper_angle: 15.0,
            step_height: 0.0,
            overhang: 0.1,
            min_wall_thickness: 0.5,
            min_rib_width: 0.4,
            seat_margin: 0.2,
            cylinder_segments: 32,
        };

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
                center: Point3::new(3.0, 0.0, 0.0), // 3mm apart
                radius: 0.5,
                normal: Vector3::z(),
                inward: -Vector3::z(),
                local_thickness: 2.0,
                boundary_vertices: vec![],
            },
        ];

        let rects = compute_bottom_rectangles(&seats, &config);

        // Available along row = (3.0 - 0.4) / 2 = 1.3
        // half_w should be 1.3
        assert!(rects[0][0] > 0.5, "half_w should be > stone radius");
        assert!(rects[0][0] <= 1.3 + 0.01, "half_w should respect rib width");
    }

    #[test]
    fn test_circle_to_rect_mapping() {
        // 32 circle segments, 8 rect points
        // Segment 0 (angle=0) should map to rect index 0
        assert_eq!(map_circle_to_rect_index(0, 32, 8), 0);
        // Segment 4 (angle=π/4) should map to rect index 1
        assert_eq!(map_circle_to_rect_index(4, 32, 8), 1);
        // Segment 16 (angle=π) should map to rect index 4
        assert_eq!(map_circle_to_rect_index(16, 32, 8), 4);
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
        assert!((t.unwrap() - 3.0).abs() < 1e-6);
    }
}
