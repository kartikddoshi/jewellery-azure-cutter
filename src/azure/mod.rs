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
//!   ZONE 2 (bottom) — TRUNCATED PYRAMID (rectangle or hexagon):
//!     Below the cylinder, the shape transitions to a polygon
//!     cross-section that widens toward the inner surface.
//!     The bottom face dimensions are governed by the spacing between
//!     neighboring stones minus rib walls.
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
//!     └────────────────┘      ← bottom polygon (rect or hex)
//!     ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔      ← inner surface
//!
//!  Bottom view — Rectangle:       Bottom view — Hexagon (honeycomb):
//!
//!     ┌──────────────┐             ╱╲────────╱╲
//!     │              │            ╱  │        │ ╲
//!     │   ╭──────╮   │           │   │╭────╮  │  │
//!     │   ╰──────╯   │           │   │╰────╯  │  │
//!     │              │            ╲  │        │ ╱
//!     └──────────────┘             ╲╱────────╲╱
//! ```
//!
//! Bottom shape selection:
//!   - Rectangle: best for linear stone rows
//!   - Hexagon: best for clustered round stones, especially on domed surfaces.
//!     Honeycomb tessellation minimizes rib waste and provides uniform walls.

use crate::config::{AzureConfig, BottomShape};
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
    // Step 1: Compute bottom rectangle extents for each seat based on neighbors + taper
    let bottom_rects = compute_bottom_rectangles(seats, ring_mesh, config);

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
/// based on the layout of neighboring stones and the taper angle.
///
/// For each seat, we find its nearest neighbors in two perpendicular
/// directions (along the row and across the row) and derive the
/// rectangle dimensions from:
///   1. The taper angle — how much the pyramid can widen per mm of depth
///   2. The inter-stone spacing — neighbor distances minus rib walls
///
/// The final rectangle is the smaller of the two constraints.
///
/// The two local axes are:
///   - "row axis" (u): direction to the nearest neighbor (typically along the band)
///   - "cross axis" (v): perpendicular to both u and the inward normal
fn compute_bottom_rectangles(
    seats: &[StoneSeat],
    ring_mesh: &TriMesh,
    config: &AzureConfig,
) -> Vec<[f64; 2]> {
    let n = seats.len();
    let mut rects = vec![[0.0f64; 2]; n];

    for i in 0..n {
        let cyl_radius = seats[i].radius + config.seat_margin;

        // Compute the taper-limited max half-extent:
        // The pyramid starts at depth = girdle_distance + step_height
        // and ends at depth = local_thickness - min_wall_thickness.
        // At the taper angle, each side expands by tan(taper_angle) * pyramid_height.
        let local_thickness = raycast_thickness(ring_mesh, &seats[i].center, &seats[i].inward);
        let pyramid_height = (local_thickness - config.min_wall_thickness
            - config.girdle_distance - config.step_height)
            .max(0.0);
        let taper_rad = config.taper_angle.to_radians();
        let taper_expansion = taper_rad.tan() * pyramid_height;
        let taper_max_half = cyl_radius + taper_expansion;

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
            // Single stone — taper-limited rectangle
            let half = taper_max_half;
            rects[i] = [half, half];
            log::debug!("Seat {}: single stone, bottom rect = {:.2} × {:.2} mm (taper-limited)",
                seats[i].id, half * 2.0, half * 2.0);
            continue;
        }

        // Nearest neighbor defines the "row" direction
        let nearest_idx = neighbor_dists[0].0;
        let nearest_dist = neighbor_dists[0].1;
        let row_dir = (seats[nearest_idx].center - seats[i].center).normalize();

        // Half-width along row direction:
        // Available space from neighbor = (distance - rib_width) / 2
        // Clamped to at least cylinder radius (otherwise cut is pointless)
        // and at most the taper-limited max.
        let neighbor_half_w = (nearest_dist - config.min_rib_width) / 2.0;
        let half_w = neighbor_half_w
            .min(taper_max_half)       // don't exceed taper limit
            .max(cyl_radius);          // at least cylinder radius (minimum useful cut)

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
            let neighbor_half_h = (min_cross_dist - config.min_rib_width) / 2.0;
            neighbor_half_h
                .min(taper_max_half)
                .max(cyl_radius)
        } else {
            // No cross-direction neighbor found — use same as width
            half_w
        };

        rects[i] = [half_w, half_h];

        log::debug!(
            "Seat {}: bottom rect = {:.2} × {:.2} mm (nearest neighbor at {:.2}mm, taper max {:.2}mm)",
            seats[i].id, half_w * 2.0, half_h * 2.0, nearest_dist, taper_max_half * 2.0
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
    // BOTTOM POLYGON (z = d_bottom)
    // Shape depends on config: rectangle (8 pts) or hexagon (12 pts)
    // =========================================
    let bottom_center_pos = seat.center + axis * d_bottom;
    let bottom_center_idx = vertices.len();
    vertices.push(bottom_center_pos);

    // Generate bottom boundary points based on shape
    let bottom_boundary_start = vertices.len();
    let bottom_boundary_2d = generate_bottom_boundary_2d(config.bottom_shape, half_w, half_h);
    let bottom_count = bottom_boundary_2d.len();

    for &(bu, bv) in &bottom_boundary_2d {
        vertices.push(bottom_center_pos + u_axis * bu + v_axis * bv);
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

    // --- Transition zone: circle (ring1 or ring2) → bottom polygon ---
    // Connects the circular ring to the bottom boundary points using
    // angular proximity mapping.
    let transition_ring_start = if ring2_exists { ring2_start } else { ring1_start };

    // Map each cylinder segment to its closest bottom boundary point
    for i in 0..segments {
        let next = (i + 1) % segments;

        let bi = map_circle_to_polygon_index(i, segments, &bottom_boundary_2d);
        let bi_next = map_circle_to_polygon_index(next, segments, &bottom_boundary_2d);

        let ri = bottom_boundary_start + bi;
        let ri_next = bottom_boundary_start + bi_next;

        if bi == bi_next {
            // Both circle points map to the same polygon point — single triangle
            faces.push([transition_ring_start + i, ri, transition_ring_start + next]);
        } else {
            // Quad between circle edge and polygon edge
            faces.push([transition_ring_start + i, ri, transition_ring_start + next]);
            faces.push([transition_ring_start + next, ri, ri_next]);
        }
    }

    // --- Bottom cap: polygon fan from center ---
    for i in 0..bottom_count {
        let p0 = bottom_boundary_start + i;
        let p1 = bottom_boundary_start + (i + 1) % bottom_count;
        faces.push([bottom_center_idx, p1, p0]);
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

/// Generate 2D boundary points for the bottom polygon in the local (u, v) frame.
///
/// Rectangle: 8 points (4 corners + 4 edge midpoints) for smoother transition.
/// Hexagon: 12 points (6 corners + 6 edge midpoints) for honeycomb tessellation.
fn generate_bottom_boundary_2d(shape: BottomShape, half_w: f64, half_h: f64) -> Vec<(f64, f64)> {
    match shape {
        BottomShape::Rectangle => {
            // 8 points in angular order (CCW from +u axis perspective):
            // corner(+w,+h), mid(0,+h), corner(-w,+h), mid(-w,0),
            // corner(-w,-h), mid(0,-h), corner(+w,-h), mid(+w,0)
            vec![
                ( half_w,  half_h),   // corner 0
                ( 0.0,     half_h),   // mid top
                (-half_w,  half_h),   // corner 1
                (-half_w,  0.0),      // mid left
                (-half_w, -half_h),   // corner 2
                ( 0.0,    -half_h),   // mid bottom
                ( half_w, -half_h),   // corner 3
                ( half_w,  0.0),      // mid right
            ]
        }
        BottomShape::Hexagon => {
            // Regular hexagon inscribed in the available space.
            // Use the smaller of half_w and half_h as the circumradius
            // to ensure the hex fits within the neighbor constraints.
            let r = half_w.min(half_h);
            // 12 points: 6 corners + 6 midpoints (for smoother circle-to-hex transition)
            let mut pts = Vec::with_capacity(12);
            for i in 0..6 {
                // Corner at angle i*60°
                let angle = PI / 3.0 * (i as f64);
                pts.push((r * angle.cos(), r * angle.sin()));
                // Midpoint between corner i and corner i+1
                let mid_angle = PI / 3.0 * (i as f64 + 0.5);
                let mid_r = r * (PI / 6.0).cos(); // inradius = r * cos(30°)
                pts.push((mid_r * mid_angle.cos(), mid_r * mid_angle.sin()));
            }
            pts
        }
    }
}

/// Map a cylinder segment index to the nearest bottom polygon boundary index.
///
/// Uses actual angular positions of the boundary points to find the closest match.
fn map_circle_to_polygon_index(circle_idx: usize, segments: usize, boundary_2d: &[(f64, f64)]) -> usize {
    let circle_angle = 2.0 * PI * (circle_idx as f64) / (segments as f64);

    let mut best_idx = 0;
    let mut best_diff = f64::MAX;

    for (i, &(x, y)) in boundary_2d.iter().enumerate() {
        let pt_angle = y.atan2(x).rem_euclid(2.0 * PI);
        let mut diff = (circle_angle - pt_angle).abs();
        if diff > PI {
            diff = 2.0 * PI - diff;
        }
        if diff < best_diff {
            best_diff = diff;
            best_idx = i;
        }
    }

    best_idx
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
            bottom_shape: BottomShape::Rectangle,
        };

        let mesh = build_chimney_cutter(&seat, 0.95, 1.5, 1.2, 3.0, &config);

        // Vertices: top_center(1) + ring0(16) + ring1(16) + bottom_center(1) + rect_boundary(8)
        // No step → no ring2
        assert_eq!(mesh.vertices.len(), 1 + 16 + 16 + 1 + 8);
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
            bottom_shape: BottomShape::Rectangle,
        };

        let mesh = build_chimney_cutter(&seat, 0.95, 1.5, 1.2, 3.0, &config);

        // With step: adds ring2(16)
        assert_eq!(mesh.vertices.len(), 1 + 16 + 16 + 16 + 1 + 8);
    }

    #[test]
    fn test_chimney_cutter_hexagon_vertex_count() {
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
            step_height: 0.0,
            overhang: 0.1,
            min_wall_thickness: 0.5,
            min_rib_width: 0.4,
            seat_margin: 0.2,
            cylinder_segments: 16,
            bottom_shape: BottomShape::Hexagon,
        };

        let mesh = build_chimney_cutter(&seat, 0.95, 1.5, 1.2, 3.0, &config);

        // Vertices: top_center(1) + ring0(16) + ring1(16) + bottom_center(1) + hex_boundary(12)
        assert_eq!(mesh.vertices.len(), 1 + 16 + 16 + 1 + 12);
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
            bottom_shape: BottomShape::Rectangle,
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

        // Create a simple mesh for raycasting (a plane at z=-2 so thickness = 2.0)
        let ring_mesh = TriMesh {
            vertices: vec![
                Point3::new(-10.0, -10.0, -2.0),
                Point3::new( 10.0, -10.0, -2.0),
                Point3::new(  0.0,  10.0, -2.0),
            ],
            faces: vec![[0, 1, 2]],
            normals: vec![Vector3::new(0.0, 0.0, 1.0)],
        };

        let rects = compute_bottom_rectangles(&seats, &ring_mesh, &config);

        let cyl_radius = 0.5 + 0.2; // radius + seat_margin = 0.7
        // Available along row = (3.0 - 0.4) / 2 = 1.3
        // Taper max = cyl_radius + tan(15°) * (2.0 - 0.5 - 0.5) = 0.7 + 0.268 = 0.968
        // half_w = min(1.3, 0.968).max(0.7) = 0.968 (taper-limited)
        assert!(rects[0][0] >= cyl_radius, "half_w should be >= cylinder radius");
        assert!(rects[0][0] <= 1.3 + 0.01, "half_w should respect rib width");
    }

    #[test]
    fn test_circle_to_polygon_mapping_rect() {
        let boundary = generate_bottom_boundary_2d(BottomShape::Rectangle, 1.0, 1.0);
        // 32 circle segments, rect has 8 boundary points
        // Segment 0 (angle=0) → (+w, 0) = mid_right = index 7
        assert_eq!(map_circle_to_polygon_index(0, 32, &boundary), 7);
        // Segment 4 (angle=π/4=45°) → corner (+w,+h) = index 0
        assert_eq!(map_circle_to_polygon_index(4, 32, &boundary), 0);
        // Segment 8 (angle=π/2=90°) → mid top (0,+h) = index 1
        assert_eq!(map_circle_to_polygon_index(8, 32, &boundary), 1);
        // Segment 16 (angle=π=180°) → mid left (-w,0) = index 3
        assert_eq!(map_circle_to_polygon_index(16, 32, &boundary), 3);
    }

    #[test]
    fn test_circle_to_polygon_mapping_hex() {
        let boundary = generate_bottom_boundary_2d(BottomShape::Hexagon, 1.0, 1.0);
        assert_eq!(boundary.len(), 12);
        // Segment 0 (angle=0) → hex corner 0 at angle 0° = index 0
        assert_eq!(map_circle_to_polygon_index(0, 32, &boundary), 0);
        // Segment 8 (angle=π/2=90°) → should be near hex at 90° (corner at 60°, mid at 90°)
        // Mid between corner1(60°) and corner2(120°) is at 90° = index 3
        assert_eq!(map_circle_to_polygon_index(8, 32, &boundary), 3);
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

    #[test]
    fn test_taper_angle_constrains_rectangle() {
        // With a small taper angle (5°), the rectangle should be limited
        // even when neighbors are far apart.
        let config = AzureConfig {
            girdle_distance: 0.5,
            taper_angle: 5.0,   // small angle → tight constraint
            step_height: 0.0,
            overhang: 0.1,
            min_wall_thickness: 0.5,
            min_rib_width: 0.4,
            seat_margin: 0.2,
            cylinder_segments: 32,
            bottom_shape: BottomShape::Rectangle,
        };

        let seats = vec![StoneSeat {
            id: 0,
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 0.5,
            normal: Vector3::z(),
            inward: -Vector3::z(),
            local_thickness: 2.0,
            boundary_vertices: vec![],
        }];

        // Plane at z=-2 for raycasting
        let ring_mesh = TriMesh {
            vertices: vec![
                Point3::new(-10.0, -10.0, -2.0),
                Point3::new( 10.0, -10.0, -2.0),
                Point3::new(  0.0,  10.0, -2.0),
            ],
            faces: vec![[0, 1, 2]],
            normals: vec![Vector3::new(0.0, 0.0, 1.0)],
        };

        let rects = compute_bottom_rectangles(&seats, &ring_mesh, &config);
        let cyl_radius = 0.5 + 0.2; // 0.7
        // pyramid_height = 2.0 - 0.5 - 0.5 = 1.0
        // taper_expansion = tan(5°) * 1.0 ≈ 0.0875
        // taper_max_half = 0.7 + 0.0875 ≈ 0.7875
        let expected_max = cyl_radius + (5.0f64.to_radians().tan() * 1.0);
        assert!((rects[0][0] - expected_max).abs() < 0.01,
            "Single stone rect should be taper-limited: got {:.3}, expected {:.3}",
            rects[0][0], expected_max);
    }

    #[test]
    fn test_chimney_cutter_faces_are_valid() {
        // Verify all face indices are within bounds and normals are unit vectors
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
            step_height: 0.0,
            overhang: 0.1,
            min_wall_thickness: 0.5,
            min_rib_width: 0.4,
            seat_margin: 0.2,
            cylinder_segments: 16,
            bottom_shape: BottomShape::Rectangle,
        };

        let mesh = build_chimney_cutter(&seat, 0.95, 1.5, 1.2, 3.0, &config);

        // All face indices should be valid
        for (fi, face) in mesh.faces.iter().enumerate() {
            for &vi in face {
                assert!(vi < mesh.vertices.len(),
                    "Face {} references vertex {} but only {} vertices exist",
                    fi, vi, mesh.vertices.len());
            }
        }

        // Each face should have a corresponding normal
        assert_eq!(mesh.faces.len(), mesh.normals.len(),
            "faces and normals count must match");

        // Normals should be approximately unit length
        for (i, n) in mesh.normals.iter().enumerate() {
            let len = n.norm();
            assert!((len - 1.0).abs() < 0.01 || len < 1e-10,
                "Normal {} has length {:.4}, expected ~1.0", i, len);
        }
    }
}
