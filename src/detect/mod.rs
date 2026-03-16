//! Stone hole detection module.
//!
//! Given a TriMesh with pre-drilled stone holes, this module:
//! 1. Finds all boundary loops (holes) in the mesh
//! 2. Fits circles to each loop to identify round stone seats
//! 3. Filters by diameter range and orientation
//! 4. Computes local metal thickness at each seat by raycasting

use crate::config::StoneConfig;
use crate::mesh::TriMesh;
use anyhow::Result;
use nalgebra::{Point3, Vector3};

/// A detected stone seat in the mesh.
#[derive(Debug, Clone)]
pub struct StoneSeat {
    /// Unique ID for this seat.
    pub id: usize,
    /// Center of the fitted circle (in mesh coordinates, mm).
    pub center: Point3<f64>,
    /// Radius of the fitted circle (mm).
    pub radius: f64,
    /// Outward-pointing normal at the stone seat (direction stone faces).
    pub normal: Vector3<f64>,
    /// Inward normal (opposite of `normal` — direction toward ring interior).
    pub inward: Vector3<f64>,
    /// Local metal thickness along the seat axis (mm).
    /// Measured from center of hole to the opposite inner surface.
    pub local_thickness: f64,
    /// Vertex indices of the boundary loop for this hole.
    pub boundary_vertices: Vec<usize>,
}

/// Detect all stone seats in the mesh.
pub fn detect_stone_seats(mesh: &TriMesh, config: &StoneConfig) -> Result<Vec<StoneSeat>> {
    let loops = mesh.boundary_loops();
    let mut seats = Vec::new();
    let mut id = 0;

    // Pre-filter: skip the largest loop (likely the finger hole)
    // by finding the loop with the largest bounding radius
    let finger_hole_idx = loops
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            let ra = bounding_radius(a, &mesh.vertices);
            let rb = bounding_radius(b, &mesh.vertices);
            ra.partial_cmp(&rb).unwrap()
        })
        .map(|(i, _)| i);

    for (loop_idx, loop_verts) in loops.iter().enumerate() {
        // Skip the finger hole
        if Some(loop_idx) == finger_hole_idx {
            log::debug!("Skipping loop {} as likely finger hole", loop_idx);
            continue;
        }

        // Fit a circle to this boundary loop
        let points: Vec<Point3<f64>> = loop_verts
            .iter()
            .map(|&vi| mesh.vertices[vi])
            .collect();

        let (center, radius, normal) = fit_circle_3d(&points)?;

        // Filter by diameter range
        let diameter = radius * 2.0;
        if diameter < config.diameter_min || diameter > config.diameter_max {
            log::debug!(
                "Loop {}: diameter {:.2}mm outside range [{}, {}], skipping",
                loop_idx, diameter, config.diameter_min, config.diameter_max
            );
            continue;
        }

        // Determine outward vs inward normal.
        // Heuristic: the outward normal points away from the mesh centroid.
        let mesh_centroid = mesh_centroid(&mesh.vertices);
        let to_center = center - mesh_centroid;
        let outward = if normal.dot(&to_center) > 0.0 {
            normal
        } else {
            -normal
        };
        let inward = -outward;

        // Measure local metal thickness by raycasting from the hole center
        // inward along the seat axis until we hit the opposite wall.
        let local_thickness = raycast_thickness(mesh, &center, &inward);

        log::info!(
            "Detected stone seat {}: center=({:.2},{:.2},{:.2}), r={:.2}mm, thickness={:.2}mm",
            id, center.x, center.y, center.z, radius, local_thickness
        );

        seats.push(StoneSeat {
            id,
            center,
            radius,
            normal: outward,
            inward,
            local_thickness,
            boundary_vertices: loop_verts.clone(),
        });
        id += 1;
    }

    log::info!("Detected {} stone seats total", seats.len());
    Ok(seats)
}

/// Compute the centroid of a set of vertices.
fn mesh_centroid(vertices: &[Point3<f64>]) -> Point3<f64> {
    let sum: Vector3<f64> = vertices.iter().map(|v| v.coords).sum();
    Point3::from(sum / vertices.len() as f64)
}

/// Compute the bounding radius of a loop (max distance from centroid).
fn bounding_radius(loop_verts: &[usize], vertices: &[Point3<f64>]) -> f64 {
    let points: Vec<Point3<f64>> = loop_verts.iter().map(|&vi| vertices[vi]).collect();
    let c = mesh_centroid(&points);
    points
        .iter()
        .map(|p| nalgebra::distance(p, &c))
        .fold(0.0f64, f64::max)
}

/// Fit a circle in 3D to a set of roughly coplanar points.
///
/// Steps:
/// 1. Compute the best-fit plane (via centroid + PCA / SVD of covariance).
/// 2. Project points onto that plane.
/// 3. Fit a 2D circle to the projected points.
/// 4. Unproject the circle center back to 3D.
///
/// Returns (center, radius, plane_normal).
fn fit_circle_3d(points: &[Point3<f64>]) -> Result<(Point3<f64>, f64, Vector3<f64>)> {
    let n = points.len() as f64;

    // Centroid
    let centroid = Point3::from(
        points.iter().map(|p| p.coords).sum::<Vector3<f64>>() / n,
    );

    // Covariance matrix for PCA
    let mut cov = nalgebra::Matrix3::zeros();
    for p in points {
        let d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= n;

    // SVD — the smallest singular value's column is the plane normal
    let svd = cov.svd(true, true);
    let u = svd.u.unwrap();
    // Columns of U sorted by singular values (descending).
    // The 3rd column (index 2) corresponds to the smallest singular value = plane normal.
    let normal = Vector3::new(u[(0, 2)], u[(1, 2)], u[(2, 2)]).normalize();

    // Two in-plane basis vectors
    let u_axis = Vector3::new(u[(0, 0)], u[(1, 0)], u[(2, 0)]).normalize();
    let v_axis = Vector3::new(u[(0, 1)], u[(1, 1)], u[(2, 1)]).normalize();

    // Project to 2D
    let pts_2d: Vec<(f64, f64)> = points
        .iter()
        .map(|p| {
            let d = p - centroid;
            (d.dot(&u_axis), d.dot(&v_axis))
        })
        .collect();

    // Fit 2D circle using algebraic method (Kasa method)
    // Solve: A * [a, b, c]^T = d  where circle eqn: x^2+y^2 + ax + by + c = 0
    let mut mat_a = nalgebra::DMatrix::zeros(pts_2d.len(), 3);
    let mut vec_d = nalgebra::DVector::zeros(pts_2d.len());
    for (i, &(x, y)) in pts_2d.iter().enumerate() {
        mat_a[(i, 0)] = x;
        mat_a[(i, 1)] = y;
        mat_a[(i, 2)] = 1.0;
        vec_d[i] = -(x * x + y * y);
    }

    let solution = mat_a
        .svd(true, true)
        .solve(&vec_d, 1e-10)
        .map_err(|e| anyhow::anyhow!("Circle fit SVD solve failed: {}", e))?;

    let cx_2d = -solution[0] / 2.0;
    let cy_2d = -solution[1] / 2.0;
    let radius = (cx_2d * cx_2d + cy_2d * cy_2d - solution[2]).sqrt();

    // Unproject center back to 3D
    let center_3d = centroid + u_axis * cx_2d + v_axis * cy_2d;

    Ok((center_3d, radius, normal))
}

/// Raycast from `origin` in direction `dir` and find the distance to the
/// first triangle hit in the mesh. Returns the distance (metal thickness).
///
/// Uses a simple brute-force ray-triangle intersection for now.
/// TODO: Accelerate with BVH (parry3d) for large meshes.
fn raycast_thickness(mesh: &TriMesh, origin: &Point3<f64>, dir: &Vector3<f64>) -> f64 {
    let mut min_t = f64::MAX;
    let dir_norm = dir.normalize();

    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0]];
        let v1 = mesh.vertices[face[1]];
        let v2 = mesh.vertices[face[2]];

        if let Some(t) = ray_triangle_intersect(origin, &dir_norm, &v0, &v1, &v2) {
            // Only consider hits in the forward direction and beyond a small epsilon
            // (to avoid self-intersection at the hole boundary)
            if t > 0.01 && t < min_t {
                min_t = t;
            }
        }
    }

    if min_t == f64::MAX {
        log::warn!("Raycast found no hit — defaulting thickness to 0.0");
        0.0
    } else {
        min_t
    }
}

/// Möller–Trumbore ray-triangle intersection.
/// Returns Some(t) if the ray hits, where t is the distance along the ray.
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
        return None; // Ray is parallel to triangle
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
    if t > epsilon {
        Some(t)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ray_triangle_hit() {
        let origin = Point3::new(0.0, 0.0, -1.0);
        let dir = Vector3::new(0.0, 0.0, 1.0);
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        let t = ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2);
        assert!(t.is_some());
        assert!((t.unwrap() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_ray_triangle_miss() {
        let origin = Point3::new(5.0, 5.0, -1.0);
        let dir = Vector3::new(0.0, 0.0, 1.0);
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        let t = ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2);
        assert!(t.is_none());
    }
}
