//! Boolean subtraction module.
//!
//! Takes the original ring mesh and subtracts all azure cutter solids from it.
//! Uses CSG (Constructive Solid Geometry) difference operations via csgrs.

use crate::azure::AzureCutter;
use crate::mesh::TriMesh;
use anyhow::Result;
use csgrs::mesh::polygon::Polygon;
use csgrs::mesh::vertex::Vertex as CsgVertex;
use csgrs::mesh::Mesh as CsgMesh;
use csgrs::traits::CSG;
use nalgebra::{Point3, Vector3};

/// Subtract all azure cutters from the ring mesh.
///
/// Strategy: iterate over cutters and subtract each one sequentially.
/// Each subtraction modifies the mesh for the next operation.
pub fn subtract_cutters(ring: &TriMesh, cutters: &[AzureCutter]) -> Result<TriMesh> {
    if cutters.is_empty() {
        log::warn!("No cutters to subtract — returning original mesh");
        return Ok(ring.clone());
    }

    log::info!("Subtracting {} azure cutters from ring mesh", cutters.len());

    // Convert the ring mesh to csgrs Mesh
    let mut result = trimesh_to_csg(ring);

    for (i, cutter) in cutters.iter().enumerate() {
        log::debug!("Subtracting cutter {} (seat {})", i, cutter.seat_id);
        let cutter_csg = trimesh_to_csg(&cutter.mesh);
        result = result.difference(&cutter_csg);
    }

    let result_mesh = csg_to_trimesh(&result);

    log::info!(
        "Boolean complete: {} faces in result (was {})",
        result_mesh.faces.len(),
        ring.faces.len()
    );

    Ok(result_mesh)
}

/// Convert our TriMesh to a csgrs Mesh<()>.
fn trimesh_to_csg(mesh: &TriMesh) -> CsgMesh<()> {
    let polygons: Vec<Polygon<()>> = mesh
        .faces
        .iter()
        .enumerate()
        .map(|(fi, face)| {
            let n = &mesh.normals[fi];
            let verts: Vec<CsgVertex> = face
                .iter()
                .map(|&vi| {
                    let p = &mesh.vertices[vi];
                    CsgVertex::new(
                        Point3::new(p.x, p.y, p.z),
                        Vector3::new(n.x, n.y, n.z),
                    )
                })
                .collect();
            Polygon::new(verts, None)
        })
        .collect();

    CsgMesh::from_polygons(&polygons, None)
}

/// Convert a csgrs Mesh back to our TriMesh.
fn csg_to_trimesh(csg: &CsgMesh<()>) -> TriMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[usize; 3]> = Vec::new();
    let mut normals: Vec<Vector3<f64>> = Vec::new();

    for poly in &csg.polygons {
        let poly_verts = &poly.vertices;
        if poly_verts.len() < 3 {
            continue;
        }

        // Record base index for this polygon's vertices
        let base_idx = vertices.len();
        for v in poly_verts {
            vertices.push(v.pos);
        }

        // Fan triangulation for polygons with > 3 vertices
        for i in 1..(poly_verts.len() - 1) {
            faces.push([base_idx, base_idx + i, base_idx + i + 1]);
            // Compute face normal from triangle edges
            let e1 = vertices[base_idx + i] - vertices[base_idx];
            let e2 = vertices[base_idx + i + 1] - vertices[base_idx];
            normals.push(e1.cross(&e2).normalize());
        }
    }

    TriMesh {
        vertices,
        faces,
        normals,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Basic smoke test — subtract an empty set of cutters
    #[test]
    fn test_no_cutters() {
        let ring = TriMesh {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            faces: vec![[0, 1, 2]],
            normals: vec![Vector3::new(0.0, 0.0, 1.0)],
        };

        let result = subtract_cutters(&ring, &[]).unwrap();
        assert_eq!(result.faces.len(), 1);
    }
}
