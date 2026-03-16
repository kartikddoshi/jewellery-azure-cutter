//! Mesh module — handles STL loading, indexed mesh representation, and
//! topology computation (boundary edges, edge loops).

use anyhow::{Context, Result};
use nalgebra::Point3;
use std::collections::HashMap;
use std::path::Path;

/// A half-edge representation of an edge: (vertex_index_a, vertex_index_b).
/// Always stored with the smaller index first for canonical form.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Edge(pub usize, pub usize);

impl Edge {
    pub fn new(a: usize, b: usize) -> Self {
        if a <= b { Edge(a, b) } else { Edge(b, a) }
    }
}

/// An indexed triangle mesh loaded from STL.
#[derive(Debug, Clone)]
pub struct TriMesh {
    /// Vertex positions (deduplicated).
    pub vertices: Vec<Point3<f64>>,
    /// Triangle faces as indices into `vertices`. Each triple is (v0, v1, v2).
    pub faces: Vec<[usize; 3]>,
    /// Per-face normals.
    pub normals: Vec<nalgebra::Vector3<f64>>,
}

impl TriMesh {
    /// Load an STL file and build an indexed mesh.
    /// Uses stl_io's built-in vertex deduplication via IndexedMesh.
    pub fn from_stl(path: &Path) -> Result<Self> {
        let mut file = std::fs::File::open(path)
            .with_context(|| format!("Cannot open STL: {}", path.display()))?;
        let stl = stl_io::read_stl(&mut file)
            .with_context(|| "Failed to parse STL file")?;

        // stl_io::IndexedMesh already provides deduplicated vertices and indexed faces
        let vertices: Vec<Point3<f64>> = stl
            .vertices
            .iter()
            .map(|v| Point3::new(v[0] as f64, v[1] as f64, v[2] as f64))
            .collect();

        let faces: Vec<[usize; 3]> = stl
            .faces
            .iter()
            .map(|f| f.vertices)
            .collect();

        let normals: Vec<nalgebra::Vector3<f64>> = stl
            .faces
            .iter()
            .map(|f| {
                nalgebra::Vector3::new(
                    f.normal[0] as f64,
                    f.normal[1] as f64,
                    f.normal[2] as f64,
                )
            })
            .collect();

        log::info!(
            "Loaded STL: {} vertices, {} faces",
            vertices.len(),
            faces.len()
        );

        Ok(TriMesh { vertices, faces, normals })
    }

    /// Build a map from Edge -> list of face indices that share that edge.
    /// Interior edges appear in 2 faces; boundary edges appear in only 1.
    pub fn edge_face_map(&self) -> HashMap<Edge, Vec<usize>> {
        let mut map: HashMap<Edge, Vec<usize>> = HashMap::new();
        for (fi, face) in self.faces.iter().enumerate() {
            for i in 0..3 {
                let edge = Edge::new(face[i], face[(i + 1) % 3]);
                map.entry(edge).or_default().push(fi);
            }
        }
        map
    }

    /// Find all boundary edges (edges belonging to only one face).
    /// These indicate holes/openings in the mesh.
    pub fn boundary_edges(&self) -> Vec<Edge> {
        self.edge_face_map()
            .into_iter()
            .filter(|(_, faces)| faces.len() == 1)
            .map(|(edge, _)| edge)
            .collect()
    }

    /// Group boundary edges into closed loops (each loop = one hole).
    /// Returns a list of loops, where each loop is an ordered list of vertex indices.
    pub fn boundary_loops(&self) -> Vec<Vec<usize>> {
        let boundary = self.boundary_edges();

        // Build adjacency: for each vertex on the boundary, which other vertices
        // does it connect to via boundary edges?
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for edge in &boundary {
            adj.entry(edge.0).or_default().push(edge.1);
            adj.entry(edge.1).or_default().push(edge.0);
        }

        let mut visited_edges: std::collections::HashSet<Edge> = std::collections::HashSet::new();
        let mut loops = Vec::new();

        for edge in &boundary {
            if visited_edges.contains(edge) {
                continue;
            }

            // Walk the boundary loop starting from this edge
            let mut loop_verts = Vec::new();
            let mut current = edge.0;
            let mut prev = usize::MAX; // sentinel

            loop {
                loop_verts.push(current);
                let neighbors = &adj[&current];
                // Pick the neighbor that isn't where we came from
                let next = neighbors
                    .iter()
                    .find(|&&n| n != prev && !visited_edges.contains(&Edge::new(current, n)))
                    .copied();

                match next {
                    Some(n) => {
                        visited_edges.insert(Edge::new(current, n));
                        prev = current;
                        current = n;
                        // If we've returned to the start, the loop is closed
                        if current == loop_verts[0] {
                            break;
                        }
                    }
                    None => break, // open boundary (shouldn't happen in valid STL)
                }
            }

            if loop_verts.len() >= 3 {
                loops.push(loop_verts);
            }
        }

        log::info!("Found {} boundary loops", loops.len());
        loops
    }

    /// Write the mesh to a binary STL file.
    pub fn write_stl(&self, path: &Path) -> Result<()> {
        let triangles: Vec<stl_io::Triangle> = self
            .faces
            .iter()
            .enumerate()
            .map(|(i, face)| {
                let n = &self.normals[i];
                stl_io::Triangle {
                    normal: stl_io::Normal::new([n.x as f32, n.y as f32, n.z as f32]),
                    vertices: [
                        stl_io::Vertex::new([
                            self.vertices[face[0]].x as f32,
                            self.vertices[face[0]].y as f32,
                            self.vertices[face[0]].z as f32,
                        ]),
                        stl_io::Vertex::new([
                            self.vertices[face[1]].x as f32,
                            self.vertices[face[1]].y as f32,
                            self.vertices[face[1]].z as f32,
                        ]),
                        stl_io::Vertex::new([
                            self.vertices[face[2]].x as f32,
                            self.vertices[face[2]].y as f32,
                            self.vertices[face[2]].z as f32,
                        ]),
                    ],
                }
            })
            .collect();

        let mut file = std::fs::File::create(path)
            .with_context(|| format!("Cannot create output STL: {}", path.display()))?;
        stl_io::write_stl(&mut file, triangles.iter())
            .with_context(|| "Failed to write STL")?;

        log::info!("Wrote STL: {} faces to {}", triangles.len(), path.display());
        Ok(())
    }
}
