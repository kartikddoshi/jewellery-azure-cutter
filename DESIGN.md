# jewellery-azure-cutter — Design Document

## 1. What This Program Does

This tool automatically generates **azure (a jour) cuts** on jewellery STL meshes.

Azure cuts are countersunk openings carved from the **underside** of a jewellery piece, directly behind each set stone. In the jewellery trade, they serve four purposes:

1. **Weight reduction** — remove precious metal from behind stones (gold/platinum is expensive)
2. **Light transmission** — let light pass through the stone for better brilliance and sparkle
3. **Cleaning access** — allow cleaning behind stone settings
4. **Professional finish** — a hallmark of quality craftsmanship

The tool takes a pre-drilled STL mesh (stone holes already exist), detects the stone seats, generates cutter solids, and boolean-subtracts them to produce the final mesh with azure cuts.

```
Input: ring.stl (with stone holes)  -->  Output: ring_azure.stl (with azure cuts)
```

---

## 2. Pipeline Overview

```
 ┌──────────────────────────────────────────────────────────────┐
 │  1. LOAD          Config (TOML) + STL mesh                   │
 │                                                              │
 │  2. DETECT         Find stone seats in the mesh              │
 │                    • Find boundary loops (holes)             │
 │                    • Fit circles (SVD/Kasa method)           │
 │                    • Filter by diameter range                │
 │                    • Skip finger hole (largest loop)         │
 │                    • Measure metal thickness (raycasting)    │
 │                                                              │
 │  3. GENERATE       Build cutter solids (one per stone)       │
 │                    • Compute bottom polygon from neighbors   │
 │                    • Build chimney mesh (cylinder + pyramid)  │
 │                                                              │
 │  4. SUBTRACT       CSG boolean difference                    │
 │                    • ring_mesh - cutter_1 - cutter_2 - ...   │
 │                                                              │
 │  5. OUTPUT         Write result STL                          │
 └──────────────────────────────────────────────────────────────┘
```

---

## 3. Cutter Geometry — The "Chimney"

Each azure cutter is a chimney-shaped solid with two zones:

```
  Side view (cross section):

        ┌────────┐          <-- Stone hole (circle, radius r)
        │        │
        │ CYLNDR │  <-- Zone 1: girdle_distance deep
        │        │
        ├─┐    ┌─┤          <-- step (optional flat ledge)
       /  │    │  \
      / TRUNCATED  \  <-- Zone 2: taper_angle controls widening
     /   PYRAMID    \
    └────────────────┘      <-- bottom polygon (shape from geometry)
    ~~~~~~~~~~~~~~~~        <-- inner surface of ring
```

**Zone 1 — Cylinder (top):**
- Straight bore matching the stone hole + seat margin
- Extends inward by `girdle_distance` (typically 0.5mm)
- This is where the diamond sits; must remain cylindrical

**Zone 2 — Truncated pyramid (bottom):**
- Below the cylinder, the cut widens toward the inner surface
- The widening rate is controlled by `taper_angle` (degrees from vertical)
- The bottom polygon shape is determined by neighboring stone layout
- An optional flat `step` ledge separates the two zones

---

## 4. Architecture

```
src/
├── main.rs              CLI entry point, pipeline orchestration
├── config/mod.rs        TOML config loading, validation, defaults
├── mesh/mod.rs          STL I/O, indexed mesh, boundary edge/loop detection
├── detect/mod.rs        Stone seat detection (circle fitting, thickness measurement)
├── azure/mod.rs         Cutter generation (chimney geometry, neighbor analysis)
└── boolean/mod.rs       CSG boolean subtraction (via csgrs library)
```

### Module Details

**config** — Loads TOML configuration. Three sections:
- `[input]` — STL file paths
- `[stone]` — Detection parameters (diameter range, stone type)
- `[azure]` — Cutter parameters (girdle, taper, walls, resolution)

**mesh** — Core mesh representation:
- `TriMesh` — Indexed triangle mesh (vertices, faces, per-face normals)
- `Edge` — Canonical half-edge (smaller index first)
- `from_stl()` / `write_stl()` — STL I/O via stl_io
- `boundary_edges()` — Find edges shared by only 1 face (= holes)
- `boundary_loops()` — Group boundary edges into closed loops

**detect** — Stone seat identification:
- `detect_stone_seats()` — Main entry: loops -> circle fit -> filter -> thickness
- `fit_circle_3d()` — SVD-based coplanarity check + Kasa 2D circle fit
- `raycast_thickness()` — Brute-force ray-mesh intersection for metal depth
- `ray_triangle_intersect()` — Moller-Trumbore algorithm

**azure** — Cutter solid generation:
- `generate_azure_cutters()` — Orchestrates bottom polygon computation + mesh building
- `compute_bottom_rectangles()` — Current: derives rectangle extents from neighbor distances
- `build_chimney_cutter()` — Constructs the chimney TriMesh (rings + transition + caps)
- `map_circle_to_polygon_index()` — Angular mapping from cylinder segments to bottom polygon

**boolean** — CSG operations:
- `subtract_cutters()` — Sequential boolean difference via csgrs
- `trimesh_to_csg()` / `csg_to_trimesh()` — Format conversion

### Key Dependencies

| Crate | Purpose |
|-------|---------|
| `stl_io` 0.8 | STL file reading/writing |
| `csgrs` 0.20 (f64) | CSG boolean operations |
| `nalgebra` 0.33 | Linear algebra, SVD, matrix ops |
| `parry3d-f64` 0.17 | 3D geometry queries (declared, not yet heavily used) |
| `clap` 4 | CLI argument parsing |
| `serde` + `toml` | Configuration deserialization |
| `anyhow` / `thiserror` | Error handling |
| `log` + `env_logger` | Logging |

---

## 5. Current Status

### What Works

| Component | Status | Notes |
|-----------|--------|-------|
| Config loading | Done | TOML parsing, validation, defaults |
| STL I/O | Done | Load and write binary STL |
| Boundary detection | Done | Edge-face map, boundary loops |
| Circle fitting | Done | SVD plane fit + Kasa circle fit |
| Finger hole skip | Done | Largest loop heuristic |
| Metal thickness | Done | Brute-force raycasting |
| Chimney mesh builder | Done | Cylinder + step + transition + bottom cap |
| Circle-to-polygon mapping | Done | Angular proximity |
| CSG boolean | Done | Sequential difference via csgrs |
| CLI | Done | clap with config/input/output/verbose |
| Test suite | 10 tests | Vertex counts, face validity, mapping, raycasting |

### What's In Progress — Voronoi Bottom Shape

**Problem identified:** The current code uses a predefined bottom shape — either a
rectangle (8 points) or hexagon (12 points) chosen in config. This is fundamentally wrong:

1. The bottom shape is NOT a choice — it's determined by the stone layout
2. On a domed surface, the "sides" between adjacent cuts are NOT at 90 degrees
3. A stone with 3 neighbors should get ~triangle; 5 neighbors should get ~pentagon
4. The shape emerges from where neighboring cuts meet, not from a preset

**Solution designed (not yet implemented in azure/mod.rs):** Voronoi tessellation.

The `BottomShape` enum has been removed from config. The bottom polygon will be
computed as the **Voronoi cell** of each stone — the intersection of half-planes
defined by perpendicular bisectors with all neighbors, offset by `min_rib_width/2`,
and clipped to the taper circle.

Algorithm:
```
For each stone S:
  1. Project all neighbor stone centers into S's local 2D plane (u, v)
  2. Start with the taper-limit circle as the initial polygon
  3. For each neighbor N:
     a. Compute midpoint M between S and N in 2D
     b. Offset M toward S by min_rib_width / 2
     c. Define the half-plane on S's side of the perpendicular bisector at M
     d. Clip the current polygon by this half-plane (Sutherland-Hodgman)
  4. The surviving polygon = bottom shape for stone S
```

This produces:
- Single stone: circle (taper-limited)
- Two stones in a row: elongated shape, narrow in the row direction
- Hexagonal packing: hexagons (naturally!)
- Irregular layouts: irregular convex polygons
- Any number of sides (3, 4, 5, 6, 7...) — whatever the geometry dictates

### What's Broken Right Now

The `azure/mod.rs` file still imports and uses `BottomShape` from config, but the
enum has been removed from `config/mod.rs`. The azure module needs to be rewritten
to use the Voronoi cell algorithm instead of the predefined shape approach.

Specifically, these functions need rewriting:
- `compute_bottom_rectangles()` -> `compute_voronoi_cells()`
- `generate_bottom_boundary_2d()` -> removed (replaced by Voronoi computation)
- `build_chimney_cutter()` -> accept arbitrary Vec<(f64, f64)> instead of half_w/half_h
- `AzureCutter` struct -> `bottom_half_extents` field replaced with `bottom_boundary`
- All tests referencing `BottomShape` need updating

---

## 6. Configuration Parameters

All dimensions in millimeters, angles in degrees.

### Stone Detection (`[stone]`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stone_type` | `"round"` | Only round stones supported |
| `diameter_min` | 0.8 | Min hole diameter to detect as stone seat |
| `diameter_max` | 10.0 | Max hole diameter to detect as stone seat |

### Azure Cutter (`[azure]`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `girdle_distance` | 0.5 | Cylinder depth below stone girdle |
| `taper_angle` | 15.0 | Pyramid wall taper from vertical (degrees) |
| `step_height` | 0.0 | Flat ledge at cylinder-pyramid transition (0 = none) |
| `overhang` | 0.1 | Extension past inner surface for clean boolean |
| `min_wall_thickness` | 0.5 | Metal remaining at inner surface |
| `min_rib_width` | 0.4 | Min metal between adjacent cuts |
| `seat_margin` | 0.2 | Safety zone around stone hole edge |
| `cylinder_segments` | 32 | Polygon segments for cylinder |

---

## 7. Where We Want to Be — Roadmap

### Phase 1: Voronoi Bottom Shape (IN PROGRESS)
- [x] Remove `BottomShape` enum from config
- [ ] Implement `compute_voronoi_cells()` using half-plane intersection
- [ ] Implement Sutherland-Hodgman polygon clipping
- [ ] Update `build_chimney_cutter()` to accept arbitrary polygon boundary
- [ ] Update `AzureCutter` struct to store actual bottom boundary
- [ ] Rewrite tests for Voronoi-based approach
- [ ] Verify on various stone layouts (linear row, hex cluster, scattered)

### Phase 2: Dome/Curvature Awareness
- [ ] Account for surface curvature when projecting neighbors into local plane
- [ ] Handle stones at varying angles on domed surfaces
- [ ] Project along surface normals, not in a flat plane
- [ ] Handle cases where the inward normal varies significantly across the cut footprint

### Phase 3: Non-Round Stones
- [ ] Oval/marquise stone support (elliptical holes)
- [ ] Baguette/emerald cut support (rectangular holes)
- [ ] Mixed stone sizes in the same piece
- [ ] The Voronoi approach naturally handles mixed sizes (just different cylinder radii)

### Phase 4: Performance
- [ ] BVH acceleration for raycasting (parry3d is already a dependency)
- [ ] Parallel cutter generation (rayon)
- [ ] Batch CSG operations instead of sequential subtraction
- [ ] Mesh simplification on output (remove tiny triangles from CSG)

### Phase 5: Quality & Validation
- [ ] Weight calculation (before/after metal volume)
- [ ] Structural analysis — warn if ribs are too thin
- [ ] Minimum metal percentage constraint
- [ ] Visual preview / report generation
- [ ] Export cutter solids separately for inspection

### Phase 6: Advanced Features
- [ ] Automatic stone detection from solid mesh (no pre-drilled holes)
- [ ] Multiple azure styles (full pierce, partial countersink, pocket)
- [ ] Edge chamfering on the inner surface opening
- [ ] Integration with CAD formats (STEP, 3MF) beyond STL

---

## 8. Commit History

| Date | Author | Summary |
|------|--------|---------|
| Mar 16 2026 | Kartik Doshi | Initial scaffold: full pipeline, frustum cutters, 5 tests |
| Mar 16 2026 | Kartik Doshi | Ray-projected ruled surface cutters (adapt to inner geometry) |
| Mar 16 2026 | Kartik Doshi | Chimney geometry: cylinder top + rectangular pyramid bottom |
| Mar 21 2026 | Claude | Fix taper angle, add hexagonal honeycomb, fix winding/mapping |
| Mar 21 2026 | (pending) | Voronoi-based bottom shape — remove predefined shapes |

---

## 9. Key Design Decisions & Rationale

### Why chimney (cylinder + pyramid) instead of a simple cone/frustum?
The top must be a **straight cylinder** to properly seat the stone — tapered walls
would push the stone out. Only below the girdle line does tapering make sense.

### Why Voronoi cells instead of predefined rectangles/hexagons?
Real jewellery has irregular stone layouts, mixed sizes, domed surfaces, and varying
angles. Predefined shapes assume regularity that doesn't exist. Voronoi cells
naturally partition space between stones, giving each stone exactly the space
available to it, regardless of layout.

### Why sequential CSG subtraction instead of one combined cutter?
Combining all cutters into a single solid first would require a CSG union (expensive
and error-prone with many touching solids). Sequential subtraction is simpler and
each operation is well-conditioned (one cutter vs. the remaining mesh).

### Why brute-force raycasting instead of BVH?
Jewellery meshes are small (typically <100K faces). Brute-force is fast enough for
now. BVH (via parry3d) is planned for Phase 4 when handling larger meshes.

### Why f64 precision throughout?
Jewellery tolerances are sub-millimeter (0.01mm matters). f32 loses precision at
these scales, especially during SVD circle fitting and CSG boolean operations.
