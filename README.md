# jewellery-azure-cutter

Automatic **azure (à jour) cut** generation for jewellery STL meshes. Reduces metal weight while maintaining structural integrity around stone settings.

## What are Azure Cuts?

Azure (from French *à jour*) cuts are countersunk openings on the underside of jewellery, behind each set stone. They:

- Reduce precious metal weight (cost savings)
- Allow light through the stone for better brilliance
- Make cleaning easier behind settings
- Add a professional finish to the piece

## Cutter Geometry — "Chimney" Shape

The azure cutter is shaped like a chimney: a **cylinder** at the top where the stone sits, transitioning to a **rectangular truncated pyramid** at the bottom.

```
  Side view (cross section):

        ┌────────┐          ← Stone hole (circle, radius r)
        │        │
        │ CYLNDR │  ← girdle_distance (straight bore)
        │        │
        ├─┐    ┌─┤          ← step (optional flat ledge)
       /  │    │  \
      / TRUNCATED  \  ← taper_angle controls widening
     /   PYRAMID    \
    └────────────────┘      ← bottom rectangle (w × h)
    ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔      ← inner surface
```

**Zone 1 (top) — Cylinder**: A straight cylindrical bore matching the stone hole diameter. Extends inward from the stone seat by `girdle_distance`. This is where the diamond sits and is held.

**Zone 2 (bottom) — Rectangular Pyramid**: Below the cylinder, the shape transitions to a rectangular cross-section that widens toward the inner surface. The bottom rectangle dimensions are derived from the spacing between neighbouring stones minus the minimum rib width between adjacent cuts.

## How It Works

```
Input STL (ring with pre-drilled stone holes)
    │
    ├─ 1. Detect stone seats (boundary loop analysis + circle fitting)
    ├─ 2. Measure local metal thickness (raycasting)
    ├─ 3. Compute bottom rectangles from neighbor layout (rib constraints)
    ├─ 4. Generate chimney cutters (cylinder + rectangular pyramid)
    ├─ 5. Boolean subtract cutters from mesh (CSG)
    │
    ▼
Output STL (ring with azure cuts applied)
```

## Architecture

```
src/
├── main.rs          # CLI entry point, pipeline orchestration
├── config/          # TOML config loading and validation
├── mesh/            # STL I/O, indexed mesh, boundary edge/loop detection
├── detect/          # Stone seat detection (circle fitting, filtering, raycasting)
├── azure/           # Azure cutter generation (chimney geometry, neighbor rectangles)
└── boolean/         # CSG boolean subtraction (csgrs-based)
```

## Usage

```bash
# Build
cargo build --release

# Run
cargo run --release -- --config example_config.toml

# With overrides
cargo run --release -- --config example_config.toml --input my_ring.stl --output my_ring_azure.stl --verbose
```

## Configuration

See [`example_config.toml`](example_config.toml) for all parameters:

### Stone Detection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stone.diameter_min` | 0.8 mm | Min hole diameter to detect as stone seat |
| `stone.diameter_max` | 10.0 mm | Max hole diameter to detect as stone seat |

### Azure Cutter — Cylinder (Top Zone)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `azure.girdle_distance` | 0.5 mm | Depth of straight cylinder below stone girdle |

### Azure Cutter — Pyramid (Bottom Zone)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `azure.taper_angle` | 15° | Pyramid wall taper from vertical |
| `azure.step_height` | 0.0 mm | Flat ledge at cylinder-to-pyramid transition (0 = none) |
| `azure.overhang` | 0.1 mm | Extension past inner surface for clean boolean |

### Wall Constraints

| Parameter | Default | Description |
|-----------|---------|-------------|
| `azure.min_wall_thickness` | 0.5 mm | Metal remaining at inner surface |
| `azure.min_rib_width` | 0.4 mm | Min metal between adjacent azure cuts |
| `azure.seat_margin` | 0.2 mm | Safety zone around stone hole edge |

### Resolution

| Parameter | Default | Description |
|-----------|---------|-------------|
| `azure.cylinder_segments` | 32 | Polygon segments for cylinder cross-section |

## Current Scope

- **Round diamonds only** (stone_type = "round")
- **Pre-drilled STL** (stone holes must already exist in the mesh)
- **Parallel top/bottom surfaces assumed** (uniform azure height)
- **Weight reduction** deferred to a second stage

## Dependencies

- [`stl_io`](https://docs.rs/stl_io) — STL file reading/writing
- [`csgrs`](https://docs.rs/csgrs) — CSG boolean operations
- [`nalgebra`](https://docs.rs/nalgebra) — Linear algebra (vectors, matrices, SVD)
- [`parry3d-f64`](https://docs.rs/parry3d-f64) — 3D geometry queries (f64 precision)
- [`clap`](https://docs.rs/clap) — CLI argument parsing
- [`serde`](https://docs.rs/serde) + [`toml`](https://docs.rs/toml) — Configuration

## License

MIT
