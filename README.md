# jewellery-azure-cutter

Automatic **azure (à jour) cut** generation for jewellery STL meshes. Reduces metal weight by 10–30% while maintaining structural integrity around stone settings.

## What are Azure Cuts?

Azure (from French *à jour*) cuts are countersunk openings on the underside of jewellery, behind each set stone. They:

- Reduce precious metal weight (cost savings)
- Allow light through the stone for better brilliance
- Make cleaning easier behind settings
- Add a professional finish to the piece

## How It Works

```
Input STL (ring with pre-drilled stone holes)
    │
    ├─ 1. Detect stone seats (boundary loop analysis + circle fitting)
    ├─ 2. Measure local metal thickness (raycasting)
    ├─ 3. Generate tapered azure cutters (frustum solids)
    ├─ 4. Resolve neighbor conflicts (maintain min rib width)
    ├─ 5. Boolean subtract cutters from mesh
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
├── azure/           # Azure cutter generation (frustum geometry, neighbor resolution)
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

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stone.diameter_min` | 0.8 mm | Min hole diameter to detect as stone seat |
| `stone.diameter_max` | 10.0 mm | Max hole diameter to detect as stone seat |
| `azure.min_wall_thickness` | 0.5 mm | Metal remaining at bottom of cut |
| `azure.min_rib_width` | 0.4 mm | Min metal between adjacent azure cuts |
| `azure.taper_angle` | 30° | Sidewall taper from vertical |
| `azure.seat_margin` | 0.2 mm | Safety zone around stone hole edge |

## Current Scope

- **Round diamonds only** (stone_type = "round")
- **Pre-drilled STL** (stone holes must already exist in the mesh)
- **Parallel top/bottom surfaces assumed** (uniform azure height)

## Dependencies

- [`stl_io`](https://docs.rs/stl_io) — STL file reading/writing
- [`csgrs`](https://docs.rs/csgrs) — CSG boolean operations
- [`nalgebra`](https://docs.rs/nalgebra) — Linear algebra (vectors, matrices, SVD)
- [`parry3d`](https://docs.rs/parry3d) — 3D geometry queries
- [`clap`](https://docs.rs/clap) — CLI argument parsing
- [`serde`](https://docs.rs/serde) + [`toml`](https://docs.rs/toml) — Configuration

## License

MIT
