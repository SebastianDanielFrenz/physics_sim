# Chassis collision hull (`physics_sim.chassis_hull/1`)

A vehicle chassis collision shape built from the car's model: a compound of convex hulls in chassis body
coordinates (ISO: x forward, y left, z up; the same frame as the vehicle file's wheel `attachment` points).
Loaded by `ps::io::load_chassis_hull_json` (`ps/io/chassis_hull_io.h`) into a `ps::ConvexCompoundShape`.

Files live in `data/collision/` and are generated, not hand-edited:

```
python tools/collisiongen/chassis_hull.py car_hyper
```

The generator reads `data/models/<car>/<car>.glb` (body nodes only; the wing and the steering wheel are left out),
moves the mesh into the chassis frame with the same offset the Godot demo uses to place the model (mean over the
wheels of attachment minus the rig's suspension pivot), clips the underbody (see below), cuts the car into slices
along x and keeps the support points of each slice in a fixed set of directions. The output is deterministic.

## Keys

| Key | Type | Meaning |
|---|---|---|
| `format` | string | `"physics_sim.chassis_hull/1"` |
| `source` | string | required, non-empty: model file, generator, offset and the [AS] numbers used |
| `convex_radius_m` | number >= 0 | Jolt convex radius of every piece (rounds edges, faces stay); default 0.05 |
| `pieces` | array | non-empty; each piece is an array of at least 4 `[x, y, z]` points (m) |

Unknown keys are ignored. Every rejection throws `std::runtime_error` naming the file.

## Using it on a body

```cpp
BodyDesc d;
d.shape = ps::io::load_chassis_hull_json(path);
d.mass_shape = BoxShape{authored_half_extents};   // mass, centre of mass and inertia as authored
d.mass = authored_mass;
```

`BodyDesc::mass_shape` keeps the vehicle's authored mass properties: the backend gives the body the mass shape's
mass properties (the same arithmetic as a plain box body) and shifts the hull's centre of mass onto the mass shape's
without moving its geometry. A box body and a hull body with that box as `mass_shape` integrate bit-identically
(`tests/unit/test_chassis_hull.cpp` H1). `IRigidBackend::set_mass` rescales the mass shape.

## Underbody clip

The wheels' cylinder casts start at full bump, inside the wheel arches. Every vehicle's wheel casts exclude its own
chassis (`VehicleInstance`, 2026-11-03), so a hull that reaches into the arches does not read as ground. The floor
is still raised to full-bump ground + 0.02 m [AS] between the axles and rises 0.08 m per m beyond them [AS], so the
hull does not ground on kerbs and crests before the suspension does.

## Shipped files

- `car_hyper_hull.json`: 10 pieces, x -2.146..2.556, floor -0.402, roof 0.628 (body z). Used by racing_game for
  car_hyper, car_hyper_n2o and their presets. The physics_sim test rigs keep their small boxes; every validation
  number was recorded on them.
