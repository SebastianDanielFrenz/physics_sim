# `physics_sim.hitch_class/1` — hitch classes (attachment A1)

`data/hitch_classes/*.json`. Loader: `core/src/io/hitch_io.cpp` (`ps::io::load_hitch_class_json`, header
`ps/io/hitch_io.h`). Descriptor: `ps::machine::HitchClassDesc` (`ps/machine/hitch_class.h`). `"extends"` works as in every
part file; an unknown key is rejected.

A class says what a hitch holds and what it leaves free. A hitch always holds its two sockets together, three rows; each of
the three rotations of the towed hitch frame in the towing one is free, limited or locked. The angles are Cardan, in the order
yaw about the towing z, pitch about the resulting y, roll about the towed x.

| Shipped class | yaw | pitch | roll | Used for |
|---|---|---|---|---|
| `ball` | ±90 | ±20 | ±25 | a car's tow ball |
| `drawbar_pin` | ±75 | ±20 | ±25 | a tractor's drawbar and a trailer eye |
| `fifth_wheel` | ±90 | ±12 | locked | a truck's turntable |
| `three_point_locked` | locked | locked | locked | a mounted implement; with a `linkage` block on the towing point the angle rows make way for the lift, [linkage.md](linkage.md) |

The numbers are `data/hitch_classes/*.json`'s own; check there before you quote one.

## Keys

| Key | Type | Required | Meaning |
|---|---|---|---|
| `format` | string | yes | `"physics_sim.hitch_class/1"` |
| `extends` | string | no | parent file, relative to this one |
| `name` | string | yes | the class's name. Two hitch points mate when their classes carry the same name |
| `source` | string | yes | where the numbers come from |
| `yaw`, `pitch`, `roll` | axis object | yes | `{"mode": "free"}`, `{"mode": "locked"}` or `{"mode": "limited", "min_deg": a, "max_deg": b}` |
| `frequency_hz` | number | yes | the rows' natural frequency |
| `damping_ratio` | number | yes | the rows' damping ratio |
| `latch.time_s` | number | yes | how long a latch ramps the sockets together |
| `latch.max_error_m` | number | yes | sockets further apart do not latch |
| `latch.max_angle_deg` | number | yes | a locked axis further off does not latch |
| `latch.max_speed_m_s` | number | yes | sockets moving faster do not latch |

`validate_hitch_class` also wants `min_deg < 0 < max_deg` on a limited axis and everything positive. Stability is measured at
240/960 Hz up to 480 Hz at `damping_ratio` 0.5; the shipped 60 Hz and 1 are the choice of attachment spike A0.

## How the rows work

Coupled vehicles step together in one island inside `vehicle_islands_substep`, the vehicles' shadow chassis held at the same
substep. Each substep the rows act as an implicit spring-damper on the shadows' velocities, normalised by the effective mass
of the row, so a light and a heavy partner settle with the same frequency. One coupling solves its rows exactly, a Cholesky of
up to 6x6 (`core/src/machine/coupling_solver.h`); a chain runs Gauss-Seidel sweeps along itself,
`WorldConfig::coupling_passes`, default 2. A limited axis is a unilateral row that only acts past its limit, so a jackknife
ends at the limit and not at box contact.

Coupled bodies neither collide nor stand on each other: `World::attach` puts the pair in Jolt's collision filter and the
partner's chassis out of each wheel's cast. A chain's non-neighbours still collide.

## Attaching

```cpp
const machine::AttachCheck check = world.check_attach(tractor, "socket_hitch", trailer, "socket_hitch_eye");
if (check.ok) hitch = world.attach(tractor, "socket_hitch", trailer, "socket_hitch_eye");
```

Between steps on the sim thread; from inside a step they throw `std::logic_error`. A refused `attach` throws
`std::invalid_argument` with the `check_attach` message. A coupling can also be built by hand, `CouplingDesc` with its sockets
and class, for a vehicle that lists no hitch points.

The latch starts at the real socket error and rate: nothing jumps, and the rows' target ramps to zero over `latch.time_s` on a
cubic that matches position and rate at both ends. Its force is what that ramp's acceleration takes, `6 e0 / T^2` at the start
on the pair's reduced mass. `CouplingState::latch_progress` runs from 0 to 1.

`World::detach` frees both vehicles at once. `World::destroy_vehicle` kills the vehicle's hitches with it; `World::destroy_body`
refuses a coupled chassis. Teleporting a coupled vehicle is the caller's job to avoid: detach first.

The state hash folds a live hitch's latch and tow brake. A world with no live hitch hashes as if the code did not exist.

## Vehicle files

A vehicle lists its points in `hitch_points` and a trailer its brakes in `tow_brake`, see
[vehicle.md](vehicle.md#hitch-points). Ratings and thresholds: [limit_policy.md](limit_policy.md), group `hitch`.
