# Engine format — `physics_sim.engine/1`

Files: `data/engines/*.json`, **or** inline inside a vehicle file's
`powertrain.components[]` entry with `"type": "engine"` (same fields, same
validation — see `vehicle.md`'s ["Components"](vehicle.md#components)
section for how `ref` vs. inline works). Loader: `read_engine_fields`
(`core/src/io/vehicle_io.cpp`), reached either via a vehicle's `{"ref":
"path/to/engine.json"}` (which additionally checks the standalone file's own
`"format"` field, see README) or by parsing the inline object directly (no
independent format check — it inherits the enclosing vehicle file's `/2`).
Public descriptor: `ps::drivetrain::TorqueMapEngineDesc`
(`core/include/ps/drivetrain/powertrain_desc.h`).

I3 has exactly one engine `kind`: `"torque_map"` — a WOT torque curve, a
motoring (closed-throttle/engine-braking) torque curve, plus idle control,
a rev limiter, a starter model and rpm thresholds for the engine's own
state machine (off → firing → idle → running).

## Fields

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required (standalone file only) | — | must equal `"physics_sim.engine/1"` | see README |
| `kind` | string | **required** | — | must equal `"torque_map"`, else `"<ctx>: engine 'kind' must be \"torque_map\" (I3's only kind)"` | engine model kind |
| `wot_torque_nm_vs_rpm` | Curve1D (N·m vs. rpm) | **required** | — | must be present (`"<ctx>: 'wot_torque_nm_vs_rpm' is required"`); `x` strictly ascending and non-empty; `x[0] >= 0` (`"<ctx>.wot_torque_nm_vs_rpm: x[0] must be >= 0"`) | wide-open-throttle torque vs. engine speed; scaled by throttle 0..1 to get commanded torque |
| `motoring_torque_nm_vs_rpm` | Curve1D (N·m vs. rpm) | **required** (I3; an I2-era file without it is rejected) | — | must be present, else `"<ctx>: 'motoring_torque_nm_vs_rpm' is required (PHYS-033 I3 - an I2-era engine block without it is no longer accepted)"`; `x` strictly ascending and non-empty; `x[0] == 0` exactly (`"<ctx>.motoring_torque_nm_vs_rpm: x[0] must be exactly 0"`); every `y <= 0` (`"<ctx>.motoring_torque_nm_vs_rpm: every y must be <= 0"`) | closed-throttle (engine-braking/friction) torque vs. rpm, always ≤ 0; `y[0]` (at rpm 0) is the Coulomb/static-friction part |
| `inertia_kgm2` | real (kg·m²) | **required** in I3 | `0` | **must be > 0**, else `"<ctx>: inertia_kgm2 must be > 0 in I3 (...the crank node needs real inertia...)"` | crank + flywheel + clutch-cover rotational inertia; also this component's drivetrain-node inertia |
| `breakaway_torque_nm` | real (N·m) | optional | `0` | must be `>= -motoring_torque_nm_vs_rpm(0)`, else `"<ctx>: breakaway_torque_nm must be >= -motoring_torque_nm_vs_rpm(0)"` | static Coulomb breakaway capacity at (near-)zero rpm |
| `idle_rpm` | real (rpm) | optional | `800` | see the ordering rule below | target idle speed |
| `stall_rpm` | real (rpm) | optional | `400` | see below | engine speed below which it stalls |
| `fire_rpm` | real (rpm) | optional | `250` | none directly (not part of the ordering rule) | speed at which the starter hand-off to combustion is considered to have "caught" |
| `start_complete_rpm` | real (rpm) | optional | `550` | see below | speed at which starting is considered finished |
| `dfco_enable_rpm` | real (rpm) | optional | `1300` | must be `> dfco_reenable_rpm` (see below) | deceleration fuel cut-off engages above this rpm |
| `dfco_reenable_rpm` | real (rpm) | optional | `1000` | **must be `< dfco_enable_rpm`**, else `"<ctx>: dfco_reenable_rpm must be < dfco_enable_rpm"` | fuel re-enables below this rpm (hysteresis band) |
| `idle_control` | object, see [below](#idle_control) | optional | see below | — | idle-speed PI controller gains |
| `limiter` | object, see [below](#limiter) | optional | see below | — | rev limiter |
| `starter` | object, see [below](#starter) | optional | see below | — | starter motor model |
| `crank_axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length (±1e-6), else `"<ctx>.crank_axis_local must be zero or a unit vector, got length <len>"` | chassis-local direction of positive crank ω, for the optional rotor-reaction torque on the chassis; zero = no rotor reaction |
| `initial_state` | string | optional | `"running"` | must be `"running"` or `"off"`, else `"<ctx>: initial_state must be \"running\" or \"off\", got \"<value>\""` | engine state at scenario start |

**rpm ordering rule** (checked as one combined condition): `stall_rpm <
start_complete_rpm <= idle_rpm < limiter.rpm`, else `"<ctx>: engine rpm
thresholds must satisfy stall_rpm < start_complete_rpm <= idle_rpm <
limiter.rpm"`.

### `idle_control`

`IdleControlDesc` — PI control on throttle (idle air bypass).

| Field | Type | Default | Meaning |
|---|---|---|---|
| `kp_per_rpm` | real (throttle fraction / rpm) | `0.0018` | proportional gain |
| `ki_per_rpm_s` | real (throttle fraction / (rpm·s)) | `0.009` | integral gain |
| `max_throttle` | real | `0.3` | ceiling the idle controller may command on its own |

### `limiter`

`EngineLimiterDesc`.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `type` | string | optional | `"fuel_cut"` | must be `"fuel_cut"` or `"soft_cut"`; `"hard_cut"` (the old I2 placeholder) is explicitly rejected with `"<ctx>: limiter.type \"hard_cut\" was the I2 placeholder; use \"fuel_cut\" or \"soft_cut\" (PHYS-033 I3)"`; any other value: `"<ctx>: limiter.type must be \"fuel_cut\" or \"soft_cut\", got \"<type>\""` | limiter behaviour |
| `rpm` | real (rpm) | optional | `6800` | see the ordering rule above (`idle_rpm < limiter.rpm`) | `fuel_cut`: cut fuel at/above this rpm. `soft_cut`: zero fuel at this rpm (governor high idle) |
| `hysteresis_rpm` | real (rpm) | optional | `150` | none | `fuel_cut` only: fuel restores at `rpm - hysteresis_rpm` |
| `band_rpm` | real (rpm) | optional | `200` | none | `soft_cut` only: linear taper over `[rpm - band_rpm, rpm]` (governor droop) |

### `starter`

`StarterDesc` — an ideal DC starter motor referred to the crank.

| Field | Type | Default | Validation | Meaning |
|---|---|---|---|---|
| `stall_torque_nm` | real (N·m) | `140.0` | none | starter torque at 0 rpm |
| `free_speed_rpm` | real (rpm) | `400.0` | none | torque falls linearly to 0 at this speed |
| `interlock` | string | `"clutch_or_neutral"` | must be `"clutch_or_neutral"` or `"none"`, else `"<ctx>: starter.interlock must be \"clutch_or_neutral\" or \"none\", got \"<value>\""` | whether the starter is inhibited unless the clutch is open or the gearbox is in neutral |

## Minimal complete example

```json
{
  "format": "physics_sim.engine/1",
  "name": "doc_example_i4",
  "kind": "torque_map",
  "source": "Documentation example, plausible small NA I4, not a real engine.",
  "wot_torque_nm_vs_rpm":      { "x": [800, 2000, 4000, 6000, 7000], "y": [90, 160, 190, 175, 130] },
  "motoring_torque_nm_vs_rpm": { "x": [0, 2000, 4000, 6000],         "y": [-15, -30, -45, -65] },
  "inertia_kgm2": 0.15,
  "breakaway_torque_nm": 40.0,
  "idle_rpm": 800, "stall_rpm": 400, "fire_rpm": 250, "start_complete_rpm": 550,
  "dfco_enable_rpm": 1300, "dfco_reenable_rpm": 1000,
  "idle_control": { "kp_per_rpm": 0.0018, "ki_per_rpm_s": 0.009, "max_throttle": 0.3 },
  "limiter": { "type": "fuel_cut", "rpm": 6800, "hysteresis_rpm": 150 },
  "starter": { "stall_torque_nm": 140.0, "free_speed_rpm": 400.0, "interlock": "clutch_or_neutral" },
  "crank_axis_local": [0, 1, 0],
  "initial_state": "running"
}
```

`"name"` and `"source"` are read and stored on the JSON object but not
consumed by `read_engine_fields` itself (`name` isn't read at all;
`source` is a convention, see README's "Discrepancies found" — include it
anyway).

## Common mistakes

- **Missing `motoring_torque_nm_vs_rpm`.** An I2-era engine block that only
  had `wot_torque_nm_vs_rpm` is rejected outright — I3 requires both curves.
- **`motoring_torque_nm_vs_rpm` with a positive `y` value**, or `x[0] !=
  0` — both are hard errors (see the field table).
- **`inertia_kgm2` left at 0 or omitted.** I3 requires a real, positive
  crank inertia; this is a common leftover from I2-era placeholder files.
- **rpm thresholds out of order**, e.g. `idle_rpm >= limiter.rpm`, or
  `dfco_reenable_rpm >= dfco_enable_rpm` — both throw the exact ordering
  rule violated.
- **`limiter.type: "hard_cut"`** — was the I2 placeholder, now a dedicated
  rejection message pointing at `fuel_cut`/`soft_cut`.
- **A `ref`'d engine file's own `"format"` wrong or missing** — e.g. a
  vehicle file's `{"id": "engine", "type": "engine", "ref": "../engines/
  foo.json"}` where `foo.json` is missing `"format": "physics_sim.
  engine/1"` throws the format-mismatch message at the *engine file's own
  path*, not the vehicle's.
