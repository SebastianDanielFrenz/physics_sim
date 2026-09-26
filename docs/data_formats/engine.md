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
| `governor` | object, see [below](#governor) | optional | absent (no governor) | — | all-speed diesel governor with droop (PHYS-033 P4 I4 C12) |
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
| `rpm` | real (rpm) | optional | `6800` | see the ordering rule above (`idle_rpm < limiter.rpm`) | `fuel_cut`: cut fuel at/above this rpm. `soft_cut`: zero fuel at this rpm (an informal "governor-like" high idle — see the note below) |
| `hysteresis_rpm` | real (rpm) | optional | `150` | none | `fuel_cut` only: fuel restores at `rpm - hysteresis_rpm` |
| `band_rpm` | real (rpm) | optional | `200` | none | `soft_cut` only: linear taper over `[rpm - band_rpm, rpm]` (an informal "governor-like" droop — see the note below) |

**Note — `soft_cut` vs. a real `governor` block.** Before PHYS-033 P4 I4
C12 added the formal [`governor`](#governor) block below, a `soft_cut`
limiter's `rpm`/`band_rpm` were the only way to get governor-like taper
behaviour, and were informally described as "governor high idle"/"governor
droop" in this file's own history — that is a description of the *shape*
of the taper, not a real all-speed governor (it only ever tapers against
the limiter's own fixed `rpm`, never against a driver-commanded lever, and
it has no `theta_req` interaction with the TCU ceiling/floor or ignition
cut the way `governor` does). A `governor` block on the same engine
duplicates this taper's effect at full lever, so combining a `governor`
with a `soft_cut` limiter whose taper band overlaps the governor's own
droop band is a loader error (see [`governor`](#governor) below) —
`data/engines/diesel_i4_4l5_turbo.json` was moved to `fuel_cut` for exactly
this reason when its `governor` block was added. A `soft_cut` limiter with
no `governor` block is unaffected by C12 and keeps its pre-existing
behaviour exactly.

### `governor`

`GovernorDesc` (PHYS-033 P4 I4 C12) — an all-speed diesel governor with
droop, sitting between the driver's throttle lever(s) and the engine's
combustion-torque request. Optional; when absent, the engine's throttle
path is byte-identical to before C12 (the pedal drives `theta_req`
directly, exactly as documented in the rest of this file).

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `type` | string | optional | `"all_speed"` | must equal `"all_speed"`, else `"<ctx>: governor.type must be \"all_speed\", got \"<type>\""` | governor kind (only one exists) |
| `high_idle_rpm` | real (rpm) | optional | `2400` | must be `> idle_rpm`, else `"<ctx>: governor.high_idle_rpm must be > idle_rpm"` | no-load engine speed at full lever (lever = 1) |
| `rated_rpm` | real (rpm) | optional | `2200` | must satisfy `idle_rpm < rated_rpm <= high_idle_rpm`, else `"<ctx>: governor.rated_rpm must be > idle_rpm and <= high_idle_rpm"` | ISO 3046-4 reference speed the droop percentage is referred to |
| `droop` | real (fraction) | optional | `0.0909` | must satisfy `0 < droop <= 0.3`, else `"<ctx>: governor.droop must be in (0, 0.3]"` | steady-state speed droop referred to `rated_rpm` (ISO 3046-4) |
| `channel` | string | optional | `"hand_throttle"` | must be non-empty, else `"<ctx>: governor.channel must not be empty"` | name of the `World::set_control`/component control channel this governor's hand-throttle lever reads; declared via `Powertrain::declare_control_channels` with default value `0` |

**Model.** Every substep:

1. `lever = max(hand_throttle, pedal_throttle)`, each already clamped to
   `[0, 1]` — the driver's foot and the hand-throttle/cruise lever compose
   as "whichever asks for more fuel", in either order.
2. `n_nl = idle_rpm + lever * (high_idle_rpm - idle_rpm)` — the governor's
   own no-load speed setpoint at the current lever position. Exposed as
   `PowertrainSnapshot::governor_setpoint_rpm` (0 with no governor); the
   `lever` value itself (specifically the `hand_throttle` channel reading)
   is exposed as `PowertrainSnapshot::hand_throttle` (also 0 with no
   governor). Neither is hashed.
3. `band = droop * rated_rpm`.
4. `theta_gov = clamp((n_nl - n) / band, 0, 1)`, where `n` is the engine's
   current rpm — below setpoint by a full band or more, full fuel; at or
   above setpoint, no fuel; linear in between.
5. `theta_gov` **replaces the pedal** as the input to the existing request
   expression — everything downstream of the pedal today is otherwise
   unchanged: `theta_req = ignition_cut ? 0 : max(min(theta_gov, ceiling),
   floor)` (the TCU's upshift-cut ceiling and idle-PI floor still apply
   exactly as they do to the pedal), `theta_eff = max(theta_req,
   theta_idle)`, `theta_c = theta_eff * f_lim`, `tau = theta_c * (w - m)`.
   The idle PI, DFCO and the rev limiter are all unchanged by the presence
   of a governor.

**Implicit slope (`dtau/domega`).** The linearly-implicit solve needs
`d(tau)/d(omega)` every substep. The governor adds a term to this
derivative **only when `theta_gov` is the value actually driving `tau` all
the way through** — i.e. not clamped away by the TCU ceiling/floor, not
beaten by the idle PI's own floor, not ignition-cut, and not sitting at one
of `theta_gov`'s own outer clamp edges (0 or 1, where its local slope is
itself 0). Concretely (matching the code's own tie-breaking convention,
`core/src/drivetrain/engine_torque_map.cpp`): the governor's contribution
is added iff `theta_req == theta_gov` (untouched by the ceiling/floor
clamp), `theta_eff >= theta_req` (a *tie* with the idle floor counts as
"still governor-driven", not "beaten"), `0 < theta_gov < 1` (strictly
inside the band, away from both clamp edges), and combustion is active and
not ignition-cut. When active, the added term is

```
dtau/domega += f_lim * (w_rpm - m_rpm) * (-kRadPerSecToRpm / band)
```

— the chain rule through `theta_gov`'s own slope `d(theta_gov)/d(n) =
-1/band` (inside the linear region) and `d(n)/d(omega) =
kRadPerSecToRpm`, applied to the existing `tau_comb = theta_eff * f_lim *
(w - m)` term, with `w`/`m` evaluated at exactly the rpm values the
existing WOT/motoring-curve evaluation already used that substep. Outside
the active window above, the governor contributes nothing to the slope
(the existing code path's own derivative terms are used unchanged) — this
is deliberately a *conservative* linearization: at a clamp edge or when
another mechanism (ceiling, floor, ignition cut) is the active constraint,
that mechanism's own (already-existing) derivative handling applies
instead, never both.

**Rejections** (`core/src/io/vehicle_io.cpp`'s `read_engine_fields`), each
with a `tests/unit/test_vehicle_io_validation.cpp` case:

- `type` not `"all_speed"`.
- `high_idle_rpm <= idle_rpm`.
- `droop` outside `(0, 0.3]`.
- `rated_rpm` outside `(idle_rpm, high_idle_rpm]`.
- `channel` empty.
- **A `governor` combined with a `soft_cut` limiter whose taper band
  overlaps the governor's own droop band** — i.e. `[limiter.rpm -
  limiter.band_rpm, limiter.rpm]` intersects `[high_idle_rpm - droop *
  rated_rpm, high_idle_rpm]`. Message: `"<ctx>: governor combined with a
  soft_cut limiter whose band [...] overlaps the governor's own droop band
  [...] - use a fuel_cut overspeed limiter instead"`. Rationale: a
  `soft_cut` limiter's own taper is exactly the same shape of mechanism a
  `governor` already provides at full lever, so an overlapping pair would
  apply droop twice; use `fuel_cut` for straightforward overspeed
  protection on a governed engine instead.

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
- **A `governor` block combined with a `soft_cut` limiter whose taper band
  overlaps the governor's own droop band** — throws the overlap-rejection
  message pointing at `fuel_cut`; see [`governor`](#governor)'s own note
  and the `limiter` section's note above. A governed engine normally wants
  `fuel_cut` for overspeed protection, not `soft_cut`.
