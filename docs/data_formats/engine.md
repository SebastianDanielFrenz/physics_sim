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

I3 shipped exactly one engine `kind`: `"torque_map"` — a WOT torque curve, a
motoring (closed-throttle/engine-braking) torque curve, plus idle control,
a rev limiter, a starter model and rpm thresholds for the engine's own
state machine (off → firing → idle → running). PHYS-033 P4 I7a C5 turned
`read_engine_fields` into a thin dispatcher on `kind` so a second kind
(`"simulated"`, a full MVEM engine) can sit next to `torque_map`; its
schema landed in I7b C10 — see the [`kind`](#kind) section below. `torque_map` also
gained an optional fuel model in C5: [`fuel`](#fuel) and
[`fuel_map`](#fuel_map) below; C5d added a no-map alternative,
[`fuels`](#fuels-ignition-indicated_efficiency)/[`ignition`](#fuels-ignition-indicated_efficiency)/
[`indicated_efficiency`](#fuels-ignition-indicated_efficiency), for an
engine that burns fuel but has no calibrated map.

### `kind`

| Value | Status | Meaning |
|---|---|---|
| absent, or `"torque_map"` | shipped | the `torque_map` engine documented in this file |
| `"simulated"` | loads (PHYS-033 P4 I7b C10); the runtime engine is I7b C11 and its Powertrain integration I7b C12 - until C12 a vehicle with one is rejected by the Powertrain constructor ("simulated engine not yet integrated (I7b C12)") | the mean-value (MVEM) engine, see [`kind: "simulated"`](#kind-simulated-the-mvem-engine) below |
| anything else | rejected — `"<ctx>: engine 'kind' must be \"torque_map\" or \"simulated\", got \"<value>\""` | — |

Before C5, `kind` was required and had to equal `"torque_map"` literally
(omitting it was a load error). C5 relaxed this to "absent means
`torque_map`" so a test fixture (or a future hand-written file) can omit
the field entirely — every shipped `data/engines/*.json` file already sets
`kind` explicitly and is unaffected either way, and `kind` never enters
`fold_state_hash` regardless. This default is pinned by
`tests/unit/test_engine_fuel_map.cpp`'s "kind absent defaults to
torque_map" case, not re-verified against every determinism scenario's
`state_hash` (no shipped scenario exercises the absent-`kind` path, since
every shipped engine file sets it).

## Fields

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required (standalone file only) | — | must equal `"physics_sim.engine/1"` | see README |
| `kind` | string | optional (PHYS-033 P4 I7a C5; was required before C5) | `"torque_map"` | see [`kind`](#kind) above | engine model kind |
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
| `throttle_input` | string | optional (PHYS-033 P4 I7a C3; a `"simulated"` engine requires it explicitly, see below) | `"electronic"` | must be `"electronic"` or `"mechanical"`, else `"<ctx>: throttle_input must be \"electronic\" or \"mechanical\", got \"<value>\""` | which throttle-command stage kind this engine expects when no explicit `throttle_control` powertrain component names it (see `vehicle.md`'s ["`throttle_control` component fields (I7a C3)"](vehicle.md#throttle_control-component-fields-i7a-c3) section) — an explicit `throttle_control` component naming this engine must declare the SAME `kind` as this field, both directions checked at load time; a mismatch is rejected (`"<ctx>: throttle_control 'kind' ... does not match engine '<id>'s throttle_input ..."`), it is never silently overridden |
| `fuel` | string (path) | optional; **required if `fuel_map` present** | absent (no fuel model) | non-empty string, else `"<ctx>: 'fuel' must be a non-empty path string"`; must resolve and load via `ps::io::load_fuel_json` | path to a `physics_sim.fuel/1` file (see `fuel.md`), resolved relative to THIS engine JSON's own directory — the referenced engine file's own directory when the engine itself was reached via a vehicle's `{"ref": ...}`, else the vehicle file's directory for an inline engine block. **Review fix B1:** `fuel` is TRUE SHORTHAND for a one-entry `fuels` list — the loader always sets `fuels = [fuel]` internally when `fuel` is present, whether or not `fuel_map` is ALSO present. So `fuel` with no `fuel_map` is NOT a no-op: it activates the crude no-map estimate below on that one fuel, exactly like writing `"fuels": ["<same path>"]`; `ignition` is then required (see the `fuels`/`ignition` row below) |
| `fuel_map` | object, see [below](#fuel_map) | optional | absent (`has_fuel_map = false`, no fuel computation anywhere) | see below | fuel flow (g/s) vs. (rpm, load) — `load` is `theta_c`, the post-limiter combustion fraction |
| `fuels` | array of string (paths) | optional; mutually exclusive with `fuel`/`fuel_map` at the JSON-key level | absent | non-empty array of non-empty path strings, no duplicates (`std::filesystem::equivalent` comparison — review fix N12, was a `weakly_canonical` string compare), each entry must be a string (else `"<ctx>: 'fuels' entries must be non-empty path strings"`) and load via `ps::io::load_fuel_json`; rejected alongside `fuel` or `fuel_map` | PHYS-033 P4 I7a C5d: a plain list of fuels this engine can run on, for the crude no-map estimate (see [below](#fuels-ignition-indicated_efficiency)) — `fuels[0]` is the ACTIVE fuel, no blending; a hypothetical per-entry weighted blend (C5e) was cancelled. Internally this vector is also populated (as a one-entry list) whenever singular `fuel` is present — see that row above |
| `ignition` | string | **required whenever a fuel model exists** (`fuel_map` present, or `fuels` non-empty); optional otherwise | none (rejected if required and absent) | must be `"spark"` or `"compression"`, else `"<ctx>: ignition must be \"spark\" or \"compression\", got \"<value>\""`; required-but-absent: `"<ctx>: an engine with a fuel ('fuel'+'fuel_map' or 'fuels') requires 'ignition' (\"spark\" or \"compression\")"` | an ENGINE property, not a fuel one (2026-09-27 owner ruling — see [below](#fuels-ignition-indicated_efficiency)); picks the crude no-map estimate's default indicated efficiency |
| `indicated_efficiency` | real (fraction) | optional; requires the crude no-map path | ignition-based Heywood default (see below) | must be in `(0, 1)` exclusive and finite; rejected without a fuel model: `"<ctx>: indicated_efficiency requires a fuel model ('fuel_map' or 'fuels')"`; rejected alongside a `fuel_map` (review fix N9): `"<ctx>: indicated_efficiency is meaningless with a 'fuel_map' present (the map is never gated by eta_i) - remove one or the other"` | override for the crude no-map estimate's `eta_i`; never consulted by the `fuel_map` path, so setting it there is now a load error rather than a silent no-op |

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

### `fuel_map`

PHYS-033 P4 I7a C5 (plan section 2.3). A bilinear-interpolated 2D table
(`ps::drivetrain::Table2D`, `x` = rpm, `y` = load) of fuel flow in g/s,
requiring a sibling `fuel` reference (above) to turn a booked mass into
energy. `load` is exactly `theta_c` — the engine's own post-limiter
combustion fraction (`ess.load`/`EngineSoundState::load`), i.e. the SAME
value already published for sound — not the raw pedal, not the pre-limiter
throttle request. At runtime `mdot = fuel_map.value(rpm, theta_c)` is
evaluated once per substep in `pre_solve` at the substep-START rpm/theta_c
(no starvation factor yet — a fuel tank/pump model is C6), and the booked
mass is `h * mdot` in `post_solve`.

| Field | Type | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `rpm` | array of real | **required** | ≥ 2 points, strictly ascending (`"<ctx>.fuel_map: 'rpm' must be strictly ascending"`) | table's `x` axis |
| `load` | array of real | **required** | ≥ 2 points, strictly ascending; **must start at exactly `0.0`** and **end at exactly `1.0`** (`"<ctx>.fuel_map: 'load' must start at exactly 0.0"` / `"...end at exactly 1.0"`) | table's `y` axis — `theta_c` |
| `fuel_flow_g_s` | array of arrays of real | **required** | outer length == `rpm.size()`, each inner row length == `load.size()` (`"<ctx>.fuel_map: 'fuel_flow_g_s' must have one row per 'rpm' point (...)"` / `"...row <i> must have one value per 'load' point (...)"`); every value finite and `>= 0` (`"<ctx>.fuel_map: fuel_flow_g_s[i][j] must be >= 0 and finite, got <v>"`) | `values[ix][iy]` = fuel flow at `(rpm[ix], load[iy])`, g/s |
| `source` | string | **required** | non-empty (`"<ctx>.fuel_map: 'source' is required (PLAN.md section 8)"`) | citation/derivation note, same convention as everywhere else in this codebase |

**The `load == 0` column must be exactly `0.0`.** Every row's
`fuel_flow_g_s[ix][0]` must equal `0.0` exactly (not merely near-zero):
`"<ctx>.fuel_map: fuel_flow_g_s[<ix>][0] (load 0) must be exactly 0.0, got
<v>"`. This is what makes DFCO, ignition cut, a fuel-cut limiter, a
pedal-0/idle-PI-floor-0 substep, and the `Stalled`/`Off` engine states all
book EXACTLY zero fuel: each of those substeps has `theta_c == 0` (`ess.load
== 0`), which lands exactly on this column via `Table2D::value`'s own
clamped-bilinear lookup — no floating-point residue for a zero-tolerance
test to trip on.

**Efficiency check** (plan section 2.3): a load-time check that the map does
not imply more than 100% thermal efficiency — `theta * (w(rpm) - m(rpm)) *
omega <= mdot * LHV`, where `w`/`m` are this engine's own
`wot_torque_nm_vs_rpm`/`motoring_torque_nm_vs_rpm` curves, `omega = rpm *
2*pi/60`, `mdot = fuel_map.value(rpm, theta) / 1000` (kg/s), and `LHV` is
the referenced fuel's `lhv_j_kg`. This is **sampled, not exhaustive**: it is
checked only at

1. every map node `(rpm[ix], load[iy])`;
2. every map-CELL MIDPOINT (the average of each adjacent pair of `rpm`/
   `load` nodes) — bilinear interpolation can under/overshoot a saturating
   `w - m` curve between nodes, so the nodes alone are not sufficient;
3. every `wot_torque_nm_vs_rpm`/`motoring_torque_nm_vs_rpm` curve
   breakpoint crossed with every map `load` node — a curve knot is where
   `w(rpm) - m(rpm)`'s own slope can kink, the worst case for a fixed-load
   power line between two map rpm nodes.

Between these sample points the check does not guarantee anything — a
pathological map could still exceed 100% efficiency strictly between a
node, its cell midpoint and a curve breakpoint. The **runtime safety net**
is `PowertrainEnergy::fuel_efficiency_violations`
(`core/include/ps/drivetrain/energy_ledger.h`): every substep where the
booked `engine_heat` delta (`fuel_energy_in_delta - indicated_work_delta`)
comes out negative — i.e. the map claimed more energy converted to work
than was burned, wherever in `(rpm, theta)` space that substep actually
landed — increments this counter instead of throwing, so a load-time gap
in the sampled check is still observable (and testable) at runtime rather
than silently producing negative heat.

A load-time violation throws `"<ctx>.fuel_map: implies >100% thermal
efficiency at rpm=<r>, load=<l> (mechanical <p1> W > fuel <p2> W)"`. `rpm <=
0` is skipped (zero mechanical power at any load, never a violation —
avoids a spurious check at the table's own `x.front()` clamp when
`rpm[0] == 0`).

### `fuels`, `ignition`, `indicated_efficiency`

**Active fuel (I7a C6):** a vehicle `fuel_tank` fluid-linked to this engine selects the burnt fuel (the tank's fuel, which must be one of `fuels`); without a tank `fuels[0]` burns - see [vehicle.md, fluid links](vehicle.md#fluid-links-i7a-c6).

PHYS-033 P4 I7a C5d (plan sections 2.1/4/4.1/4.2). For an engine that burns
fuel but has no calibrated `fuel_map` — every shipped engine as of this
commit — a crude constant-efficiency estimate replaces the map lookup:

```
mdot = max(0, tau_comb * omega_old) / (eta_i * LHV)
```

where `tau_comb` is the same post-limiter combustion torque the map path
would otherwise convert into `indicated_work` (`PostSolveResult::
indicated_work_delta`, booked for every `torque_map` engine regardless of
fuel model), `omega_old` is the crank's pre-solve angular velocity, `LHV` is
`fuels[0].lhv_j_kg`, and `eta_i` is `indicated_efficiency` if set, else an
ignition-based default (Heywood, John B., *Internal Combustion Engine
Fundamentals*, McGraw-Hill, 1988). **Review fix N5:** the two defaults come
from two DIFFERENT kinds of operating point — an acknowledged
inconsistency, not hidden:

- **spark, 0.325** — Sec. 15.2, p.827, full load: *"indicated fuel
  conversion efficiency increases by about 10 percent from 0.31 to 0.34
  over the speed range 1000 to 4000 rev/min."* 0.325 is the DERIVED midpoint
  of `[0.31, 0.34]`, not a direct quote from the text.
- **compression, 0.48** — Sec. 15.5.1 "Load and Speed", p.859, at the
  minimum-bsfc operating point (220 g/kWh, 1600 rev/min, 580 kPa bmep,
  medium-swirl NA DI diesel): *"The gross indicated fuel conversion
  efficiency would be about 48 percent."*

This is deliberately crude — a single constant `eta_i` per engine, no load
or rpm dependence, no pumping-loss/friction split beyond what the motoring
curve already folds in — so it is biased in both directions, for DIFFERENT
reasons (review fix B4, correcting a prior version of this section that
described both biases backwards):

- **Near WOT, `w - m` OVERSTATES gross indicated torque.** The motoring
  curve `m` is measured/estimated at CLOSED throttle, so it already
  includes closed-throttle pumping losses; at WOT those pumping losses are
  much smaller (or reversed — supercharged/turbocharged engines can pump
  positive), so `w - m` (which subtracts the closed-throttle pumping loss
  from the WOT curve) overstates how much *gross* indicated torque a WOT
  cycle is actually producing. This makes the crude model's WOT `mdot`
  estimate for a given `tau_comb` too CONSERVATIVE at a fixed `eta_i` (the
  model attributes more of the delivered torque to combustion than pumping
  losses actually allow) — no numeric magnitude is given here because none
  in this file has been DERIVED from a specific engine data file's own
  `"source"` field; a magnitude would need to be computed from that engine's
  own curves and labelled "derived" if added.
- **At idle, real gross indicated efficiency is BELOW the full-load
  `eta_i` default.** Both Heywood defaults above are full-load/minimum-bsfc
  operating points, not idle ones — a real engine's indicated efficiency at
  idle is lower (poor combustion phasing, high relative friction/pumping
  share of a small net output). Critically, this shortfall belongs to the
  ENGINE'S OWN pumping losses, which already live in the motoring curve `m`
  (folded into `tau_comb = theta_c*(w-m)` before this formula ever sees
  it) — `eta_i` itself is a single indicated-efficiency constant and does
  NOT separately account for pumping loss; a prior version of this
  paragraph incorrectly implied `eta_i` absorbs the idle pumping-loss gap.

A calibrated `fuel_map` is the accurate alternative once one exists for a
given engine.

`has_fuel_model(TorqueMapEngineDesc)` (`ps/drivetrain/powertrain_desc.h`) is
`has_fuel_map || !fuels.empty()` — distinct from `has_fuel_map` (a
calibrated map specifically): it gates whether fuel/heat booking
(`fuel_mass`/`fuel_energy_in`/`engine_heat`/`fuel_efficiency_violations` in
`PowertrainEnergy`) and the `vehicle<i>.fuel.*` telemetry channels are
active at all, while `has_fuel_map` alone still picks map-vs-crude *within*
that. **Review fix N6:** `fuel_efficiency_violations` is NOT always 0 for
the crude path despite its constant `eta_i` — a prior version of this
paragraph claimed it was. With `tau_comb > 0`, `engine_heat`'s substep delta
is `h*tau_comb*(omega_old/eta_i - omega_new)`, which is negative (a
violation) exactly when `omega_new/omega_old > 1/eta_i` — a large enough
one-substep speed-up (e.g. a stiff/lightly-loaded crank accelerating fast
relative to `h`) can still trip it even at a constant assumed efficiency.

**Why `ignition` is an engine field, not a fuel field.** C5c briefly added
an ignition/combustion-cycle field to `FuelDesc` itself, then reversed it
(2026-09-27 owner ruling, commit reachable via this branch's own history):
*any* fuel autoignites given enough compression — fuels differ only in
ignition QUALITY (octane/cetane rating), not in which cycle they are "for".
`ignition` therefore lives on the engine (this file), never on `fuel.md`'s
`FuelDesc`, and its only effect is choosing the crude estimate's default
`eta_i` above — it never dispatches any other physics (combustion timing,
knock, etc. are all out of scope for the `torque_map` engine kind).

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

## Kind "simulated": the MVEM engine

PHYS-033 P4 I7b C10. Public descriptor: `ps::drivetrain::SimulatedEngineDesc`
(`core/include/ps/drivetrain/powertrain_desc.h`), a `ComponentParams`
alternative next to `TorqueMapEngineDesc`. Loader: `read_simulated_fields`
(`core/src/io/vehicle_io.cpp`). A mean-value engine: manifold pressure is a
state, torque follows from the air mass the cylinders take in, the fuel the
ECU adds to it and the maps below. It therefore has **no** WOT or motoring
curve. The runtime model is `core/src/drivetrain/engine_simulated.h`
(I7b C11); the Powertrain accepts it from I7b C12 on.

Every table is **inline** in the engine object, or (I7c C16a) generated at
load time from a [`cycle`](#cycle-maps-generated-from-the-0d-cycle-model-i7c-c16a)
block. Inline tables are
first-class schema, not a test shortcut: I7c adds generated maps behind the
same descriptor, and a hand-authored or literature-derived engine may keep
its tables inline for good.

### Unit conventions (file to descriptor)

| File unit | Descriptor unit | Where |
|---|---|---|
| mm | m | `bore_mm`, `stroke_mm`, rotary dimensions, `throttle_body.diameter_mm` |
| L | m^3 | `intake_manifold_volume_l` |
| mm^2 | m^2 | `idle_control.bypass_area_mm2` |
| kPa | Pa | every `p_im_kpa` axis, `pmep_excess_kpa` values |
| g per crank revolution | kg | `maps.eta_i`'s `air_mass_per_rev_g` axis |

### Table format

A 2D map is `{"rpm": [...], "<load key>": [...], "values": [[...], ...]}`
with one `values` row per `rpm` point and one entry per load point
(row-major, same as `Table2D`: `values[i_rpm][i_load]`). Both axes need at
least 2 points, strictly ascending; `rpm >= 0`. Lookup is bilinear and
clamped at every edge. A 1D curve is the usual `{"x": [...], "y": [...]}`.

### Fields

| Field | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `architecture` | string | yes | `"piston_4stroke"` or `"rotary"` | selects the geometry block and the cycle factor (1/2 per crank revolution for a 4-stroke, 1 per e-shaft revolution for a rotary) |
| `bore_mm`, `stroke_mm` | number | piston | > 0 | displacement = cylinders * pi/4 * bore^2 * stroke |
| `cylinders` | integer | piston | [1, 64] | |
| `rotor_radius_mm`, `eccentricity_mm`, `rotor_width_mm` | number | rotary | > 0, eccentricity < rotor radius | displaced volume per face 3*sqrt(3)*R*e*b |
| `rotors` | integer | rotary | [1, 64] | |
| `intake_manifold_volume_l` | number | yes | > 0 | plenum plus runners |
| `throttle_body.diameter_mm` | number | yes | > 0 | per throttle body |
| `throttle_body.count` | integer | no (1) | [1, 64] | individual throttle bodies sharing one position |
| `throttle_body.discharge_coefficient` | number | yes | (0, 1] | |
| `throttle_body.area_fraction_vs_position` | curve | yes | x exactly 0..1 ascending; y in [0, 1], non-decreasing; y(0) > 0 | open-area fraction over plate position; a closed plate still leaks |
| `throttle_input` | string | **yes** | `"electronic"` or `"mechanical"` | see "Input types" below |
| `throttle_actuator_tau_s` | number | electronic only | >= 0 (0 = no lag) | first-order plate actuator time constant |
| `idle_control.bypass_area_mm2` | number | mechanical only | > 0 | idle bypass valve area at full opening |
| `ecu.lambda_target` | table, load `p_im_kpa` | yes | values in [0.5, 2] | open-loop air/fuel ratio target |
| `ecu.spark_advance_deg` | table, load `p_im_kpa` | yes, unless `cycle.spark_advance_deg` is given (then it must be absent and is filled from the cycle table) | values in [-30, 70] | spark advance target [deg BTDC] |
| `maps.source` | string | yes | non-empty | provenance (PLAN.md section 8) |
| `maps.ve` | table, load `p_im_kpa` | yes | values in (0, 2) | volumetric efficiency |
| `maps.eta_i` | table, load `air_mass_per_rev_g` | yes | load axis >= 0; values in (0, 1) | **gross** indicated efficiency at MBT spark and lambda = 1 (see below) |
| `maps.f_lambda` | curve, x = lambda | yes | >= 2 points, x > 0, y >= 0, **f_lambda(1) == 1** | mixture correction on eta_i |
| `maps.eta_spark` | curve, x = spark - MBT [deg] | yes | >= 2 points, y in [0, 1], **eta_spark(0) == 1** | spark-timing correction on eta_i |
| `maps.theta_mbt_deg` | table, load `p_im_kpa` | yes | values in [-30, 70] | MBT spark advance [deg BTDC] |
| `maps.friction_torque_nm` | curve, x = rpm | yes | x[0] == 0; y >= 0 | friction torque magnitude; y(0) is the Coulomb part |
| `maps.pmep_excess_kpa` | table, load `p_im_kpa` | no (absent = 0) | signed, finite, \|value\| <= 1e6 | reserved: pumping loss beyond the ideal (p_em - p_im) term |
| `maps.theta_kl_deg` | table, load `p_im_kpa` | no (absent = no limit) | values in [-30, 70] | reserved: knock-limited spark advance; spark used = min(ECU target, theta_kl) |
| `inertia_kgm2` | number | yes | > 0 | as `torque_map` |
| `breakaway_torque_nm` | number | no (0) | >= `maps.friction_torque_nm(0)` | static Coulomb capacity |
| `idle_rpm`, `stall_rpm`, `fire_rpm`, `start_complete_rpm`, `dfco_enable_rpm`, `dfco_reenable_rpm` | number | no | same ordering rules as `torque_map` | |
| `idle_control.kp_per_rpm`, `ki_per_rpm_s`, `max_throttle` | number | no | max_throttle in (0, 1] | electronic: PI on the plate; mechanical: PI on the bypass valve (max_throttle = largest valve opening). Pedal released (request < 0.01): the P term only adds air below idle_rpm and the integrator stays in [0, cap - ff] (ff = the no-load idle position), during the overrun fuel cut the position is held at ff (dashpot), and a manifold-fill term opens further while the manifold is below its target (x with f(p, x) = 4 f(p, theta_idle)), so a lift-off settles onto idle from above. Pedal pressed: the two-sided PI. |
| `idle_control.max_torque_nm` | number | no | >= 0, default 0 | caps the idle controller at the position whose steady net torque at idle_rpm is this much above zero - enough to creep in gear, not a launch aid; 0 = max_throttle alone. Per engine file (a truck engine sets its own). Folded into the maps content hash only when > 0 |
| `limiter`, `starter`, `crank_axis_local`, `initial_state` | | no | as `torque_map` | |
| `fuels` / `fuel` | array / string | **yes** | as `torque_map`'s [`fuels`](#fuels-ignition-indicated_efficiency) | the engine's torque comes from burnt fuel, so a fuel is mandatory |
| `ignition` | string | **yes** | `"spark"` only | `"compression"` is rejected until a CI combustion model exists |

**eta_i is GROSS indicated efficiency at MBT spark and lambda = 1.** The
indicated work is `W_ind = eta_i(rpm, m_air_per_rev) * f_lambda(lambda) *
eta_spark(spark - MBT) * LHV * m_fuel`. `f_lambda` and `eta_spark` are
applied on top of it, so the loader requires them to be exactly 1 at their
reference points. Pumping (`(p_em - p_im + pmep_excess)` times the
displaced volume) and friction are separate torque terms and are never
folded into eta_i. The loader also rejects a map set whose
`max(eta_i) * max(f_lambda) * max(eta_spark)` reaches 1.

### Input types (I7 plan section 3.4.5)

- **Electronic** (drive-by-wire): the throttle command is a torque request.
  The ECU turns it into a plate target through an inverse table built at
  load time; the plate follows through a first-order actuator
  (`throttle_actuator_tau_s`). The idle PI raises the plate. A
  `bypass_area_mm2` is rejected (idle air comes from the plate).
- **Mechanical** (cable): the command IS the plate position. Idle air comes
  from the bypass valve (`bypass_area_mm2`, required), driven by the idle
  PI. `throttle_actuator_tau_s` is rejected (a cable has no actuator). As
  for a mechanical `torque_map` engine, a `manual_tcu` with nonzero
  `auto_blip` blip/ramp values (after defaults) is rejected.

An explicit `throttle_control` component must agree with `throttle_input`
in both directions, exactly as for `torque_map`.

### Rejected on a simulated engine

`wot_torque_nm_vs_rpm`, `motoring_torque_nm_vs_rpm` (derived, not given),
`governor` (torque_map-only in I7), `fuel_map` (fuel follows from the air
path and the lambda target), `indicated_efficiency` (use `maps.eta_i`).
A piston key on a rotary engine and a rotary key on a piston engine are
rejected by name.

### Rejected on a torque_map engine

Every simulated-only key (`architecture`, the geometry keys,
`intake_manifold_volume_l`, `throttle_body`, `throttle_actuator_tau_s`,
`ecu`, `maps`) and `idle_control.bypass_area_mm2`. A torque_map file
carrying one almost certainly meant `"kind": "simulated"`.

### Example (fragment)

```json
{
  "id": "engine", "type": "engine", "kind": "simulated",
  "architecture": "piston_4stroke", "bore_mm": 86.0, "stroke_mm": 86.0, "cylinders": 4,
  "intake_manifold_volume_l": 3.0,
  "throttle_body": { "diameter_mm": 60.0, "discharge_coefficient": 0.8,
                     "area_fraction_vs_position": { "x": [0, 0.5, 1], "y": [0.01, 0.4, 1.0] } },
  "throttle_input": "electronic", "throttle_actuator_tau_s": 0.04,
  "ecu": {
    "lambda_target":     { "rpm": [0, 7000], "p_im_kpa": [20, 100], "values": [[1.0, 1.0], [1.0, 0.9]] },
    "spark_advance_deg": { "rpm": [0, 7000], "p_im_kpa": [20, 100], "values": [[30, 20], [40, 30]] }
  },
  "maps": {
    "source": "documentation example, not physical data",
    "ve":            { "rpm": [0, 3000, 7000], "p_im_kpa": [20, 100], "values": [[0.80, 0.85], [0.88, 0.95], [0.85, 0.90]] },
    "eta_i":         { "rpm": [0, 7000], "air_mass_per_rev_g": [0, 1.5], "values": [[0.34, 0.37], [0.36, 0.38]] },
    "f_lambda":      { "x": [0.7, 1.0, 1.5], "y": [0.85, 1.0, 0.9] },
    "eta_spark":     { "x": [-40, 0, 10], "y": [0.5, 1.0, 0.95] },
    "theta_mbt_deg": { "rpm": [0, 7000], "p_im_kpa": [20, 100], "values": [[30, 20], [40, 30]] },
    "friction_torque_nm": { "x": [0, 1000, 7000], "y": [15, 18, 40] }
  },
  "inertia_kgm2": 0.15, "breakaway_torque_nm": 20.0,
  "fuel": "../fuels/gasoline.json", "ignition": "spark"
}
```

### `cycle`: maps generated from the 0D cycle model (I7c C16a)

A piston simulated engine may carry a `"cycle"` block instead of (or in
addition to) the `maps` block. The loader then runs the map generator
(`ps::drivetrain::generate_engine_maps`, `ps/drivetrain/engine_maps.h`,
algorithm in `core/src/engine_cycle/map_generator.cpp`) on the 0D
single-cylinder cycle model (`ps::engine_cycle`, descriptor
`ps/drivetrain/engine_cycle_desc.h`) at **load time** and fills every
runtime map the file does not give. Nothing is generated at runtime and
nothing is cached on disk yet (C16b adds the `.psmaps` cache).

**Geometry is the engine's own.** `bore_mm`, `stroke_mm` and `cylinders`
come from the engine object; repeating them inside `cycle` is rejected.
Only `"architecture": "piston_4stroke"` may carry a cycle block (the rotary
cycle model is I7e).

| Key | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `source` | string | yes | non-empty | provenance of the cycle inputs (PLAN.md section 8) |
| `*_source` | string | no | any key ending in `_source` | per-value citation notes; ignored by the loader |
| `rod_mm` | number | yes | > stroke/2 + \|pin offset\| | connecting-rod length |
| `pin_offset_mm` | number | no (0) | | piston-pin offset |
| `compression_ratio` | number | yes | > 1 | |
| `intake_valves`, `exhaust_valves` | integer | yes | > 0 | valves per cylinder |
| `intake_valve_d_mm`, `exhaust_valve_d_mm` | number | yes | > 0 | valve head diameter |
| `intake_lift`, `exhaust_lift` | object | yes | see below | `{"lift_mm_vs_deg": curve, "phase_ref_deg": number}` |
| `runner_length_mm`, `runner_diameter_mm` | number | yes | > 0 | intake runner (a lumped A/L inertia) |
| `intake_port_cc`, `exhaust_port_cc` | number | yes | > 0 | port volumes |
| `cd_vs_l_over_d` | curve | yes | >= 2 points, x (L/D) >= 0 ascending, Cd in (0, 1] | valve discharge coefficient over lift/diameter |
| `cam` | object | no (fixed cams) | keys `intake_advance_deg`, `exhaust_retard_deg`: tables over `rpm` x `p_im_kpa` | cam-phaser schedule, sampled at each operating point |
| `spark_advance_deg` | table over `rpm` x `p_im_kpa` | no (empty = MBT tracking) | values in [-30, 70] | Tier 1b ECU spark schedule [deg BTDC]. Every grid node's cycle then runs with its Wiebe start at -(schedule) + `ignition_delay_deg` (deg ATDC); the stored `eta_i` is the MBT-equivalent value eta_gross(scheduled)/eta_spark(scheduled - theta_mbt), so the runtime's eta_i x eta_spark reproduces the cycle at every node. The same table becomes `ecu.spark_advance_deg` (which must then be absent). Without it every node runs at MBT and the maps are bit-identical to before the key existed |
| `ignition_delay_deg` | number | no (0) | [0, 60] | Tier 2 flame-development delay, crank deg: Wiebe start = spark + delay; generated `theta_mbt_deg` = -(MBT Wiebe start - delay) |
| `wiebe_a`, `wiebe_m`, `burn_duration_deg`, `ca50_mbt_deg_atdc` | number | no (5, 2, 60, 8) | a > 0, m >= 0, duration > 0 | Wiebe burn; MBT = the spark that puts CA50 at `ca50_mbt_deg_atdc` |
| `woschni_c1_gx`, `woschni_c1_cc`, `woschni_c2`, `wall_temperature_k`, `head_area_factor` | number | no (6.18, 2.28, 3.24e-3, 450, 1.1) | > 0 | Woschni heat transfer |
| `combustion_eff_lean` | number | no (0.98) | (0, 1] | combustion efficiency at lambda >= 1 (times lambda below 1) |
| `exhaust_backflow_temperature_k` | number | no (800) | > 0 | temperature of exhaust gas flowing back into the cylinder |
| `friction` | object | unless `maps.friction_torque_nm` is given | see below | FMEP correlation |
| `firing_order` | array of integers | no (`[1, 3, 4, 2]` only for `cylinders` == 4; required otherwise) | a permutation of `1..cylinders` | 1-based cylinder numbers, in FIRING order (first entry fires first) |
| `firing_angles_deg` | array of numbers | no (derived) | one value per cylinder, in CYLINDER-NUMBER order (index 0 = cylinder 1), each in `[0, 720)`, pairwise distinct | explicit per-cylinder firing angle, crank degrees, 0 = cylinder 1's firing TDC; omit to derive even `720/cylinders` spacing in `firing_order`'s order - REQUIRED for an odd-fire engine (a V-twin, some V6) |
| `banks` | array of arrays of integers | no (one bank, every cylinder) | every `1..cylinders` covered exactly once | groups cylinder numbers into exhaust banks (source positions, A8+) - does not affect firing timing |
| `bank_angle_deg` | number | no (0) | | the V angle between banks, geometry only |
| `grid_rpm` | array | yes | >= 2 values, > 0, strictly ascending | Full-quality rpm grid of the generated tables |
| `grid_p_im_kpa` | array | yes | >= 2 values, > 0, strictly ascending | Full-quality manifold-pressure grid |

Defaults in parentheses are the descriptor's literature values
(`engine_cycle_desc.h`, each with its citation there). Unknown keys are
rejected. `back_pressure_kpa_vs_exh_kg_s` is rejected: the simulated
engine's runtime has no exhaust back pressure until I7d.

A lift curve's x is crank degrees relative to `phase_ref_deg`; the span
(last x - first x) must be < 720 deg, the first and last lift exactly 0 and
every lift >= 0.

**Firing order and banks (ps_audio A1, out/audio_plan.md section 1/3).**
Descriptor only for now: NEITHER the map generator nor the single-cylinder
0D cycle model this section otherwise describes reads `firing_order`,
`firing_angles_deg`, `banks` or `bank_angle_deg` yet, so today's generated
maps and an engine's `state_hash` are unaffected by them (a later
increment, PLAN.md's ps_audio A5, wires a multi-cylinder cycle model to
them). `firing_order` lists cylinders in the order they fire (1-based,
e.g. `[1, 3, 4, 2]` for the common inline-4 order); `firing_angles_deg`,
when given, is the explicit per-cylinder crank angle (cylinder-number
order, not firing-order position) at which that cylinder reaches its own
firing TDC, needed for an odd-fire engine where no even-spacing rule
applies. `banks`/`bank_angle_deg` are exhaust-bank geometry only (source
positions once ps_audio lands) and never affect firing timing.
`ps::engine_cycle::resolve_firing_angles_deg`/`validate_firing_order`/
`validate_firing_angles` (`ps/engine_cycle/firing_order.h`) implement the
derivation and validation; the loader (`read_cycle_block`) calls them and
wraps any rejection into its own `"<ctx>: ..."` message.

**Friction.** `{"model": ..., "a_kpa", "b_kpa", "c_kpa"}`, FMEP = a + b*s +
c*s^2 [kPa] with s = rpm/1000 (`"polynomial_rpm"`) or the mean piston speed
in m/s (`"polynomial_piston_speed"`); `a_kpa` >= 0 is the Coulomb part.
The generated `friction_torque_nm` is FMEP * V_d / (4 pi) sampled at rpm 0,
every grid rpm and every midpoint between two grid rpms. The cycle model
itself has no friction: the correlation is the only friction source.

**Which maps are generated (override precedence).** A key in `maps` always
wins; the generator fills the rest. The generator runs only if at least one
of the seven keys `ve`, `eta_i`, `f_lambda`, `eta_spark`, `theta_mbt_deg`,
`friction_torque_nm`, `pmep_excess_kpa` is missing. A file that gives all
seven carries a complete override: the generator is not called at all, and
the cycle block is only parsed and validated. With a cycle block every
`maps.*` key is optional and the `maps` block itself may be absent. If
`maps` is present, its `source` stays required and the loader appends
`" | generated from the cycle block (<keys>): <generator source>"`; without
`maps`, `source` is the generator's own. `theta_kl_deg` is never generated
(knock is C17b). The efficiency sanity bound and the breakaway rule
(`breakaway_torque_nm >= maps.friction_torque_nm(0)`) are checked on the
MERGED maps; the value range checks above apply to file values only.
`pmep_excess_kpa` values are signed, both generated and file-provided (I7c
C16b review N1: the runtime - `engine_simulated.cpp`'s `tau_pump` term -
takes them as a plain signed additive term with no sign special-casing, so
a file value follows the same rule as a generated one; the loader bounds
the magnitude to <= 1e6 Pa as a sanity check, not a physical constraint).
When the generator runs from a cycle block, its `source` note also carries
the cycle block's own `source` citation (I7c C16b review N4) so a
generated engine's data provenance survives past load time.

What the generator computes (full detail in `engine_maps.h`): every grid
node at lambda 1, MBT spark and ISA intake state gives `ve` (net fresh air
through the intake valve over the swept volume at manifold density
p_im/(287.05*288.15 K)), `theta_mbt_deg`, `pmep_excess_kpa`
(the cycle's pumping work beyond the ideal `p_amb - p_im` term) and the
gross indicated efficiency at that node; `eta_i` is resampled onto a union
air-mass-per-revolution axis per rpm line. `eta_spark` is the mean of a
spark sweep at three sites, `f_lambda` a lambda sweep at 3000 rpm / 80 kPa.
The fuel is the engine's first `fuels` entry. At every grid node the
runtime reproduces the cycle model's net torque exactly (1e-9).

**Quality.** `ps::io::EngineMapOptions` (`ps/io/vehicle_io.h`, the
`load_vehicle_json(path, options)` / `load_scenario(path, options)`
overloads): `quality` `Full` (default; the whole grid, 5 cycles per point,
step 1/(960*48) s) or `Preview` (at most 10 x 6 of the grid nodes, 4 cycles,
step 1/(960*24) s), `pool` (a `JobSystem` to run the operating points on;
`nullptr` = the calling thread). The result is bit-identical for any worker
count. `ps_run --map-workers N` generates on an N-worker pool that lives
only for the load. Measured on the ASSUMED 2 L I4 test fixture with a
12 x 20 grid (release build): Preview 0.33 s on one thread / 0.034 s on
10 workers, Full 2.24 s / 0.23 s.

**Off-node accuracy caveat.** The runtime interpolates the generated tables
bilinearly. eta_i, pumping and friction stay within 1 % between nodes on a
500 rpm x 5 kPa grid, but the trapped air mass does not: the intake runner
(a lumped Helmholtz resonator) produces a real, step- and cycle-converged
ripple of about 100 rpm period and up to +-2.5 % at light load (measured
+-2.5 % near 1700 rpm / 22.5 kPa on the fixture, worst cell-centre error
7.7 %, mean 1.6 %). No practical rpm grid resolves it; treat it as sub-grid
detail. A generated engine's `state_hash` folds the maps' content hash, so
two engines on different maps never hash alike.

Rejections specific to the cycle block: a rotary engine; `bore_mm` /
`stroke_mm` / `cylinders` inside `cycle`; `back_pressure_kpa_vs_exh_kg_s`;
an unknown key; a missing `friction` without `maps.friction_torque_nm`;
`firing_order` not a permutation of `1..cylinders` (wrong length, a value
out of range, or a repeat), missing with no default (`cylinders` != 4);
`firing_angles_deg` not exactly one value per cylinder, a value outside
`[0, 720)`, or two cylinders sharing an angle; `banks` not covering every
cylinder exactly once, or an empty bank list; every range rule in the
table above. Generator errors (e.g. an operating point whose cycle does
not converge) propagate with their own `"engine map generator: ..."` /
`"CycleModelDesc: ..."` messages.

### `intake`/`exhaust`: the finite-volume pipe pair (ps_audio A4)

A4 (out/audio_plan.md sections 3.1/3.2) is **data only**: the loader
validates and round-trips these fields, nothing consumes them yet (the map
generator still uses `runner_length_mm`/`runner_diameter_mm`; a future
ps_audio A5 wires the pipe pair into the generator, and the runtime engine
sound/audio path into the pipe network). `intake` and `exhaust` must be
given **together** (a manifold with no exhaust plumbing modeled, or vice
versa, is not a supported configuration) or **not at all**; when given, the
pre-A4 lumped `runner_length_mm`/`runner_diameter_mm` pair is rejected
instead (the pipe blocks replace it) - a cycle block without either pair is
also rejected, exactly as before A4. `default_installation` (below) and
`resolution` are only allowed together with the pipe pair.

`intake` fields:

| Key | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `source` | string | yes | non-empty | provenance (PLAN.md section 8) |
| `throttle_bore_m` | number | yes | > 0 | throttle body bore, metres |
| `plenum_m3` | number | yes | > 0 | plenum/manifold volume |
| `runners` | array of objects | yes | non-empty | one entry per runner or runner group |
| `runners[].cylinders` | `"all"` or array of integers | yes | `"all"` only as the sole entry, else every `1..cylinders` covered exactly once | which cylinders this runner feeds |
| `runners[].length_m`, `d_in_m`, `d_out_m` | number | yes | > 0 | runner length and inlet/outlet diameter (may taper) |
| `interface` | string | yes | non-empty | the pipe network node name where this intake meets the installation file's own network (A5) |

`exhaust` fields:

| Key | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `source` | string | yes | non-empty | provenance |
| `primaries` | array of objects | yes | non-empty, `cylinders` covers every `1..cylinders` exactly once (same `"all"` convention as `intake.runners`) | one primary pipe per cylinder or cylinder group |
| `primaries[].length_m`, `d_m` | number | yes | > 0 | |
| `merges` | array of objects | yes | non-empty | the collector tree, one entry per junction, in the order pipes reach the flange |
| `merges[].from` | array, >= 2 entries | yes | each entry a 1-based cylinder number (must be unique across every merge's `from`, and together with every other merge cover every cylinder exactly once) or a string naming an EARLIER merge's `into` (forward references rejected; each name consumed by at most one later merge) | what feeds this junction |
| `merges[].into` | string | yes | non-empty, no duplicate `into` name | this junction's own name, referenceable by a later merge's `from` |
| `merges[].length_m`, `d_m` | number | yes | > 0 | the merged pipe leaving this junction |
| `interface` | string | yes | non-empty, must equal the LAST merge's `into` | the flange - every non-last merge's `into` must be consumed by exactly one later merge (no dangling branch) |

`default_installation` (string, path relative to the ENGINE file's own
directory, required with the pipe pair, rejected without it): the
`physics_sim.installation/1` file (below) this engine plugs into unless a
vehicle overrides it (`docs/data_formats/vehicle.md`'s per-engine
`"installation"` component field - the vehicle-component JSON's own
directory-relative override, resolved and loaded once both the vehicle
component JSON and this engine's parsed cycle block are in hand; only
allowed for a simulated engine whose cycle block has pipe blocks).

`resolution` (object, optional, only allowed with the pipe pair): the
finite-volume cell size used by the pipe network (A5/A7), CFL-checked at
load time against the worst-case wave speed/step-rate combination the plan
lists (exhaust 900 m/s, intake 500 m/s; `power_dx_m` against the map
generator's Preview step 1/(960*24) s, `audio_dx_exhaust_m`/
`audio_dx_intake_m` against the live 44.1 kHz audio rate) - a value below
the CFL minimum is rejected with that minimum in the message.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `power_dx_m` | number | 0.05 | cell size for load-time map generation (coarse, cheap) |
| `audio_dx_exhaust_m` | number | 0.032 | live exhaust-pipe cell size |
| `audio_dx_intake_m` | number | 0.017 | live intake-pipe cell size |

An engine WITHOUT `intake`/`exhaust` carries `resolution` at these defaults
but never writes `default_installation`/`installation`/`installation_path`
- bit-identical to a pre-A4 `CycleModelDesc` (`tests/unit/
test_engine_installation_io.cpp`'s own regression TEST_CASE checks this
directly; `test_engine_map_cache.cpp`'s H3 section checks the cache key is
unaffected). See `ps/drivetrain/engine_cycle_desc.h`'s own field comments
for the exact struct shapes (`IntakeCycleDesc`, `ExhaustCycleDesc`,
`MergeSourceRef`, `ResolutionCycleDesc`) and `docs/data_formats/
installation.md` for the installation file format.

Rejections specific to these blocks: every rule in the two tables above;
`intake`/`exhaust` given one without the other; `runner_length_mm`/
`runner_diameter_mm` given together with them; `default_installation`
missing with them present or given without them; `resolution` given
without them; an unknown key at any level; the vehicle-component
`"installation"` override on an engine without pipe blocks, on a
non-simulated engine, or as an empty string.

### Shipped simulated engine: `toyota_a25a_fks_sim.json` (measured maps)

PHYS-033 P4 I7c C17a (fallback M-B). `data/engines/toyota_a25a_fks_sim.json`
is the Toyota A25A-FKS on maps back-solved from the US EPA NVFEL Tier 3
test-data xlsx (the same package as the torque-map
`toyota_a25a_fks.json`), written by `tools/enginedata/a25a_mb_engine.py`
(rerun it to regenerate; it also writes `out/a25a_mb_report.txt`).
`data/vehicles/car_sedan_sim.json` is `car_sedan.json` with this engine and a
350 N·m clutch. The `ve`, `lambda_target`, `spark_advance_deg` and
`pmep_excess_kpa` tables are gridded from measured points over (rpm, MAP).
`eta_i` is an *effective* indicated efficiency: it is back-solved per point
through the runtime's own torque equation, with the file's own
`friction_torque_nm`, the file's own pumping term and the first `fuels`
entry. It therefore absorbs any friction or pumping model error.
`friction_torque_nm` is fitted to the engine's own measured motoring rows.
`theta_mbt_deg` equals the spark table, so `eta_spark` is 1 throughout.

Because the maps were solved from the WOT points, reproducing those points is
tautological. It proves the file round-trips through the loader and runtime,
not that a cycle generator predicts anything
(`tests/unit/test_a25a_sim_engine_data.cpp`). The (rpm, MAP) tables have no
cam-phase input. Steady-state points just below WOT MAP, measured at a
different cam phasing, sit 30–60 N·m under the model. The per-point
residuals are in the report.
