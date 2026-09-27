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
(`"simulated"`, a full MVEM engine) can sit next to `torque_map` once it
exists (I7b) — see the [`kind`](#kind) section below. `torque_map` also
gained an optional fuel model in C5: [`fuel`](#fuel) and
[`fuel_map`](#fuel_map) below; C5d added a no-map alternative,
[`fuels`](#fuels-ignition-indicated_efficiency)/[`ignition`](#fuels-ignition-indicated_efficiency)/
[`indicated_efficiency`](#fuels-ignition-indicated_efficiency), for an
engine that burns fuel but has no calibrated map.

### `kind`

| Value | Status | Meaning |
|---|---|---|
| absent, or `"torque_map"` | shipped | the `torque_map` engine documented in this file |
| `"simulated"` | rejected — `"<ctx>: engine kind \"simulated\" is not yet supported (PHYS-033 I7b)"` | reserved for a future full MVEM engine |
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
| `throttle_input` | string | optional (PHYS-033 P4 I7a C3; a simulated MVEM engine, out of this commit's scope, will require it explicitly) | `"electronic"` | must be `"electronic"` or `"mechanical"`, else `"<ctx>: throttle_input must be \"electronic\" or \"mechanical\", got \"<value>\""` | which throttle-command stage kind this engine expects when no explicit `throttle_control` powertrain component names it (see `vehicle.md`'s ["`throttle_control` component fields (I7a C3)"](vehicle.md#throttle_control-component-fields-i7a-c3) section) — an explicit `throttle_control` component naming this engine must declare the SAME `kind` as this field, both directions checked at load time; a mismatch is rejected (`"<ctx>: throttle_control 'kind' ... does not match engine '<id>'s throttle_input ..."`), it is never silently overridden |
| `fuel` | string (path) | optional; **required if `fuel_map` present** | absent (no fuel model) | non-empty string, else `"<ctx>: 'fuel' must be a non-empty path string"`; must resolve and load via `ps::io::load_fuel_json` | path to a `physics_sim.fuel/1` file (see `fuel.md`), resolved relative to THIS engine JSON's own directory — the referenced engine file's own directory when the engine itself was reached via a vehicle's `{"ref": ...}`, else the vehicle file's directory for an inline engine block |
| `fuel_map` | object, see [below](#fuel_map) | optional | absent (`has_fuel_map = false`, no fuel computation anywhere) | see below | fuel flow (g/s) vs. (rpm, load) — `load` is `theta_c`, the post-limiter combustion fraction |
| `fuels` | array of string (paths) | optional; mutually exclusive with `fuel`/`fuel_map` | absent | non-empty array of non-empty path strings, no duplicates (resolved-path comparison), each must resolve and load via `ps::io::load_fuel_json`; rejected alongside `fuel` or `fuel_map` | PHYS-033 P4 I7a C5d: a plain list of fuels this engine can run on, for the crude no-map estimate (see [below](#fuels-ignition-indicated_efficiency)) — `fuels[0]` is the ACTIVE fuel, no blending; a hypothetical per-entry weighted blend (C5e) was cancelled |
| `ignition` | string | **required whenever a fuel model exists** (`fuel_map` present, or `fuels` non-empty); optional otherwise | none (rejected if required and absent) | must be `"spark"` or `"compression"`, else `"<ctx>: ignition must be \"spark\" or \"compression\", got \"<value>\""`; required-but-absent: `"<ctx>: an engine with a fuel ('fuel'+'fuel_map' or 'fuels') requires 'ignition' (\"spark\" or \"compression\")"` | an ENGINE property, not a fuel one (2026-09-27 owner ruling — see [below](#fuels-ignition-indicated_efficiency)); picks the crude no-map estimate's default indicated efficiency |
| `indicated_efficiency` | real (fraction) | optional; requires a fuel model | ignition-based Heywood default (see below) | must be in `(0, 1)` exclusive and finite; rejected without a fuel model: `"<ctx>: indicated_efficiency requires a fuel model ('fuel_map' or 'fuels')"` | override for the crude no-map estimate's `eta_i`; ignored (never read) by the `fuel_map` path |

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
ignition-based default (Heywood, *Internal Combustion Engine Fundamentals*,
1988): **0.325** for `"spark"` (Table/p.827-region typical SI indicated
efficiency), **0.48** for `"compression"` (p.859-region typical CI indicated
efficiency). This is deliberately crude — no pumping-loss/friction split
beyond what the motoring curve already folds in, no load or rpm dependence
of `eta_i` — and is expected to overstate fuel consumption somewhat near
WOT (where a real engine's indicated efficiency typically peaks above a
flat constant) and understate it at idle/light load (where real indicated
efficiency drops well below the constant, e.g. pumping losses at closed
throttle); a calibrated `fuel_map` is the accurate alternative once one
exists for a given engine.

`has_fuel_model(TorqueMapEngineDesc)` (`ps/drivetrain/powertrain_desc.h`) is
`has_fuel_map || !fuels.empty()` — distinct from `has_fuel_map` (a
calibrated map specifically): it gates whether fuel/heat booking
(`fuel_mass`/`fuel_energy_in`/`engine_heat`/`fuel_efficiency_violations` in
`PowertrainEnergy`) and the `vehicle<i>.fuel.*` telemetry channels are
active at all, while `has_fuel_map` alone still picks map-vs-crude *within*
that. `fuel_efficiency_violations` never increments for the crude path (a
constant `eta_i` cannot itself imply >100% efficiency the way a map can at
an unchecked point between samples).

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
