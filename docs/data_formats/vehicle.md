# Vehicle format — `physics_sim.vehicle/2`

Files: `data/vehicles/*.json`. Loader: `ps::io::load_vehicle_json`
(`core/src/io/vehicle_io.cpp`); public descriptor:
`ps::vehicle::VehicleDesc` (`core/include/ps/vehicle/vehicle_desc.h`).
Powertrain-graph validation beyond parsing happens separately in
`core/src/drivetrain/powertrain.cpp` (`Powertrain::Powertrain`) — see
["Powertrain graph"](#powertrain-graph) below.

A vehicle describes: a set of wheels (each with suspension + tyre), a
steering model, a brake system, a powertrain graph (engine → clutch →
gearbox → differential → wheels, or any subset), aerodynamic drag, and at
most one driver-assist controller. It does **not** describe the chassis
rigid body itself — a vehicle is always attached to an already-created
`BodyId` (`World::create_vehicle(desc, chassis)`; in a scenario file, see
`scenario.md`'s `vehicles[]`).

See `README.md` for: format versioning, the shared `Curve1D` format, path
resolution for `"tyre"`/`"ref"` references, coordinate conventions (ISO
8855: X forward, Y left, Z up), and the `"source"` field convention.

## Top level

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required | — | must equal `"physics_sim.vehicle/2"` | format/version tag; see README |
| `name` | string | optional | `""` | none | vehicle name, free text |
| `source` | string | **required** | — | must be non-empty, else `"vehicle '<name>': 'source' is required (PLAN.md section 8)"` | provenance note (see README's "source" convention) |
| `wheels` | array of [wheel objects](#wheel) | **required** | — | must exist, be an array, and be non-empty, else `"vehicle <path> has no 'wheels' array"` | one entry per wheel |
| `steering` | [steering object](#steering) | optional | all fields at their own defaults | see below | Ackermann steering geometry |
| `brake_system` | [brake_system object](#brake_system) | optional | no corners, no handbrake | see below | direct-torque friction brakes |
| `aero` | object `{drag_area_m2}` | optional | `drag_area_m2 = 0` | `drag_area_m2 >= 0`, else `"vehicle '<name>': aero.drag_area_m2 must be >= 0"` | bluff-body drag: `F_drag = -0.5*rho*CdA*|v|*v` at the chassis CoM; `drag_area_m2` **is** `Cd*A` already combined, not a separate `Cd`/frontal-area pair |
| `powertrain` | [powertrain object](#powertrain-graph) | optional | empty (no components, no shafts) | see below | the engine/clutch/gearbox/differential graph |
| `controllers` | array of [controller objects](#controllers) | optional | empty | at most 1 entry, else `"vehicle '<name>': at most one controller (manual_tcu) is supported in I3"` | driver-assist logic (auto-clutch, auto-blip, auto-shift) |

**Rejected /1 fields.** A `physics_sim.vehicle/2` file must not contain a
top-level `"drive"` or `"brakes"` block (the `/1` placeholder drive/brake
model) — either one throws immediately:
`"vehicle <path>: 'drive' is no longer a vehicle field (physics_sim.vehicle/2) - see 'powertrain' (PHYS-033 Q7)"`
(and the analogous message for `"brakes"` → `"brake_system"`).

## Wheel

One entry of the top-level `wheels` array (`ps::vehicle::WheelDesc`).

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `name` | string | optional | `"wheel"` | none, but must be **unique** to be usable by `brake_system`/`powertrain`/`arb_partner`/telemetry (a duplicate name silently makes later references ambiguous — not itself an error at this point) | wheel identifier, e.g. `"FL"`; used in telemetry channel names, `brake_system.corners[].wheel`, `handbrake.wheels[]`, and powertrain `"wheel:<name>"` port refs |
| `attachment_local` | Vec3 (m) | optional | `[0,0,0]` | must be a 3-element array if present | wheel centre position in the chassis' local frame at zero suspension travel (full extension); ISO 8855 (X forward, Y left, Z up) |
| `static_camber` | real (rad) | optional | `0` | none | added to `suspension.camber_vs_travel(x)` |
| `static_toe` | real (rad) | optional | `0` | none | added to `suspension.toe_vs_travel(x)`; positive = toe-in for a front-left-style wheel |
| `wheel_radius` | real (m) | optional | `0.31` | none | tyre-shape (`CylinderShape`) radius for the wheel's contact query |
| `wheel_width` | real (m) | optional | `0.205` | none | `CylinderShape` half_height*2 |
| `wheel_mass` | real (kg) | optional | `18.0` | none | spin inertia only — unsprung body mass is not separately simulated |
| `wheel_inertia` | real (kg·m²) | optional | `0.9` | **must be > 0**, checked not by `vehicle_io.cpp` but by `Powertrain::Powertrain` (`core/src/drivetrain/powertrain.cpp`), which every `World::create_vehicle` call runs unconditionally (even for a vehicle with an empty `powertrain` block) — so `load_vehicle_json` alone won't catch a bad value, but building the vehicle into a `World` will: `"vehicle '<name>': wheel '<wheel>' has non-positive wheel_inertia (needed as the drivetrain hub's own inertia)"` | spin inertia about the wheel's own axis; also the drivetrain hub node's inertia |
| `suspension` | [suspension object](#suspension) | optional | all fields at their own defaults | see below | per-corner spring/damper/ARB |
| `arb_partner` | int | optional | `-1` | **not range-checked** by the loader (see README "Discrepancies found") | index into `wheels[]` of this wheel's anti-roll-bar pair; `-1` = none |
| `tyre` | [tyre reference](tyre.md) | optional | empty `TyreDesc` (all its own defaults) | see `tyre.md` | inline tyre object, a path to a standalone tyre JSON, or a path to a `.tir` file |
| `steered` | bool | optional | `false` | none | whether this wheel responds to the `"steer"` control channel |
| `is_front` | bool | optional | `true` | none | selects the Ackermann-axle-averaging group and the rear-steer sign (see below); does **not** by itself pick a `brake_system` corner (that matches by `name`) |
| `driven` | — | **rejected** | — | presence throws `"wheel '<name>': 'driven' is no longer a wheel field (physics_sim.vehicle/2) - which wheels are driven is now determined by the 'powertrain' graph (PHYS-033 Q7)"` | superseded `/1` field; whether a wheel is driven is now whatever the `powertrain` graph connects to `wheel:<name>` |

`is_front` in practice (`core/src/vehicle/vehicle_instance.cpp`): the
average `attachment_local.x` of every `is_front == true` wheel is the front
axle x; the average of every other wheel is the rear axle x; wheelbase is
their difference (floored at 0.5 m). A steered wheel with `is_front ==
false` uses `-steering.rear_steer_fraction * delta_avg` instead of Ackermann
geometry.

## Suspension

`SuspensionDesc`, nested under a wheel's `"suspension"` key. Suspension
travel is a scalar `x` in `[0, travel_max]` along `axis`: `x = 0` is full
extension (spring free length), `x = travel_max` is full bump. Measured
kinematically each substep (a shape cast), not integrated as a separate
unsprung-mass state.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `axis` | Vec3 | optional | `[0, 0, -1]` | 3-element array if present | chassis-local direction from full extension toward full bump |
| `travel_max` | real (m) | optional | `0.09` | none | maximum compression |
| `spring_rate` | real (N/m) | optional | `35000` | none | used only if `spring_curve` is empty |
| `spring_curve` | Curve1D (force N vs. compression x, m) | optional | empty | see README's Curve1D rules | overrides `spring_rate` when non-empty |
| `spring_preload` | real (N) | optional | `0` | none | added to the spring force (sets ride height) |
| `damper_bump_rate` | real (N·s/m) | optional | `3000` | none | used for `x` increasing (bump), only if `damper_curve` is empty |
| `damper_rebound_rate` | real (N·s/m) | optional | `4000` | none | used for `x` decreasing (rebound), only if `damper_curve` is empty |
| `damper_curve` | Curve1D (force N vs. compression rate ẋ, m/s) | optional | empty | see README's Curve1D rules | overrides both rates above when non-empty |
| `bump_stop_rate` | real (N/m) | optional | `400000` | none | engages only past `travel_max` |
| `camber_vs_travel` | Curve1D (rad vs. m) | optional | empty | see README's Curve1D rules | empty = no camber change with travel |
| `toe_vs_travel` | Curve1D (rad vs. m) | optional | empty | see README's Curve1D rules | empty = no toe change with travel |
| `arb_rate` | real (N/m) | optional | `0` | none | anti-roll-bar rate; force = `arb_rate * (x_self - x_partner)`; `0` = no ARB on this corner |

## Steering

`SteeringDesc`, top-level `"steering"`.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `max_wheel_angle` | real (rad) | optional | `0.5236` (30°) | none | wheel angle at full `"steer"` input (±1) |
| `ackermann_fraction` | real | optional | `1.0` | **must be in `[-1, 1]`**, else `"steering.ackermann_fraction must be in [-1, 1] (1 = full geometric Ackermann, -1 = full anti-Ackermann, 0 = parallel steer), got <value>"` | blends parallel steer (0) toward full geometric Ackermann (1) or full anti-Ackermann (-1); magnitude is how far the wheel is pushed from parallel steer toward/away from the geometric-Ackermann angle |
| `rear_steer_fraction` | real | optional | `0` | none | rear (non-front) steered wheels turn this fraction of the equivalent front angle, opposite sign; `0` = no rear steer |
| `steer_rate_limit` | real (rad/s) | optional | `0` | none | `0` = unlimited (instantaneous) |

## Brake system

`BrakeSystemDesc`, top-level `"brake_system"`. Optional — an absent block
means no brakes at all.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `type` | string | optional | `"direct_torque"` | must equal `"direct_torque"` (the only type in I3), else `"brake_system.type must be \"direct_torque\" (I5 adds \"hydraulic\"), got \"<type>\""` | brake model kind |
| `corners` | array of `{wheel, max_torque_nm}` | optional | empty | each `wheel` required, non-empty string (`"brake_system.corners[]: 'wheel' is required and must be a non-empty string"`), unique across corners (`"brake_system.corners[]: duplicate wheel '<name>'"`), and must name a wheel that actually exists (checked in `powertrain.cpp`: `"brake_system corner references unknown wheel '<name>'"`, and `"...has more than one corner for wheel '<name>'"`); `max_torque_nm >= 0` (`"brake_system.corners['<wheel>'].max_torque_nm must be >= 0"`) | per-corner service-brake friction-torque capacity |
| `handbrake` | object `{wheels[], max_torque_nm}` | optional | no wheels, `max_torque_nm = 0` | `max_torque_nm >= 0` (`"brake_system.handbrake.max_torque_nm must be >= 0"`); each name in `wheels[]` must exist (`"brake_system.handbrake references unknown wheel '<name>'"`) | wheels the handbrake channel acts on, and its capacity |
| `static_factor` | real | optional | `1.0` | **must be >= 1**, else `"brake_system.static_factor must be >= 1, got <value>"` | static brake-torque capacity = `static_factor * kinetic capacity`; `1.0` = no static/kinetic distinction |

A corner's/handbrake's `max_torque_nm` becomes a per-hub friction-row
capacity in the powertrain shaft network (`core/src/drivetrain/
components.cpp`'s `DirectTorqueBrakes`) — service and handbrake capacities
for the same hub are merged at graph-build time.

## Powertrain graph

`PowertrainDesc`, top-level `"powertrain"`. Optional; an absent/empty block
means the vehicle has no drivetrain at all (every wheel free-rolling, no
engine, no brakes beyond `brake_system`). Two things read this block:
`vehicle_io.cpp` (JSON → `PowertrainDesc`, mostly per-field validation) and
then `core/src/drivetrain/powertrain.cpp`'s `Powertrain` constructor (graph
validation — this is where most "does the graph make sense" errors come
from).

### Components

`"components"` is an array; **array order is evaluation order** and is
significant for scenario determinism. Each entry:

```json
{ "id": "<unique id>", "type": "engine" | "clutch" | "gearbox" | "gear_pair" | "differential" | "transfer_case" | "park_lock" | "pto" | "load_dyno", ... }
```

- `id`: required, non-empty, unique across `components[]`
  (`"powertrain.components[]: 'id' is required and must be a non-empty string"`,
  `"powertrain.components[]: duplicate id '<id>'"`, the latter re-checked
  again in `powertrain.cpp` as `"powertrain component id '<id>' is not
  unique"`).
- `type`: required (`"powertrain.components['<id>']: 'type' is required"`);
  unknown type throws `"powertrain.components['<id>']: unknown type
  \"<type>\""`.
- `engine`/`gearbox` components may use `{"ref": "<path>"}` **instead of**
  inline fields, to load a standalone [engine](engine.md)/[gearbox](gearbox.md)
  file. A `ref`'d entry may contain **only** `id`, `type`, `ref` — any other
  key alongside `ref` throws `"<ctx>: a component with 'ref' may only
  contain 'id', 'type' and 'ref' (found extra field '<key>')"`; there is no
  merging of inline overrides on top of a ref'd file. `clutch`, `gear_pair`,
  `differential`, `transfer_case`, `park_lock`, `pto` and `load_dyno`
  components are **always inline** — `{"ref": ...}` on a clutch throws
  `"<ctx>: 'clutch' components are always inline in I3 (no 'ref')"`.
- **At most one** `engine`, one `clutch`, one `gearbox`, one `transfer_case`,
  one `park_lock`, one `pto` component is allowed (checked in
  `powertrain.cpp`): a second `engine`/`clutch`/`gearbox` throws `"more than
  one <kind> component - I3 supports at most one"`; a second `transfer_case`
  throws `"vehicle '<v>': more than one transfer_case component - I4 C9
  supports at most one"` (I4 C9 - a second transfer case would need
  per-instance channel/hash disambiguation the plan never specifies). A
  second `park_lock` **or `pto`** is rejected **at load time**
  (`ps::io::load_vehicle_json`, not just later at `Powertrain` graph-build
  time like the other single-instance components above) —
  `"<ctx>: more than one park_lock component - at most one is supported"` /
  `"<ctx>: more than one pto component - at most one is supported"` — since a
  real vehicle only ever has one park pawl or one PTO shaft, and both I4
  C10's and I4 C11's own loader-validation tests exercise `load_vehicle_json`
  directly (see [`park_lock` component fields](#park_lock-component-fields-i4-c10)
  and [`pto` component fields](#pto-component-fields-i4-c11) below).
  Any number of `gear_pair`, `differential` **or `load_dyno`**
  components is fine (I4 C4 lifted the earlier "at most one differential"
  limit; `load_dyno` never had one — it is a test/bench-only load, see
  [`load_dyno` component fields](#load_dyno-component-fields-i4-c11) below) -
  a centre diff feeding two axle diffs, or any other multi-diff
  topology, is built by wiring their `carrier`/`left`/`right` ports together
  with ordinary shafts (see [Ports, by component
  type](#ports-by-component-type-fixed-names) and [What the graph builder
  additionally rejects](#what-the-graph-builder-additionally-rejects)
  below).

Field tables for `engine` and `gearbox` components (inline or `ref`'d) are
in [`engine.md`](engine.md) and [`gearbox.md`](gearbox.md) — the same
fields, same validation, whether the JSON lives in a standalone file or
directly inside `powertrain.components[]`.

#### `clutch` component fields

`ClutchDesc`. Always inline.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `capacity_nm` | real (N·m) | optional | `0` | **must be > 0**, else `"<ctx>: clutch capacity_nm must be > 0"` | kinetic torque capacity at engagement = 1 |
| `static_factor` | real | optional | `1.15` | **must be >= 1**, else `"<ctx>: clutch static_factor must be >= 1"` | static capacity = `static_factor * kinetic` |
| `engagement_vs_pedal` | Curve1D | **required** | — | must be present (`"<ctx>: clutch 'engagement_vs_pedal' is required"`), `x` strictly ascending and non-empty, `x` within `[0, 1]` (`"<ctx>.engagement_vs_pedal: x must be within [0, 1]"`), `y` **non-increasing** (`"<ctx>.engagement_vs_pedal: y must be non-increasing"`), `y` within `[0, 1]` (`"<ctx>.engagement_vs_pedal: y must be within [0, 1]"`) | pedal position (0 = released, 1 = fully pressed) → engagement fraction (1 = locked, 0 = fully open) |
| `in_inertia_kgm2` | real (kg·m²) | optional | `0` | none | inertia at the `in` port (pressure plate; usually folded into the engine's own inertia) |
| `out_inertia_kgm2` | real (kg·m²) | optional | `0` | none | inertia at the `out` port (driven disc) |
| `axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length (±1e-6), else `"<ctx>.axis_local must be zero or a unit vector, got length <len>"` | shared rotor-reaction axis for both ports (one physical shaft); zero = no rotor reaction |

Ports: `<id>.in`, `<id>.out`.

#### `gear_pair` component fields

`GearPairDesc`. Always inline.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `ratio` | real | optional | `1.0` | **must be != 0**, else `"<ctx>: gear_pair 'ratio' must be != 0"` | `omega_in = ratio * omega_out` |
| `efficiency` | real | optional | `1.0` | must be in `(0, 1]`, else `"<ctx>: gear_pair 'efficiency' must be in (0, 1]"` | mesh efficiency; `< 1` needs the pair's input node to carry no *other* rigid row, else `powertrain.cpp` throws `"gear_pair '<id>' has efficiency < 1 but its input node carries another rigid row - gear efficiency needs a single rigid coupling at its input node"` |
| `inertia_in_kgm2` | real (kg·m²) | optional | `0` | none | inertia at `in` |
| `inertia_out_kgm2` | real (kg·m²) | optional | `0` | none | inertia at `out` |
| `in_axis_local` / `out_axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, same rule/message as clutch `axis_local` | rotor-reaction axes |

Ports: `<id>.in`, `<id>.out`.

#### `differential` component fields

`DifferentialDesc`. Always inline. Any number of `differential` components
is allowed (I4 C4) — wiring more than one together via shafts builds a
centre/axle topology; see [What the graph builder additionally
rejects](#what-the-graph-builder-additionally-rejects) for the "no other
rigid coupling" leaf rule this depends on.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `kind` | string | **effectively required** (defaults to `""`, which fails the check below — must be given explicitly) | `""` | **must be `"open"`, `"locked"`, `"viscous"`, `"clutch_lsd"` or `"torsen"`** (I4 C6/C7), else `"<ctx>: differential 'kind' must be \"open\", \"locked\", \"viscous\", \"clutch_lsd\" or \"torsen\""`; `"active"` throws `"<ctx>: differential kind \"active\" is not supported until a torque-vectoring controller exists"` | differential type — `open` (free, Willis-constrained split), `locked` (spool: an unconditional `omega_left == omega_right` row alongside the diff row, present in every topology state — never spends a composed-state bit, see `lockable` below for the togglable equivalent), `viscous` (the open row plus an implicit `L-R` coupling of rate `viscous_nm_s_rad`), `clutch_lsd` / `torsen` (I4 C7: the open row plus one Coulomb friction row between `left` and `right`, present in every topology state, whose capacity limits `T_high - T_low`; see [clutch_lsd / torsen](#clutch_lsd--torsen-i4-c7) below) |
| `ratio` | real | optional | `1.0` | **must be != 0**, else `"<ctx>: differential 'ratio' must be != 0"` | final-drive ratio; the carrier's Willis constraint is `omega_carrier = (ratio/2) * (omega_left + omega_right)` |
| `efficiency` | real | optional | `1.0` | must be in `(0, 1]`, else `"<ctx>: differential 'efficiency' must be in (0, 1]"` | mesh efficiency (I3 lifts the earlier "must be exactly 1" restriction) |
| `carrier_inertia_kgm2` | real (kg·m²) | optional | `0` | none | inertia at `carrier` |
| `carrier_axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, same rule as above | rotor-reaction axis |
| `viscous_nm_s_rad` | real (N·m·s/rad) | optional, **only valid for `kind: "viscous"`** | `0` | present on a non-`viscous` kind throws `"<ctx>: differential 'viscous_nm_s_rad' is only valid for kind \"viscous\""`; on `kind: "viscous"` **must be > 0**, else `"<ctx>: differential kind \"viscous\" requires 'viscous_nm_s_rad' > 0"` | the coupling's implicit viscous rate `c`: reaction torque `c * (omega_left - omega_right)`, booked every substep as `diff_loss += h * c * v^2` (`PowertrainEnergy::diff_loss`, I4 C6) |
| `preload_nm` | real (N·m) | optional, **only valid for `kind: "clutch_lsd"`** | `0` | present on another kind throws `"<ctx>: differential 'preload_nm' is only valid for kind \"clutch_lsd\""`; **must be >= 0**, else `"<ctx>: differential 'preload_nm' must be >= 0"` | I4 C7: clutch-pack preload — the locking torque `T_high - T_low` available at zero axle torque |
| `ramp` | object `{drive_factor, coast_factor}` | **required for `kind: "clutch_lsd"`**, invalid on every other kind | — | present on another kind throws `"<ctx>: differential 'ramp' is only valid for kind \"clutch_lsd\""`; missing on `clutch_lsd` throws `"<ctx>: differential kind \"clutch_lsd\" requires a 'ramp' block {\"drive_factor\", \"coast_factor\"}"`; either key missing throws `"<ctx>: differential 'ramp' requires both 'drive_factor' and 'coast_factor'"`; each **must be in [0, 1)**, else `"<ctx>: differential ramp.drive_factor must be in [0, 1)"` / `"... ramp.coast_factor must be in [0, 1)"` | I4 C7: ramp-angle locking factors `f` — `T_high - T_low <= preload_nm + f * T_axle`, `f = drive_factor` while the carrier's input power was positive last substep (drive), else `coast_factor` (coast/overrun) |
| `static_factor` | real | optional, **only valid for `kind: "clutch_lsd"`** | `1.0` | present on another kind throws `"<ctx>: differential 'static_factor' is only valid for kind \"clutch_lsd\""`; **must be >= 1**, else `"<ctx>: differential 'static_factor' must be >= 1"` | I4 C7: stuck (static) capacity = `static_factor` × kinetic capacity |
| `torque_bias_ratio` | real | **required for `kind: "torsen"`**, invalid on every other kind | — | present on another kind throws `"<ctx>: differential 'torque_bias_ratio' is only valid for kind \"torsen\""`; missing on `torsen` throws `"<ctx>: differential kind \"torsen\" requires 'torque_bias_ratio' > 1"`; **must be > 1**, else `"<ctx>: differential 'torque_bias_ratio' must be > 1"` | I4 C7: TBR — `T_high / T_low <= torque_bias_ratio` |
| `lockable` | bool | optional | `false` | **must not be combined with `kind: "locked"`**, else `"<ctx>: differential 'lockable' is redundant on kind \"locked\" (already always locked)"` | I4 C5: a *runtime-togglable* lock — adds one composed-state bit and a `{L:+1, R:-1}` lock row only in the states where the bit is set (unlike `kind: "locked"`'s unconditional row); must be a leaf diff (its `left`/`right` carry no other rigid row). Allowed on `open`, `viscous`, `clutch_lsd` and `torsen` (I4 C7: a lockable `clutch_lsd`/`torsen` diff's friction row gets capacity 0 while the lock is engaged) |
| `lock` | object (`DiffLockDesc`) | optional, **only valid with `lockable: true`** | `{max_engage_mismatch_rad_s: 2.0, channel: "diff_lock"}` | present without `lockable: true` throws `"<ctx>: differential 'lock' requires \"lockable\": true"`; `max_engage_mismatch_rad_s` **must be >= 0**, else `"<ctx>: differential lock.max_engage_mismatch_rad_s must be >= 0"`; `channel` **must be non-empty**, else `"<ctx>: differential lock.channel must be non-empty"` | I4 C5: `max_engage_mismatch_rad_s` — the lock only engages while `|omega_left - omega_right|` is at or below this (else stays pending, retried every substep); `channel` — the declared control channel (default value 0) whose value `>= 0.5` requests the lock; releases the substep it reads `< 0.5` |

Ports: `<id>.carrier`, `<id>.left`, `<id>.right` (`left`/`right` carry zero
inertia of their own — they exist purely so a shaft can name them).

**Reconstruction/leaf rule (I4 C4/C5/C6/C7, plan section 2.7):** a diff of any
kind whose `efficiency < 1` (a lossy mesh) must be a **leaf** — its `left`
and `right` ports may carry no other rigid row (discounting the diff's own
`left`/`right` rows, e.g. a `locked`/`lockable` diff's own lock row), or the
graph builder throws `"vehicle '<v>': differential '<id>' of kind <k> must
drive nodes with no other rigid coupling (centre LSD/Torsen is not
supported in I4)"` (a `lockable: true` diff gets the same rejection with
`"of kind lockable <k>"`, e.g. `"of kind lockable open"`, instead of the
plain kind name). A `clutch_lsd`/`torsen` diff (I4 C7) must be a leaf
**whatever its efficiency** — same message. A **lossless**
(`efficiency == 1`) diff of kind `open`/`locked`/`viscous` may instead be a
**centre** diff — `left`/`right` feeding two child diffs' `carrier` ports —
reconstructed from both children's own row lambdas. Plain `open` diffs are
unaffected by any of this (same leaf/centre rules and float ops as before I4).

#### clutch_lsd / torsen (I4 C7)

Both add one Coulomb friction row between `left` and `right` (reaction
`+lambda` on `left`, `-lambda` on `right`, so `T_high - T_low = 2|lambda|`)
alongside the diff's own open row, in every topology state. Its capacity
(half the allowed torque difference) is set every substep from `T_axle =
|ratio * lambda_diff|`, the diff row's reconstructed multiplier:

- `clutch_lsd`: kinetic capacity `(preload_nm + f * T_axle) / 2`, `f =
  drive_factor` if `T_hub * omega_hub` summed over both hubs (the diff row's
  own share of each hub torque, equal to the carrier's input power) was
  `> 0` last substep, else `coast_factor`; static capacity `static_factor *
  kinetic`.
- `torsen`: capacity `(T_axle / 2) * (TBR - 1) / (TBR + 1)`, static ==
  kinetic, so `T_high / T_low <= torque_bias_ratio`.

**One-substep lag:** `T_axle` and the drive/coast sign come from the
**previous** substep's solve (1.04 ms at the default 960 Hz substep rate),
so the capacity is a known number when the substep's friction LCP runs —
no fixed-point iteration. The slip dissipation `-h * v * lambda` is booked
as `PowertrainEnergy::diff_loss` (same channel as `viscous`).

#### `transfer_case` component fields (I4 C9)

`TransferCaseDesc`. Always inline; at most one per vehicle. Models a
mechanical-front-wheel-drive (MFWD) transfer case: a **rigid** kinematic
coupling from `in` to `rear` (present in every topology state, exactly like
a `gear_pair`'s own rigid row), plus a **friction** (Coulomb, box-LCP) row
between `in` and `front` — the AWD clutch pack a driver/controller engages
to add front-axle drive. Unlike a `differential`, `front` is not
constrained to follow `in` at all when the clutch capacity is 0 (fully
disengaged) — it simply free-wheels at whatever speed its own downstream
inertia carries it to.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `rear_ratio` | real | optional | `1.0` | **must be != 0**, else `"<ctx>: transfer_case 'rear_ratio' must be != 0"` | the rigid row: `omega_in = rear_ratio * omega_rear` |
| `front_ratio` | real | optional | `1.0` | **must be != 0**, else `"<ctx>: transfer_case 'front_ratio' must be != 0"` | the friction row's `ratio_b`: slip velocity `v = omega_in - front_ratio * omega_front` |
| `clutch` | object (`TransferCaseClutchDesc`) | **required** | — | must be present, else `"<ctx>: transfer_case requires a 'clutch' block {\"capacity_nm\", \"static_factor\", \"engage_time_s\"}"` | the AWD clutch pack's own parameters, see below |
| `clutch.capacity_nm` | real (N·m) | optional | `0` | **must be > 0**, else `"<ctx>: transfer_case clutch.capacity_nm must be > 0"` | kinetic torque capacity at engagement `e = 1` |
| `clutch.static_factor` | real | optional | `1.15` | **must be >= 1**, else `"<ctx>: transfer_case clutch.static_factor must be >= 1"` | static capacity = `static_factor * kinetic` (same mechanism as `clutch.static_factor` / `Assembly::set_friction_capacity`) |
| `clutch.engage_time_s` | real (s) | optional | `0.3` | **must be >= 0**, else `"<ctx>: transfer_case clutch.engage_time_s must be >= 0"` | engagement `e` ramps toward the channel's target (1 if channel value `>= 0.5`, else 0) at rate `1 / engage_time_s`; `0` = instant |
| `channel` | string | optional | `"awd"` | **must be non-empty**, else `"<ctx>: transfer_case 'channel' must be non-empty"` | declared control channel (default value `0`, via `Powertrain::declare_control_channels`) whose value `>= 0.5` requests engagement |
| `in_inertia_kgm2` / `front_inertia_kgm2` | real (kg·m²) | optional | `0` | none | inertia at `in` / `front` (`rear` always carries zero, a pure kinematic passthrough port) |
| `axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, same rule/message as clutch `axis_local` | rotor-reaction axis shared by `in` and `front` (one physical shaft) |

Ports: `<id>.in`, `<id>.rear`, `<id>.front`.

Telemetry/debug: `PowertrainSnapshot::awd_engagement` (the clutch's current
`e`, 0 for a vehicle with no transfer case) and `PowertrainDebug::
awd_clutch_torque`/`awd_clutch_slip` (the friction row's own lambda/
velocity, tests only). Dissipation is booked as `PowertrainEnergy::
awd_clutch_loss` (`-h * v * lambda` per substep, same form as `clutch_loss`/
`diff_loss`), included in `end_tick()`'s `balance_error`. `e` is the
transfer case's only persistent per-substep state and is folded into
`Powertrain::fold_state_hash` — a vehicle with no `transfer_case` component
hashes bit-identically to before this component existed.

**Speedo interaction:** the gearbox-output speedo walk (see
`powertrain.cpp`'s `speedo_ratio_`/`speedo_radius_`) follows a transfer
case's `in -> rear` edge only, never `in -> front` — adding a front axle
behind a transfer case cannot change a vehicle's speedo reading.

#### `park_lock` component fields (I4 C10)

`ParkLockDesc`. Always inline; at most one per vehicle (rejected at load
time, see above). Models a park pawl: a single Coulomb friction row from
its own `shaft` port to GROUND (no rigid row, no topology-state bit — it
never changes network topology, only a friction row's capacity, exactly
like a `DirectTorqueBrakes` corner row). The port is wired to wherever the
pawl physically clamps (e.g. `gearbox.out`) via an ordinary shaft entry
alongside that port's other connections (a port may belong to only one
shaft — extend the existing shaft's `ports` array rather than adding a new
one).

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `capacity_nm` | real (N·m) | optional | `0` | **must be > 0**, else `"<ctx>: park_lock 'capacity_nm' must be > 0"` | kinetic friction-row capacity while engaged |
| `static_factor` | real | optional | `1.0` | **must be >= 1**, else `"<ctx>: park_lock 'static_factor' must be >= 1"` | static capacity = `static_factor * capacity_nm`; `1.0` (unlike a clutch's `1.15`) because a pawl tooth has no separate static/kinetic friction regime — it holds or it shears |
| `max_engage_speed_rad_s` | real (rad/s) | optional | `5.0` | **must be >= 0**, else `"<ctx>: park_lock 'max_engage_speed_rad_s' must be >= 0"` | the pawl can only engage while `|omega|` at its `shaft` node (read **before** the current substep's solve — the value the previous substep's solve left behind) is at or below this; above it the request stays pending, retried every substep |
| `channel` | string | optional | `"park_lock"` | **must be non-empty**, else `"<ctx>: park_lock 'channel' must be non-empty"` | declared control channel (default value `0`, via `Powertrain::declare_control_channels`) whose value `>= 0.5` requests engagement; releases (capacity 0, no ramp) the first substep it reads `< 0.5` |

Port: `<id>.shaft`.

Engagement is a simple two-state latch, unlike the transfer case's ramped
`e`: released capacity is always exactly `0`; engaged capacity is always
exactly `capacity_nm`/`static_factor * capacity_nm` (no ramp — "a pawl
either holds or it shears"). Slip dissipation (`-h * v * lambda`, same form
as every other friction row) is booked into `PowertrainEnergy::brake_loss`
(a park pawl is a brake, and I4 C10 adds no new ledger channel). Telemetry:
`PowertrainSnapshot::park_lock_engaged` (1 = holding, 0 = released or no
`park_lock` at all) and `PowertrainDebug::park_lock_torque` (the row's own
lambda, tests only). Only the engaged/released latch is hashed (the pending
state, like a diff lock's `lock_pending_`, is recomputed every substep from
the channel plus the network's own already-hashed omega) — a vehicle with
no `park_lock` hashes bit-identically to before this component existed.

**Only holds through an open differential when nothing else does:** a
`park_lock` clamps one shaft-network node, not the wheel hubs directly — an
`open` differential downstream still lets its two output hubs counter-rotate
while the clamped node's own average stays at rest, so on a *cross* slope
(not a straight climb) a park-lock-only hold can let the vehicle creep via
wheel slip even though the network's own kinematics are satisfied. See
`tests/validation/test_vehicle_tractor_cross_slope.cpp`'s park-lock cases
for the measured numbers this produces on `data/vehicles/tractor.json`.

#### `pto` component fields (I4 C11)

`PtoDesc`. Always inline; at most one per vehicle (rejected at load time,
same mechanism as `park_lock` above). Models a power take-off: a single
Coulomb friction row from the engine/crank-side `in` port to the PTO-side
`out` port, with a fixed **ratio** applied at that row (`out` spins at
`in / ratio` once the row is stuck, per the vehicle's chosen 540/1000 rpm
mode) — the tractor's real PTO shaft is usually tapped straight off the
crank ahead of the clutch/gearbox entirely (see `data/vehicles/tractor.json`'s
own `shafts[]`), so `in` is typically unioned onto the same node as
`engine.crank`, making the PTO branch's dynamics independent of
`gear_request`/`park_lock`.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `ratios` | object, keys `"540"`/`"1000"` | **required, non-empty** | — | must be a non-empty object, else `"<ctx>: pto requires a non-empty 'ratios' object with key(s) \"540\" and/or \"1000\""`; an unrecognised key throws `"<ctx>: pto ratios key '<key>' is not \"540\" or \"1000\""`; each value **must be > 0**, else `"<ctx>: pto ratios['<key>'] must be > 0"` | `omega_in = ratio * omega_out` while the row is stuck; at least one of `"540"`/`"1000"` must be given, both may be |
| `capacity_nm` | real (N·m) | optional | `0` | **must be > 0**, else `"<ctx>: pto 'capacity_nm' must be > 0"` | kinetic friction-row capacity while engaged |
| `static_factor` | real | optional | `1.2` | **must be >= 1**, else `"<ctx>: pto 'static_factor' must be >= 1"` | static capacity = `static_factor * capacity_nm` |
| `engage_time_s` | real (s) | optional | `1.5` | **must be >= 0**, else `"<ctx>: pto 'engage_time_s' must be >= 0"` | engagement ramps toward the channel's target (1 if channel value `>= 0.5`, else 0) at rate `1 / engage_time_s`; `0` = instant |
| `efficiency` | real | optional | `1.0` | must be in `(0, 1]`, else `"<ctx>: pto 'efficiency' must be in (0, 1]"` | mesh efficiency; `< 1` books an efficiency-loss torque at `in` (same lagged-reconstruction pattern as `gear_pair`/`differential`) |
| `out_inertia_kgm2` | real (kg·m²) | optional | `0` | **must be >= 0**, else `"<ctx>: pto 'out_inertia_kgm2' must be >= 0"` | inertia at `out` (the PTO shaft/stub itself, before whatever implement is attached) |
| `disengaged_brake_time_s` | real (s) | optional | `1.0` | **must be > 0**, else `"<ctx>: pto 'disengaged_brake_time_s' must be > 0"` | while fully disengaged (`e == 0` exactly), the `out` node is given an implicit stub-brake damping term sized so its own free-spin-down time constant is this value — a disengaged PTO shaft simply drifts to rest instead of coasting forever |
| `channel` | string | optional | `"pto"` | **must be non-empty**, else `"<ctx>: pto 'channel' must be non-empty"` | declared control channel (default value `0`) whose value `>= 0.5` requests engagement |
| `speed_channel` | string | optional | `"pto_speed"` | **must be non-empty**, else `"<ctx>: pto 'speed_channel' must be non-empty"` | declared control channel (default value `540.0`) selecting the mode: `>= 750` selects `"1000"`, else `"540"` — read only while eligible to switch (see below) |
| `axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, same rule/message as clutch `axis_local` | rotor-reaction axis shared by `in` and `out` (one physical shaft) |

Ports: `<id>.in`, `<id>.out`. Unlike every other component's ports, `<id>.out`
is **exempt** from the "every port must be connected by exactly one shaft"
rule below (`in` is not) — a PTO stub is routinely left unconnected in a
vehicle file, with a test/bench `load_dyno` or a future implement model
wired onto it only in a test or a specific scenario, not in the shipped
vehicle data itself.

**Mode switching:** a `speed_channel` change is only honoured while the PTO
is disengaged (`engagement == 0` exactly) **and** `|omega_out| < 1 rad/s` —
requesting a mode change at any other time is simply ignored (the PTO stays
in its current mode) until both conditions are true, so a mode never changes
out from under a spinning implement.

Telemetry/debug: `PowertrainSnapshot::{pto_engagement, pto_mode, pto_rpm}`
(`pto_mode` is `540.0`/`1000.0` as a `real`; `pto_rpm` is always
non-negative — magnitude only) and `PowertrainDebug::{pto_clutch_torque,
pto_clutch_slip}` (tests only). Dissipation (both the friction row's own
slip and the disengaged stub-brake's) is booked as
`PowertrainEnergy::pto_clutch_loss`, included in `end_tick()`'s
`balance_error`. `engagement_`, `mode_1000_` and (only when `efficiency < 1`)
the previous substep's row lambda are the PTO's persistent per-substep
state and are folded into `Powertrain::fold_state_hash` — a vehicle with no
`pto` component hashes bit-identically to before this component existed.

#### `load_dyno` component fields (I4 C11)

`LoadDynoDesc`. Always inline; **any number** per vehicle is allowed (no
"at most one" rule — see above). A test/bench-only load on a single `shaft`
port: **no shipped vehicle carries one.** `tests/unit/test_powertrain_pto.cpp`
and `tests/validation/test_powertrain_pto.cpp` both add one to a
`VehicleDesc` mutated in code (never in a `data/vehicles/*.json` file) to
give the `pto` component above something to drive.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `mode` | string | optional | `"brake"` | must be `"brake"`, `"viscous"` or `"quadratic"`, else `"<ctx>: load_dyno 'mode' must be \"brake\", \"viscous\" or \"quadratic\""` | load law — `brake` (a fixed Coulomb friction row to GROUND), `viscous` (`tau = -viscous_nm_s_rad * omega`), `quadratic` (`tau = -quadratic_nm_s2_rad2 * omega * \|omega\|`) |
| `torque_nm` | real (N·m) | **required for `mode: "brake"`**, invalid on every other mode | `0` | present on another mode throws `"<ctx>: load_dyno 'torque_nm' is only valid for mode \"brake\""`; on `"brake"` **must be > 0**, else `"<ctx>: load_dyno 'torque_nm' must be > 0 for mode \"brake\""` | the friction row's fixed capacity (both static and kinetic) |
| `viscous_nm_s_rad` | real (N·m·s/rad) | **required for `mode: "viscous"`**, invalid on every other mode | `0` | present on another mode throws `"<ctx>: load_dyno 'viscous_nm_s_rad' is only valid for mode \"viscous\""`; on `"viscous"` **must be > 0**, else `"<ctx>: load_dyno 'viscous_nm_s_rad' must be > 0 for mode \"viscous\""` | the viscous coefficient `c` above |
| `quadratic_nm_s2_rad2` | real (N·m·s²/rad²) | **required for `mode: "quadratic"`**, invalid on every other mode | `0` | present on another mode throws `"<ctx>: load_dyno 'quadratic_nm_s2_rad2' is only valid for mode \"quadratic\""`; on `"quadratic"` **must be > 0**, else `"<ctx>: load_dyno 'quadratic_nm_s2_rad2' must be > 0 for mode \"quadratic\""` | the quadratic (aerodynamic-like) coefficient above |
| `inertia_kgm2` | real (kg·m²) | optional | `0` | **must be >= 0**, else `"<ctx>: load_dyno 'inertia_kgm2' must be >= 0"` | inertia at `shaft` |
| `scale_channel` | string | optional | `""` (empty) | none | **optional** declared control channel (default value `1.0`) multiplying the load; empty (the default) means a fixed scale of `1.0` and declares no channel at all — unlike every `channel`/`speed_channel` field elsewhere in this document, an empty `scale_channel` is valid, not rejected |
| `axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, same rule/message as clutch `axis_local` | rotor-reaction axis at `shaft` |

Port: `<id>.shaft` — **not** exempt from the "every port must be connected by
exactly one shaft" rule (unlike `pto.out` above): a `load_dyno` with nothing
wired to it would be pointless, so it is held to the ordinary rule.

Telemetry/debug: `PowertrainDebug::dyno_torque` (tests only, the row's/law's
own current torque). Mechanical work this component absorbs is booked as
`PowertrainEnergy::external_absorbed` (cumulative since the vehicle was
created, always `>= 0` — brake: `-h * v * lambda`; viscous/quadratic:
`-h * tau_applied * omega'`), included in `end_tick()`'s `balance_error`.
Stateless otherwise — no `fold_state_hash` override, so adding a `load_dyno`
to a vehicle changes nothing else that is hashed beyond the network's own
(already-hashed) node omegas.

### Ports, by component type (fixed names)

| Component | Ports |
|---|---|
| `engine` | `crank` |
| `clutch` | `in`, `out` |
| `gearbox` | `in`, `out` |
| `gear_pair` | `in`, `out` |
| `differential` | `carrier`, `left`, `right` |
| `transfer_case` | `in`, `rear`, `front` |
| `park_lock` | `shaft` |
| `pto` | `in`, `out` (`out` exempt from the "must be connected" rule, see above) |
| `load_dyno` | `shaft` |
| a wheel (implicit, one per entry in the top-level `wheels[]`) | `wheel:<wheel name>` |

A port reference string is written `"<component id>.<port>"` (e.g.
`"engine.crank"`, `"diff.left"`) or `"wheel:<wheel name>"` (e.g.
`"wheel:FL"`). A malformed reference (no `.`, or a `.` at position 0 or at
the very end) throws `"powertrain.shafts[]: bad port ref \"<ref>\" (expected
\"<id>.<port>\" or \"wheel:<name>\")"`; an empty wheel name after `"wheel:"`
throws the same message.

### Shafts

`"shafts"` is an array of `{ "ports": ["<ref>", ...], "stiffness": 0,
"damping": 0 }`. Each shaft rigidly joins every port it lists into one node.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `ports` | array of port-ref strings | **required** | — | must be present (`"powertrain.shafts[]: 'ports' is required"`); each ref must resolve to a real port, else `"powertrain shaft references unknown port '<key>'"`; a port referenced by more than one shaft, or twice within the same shaft, throws `"powertrain port '<key>' is connected by more than one shaft, or twice in one shaft"` | which ports are rigidly joined |
| `stiffness` | real (N·m/rad) | optional | `0` | **must be exactly `0`** in I3, else `"powertrain.shafts[]: 'stiffness' must be 0 in I3 (I4 adds compliant shafts)"` | shaft torsional stiffness; compliant shafts are not yet implemented |
| `damping` | real (N·m·s/rad) | optional | `0` | none (unused while `stiffness == 0`) | shaft damping |

**Every non-wheel port must be connected by exactly one shaft.** A port
that belongs to no shaft at all throws `"powertrain port '<key>' is not
connected by any shaft"` — this applies to `engine.crank`, every
`clutch`/`gearbox`/`gear_pair`/`differential`/`transfer_case`/`park_lock`/
`load_dyno` port and `pto.in`, but **not** to a wheel port (`wheel:<name>`
may be left unconnected — that wheel is simply free-rolling/undriven) nor
to `pto.out` (I4 C11 — see the [`pto` component fields](#pto-component-fields-i4-c11)
section above).

Top-level powertrain fields beyond `components`/`shafts`:

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `compliance` | string | optional | `"rigid"` | must be `"rigid"` in I3, else `"powertrain.compliance must be \"rigid\" in I3 (I4 adds \"compliant\"), got \"<value>\""` | shaft-network compliance model |
| `rotor_reaction` | bool | optional | `true` | none | whether spinning components apply a reaction torque back onto the chassis about their own axis; `false` only for a temporary data-migration step |

### What the graph builder additionally rejects

Beyond the per-field checks above, `Powertrain::Powertrain`
(`core/src/drivetrain/powertrain.cpp`) walks the whole graph once at
vehicle-construction time and additionally rejects:

- A wheel with `wheel_inertia <= 0` (see the [wheel table](#wheel) above).
- More than one engine/clutch/gearbox/transfer_case/park_lock/pto component
  (duplicated check from the field-level notes above — `park_lock` and `pto`
  are additionally rejected earlier, at load time; any number of
  `differential`/`gear_pair`/`load_dyno` components is fine, see above).
- A lossy (`efficiency < 1`) or `lockable: true` differential that is not a
  leaf (its `left`/`right` carry another rigid row) — the exact rejection
  message is under the [differential field table](#differential-component-fields)
  above.
- More `lockable: true` differentials than the composed-topology-state
  budget allows (I4 C5: `kMaxStates` = 128 — `num_gear_states * 2^(number
  of lockable diffs)` must not exceed it).
- A `brake_system` corner or handbrake wheel name that doesn't match any
  wheel, or a duplicate corner for the same wheel.
- A controller (see [below](#controllers)) referencing a component id that
  doesn't exist, or that exists but is the wrong component type.

## Controllers

`"controllers"` is an array of `{id, type, ...}`; at most one entry, and I3
supports exactly one `type`: `"manual_tcu"` (`ManualTcuDesc`). An unknown
type throws `"controllers['<id>']: unknown type \"<type>\""`.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `id` | string | required | — | non-empty (`"controllers[]: 'id' is required and must be a non-empty string"`) | controller id |
| `type` | string | required | — | must be `"manual_tcu"` | controller kind |
| `gearbox`, `clutch`, `engine` | string (component id) | **all required** | — | all three must be present (`"'gearbox', 'clutch' and 'engine' (component id references) are all required"`); each must reference an existing `powertrain.components[]` id (`"controllers['<id>']: references unknown component '<ref>'"`, re-checked with the component-type match in `powertrain.cpp`: `"controller '<id>' component '<ref>' is not a <kind>"`) | which powertrain components this TCU drives |
| `auto_clutch` | object | optional | see [`AutoClutchDesc`](#auto_clutch) | see below | automatic clutch modulation |
| `auto_blip` | object | optional | see [`AutoBlipDesc`](#auto_blip) | none | throttle blip on downshift |
| `auto_shift` | object | optional | see [`AutoShiftDesc`](#auto_shift) | none | automatic up/down shifting |
| `max_pending_steps` | int | optional | `2` | none | queued shift-request depth |

#### `auto_clutch`

| Field | Type | Default | Validation | Meaning |
|---|---|---|---|---|
| `enabled` | bool | `true` | — | default of the runtime `assist.auto_clutch` channel (the channel is the on/off switch, see below) |
| `bite_rpm` | real (rpm) | `1100` | must satisfy `bite_rpm < full_rpm_min` (see below) | engagement 0 at/below this engine speed (launch law) |
| `full_rpm_min` | real (rpm) | `1600` | `bite_rpm < full_rpm_min <= full_rpm_max`, else `"<ctx>.auto_clutch: must satisfy bite_rpm < full_rpm_min <= full_rpm_max"` | engagement 1 at this rpm with throttle 0 |
| `full_rpm_max` | real (rpm) | `3000` | see above | engagement 1 at this rpm with throttle 1 |
| `lock_slip_rpm` | real (rpm) | `80` | none | `|engine - input|` below this (and rpm > bite) locks the clutch |
| `anti_stall_rpm` | real (rpm) | `950` | **must be `< bite_rpm`**, else `"<ctx>.auto_clutch: anti_stall_rpm must be < bite_rpm"` | below this and locked, falls back to the launch law |
| `engage_rate_per_s` / `release_rate_per_s` | real | `4.0` / `12.0` | none | ordinary engagement ramp rates |
| `anti_stall_release_rate_per_s` | real | `40.0` | none | faster release rate used only while releasing below `anti_stall_rpm` |
| `rolling_reengage_rpm` | real (rpm) | `1300` | **must be `> anti_stall_rpm`**, else `"<ctx>.auto_clutch: rolling_reengage_rpm must be > anti_stall_rpm"` | after an anti-stall release, the clutch closes again on its own once the gearbox input shaft turns faster than this while in gear (engine braking resumes while coasting) |
| `open_time_s` / `close_time_s` | real (s) | `0.08` / `0.20` | none | shift-mode open ramp duration / the rate the post-shift close (and a rolling re-engagement) follows its target at; with the torque-following close below that target is a torque, not engagement 1 |
| `sync_time_s` | real (s) | `0.30` | **must be `> 0`**, else `"<ctx>.auto_clutch: sync_time_s must be > 0"` | torque-following close after a shift and on a rolling re-engagement: the clutch capacity is set to `max(0, sign(slip)*T_engine + min(J_engine*|slip|/sync_time_s, max_sync_torque_nm))` (engine demand torque, rev limiter ignored), so the engine speed converges on the input shaft with this time constant and the wheels never get more than engine torque plus `max_sync_torque_nm` through the slipping clutch; the clutch locks once `|slip| < lock_slip_rpm` |
| `max_sync_torque_nm` | real (N*m) | `50` | **must be `> 0`**, else `"<ctx>.auto_clutch: max_sync_torque_nm must be > 0"` | cap on the synchronising term above |
| `anti_stall_release_rate_per_s`, `launch_speed_threshold_mps` | real | `40.0`, `1.0` | none | see `ps/drivetrain/controller_desc.h`'s `AutoClutchDesc` comment for the launch-vs-shift distinction this threshold implements |

#### `auto_blip`

| Field | Type | Default | Meaning |
|---|---|---|---|
| `enabled` | bool | `true` | default of the runtime `assist.auto_blip` channel |
| `kp_per_rpm` | real | `0.002` | throttle-blip gain |
| `max_throttle` | real | `1.0` | blip ceiling |
| `upshift_cut` | bool | `true` | cut ignition on an upshift: through the whole gearbox shift and, with the auto-clutch on, through the post-shift clutch close until the engine is speed-matched (below) |
| `upshift_cut_release_slip_rpm` | real (rpm) | `300` | with the auto-clutch on, the cut is held after the dog is in until engine rpm minus the new gear's input-shaft rpm drops to this; the car coasts while it holds. **Must be `> 0`**, else `"<ctx>.auto_blip: upshift_cut_release_slip_rpm must be > 0"` |
| `upshift_cut_max_hold_s` | real (s) | `0.6` | the cut is released at the latest this long after the dog is in (a close that never synchronises cannot hold it forever). **Must be `> 0`**, else `"<ctx>.auto_blip: upshift_cut_max_hold_s must be > 0"` |
| `upshift_torque_ramp_s` | real (s) | `0.10` | after an upshift cut ends, the driver's pedal comes back through a ceiling ramped 0 -> 1 over this time (the blip floor is not capped); `0` = step. **Must be `>= 0`**, else `"<ctx>.auto_blip: upshift_torque_ramp_s must be >= 0"` |

#### `auto_shift`

| Field | Type | Default | Meaning |
|---|---|---|---|
| `enabled` | bool | `false` | default of the runtime `assist.auto_shift` channel |
| `up_rpm_at_zero_throttle` / `up_rpm_at_full_throttle` | real (rpm) | `2500` / `6300` | upshift rpm thresholds, interpolated by throttle |
| `down_rpm_at_zero_throttle` / `down_rpm_at_full_throttle` | real (rpm) | `1300` / `2800` | downshift rpm thresholds |
| `min_time_in_gear_s` | real (s) | `1.0` | shift-hysteresis timer |

Apart from the three `upshift_*` fields above, none of the numeric
sub-fields inside `auto_blip`/`auto_shift` are range-validated by the
loader beyond the defaults shown.

## Control channels a vehicle reads

Not part of the JSON format, but relevant context for anyone editing a
vehicle file and then driving it from a scenario's `control_events` (see
`scenario.md`): `World::create_vehicle` resolves these channel names once,
at vehicle-creation time (`core/src/world/world.cpp`) — `"steer"`,
`"throttle"`, `"brake"`, `"handbrake"`, `"clutch"`, `"shift_up_count"`,
`"shift_down_count"`, `"gear_request"`, `"ignition"`, `"starter"`,
`"assist.auto_clutch"`, `"assist.auto_blip"`, `"assist.auto_shift"`. The
first vehicle created in a `World` reads the bare names; every subsequent
vehicle reads a `"v<index>."`-prefixed version (e.g. `"v1.steer"`).
The three `assist.*` channels (> 0.5 means on) are the runtime on/off
switches for the `manual_tcu` driver aids; each one's value when the
vehicle is created is its controller block's `enabled` flag (0 if the
vehicle has no `manual_tcu`).

## Telemetry channels a differential exposes

PHYS-033 P4 I4 C8 (plan section 3.1): every differential component whose
`kind` is not plain `"open"` — or that has `lockable: true` (a lockable
`"open"` diff still needs its own "am I currently locked" telemetry even
though its `kind` reads `open`) — registers two extra CSV/telemetry
channels, in `powertrain.components[]` array order:

| Channel | Meaning |
|---|---|
| `vehicle<i>.<diff id>.bias` | signed `(T_left - T_right) / (\|T_left\| + \|T_right\|)`, in `[-1, 1]`; `0` at zero axle torque (never NaN) |
| `vehicle<i>.<diff id>.locked` | `1` while a rigid lock is currently active (`kind: "locked"`, or a `lockable` diff with its lock bit set this substep), else `0` |

`<i>` is the vehicle index (see ["Control channels a vehicle reads"](#control-channels-a-vehicle-reads)
above for the same first-vehicle-is-bare-name / later-vehicles-`v<i>.`-prefixed
convention — the diff channels instead always carry the `vehicle<i>.` prefix,
matching the existing `vehicle<i>.energy.*`/`vehicle<i>.powertrain.*`
channels' own naming). A vehicle whose only differential(s) are plain
non-lockable `open` (both shipped `car_sedan.json`/`tractor.json` diffs)
registers none of these — its CSV header is byte-identical to before this
existed. Up to 4 differentials per vehicle are supported
(`ps::drivetrain::kMaxPowertrainDiffs`, `ps/drivetrain/powertrain_state.h`);
this is a snapshot/debug-only addition — no new hashed simulation state, no
change to `World::state_hash()`.

## Minimal complete example

A single driven axle, no clutch/gearbox (the engine drives the differential
directly through one `gear_pair`), no controller:

```json
{
  "format": "physics_sim.vehicle/2",
  "name": "minimal_example",
  "source": "Documentation example only, not a real vehicle - physics_sim.vehicle/2.",
  "wheels": [
    { "name": "FL", "attachment_local": [1.2, 0.75, -0.3], "wheel_radius": 0.3, "wheel_inertia": 0.8,
      "suspension": { "axis": [0, 0, -1], "travel_max": 0.12, "spring_rate": 40000, "damper_bump_rate": 3000, "damper_rebound_rate": 4000 },
      "tyre": "../tyres/passenger_car_225_45r17.json", "steered": true, "is_front": true },
    { "name": "FR", "attachment_local": [1.2, -0.75, -0.3], "wheel_radius": 0.3, "wheel_inertia": 0.8,
      "suspension": { "axis": [0, 0, -1], "travel_max": 0.12, "spring_rate": 40000, "damper_bump_rate": 3000, "damper_rebound_rate": 4000 },
      "tyre": "../tyres/passenger_car_225_45r17.json", "steered": true, "is_front": true },
    { "name": "RL", "attachment_local": [-1.2, 0.75, -0.3], "wheel_radius": 0.3, "wheel_inertia": 0.8,
      "suspension": { "axis": [0, 0, -1], "travel_max": 0.12, "spring_rate": 40000, "damper_bump_rate": 3000, "damper_rebound_rate": 4000 },
      "tyre": "../tyres/passenger_car_225_45r17.json", "is_front": false },
    { "name": "RR", "attachment_local": [-1.2, -0.75, -0.3], "wheel_radius": 0.3, "wheel_inertia": 0.8,
      "suspension": { "axis": [0, 0, -1], "travel_max": 0.12, "spring_rate": 40000, "damper_bump_rate": 3000, "damper_rebound_rate": 4000 },
      "tyre": "../tyres/passenger_car_225_45r17.json", "is_front": false }
  ],
  "steering": { "max_wheel_angle": 0.5, "ackermann_fraction": 1.0 },
  "brake_system": {
    "type": "direct_torque",
    "corners": [ { "wheel": "FL", "max_torque_nm": 1500 }, { "wheel": "FR", "max_torque_nm": 1500 },
                 { "wheel": "RL", "max_torque_nm": 900 }, { "wheel": "RR", "max_torque_nm": 900 } ],
    "handbrake": { "wheels": ["RL", "RR"], "max_torque_nm": 900 }
  },
  "aero": { "drag_area_m2": 0.6 },
  "powertrain": {
    "components": [
      { "id": "engine", "type": "engine", "ref": "../engines/i4_2l_na.json" },
      { "id": "final_drive", "type": "gear_pair", "ratio": 4.0, "efficiency": 0.97 }
    ],
    "shafts": [
      { "ports": ["engine.crank", "final_drive.in"] },
      { "ports": ["final_drive.out", "wheel:RL", "wheel:RR"] }
    ]
  }
}
```

(This directly locks both rear wheels together, no differential — a
simplification fine for documentation, not for a real drivable vehicle.)

## Common mistakes

- **Forgetting `source`.** Throws `"vehicle '<name>': 'source' is required
  (PLAN.md section 8)"`.
- **Empty or missing `wheels`.** Throws `"vehicle <path> has no 'wheels'
  array"`.
- **A leftover `driven` key on a wheel**, or top-level `drive`/`brakes`
  blocks from the old `/1` format — all three are hard errors under `/2`
  (see the tables above).
- **A powertrain port left unconnected.** Every non-wheel port (engine,
  clutch, gearbox, gear_pair, differential ports) must appear in exactly one
  shaft. Forgetting to wire `engine.crank` into a shaft throws `"powertrain
  port 'engine.crank' is not connected by any shaft"`.
- **A `"wheel:<name>"` port ref with a typo'd or non-existent wheel name.**
  Throws `"powertrain shaft references unknown port 'wheel:<name>'"` — the
  name must exactly match a `wheels[].name`.
- **A `gear_pair`/`differential` with `efficiency < 1` sharing its input
  node with something else.** Throws `"gear_pair '<id>' has efficiency < 1
  but its input node carries another rigid row..."` — give the lossy gear
  its own dedicated input node (a shaft with only that port and its
  upstream neighbour).
- **`ackermann_fraction` outside `[-1, 1]`** (e.g. accidentally in degrees,
  or a stray extra digit) — rejected with the exact valid range in the
  message.
- **A controller naming a component of the wrong kind**, e.g. `"clutch":
  "gearbox"` — throws `"controller '<id>' component 'gearbox' is not a
  clutch"`.
