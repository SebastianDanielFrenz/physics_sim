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
{ "id": "<unique id>", "type": "engine" | "clutch" | "gearbox" | "gear_pair" | "differential", ... }
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
  merging of inline overrides on top of a ref'd file. `clutch`, `gear_pair`
  and `differential` components are **always inline** — `{"ref": ...}` on a
  clutch throws `"<ctx>: 'clutch' components are always inline in I3 (no
  'ref')"`.
- **At most one** `engine`, one `clutch`, one `gearbox` component is allowed
  (checked in `powertrain.cpp`): a second one throws `"more than one <kind>
  component - I3 supports at most one"`. Any number of `gear_pair` **or**
  `differential` components is fine (I4 C4 lifted the earlier "at most one
  differential" limit) - a centre diff feeding two axle diffs, or any other
  multi-diff topology, is built by wiring their `carrier`/`left`/`right`
  ports together with ordinary shafts (see [Ports, by component
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

### Ports, by component type (fixed names)

| Component | Ports |
|---|---|
| `engine` | `crank` |
| `clutch` | `in`, `out` |
| `gearbox` | `in`, `out` |
| `gear_pair` | `in`, `out` |
| `differential` | `carrier`, `left`, `right` |
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
`clutch`/`gearbox`/`gear_pair`/`differential` port, but **not** to a wheel
port (`wheel:<name>` may be left unconnected — that wheel is simply
free-rolling/undriven).

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
- More than one engine/clutch/gearbox component (duplicated check from the
  field-level notes above — this is where it's actually enforced; any
  number of `differential`/`gear_pair` components is fine, see above).
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
