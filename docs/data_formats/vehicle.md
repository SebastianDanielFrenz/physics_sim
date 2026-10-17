# Vehicle format — `physics_sim.vehicle/2`

Files: `data/vehicles/*.json` (five shipped cars; the other reference variants are test fixtures in
`tests/support/fixtures/vehicles/`, same format, part paths `../../../../data/...`). Loader: `ps::io::load_vehicle_json`
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
| `aero` | [aero object](#aero) | optional | all zero (no aerodynamics) | see [Aero](#aero) | legacy `drag_area_m2` bluff-body drag, or the explicit model: anisotropic body drag/moments, lifting surfaces with actuators, ground effect, suction fans and a drafting wake; model description in `AERODYNAMICS.md` |
| `powertrain` | [powertrain object](#powertrain-graph) | optional | empty (no components, no shafts) | see below | the engine/clutch/gearbox/differential graph |
| `controllers` | array of [controller objects](#controllers) | optional | empty | at most one `manual_tcu` or `automatic_tcu` entry (else `"vehicle '<name>': at most one controller (manual_tcu or automatic_tcu) is supported"`); `electronics` entries are additional and are not counted by that cap | driver-assist logic (auto-clutch, auto-blip, auto-shift); P8 adds `electronics` entries (launch control, assist fitment), see [below](#controllers) |
| `cooling_system` | string | optional | path relative to the vehicle file; a `physics_sim.cooling_system/1` file ([cooling_system.md](cooling_system.md)); needs an engine component whose engine file names a `default_thermal` (or whose component carries a `thermal` override), else the load throws | **opts the vehicle in to the lumped engine/oil thermal model** (coolant and oil temperature, thermostat, radiator, fan, oil-viscosity friction, gauge, `vehicle<i>.thermal.*` telemetry). Absent = no thermal state, every hash and CSV header as before |
| `engine_thermal_start` | string | optional | `"warm"` (default) or `"ambient"`; rejected without a `cooling_system` | `"warm"`: coolant at the thermostat's 25 % opening point (91 C by default), oil at the friction reference temperature (95 C, friction scale exactly 1); `"ambient"`: cold soak at the first tick's ambient temperature. A scenario's vehicle entry may give its own `engine_thermal_start`, which wins |

The engine component of a vehicle with a `cooling_system` may carry a
`"thermal"` path (relative to the vehicle file; also allowed next to `"ref"`) that
overrides the engine file's `default_thermal` - see
[engine_thermal.md](engine_thermal.md).

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
| `tyre` | [tyre reference](tyre.md) | **required** unless `VehicleLoadOptions::tyres` injects one for this wheel's `name` | none (a wheel with neither is a load error: `vehicle <file>: wheel '<name>': no tyre (give a "tyre" path/object or VehicleLoadOptions::tyres)`; it used to get an empty default `TyreDesc`) | see `tyre.md` | inline tyre object, a path to a standalone tyre JSON, or a path to a `.tir` file; an injected desc replaces it (see "Injected tyres" below) |
| `tyre_profile` | string | optional | `""` | none (stored as-is; not resolved yet) | path or name of a [`physics_sim.tyre_profile/1`](tyre_profile.md) file for the surface condition grip. W6 stores it but resolves every wheel through the environment's `default_tyre_profile`; the per-wheel lookup is W7 |
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
{ "id": "<unique id>", "type": "engine" | "clutch" | "gearbox" | "gear_pair" | "differential" | "transfer_case" | "park_lock" | "pto" | "load_dyno" | "throttle_control" | "fuel_tank" | "n2o_bottle" | "nitrous_kit", ... }
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
  `differential`, `transfer_case`, `park_lock`, `pto`, `load_dyno`,
  `throttle_control` and `fuel_tank` components are **always inline** — `{"ref": ...}` on a
  clutch throws `"<ctx>: 'clutch' components are always inline in I3 (no
  'ref')"`.
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
  and [`pto` component fields](#pto-component-fields-i4-c11) below). A second
  `throttle_control` component is likewise rejected at load time
  (`"<ctx>: more than one throttle_control component - at most one is
  supported"`) and again, redundantly, at `Powertrain` graph-build time
  (`"vehicle '<v>': more than one throttle_control component - I7a C3
  supports at most one"`) — see [`throttle_control` component
  fields](#throttle_control-component-fields-i7a-c3) below.
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

An `engine` component (inline or `ref`'d) may additionally carry a
vehicle-level `"installation"` field (ps_audio A4, a string path resolved
relative to the VEHICLE file's own directory, **not** the engine file's) —
overrides the engine's own `cycle.default_installation` for this vehicle
only. Only legal when the engine is a simulated engine whose cycle block
has `intake`/`exhaust` pipe blocks (`"'installation' is only allowed for
an engine whose cycle block has 'intake'/'exhaust' pipe blocks"` /
`"'installation' is only allowed on a simulated engine with
'intake'/'exhaust' pipe blocks"`); must be non-empty when given. See
[`engine.md`](engine.md)'s `default_installation` field and
[`installation.md`](installation.md) for the referenced file's own format.

Likewise an `engine` component may carry a vehicle-level `"forced_induction"`
field (a string path resolved relative to the VEHICLE file's own directory, or
`"none"`) that replaces the engine's own `default_forced_induction` for this
vehicle only. Engine garage P2 renamed the key (it was `"turbo_configuration"`,
now rejected: `"'turbo_configuration' was renamed 'forced_induction' (engine
garage P2; a path or \"none\")"`). The replacement happens before the turbo
tables and the engine maps are built, so the derived turbo cache key and the
generated maps follow the file actually used; `compressor_supply` and the map
keys behave exactly as with the engine's default. Legal on any simulated engine
(since P2 also one whose file names no forced induction - the vehicle adds it);
a torque-map engine is rejected (`"'forced_induction' is only allowed on a
simulated engine ..."`); must be a non-empty string (`"'forced_induction' must
be a non-empty path string or \"none\""`). Absent = bit-identical to before.
See [Engine component slots](#engine-component-slots-engine-garage-p2) below for
the other slots and `"none"`.

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

**Reconstruction/leaf rule (I4 C4/C5/C6/C7, plan section 2.7):** a `lockable:
true` diff, or any `clutch_lsd`/`torsen` diff **whatever its efficiency**,
must be a **leaf** — its `left` and `right` ports may carry no other rigid
row (discounting the diff's own `left`/`right` rows, e.g. a `locked`/
`lockable` diff's own lock row), or the graph builder throws `"vehicle '<v>':
differential '<id>' of kind <k> must drive nodes with no other rigid
coupling (centre LSD/Torsen is not supported in I4)"` (a `lockable: true`
diff gets the same rejection with `"of kind lockable <k>"`, e.g. `"of kind
lockable open"`, instead of the plain kind name). A plain (non-lockable)
`open` diff may be a **leaf** or a **centre** diff — `left`/`right` feeding
two child diffs' `carrier` ports, reconstructed from both children's own row
lambdas — at **any** efficiency, lossy or not (the mesh-loss torque is
applied at `carrier`, so it does not care whether `left`/`right` also drive
child diffs). A non-lockable `locked`/`viscous` diff may likewise be a leaf
or a centre diff, but a **lossy** (`efficiency < 1`) `locked`/`viscous`
*centre* diff is rejected with the same "must drive nodes with no other
rigid coupling" message above — only a **lossless** (`efficiency == 1`)
`locked`/`viscous` diff may be a centre diff. A diff whose `left`/`right`
pattern matches neither the leaf nor the centre shape at all, and whose
`efficiency < 1`, throws instead `"vehicle '<v>': differential '<id>' has
efficiency < 1 but its output nodes carry rigid couplings other than child
differentials' carriers - mesh-loss reconstruction is not available there"`
(this exact wording only for a plain `open` diff — every other kind reaching
this point already threw the "no other rigid coupling" message above at the
centre-diff check). A diff graph that loops back on itself while resolving
centre diffs (`left`/`right` feeding a chain of carriers that eventually
feeds back to the same diff) throws `"vehicle '<v>': differential '<id>' is
part of a cycle of differentials feeding each other's carriers"`. Plain
`open` diffs are otherwise unaffected by any of this (same leaf/centre rules
and float ops as before I4).

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

#### `throttle_control` component fields (I7a C3)

`ThrottleControlDesc`. Always inline; at most one per vehicle, and only
when its `engine` names this vehicle's engine component (rejected both at
load time and again in `Powertrain`'s constructor — see the duplicate/
existence checks under ["Components"](#components) above). A pure
control-signal seam: it owns no shaft-network port and contributes no rows
to the graph — `Powertrain` instead uses it to configure a typed
`ThrottleStage` member, evaluated immediately before the named engine's own
`pre_solve` every substep. This is where a pedal or hand-throttle-lever
signal is turned into the `command`/`floor`/`ceiling` triple
(`ps::drivetrain::ThrottleCommand`) the engine actually receives — the
engine itself never reads a raw pedal value.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `engine` | string (component id) | **required** | — | must be present and non-empty (`"<ctx>: throttle_control 'engine' (a component id reference) is required"`); once every `powertrain.components` entry has been parsed (order-independent — a `throttle_control` listed BEFORE its engine in the array still loads), must name a `torque_map` engine component in this vehicle's powertrain, else `"vehicle '<v>': powertrain.components['<id>']: 'engine' (\"<ref>\") must name a \"torque_map\" engine component in this vehicle's powertrain"` (covers an unknown id and a non-engine component); a vehicle with NO engine component at all is rejected first with its own message, `"vehicle '<v>': powertrain.components['<id>']: a throttle_control component in a vehicle with no engine component"`; `Powertrain`'s constructor re-checks it names THIS vehicle's actual engine component, `"vehicle '<v>': throttle_control component '<id>' names engine '<engine>', which is not this vehicle's engine ('<actual>')"` | which engine component this stage feeds |
| `kind` | string | **required** | — | must be `"electronic"` or `"mechanical_linkage"`, else `"<ctx>: throttle_control 'kind' must be \"electronic\" or \"mechanical_linkage\", got \"<value>\""` — **note**: deliberately NOT the same string set as the engine's own [`throttle_input`](engine.md) field (`"electronic"`/`"mechanical"`) — the two fields name different things (the engine's own wiring vs. this stage's kind) and the plan spec gives them different literal strings on purpose. Once the named engine is resolved (see `engine` above), `kind` must AGREE with that engine's own `throttle_input` (`"electronic"` <-> `"electronic"`, `"mechanical_linkage"` <-> `"mechanical"`) in both directions, else `"vehicle '<v>': powertrain.components['<id>']: 'kind' (\"<stage-kind>\") does not match engine '<engine>'.throttle_input (\"<engine-kind>\") - an explicit throttle_control component must agree with its engine, it no longer silently overrides it"` (PHYS-033 P4 I7a C3 review item 2 — an earlier revision let an explicit stage silently win over a disagreeing `throttle_input`; that is now rejected instead) | electronic (pedal map, TCU-forwarded floor/ceiling) or mechanical linkage (fixed floor 0/ceiling 1, no TCU intervention beyond the upshift ignition cut) |
| `pedal_map` | Curve1D | optional, **only valid when `kind: "electronic"`** | none (pedal passes through unmodified) | present with the wrong `kind` throws `"<ctx>: throttle_control 'pedal_map' is only valid for kind \"electronic\" (a mechanical linkage uses 'linkage_curve' instead)"`; `x` strictly ascending (`require_ascending`); additionally `x.front()` must be exactly `0` and `x.back()` exactly `1` (the curve must cover the whole pedal range — `"<ctx>.pedal_map: x.front() must be exactly 0 ..."` / `"... x.back() must be exactly 1 ..."`), every `y` within `[0, 1]` (`"<ctx>.pedal_map: y must be within [0, 1]"`) and `y` non-decreasing (`"<ctx>.pedal_map: y must be non-decreasing"`) | reshapes the raw pedal 0..1 before it reaches the stage's `max()` with any hand lever |
| `linkage_curve` | Curve1D | optional, **only valid when `kind: "mechanical_linkage"`** | none (pedal passes through unmodified) | present with the wrong `kind` throws `"<ctx>: throttle_control 'linkage_curve' is only valid for kind \"mechanical_linkage\" (electronic uses 'pedal_map' instead)"`; same `x`/`y` ascending/endpoint/monotonicity validation as `pedal_map` | same role as `pedal_map`, for a mechanical (cable/rod) linkage's own non-linear travel curve |
| `hand_lever_channel` | string | optional | `""` (no hand lever) | none, UNLESS the named engine has a `governor` block (`engine.md`'s `governor` section): then a non-empty `hand_lever_channel` must equal that engine's `governor.channel` exactly, else `"vehicle '<v>': powertrain.components['<id>']: 'hand_lever_channel' (\"<channel>\") differs from engine '<engine>'.governor.channel (\"<gov-channel>\") - a governed engine has one hand lever; leave hand_lever_channel unset to use the governor's own channel, or set it to that same channel"` — a governed engine has exactly one physical hand lever, so a stage naming a DIFFERENT channel makes no sense; leaving it empty on a governed engine falls back to `governor.channel` (see "Implicit default stage" below). An explicit `hand_lever_channel` on an UNGOVERNED engine is always allowed (a hand lever on the same linkage/ECU as an electronic/mechanical throttle is physical) | declared control channel this stage reads as a hand-throttle lever, `max()`-combined with the (curve-mapped) pedal — see below |

Port: none (see above — `throttle_control` never appears in `shafts[]`).

**Stage arithmetic** (plan section 3.4.4, evaluated every substep before the
named engine's `pre_solve`):

```
pedal'          = pedal_map/linkage_curve present ? curve(pedal) : pedal
command         = hand lever present ? max(clamp(hand, 0, 1), clamp(pedal', 0, 1)) : pedal'
floor, ceiling  = kind == "electronic" ? (ctx.tcu_throttle_floor, ctx.tcu_throttle_ceiling) : (0, 1)
```

The engine then applies its own governor/ignition-cut/idle-PI logic
unchanged, using `command` wherever it used to read a raw pedal value and
clamping to `[floor, ceiling]` wherever it used to clamp to the TCU's own
floor/ceiling — see `engine.md` and the "Drivetrain" section of the
top-level `CLAUDE.md` for `IEngine::pre_solve`'s exact signature.

**Implicit default stage.** A vehicle with no `throttle_control` component
still gets one, built by `Powertrain` itself from the engine's own
declared [`throttle_input`](engine.md) field: `kind` = that field's value,
no `pedal_map`/`linkage_curve`, and a hand lever **only when the engine has
a `governor` block** (`engine.md`'s `governor` section) — in that case the
implicit stage's hand lever is exactly the governor's own `channel` (the
same channel the pre-I7a-C3 code resolved directly; nothing about
`data/vehicles/tractor.json`'s `hand_throttle` channel or its governor
behaviour changed). Every shipped vehicle (`car_sedan.json` and its
variants, `tractor.json`) relies on this implicit stage — none declares an
explicit `throttle_control` component.

**Explicit stage on a governed engine.** An explicit `throttle_control`
component that does not set its own `hand_lever_channel` behaves the same
way as the implicit stage above when its engine has a `governor` block:
`Powertrain` (Phase K) falls back to that engine's `governor.channel`
rather than leaving the stage with no hand lever at all — a governed
tractor engine keeps its hand throttle even if an explicit stage is later
added to give it a `pedal_map`/`linkage_curve`. See `hand_lever_channel`'s
own row above for the (only remaining) rejection this interacts with: an
explicit channel that names something OTHER than that same
`governor.channel`.

**Which TCU intervention lives where** (plan section 3.4.5):

| Intervention | Lives in | Mechanical linkage |
|---|---|---|
| Pedal map / linkage curve | this stage component | works (that is the point of `linkage_curve`) |
| Hand lever | this stage component | works |
| Blip **and** post-upshift pedal-ceiling ramp (`auto_blip.kp_per_rpm`/`max_throttle`/`upshift_torque_ramp_s`) | TCU → forwarded through `floor`/`ceiling`, applied by an electronic stage only | **rejected at load time**, on the EFFECTIVE VALUES rather than `auto_blip.enabled` (PHYS-033 P4 I7a C3 review item 1 — an earlier revision checked `enabled`, which wrongly rejected `enabled: true` with all three values already zeroed, a no-op configuration that must load): once every controller's `auto_blip` block has its defaults applied (`AutoBlipDesc`'s own defaults — `kp_per_rpm 0.002`, `max_throttle 1.0`, `upshift_torque_ramp_s 0.10` — apply even when the JSON has no `auto_blip` block at all), the effective throttle kind (this component if present, else the engine's `throttle_input`) resolving to Mechanical with `kp_per_rpm > 0 \|\| max_throttle > 0 \|\| upshift_torque_ramp_s > 0` throws `"<ctx>.auto_blip: engine '<id>' has a mechanical throttle - a mechanical throttle cannot open the plate (blip) or close it against the driver (ceiling/ramp); the upshift ignition cut stays available (set those three to 0)"`. A mechanical vehicle must therefore set all three to `0` explicitly (`enabled` itself is irrelevant to this check either way) |
| Upshift ignition cut (`auto_blip.upshift_cut`) | forwarded directly to the engine as `TcuRequest::ignition_cut`, never through `ThrottleCommand` | works on both kinds |
| Idle PI / limiter / DFCO / governor / starter | engine, unchanged | works on both kinds |

**Open design question — the TCU still reads the raw pedal.** `ManualTcu`
(the launch law, the auto-shift up/down rpm thresholds) reads the pedal
value `World`/the adapter sets on the `"throttle"` control channel
directly, NOT this stage's `pedal_map`-reshaped `command` — the pedal map
only reshapes what reaches the engine's fuelling, not what the TCU uses to
decide when to shift or how to launch. For every shipped vehicle this is
moot (neither `car_sedan.json`/its variants nor `tractor.json` declares a
`pedal_map`), but a vehicle that DOES add one would see the TCU's shift
points keyed to the raw pedal position while the engine itself sees the
reshaped value. Left as-is for now (PHYS-033 P4 I7a C3 review item 7) —
not attempted in this commit.

`EngineSoundState::throttle_command` (sound-only, never hashed) publishes
this stage's `command` for a listening/telemetry consumer, alongside the
engine's existing `load`/`throttle_eff` fields.

#### `fuel_tank` component fields (I7a C6)

`FuelTankDesc`. Always inline (`{"ref": ...}` throws `"<ctx>: 'fuel_tank'
components are always inline (no 'ref')"`). A fuel store, not a shaft
component: it has **no shaft port** and adds no node, row or inertia to the
network. It is reached only through a [fluid link](#fluid-links-i7a-c6).
Internally it is a `FuelTank` behind the generic `IEnergyStore` interface
(`core/src/drivetrain/energy_store.h`), so a later battery or hydrogen
store can use the same seam.

```json
{ "id": "tank", "type": "fuel_tank", "fuel": "../fuels/e5.json", "capacity_kg": 40.0, "initial_kg": 30.0 }
```

| Field | Type | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `fuel` | string (path) | **required** | non-empty string, else `"<ctx>: fuel_tank 'fuel' is required and must be a non-empty path string"`. Resolved relative to the **vehicle file's** directory (like every inline component path), then loaded with `load_fuel_json` ([fuel.md](fuel.md) rejections apply) | the fuel the tank holds |
| `capacity_kg` | real (kg) | **required** | a number (`"<ctx>: fuel_tank 'capacity_kg' is required and must be a number"`), finite (`"... must be finite"`, a backstop only: nlohmann rejects an overflowing literal such as `1e400` itself with `"number overflow parsing '1e400'"`), not negative (`"<ctx>: fuel_tank 'capacity_kg' must not be negative, got <v>"`) and `> 0` (`"<ctx>: fuel_tank 'capacity_kg' must be > 0"`) | usable capacity |
| `initial_kg` | real (kg) | **required** | a number, finite, not negative (same messages with `'initial_kg'`), and `<= capacity_kg` (`"<ctx>: fuel_tank 'initial_kg' (<v>) must not exceed 'capacity_kg' (<c>)"`) | content at vehicle creation (`World::reset_vehicle` leaves the tank as it is; refill with `World::set_fuel_tank_mass`) |

At load time a tank must also feed something: a tank with no fluid link from
its `out` port throws `"vehicle '<v>': powertrain.components['<id>']:
fuel_tank has no consumer (no powertrain.fluid link from \"<id>.out\")"`.
`Powertrain`'s constructor re-checks every rule above for a code-built
`VehicleDesc` (`"vehicle '<v>': fuel_tank '<id>' has no consumer"`,
`"vehicle '<v>': fuel_tank '<id>' needs finite capacity_kg > 0 and 0 <=
initial_kg <= capacity_kg"`, `"vehicle '<v>': engine '<e>' has no fuel
model - fuel_tank '<id>' cannot feed it"`, `"vehicle '<v>': tank fuel
'<name>' is not in engine '<e>''s fuels list"`).

In I7a the fuel mass does not change the chassis mass (the chassis body
keeps its `chassis.mass` whatever the tank holds); a later increment adds it
as a point mass at the tank position (PLAN.md "Fuel mass moves the
chassis").

### Fluid links (I7a C6)

`"fluid"` (optional, inside `"powertrain"`, stored as
`PowertrainDesc::fluid`) is an array of `["<tank>.out", "<engine>.fuel"]`
pairs. It is separate from `shafts[]`: a fluid link carries fuel mass, not
torque, and never creates a network node.

```json
"fluid": [["tank.out", "engine.fuel"]]
```

Loader rejections (`<lctx>` = `vehicle '<v>': powertrain.fluid[<n>]`):

| Case | Message |
|---|---|
| `fluid` is not an array | `"vehicle '<v>': powertrain.fluid must be an array of [\"<tank>.out\", \"<engine>.fuel\"] pairs"` |
| an entry is not a pair of strings | `"<lctx>: must be a [\"<tank>.out\", \"<engine>.fuel\"] pair of port strings"` |
| a malformed port ref (no `.`, or `.` first/last) | `"<lctx>: bad port ref \"<ref>\" - must be a ..."` |
| unknown component on either side | `"<lctx>: unknown component '<id>'"` |
| source is not a `fuel_tank` | `"<lctx>: '<id>' is not a fuel_tank (a fluid link must start at \"<tank>.out\")"` |
| source port is not `out` | `"<lctx>: unknown port \"<ref>\" (a fuel_tank's only fluid port is \"out\")"` |
| target is not an engine | `"<lctx>: '<id>' is not an engine (a fluid link must end at \"<engine>.fuel\")"` |
| target port is not `fuel` | `"<lctx>: unknown port \"<ref>\" (an engine's only fluid port is \"fuel\")"` |
| target engine has no fuel model | `"<lctx>: engine '<e>' has no fuel model (no 'fuels', 'fuel' or 'fuel_map') - a fluid link cannot feed it"` |
| the same link twice | `"<lctx>: duplicate fluid link \"<from>\" -> \"<to>\""` |
| two tanks feed one engine | `"<lctx>: engine '<e>' fuel port is fed by two tanks ('<a>' and '<b>')"` |
| the tank's fuel file is not one of the engine's fuel files (`std::filesystem::equivalent`) | `"<lctx>: tank fuel \"<path as written>\" is not in engine \"<e>\"'s fuels list"` |

A tank on a map-less engine (the [crude fuel model](engine.md)) is allowed.

#### Nitrous links (N2O N1)

The same `fluid` array also carries the nitrous wiring (N2O plan N1, the three part files are in
[nitrous.md](nitrous.md); a `n2o_bottle`/`nitrous_kit` component is always a `"ref"` to a part
file plus the documented overrides):

```json
"fluid": [["tank.out", "engine.fuel"], ["n2o_bottle.out", "n2o_kit.n2o"], ["n2o_kit.out", "engine.nitrous"]]
```

A link whose source is an `n2o_bottle` must be `"<bottle>.out" -> "<kit>.n2o"`; one whose source
is a `nitrous_kit` must be `"<kit>.out" -> "<engine>.nitrous"`. The fuel-link rejections above
are unchanged (and the "two tanks feed one engine" check only looks at links into the `fuel` port).
Additional rejections (`<lctx>` as above):

| Case | Message |
|---|---|
| bottle source port is not `out` | `"<lctx>: unknown port \"<ref>\" (an n2o_bottle's only fluid port is \"out\")"` |
| bottle target is not a `nitrous_kit` | `"<lctx>: '<id>' is not a nitrous_kit (an n2o_bottle's link must end at \"<kit>.n2o\")"` |
| bottle target port is not `n2o` | `"<lctx>: unknown port \"<ref>\" (a nitrous_kit's bottle input is \"n2o\")"` |
| two bottles into one kit | `"<lctx>: nitrous_kit '<k>' n2o port is fed by two bottles ('<a>' and '<b>')"` |
| a bottle feeding a second kit | `"<lctx>: n2o_bottle '<b>' already feeds a kit"` |
| kit source port is not `out` | `"<lctx>: unknown port \"<ref>\" (a nitrous_kit's only output port is \"out\")"` |
| kit target is not an engine | `"<lctx>: '<id>' is not an engine (a kit's link must end at \"<engine>.nitrous\")"` |
| kit target port is not `nitrous` | `"<lctx>: unknown port \"<ref>\" (an engine's nitrous input is \"nitrous\")"` |
| target engine has no fuel model | `"<lctx>: engine '<e>' has no fuel model (...) - a nitrous_kit's fuel jet needs its fuel"` |
| two kits into one engine | `"<lctx>: engine '<e>' nitrous port is fed by two kits ('<a>' and '<b>')"` |
| a bottle without a kit link / a kit without a bottle or engine link | `"vehicle '<v>': powertrain.components['<id>']: n2o_bottle has no consumer (no powertrain.fluid link from \"<id>.out\")"`, `"... nitrous_kit has no bottle (no powertrain.fluid link to \"<id>.n2o\")"`, `"... nitrous_kit has no consumer ..."` |

`Powertrain::Powertrain` repeats the wiring checks for a code-built `VehicleDesc`
(`"vehicle '<v>': n2o_bottle '<id>' is not linked to a nitrous_kit"`, `"... nitrous_kit '<id>' has no n2o_bottle"`,
`"... nitrous_kit '<id>' has no consumer"`, `"... a nitrous_kit's \"out\" port feeds an engine's \"nitrous\" port"`,
`"... an n2o_bottle's \"out\" port feeds a nitrous_kit's \"n2o\" port"`). The nitrous system is a
Powertrain-owned typed member (no shaft port, node or row); a vehicle without a kit is bit-identical to before.

The loader's same-fuel check compares **file paths**
(`std::filesystem::equivalent`), not contents: a byte-identical copy of
`e5.json` under another name is rejected. The `Powertrain` constructor's
backstop for a code-built `VehicleDesc` has no paths and compares **values**
instead (name, phase, `lhv_j_kg`, `afr_stoich`, `density_kg_m3`, all
exact).

**Which fuel burns.** With a tank, the engine burns the tank's fuel (which
must be one of its `fuels`); without one, it burns `fuels[0]`. The choice is
fixed when the vehicle is built and is not hashed.

**Starvation.** Each substep the engine computes its fuel demand
`m_demand = h * mdot_demand` (fuel map or crude model) and the supply
fraction

```
s = clamp(m_tank / m_demand, 0, 1)      (s = 1 when m_demand == 0)
```

The combustion torque `tau_comb`, its share of the linearised damping `d`,
the combustion fraction `theta_c` (published as `EngineSoundState::load`)
and the booked fuel mass all scale by `s` (held constant within the
substep: on the emptying substep `d` omits the `ds/domega` term by design,
since `s` depends on `omega` only through that substep's demand); the tank
loses exactly the
booked mass and is clamped at `0`. At `s == 1` the result is bit-identical
to the same vehicle without a tank (the engine's torque demand, the ECU's
request, is not scaled). An empty tank gives `tau_comb = 0`: a running
engine coasts down on its motoring torque to `Stalled`, and cranking with an
empty tank never reaches the fire speed. The idle PI integrator winds up to
its upper clamp while the engine starves at idle and is reset to `0` when
the engine stalls.

**Runtime access.** `PowertrainSnapshot::fuel_mass_kg` (the tank content, 0
without a tank); `World::fuel_tank_mass(vehicle)`; `World::set_fuel_tank_mass(
vehicle, kg)` - applied at the next tick boundary, clamped to
`[0, capacity_kg]`, and booked signed (`new - old`) into
`PowertrainEnergy::fuel_external_added` (throws for a vehicle without a tank
or a NaN mass). The tank content is folded into `World::state_hash()` only
when the vehicle has a tank, so every tank-less vehicle hashes as before.

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
| `fuel_tank` | none (fluid port `out`, used only in [`fluid`](#fluid-links-i7a-c6)); an engine's fluid port is `fuel` |
| `n2o_bottle` | none (fluid port `out`); `nitrous_kit`: fluid ports `n2o` (in) and `out`; an engine's second fluid port is `nitrous` - see [nitrous.md](nitrous.md) and [Nitrous links](#nitrous-links-n2o-n1) |
| a wheel (implicit, one per entry in the top-level `wheels[]`) | `wheel:<wheel name>` |

A port reference string is written `"<component id>.<port>"` (e.g.
`"engine.crank"`, `"diff.left"`) or `"wheel:<wheel name>"` (e.g.
`"wheel:FL"`). A malformed reference (no `.`, or a `.` at position 0 or at
the very end) throws `"powertrain.shafts[]: bad port ref \"<ref>\" (expected
\"<id>.<port>\" or \"wheel:<name>\")"`; an empty wheel name after `"wheel:"`
throws the same message.

### Shafts

`"shafts"` is an array of `{ "ports": ["<ref>", ...], "stiffness": 0,
"damping": 0 }`. Each shaft rigidly joins every port it lists into one node
— unless the powertrain is `"compliance": "compliant"` and the shaft has
`stiffness > 0`, in which case it is a torsional spring-damper between two
nodes instead (see [Compliant shafts](#compliant-shafts-i4-c13) below).

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `ports` | array of port-ref strings | **required** | — | must be present (`"powertrain.shafts[]: 'ports' is required"`); each ref must resolve to a real port, else `"powertrain shaft references unknown port '<key>'"`; a port referenced by more than one shaft, or twice within the same shaft, throws `"powertrain port '<key>' is connected by more than one shaft, or twice in one shaft"` | which ports are rigidly joined |
| `stiffness` | real (N·m/rad) | optional | `0` | **must be `>= 0`**, else `"vehicle '<name>': powertrain.shafts[<i>]: 'stiffness' must be >= 0"` (checked whatever `compliance` is) | torsional stiffness `k`; only used when `compliance` is `"compliant"` — under `"rigid"` it is read, validated and ignored |
| `damping` | real (N·m·s/rad) | optional | `0` | **must be `>= 0`**, else `"...: 'damping' must be >= 0"`; `damping > 0` with `stiffness == 0` throws `"...: 'damping' > 0 requires 'stiffness' > 0 (a pure damper shaft is not supported)"` | torsional damping `c` on the twist rate; same "only when compliant" rule as `stiffness` |

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
| `compliance` | string | optional | `"rigid"` | must be `"rigid"` or `"compliant"`, else `"vehicle '<name>': powertrain.compliance must be \"rigid\" or \"compliant\", got \"<value>\""`; `"compliant"` with no shaft at `stiffness > 0` throws `"...: powertrain.compliance is \"compliant\" but no shaft has stiffness > 0"` | shaft-network compliance model: `"rigid"` = every shaft is a rigid join (every shipped vehicle); `"compliant"` = shafts with `stiffness > 0` become spring-dampers, see below |
| `rotor_reaction` | bool | optional | `true` | none | whether spinning components apply a reaction torque back onto the chassis about their own axis; `false` only for a temporary data-migration step |

### Compliant shafts (I4 C13)

With `"compliance": "compliant"`, every shaft with `stiffness > 0` is a
torsional spring-damper between two separate network nodes instead of a
rigid join; shafts with `stiffness == 0` stay rigid joins. No shipped
vehicle uses it.

```json
"powertrain": {
  "compliance": "compliant",
  "shafts": [
    { "ports": ["engine.crank", "clutch.in"] },
    { "ports": ["clutch.out", "gearbox.in"], "stiffness": 600, "damping": 2 }
  ]
}
```

- A compliant shaft must list **exactly 2 ports**, else `"vehicle '<name>':
  powertrain.shafts[<i>]: a compliant shaft (stiffness > 0) must list
  exactly 2 ports, got <n>"`. `ports[0]` is end `a`, `ports[1]` end `b`.
- Its two ends must end up on **different** nodes (other rigid shafts
  must not already join them): else `"...: compliant shaft's two ports
  '<a>' and '<b>' are already rigidly joined by other shafts (a compliant
  shaft must join two distinct nodes)"` (thrown when the vehicle is built,
  not at load).
- Both ends need **inertia > 0** on their node, else `"...: compliant shaft
  endpoint '<port>' has zero inertia (...)"` (also at build). In
  `car_sedan.json`, `clutch.in` has no inertia of its own and a
  differential's `left`/`right` have none, so only `clutch.out`–`gearbox.in`
  (0.004/0.010 kg·m²) and `gearbox.out`–`diff.carrier` (0.015/0.02 kg·m²)
  can be made compliant without adding inertia.
- At most `kMaxCompliantShafts` (8, `ps/drivetrain/powertrain_state.h`)
  compliant shafts per vehicle, else `"vehicle '<name>': at most 8 compliant
  shafts are supported, got <n>"` (also at build).

Model: twist `theta = ∫(omega_a - omega_b) dt`, shaft torque
`tau_s = k*theta + c*(omega_a - omega_b)`, acting `-tau_s` on `a` and
`+tau_s` on `b`. It is solved implicitly with the rest of the network (one
two-node coupling per compliant shaft, after any viscous-diff couplings).
The integrator is **trapezoidal** by default; with `c = 0` this is the
implicit midpoint rule, so kinetic plus spring energy is conserved exactly
and the discrete oscillation frequency is `(2/h)*atan(omega_n*h/2)`. The
period is about `(omega_n*h)^2/12` longer than `2*pi/omega_n` (about 0.02 %
at 50 rad/s and 960 Hz; about 1.9 % for the sedan's `clutch.out`–
`gearbox.in` pair at `k = 600`, `omega_n` = 458 rad/s).

The **first substep after a mode change** uses **backward Euler**
instead, which damps any step the change introduced rather than letting it
ring. A mode change is a topology switch (gear or diff-lock change), any
friction row's stick/slip flag changing in a substep's solve, or a
`reset()`.

A reset (`World::reset_vehicle`) relaxes every compliant shaft to
`theta = 0` and equal end speeds. If the reset sets exactly one end itself
(hubs, derived diff carriers/gear inputs, the engine crank), the other end
takes that end's speed. If it sets neither, `b` takes `a`'s speed. If it
sets both (e.g. `clutch.out` and `gearbox.in`, which it sets to the same
speed), neither is changed.

Energy ledger (`PowertrainEnergy`): `shaft_pe` (`sum 1/2*k*theta^2`,
instantaneous, stored like `rotor_ke`) and `shaft_damping_loss`
(cumulative `c`-term work). Whatever the discrete step adds or removes
beyond those, including backward Euler's own dissipation, is booked into
`integrator_loss`, so `balance_error` still closes. The
`vehicle<i>.energy.stored` telemetry channel is `rotor_ke + shaft_pe` for a
compliant vehicle.

`PowertrainDebug::shaft_twist_rad[i]`/`shaft_torque[i]` (tests only) give
the i-th compliant shaft's `theta` and last `tau_s`. The state hash folds
each compliant shaft's `theta` plus the pending-backward-Euler flag. A
`"rigid"` vehicle, including one whose shafts carry `stiffness`/`damping`
values, gets no coupling, no ledger term and no hash byte, so it is
bit-identical to before C13.

### What the graph builder additionally rejects

Beyond the per-field checks above, `Powertrain::Powertrain`
(`core/src/drivetrain/powertrain.cpp`) walks the whole graph once at
vehicle-construction time and additionally rejects:

- A wheel with `wheel_inertia <= 0` (see the [wheel table](#wheel) above).
- More than one engine/clutch/gearbox/transfer_case/park_lock/pto component
  (duplicated check from the field-level notes above — `park_lock` and `pto`
  are additionally rejected earlier, at load time; any number of
  `differential`/`gear_pair`/`load_dyno` components is fine, see above).
- A differential whose `left`/`right` ports don't match a resolvable
  leaf/centre pattern for its kind and efficiency (a `lockable: true` or
  `clutch_lsd`/`torsen` diff that isn't a leaf; a lossy `locked`/`viscous`
  centre diff; a plain lossy `open` diff whose ports match neither shape at
  all) — the exact rejection messages are under the ["Reconstruction/leaf
  rule"](#differential-component-fields) above.
- A cycle of differentials feeding each other's carriers (resolving centre
  diffs loops back on itself) — see the same section above.
- More `lockable: true` differentials than the composed-topology-state
  budget allows (I4 C5: `kMaxStates` = 128 — `num_gear_states * 2^(number
  of lockable diffs)` must not exceed it).
- More than `kMaxCompliantShafts` (8) compliant (`stiffness > 0` under
  `"compliance": "compliant"`) shafts (I4 C13, see [Compliant
  shafts](#compliant-shafts-i4-c13) above).
- A `brake_system` corner or handbrake wheel name that doesn't match any
  wheel, or a duplicate corner for the same wheel.
- A controller (see [below](#controllers)) referencing a component id that
  doesn't exist, or that exists but is the wrong component type.
- A malformed fluid link or fuel tank in a code-built `VehicleDesc` (the
  backstops listed under [`fuel_tank`](#fuel_tank-component-fields-i7a-c6)
  and `"vehicle '<v>': fluid link source '<ref>' is not a fuel_tank's
  \"out\" port"` / `"... fluid link target '<ref>' is not an engine's
  \"fuel\" port"` / `"vehicle '<v>': engine '<e>' fuel port is fed by more
  than one fluid link"`).

## Controllers

`"controllers"` is an array of `{id, type, ...}`: at most one transmission controller, `"manual_tcu"`
(`ManualTcuDesc`) or `"automatic_tcu"` (`AutomaticTcuDesc`, see [below](#automatic_tcu)) (a second one
throws `"vehicle '<name>': at most one controller (manual_tcu or automatic_tcu) is supported"`) plus any number of
`"electronics"` entries (see the P8 and assist-unit paragraphs below and [electronics.md](electronics.md)). An unknown
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
| `max_limiter_fraction` | real | `0.985` `[AS]` | G1: when the higher upshift point (the car's own or a drive mode's override) lies above this fraction of the engine's effective limiter (the lowest of the FuelCut rpm, a SoftCut's taper start, a governor's high idle and an armed roll-off end, engine.md "Beyond the authored rpm range"), the TCU multiplies all four points by `cap / max(up)`, one factor, so the down/up ratios are kept. The load reports it with `warnings_out` (`"... auto_shift: Limiting: the upshift at U rpm is above C rpm (98.5 % of the effective limiter): the shift points are scaled by k (...)"`, with ` (drive mode '<name>')` for an override). Must be in `(0, 1]`. Every shipped car's points sit below it (highest: car_hyper_n2o Sport, 8100 / 8250 = 0.982) |
| `kickdown_throttle` | real | `0` | assists I7: a **downshift** whose rpm condition holds at a throttle command `>=` this value ignores `min_time_in_gear_s` (upshifts always wait for it). The bypass is a driver action: a gear change spends it and it is re-armed only once the command has fallen below this value, so a pedal held at full throttle through an upshift cannot bounce back down (hashed only while spent). The compared value is the throttle COMMAND (after the pedal curve and a drive mode's bend). `0` = off (every shipped car except `car_hyper_n2o`, `0.95` `[AS]`). Must be in `[0, 1]`, else `"auto_shift: kickdown_throttle must be in [0, 1] (0 = no bypass)"` |

Apart from the three `upshift_*` fields above, none of the numeric
sub-fields inside `auto_blip`/`auto_shift` are range-validated by the
loader beyond the defaults shown.

**Engine garage P8 (fitment, [electronics.md](electronics.md)).** `auto_clutch`, `auto_blip` and `auto_shift` each take an
optional `"fitted"` bool (default `true`); `false` makes the assist ignore its `assist.*` channel (the channel is still
registered, default 0). The vehicle-level `traction_control` block takes the same `fitted` key, and an optional
top-level `"anti_lag_switch": {"fitted": false}` removes the car-side anti-lag switch (the engine's anti-lag channel is
then ignored; `default_enabled` rules). The `controllers` array also accepts entries `{"id", "type": "electronics",
"ref": "<physics_sim.electronics/1 file>"}` (path relative to the vehicle file) next to the one `manual_tcu`: a
`launch_control` part (second rev limit, channel `assist.launch_control`, registered only on a car that has one) or an
`assist_fitment` part (sets `fitted` to `true` for the assists it lists). At most one `manual_tcu` and one
`launch_control`; ids are unique across all controllers.

**Assist unit (assists I1).** The `electronics` entry may also be an `assist_unit` part (grade Street / Sport / Racing and a
traction-control mode table, channel `assist.traction_control` codes 0 Off, 1 On, 2 Relaxed, 3 Sport, 4 Drift). It adds no
controller: the loader stores it as `VehicleDesc::assist_unit` (at most one per vehicle). The car's own `traction_control`
block is mode On. Without a unit the channel keeps its pre-unit meaning (`> 0.5` is on). A Racing unit's `aero_control` block (assists I5) additionally automates the car's `rear_wing` surface through the channel `assist.aero_control`, registered only when the car's aero block has the named surface. A Sport or Racing unit's `launch_control` block (assists I4) makes the unit a launch control too (channel `assist.launch_control`, registered on a car with either that block or a P8 `launch_control` part; both on one car is rejected). See [electronics.md](electronics.md#assist-unit).

**Drive modes (assists I7).** The `electronics` entry may also be a `drive_modes` part (car-specific: pedal bend, auto-shift
rpm points, post-shift clutch close time, a traction-control default, launch-control and N2O gates; at most one per
vehicle; needs the car's `manual_tcu`). Channel `assist.drive_mode`. See [electronics.md](electronics.md#drive-modes).
A mode's `traction_control_default` / `launch_control_default` is written into the `assist.traction_control` /
`assist.launch_control` channel by the physics (assists I7c; `World::step` applies it after the tick, so `get_control` reads it right
after `step()` and one driver write of any value wins) - clients that mirror the channels must adopt it (electronics.md rule 4c).

### `automatic_tcu`

Automatic gearbox plan A1. The controller of an `"automatic"`, `"dual_clutch"` or (CVT plan C1) `"cvt"` gearbox
([gearbox.md](gearbox.md#automatic-and-dual-clutch-gearboxes), [CVT](gearbox.md#cvt-gearboxes)); the cvt kind counts
as automatic in every rule below. Pairing rules (Powertrain build):
an automatic/dual_clutch gearbox needs one (`"an automatic or dual_clutch gearbox needs an automatic_tcu controller"`),
a `manual_tcu` cannot drive one (`"a manual_tcu cannot drive an automatic or dual_clutch gearbox - use automatic_tcu"`),
an `automatic_tcu` needs one (`"an automatic_tcu needs an automatic or dual_clutch gearbox"`) and such a vehicle has no
`clutch` component (`"an automatic or dual_clutch gearbox has no separate clutch component"`). A `drive_modes`
electronics part needs a `manual_tcu`, so it cannot be fitted with an automatic_tcu. Unknown keys are rejected
(`"<ctx>: unknown key '<key>'"`).

**Selector** through the existing channels (no new selector channel): `gear_request` 0 = N, any negative value = R,
`k > 0` = D with gears up to `min(k, forward gears)` (a large value is plain D, 1..n hold the box at or below that gear
like L/2/3); P = N plus a `park_lock` component (channel `park_lock`). `shift_up_count` / `shift_down_count` in D step
one gear (manual mode, paddles); manual mode ends after `manual_timeout_s` without a press (0 = never) or on a
`gear_request` change. Channel `transmission.mode` (declared with default 0 by every automatic/dual_clutch gearbox):
`>= 0.5` selects `sport_schedule`. Driver demand = the throttle stage's command.

| Field | Type | Default | Validation | Meaning |
|---|---|---|---|---|
| `gearbox`, `engine` | string (component id) | **required** | must reference components (`"'gearbox' and 'engine' (component id references) are required"`) | what the TCU drives |
| `schedule` | object `{up_rpm_at_zero_throttle, up_rpm_at_full_throttle, down_rpm_at_zero_throttle, down_rpm_at_full_throttle}` | 2000 / 6000 / 1100 / 3200 | every downshift rpm `> 0` and below the upshift rpm at the same throttle | D schedule in gearbox INPUT rpm (turbine of an automatic, engine of a dual clutch), interpolated linearly in the throttle command |
| `sport_schedule` | same | same as the defaults | same | S schedule |
| `min_time_in_gear_s` | real (s) | `0.8` | `>= 0` | upshift dwell (kickdown and the stall guard ignore it) |
| `kickdown_throttle`, `kickdown_max_rpm` | real, real (rpm) | `0.9`, `5500` | `[0, 1]`, `> 0` | at or above this command: downshift to the lowest gear whose input rpm stays at or below `kickdown_max_rpm` (automatic; a dual_clutch steps one gear - see Limits) |
| `manual_timeout_s` | real (s) | `0` | `>= 0` | see above |
| `manual_up_rpm` | real (rpm) | `0` | `>= 0` | manual mode: forced upshift at this input rpm (0 = the full-throttle up line) |
| `upshift_torque_fraction` | real | `0.7` | `(0, 1]` | engine throttle cap during a power-on upshift's inertia phase |
| `lockup` | object `{enabled, min_gear, engage_rpm, release_rpm, max_throttle}` | true, 3, 1400, 1100, 0.8 | `min_gear >= 1`, `0 < release_rpm < engage_rpm`, `max_throttle` in [0, 1] | automatic only: lock-up engaged in gear >= `min_gear` with the turbine above `engage_rpm` and the command at or below `max_throttle`; released below `release_rpm`, above `max_throttle + 0.1` and during every shift |
| `launch` | object `{bite_rpm, full_rpm_min, full_rpm_max, lock_slip_rpm, anti_stall_rpm}` | 1000, 1500, 3500, 100, 600 | `anti_stall_rpm > 0`, `0 < bite_rpm < full_rpm_min <= full_rpm_max`, `lock_slip_rpm > 0` | dual_clutch only: launch on clutch slip - engagement 0 at `bite_rpm` -> 1 at `lerp(full_rpm_min, full_rpm_max, throttle)` of engine rpm; locks once `|slip| < lock_slip_rpm` until the engine falls below `anti_stall_rpm` |
| `creep_torque_nm` | real (N·m) | `40` | `>= 0` | dual_clutch: clutch capacity kept at idle in D/R without throttle (creep); an automatic creeps through its converter |
| `cvt` | object, see below | **required** on a cvt gearbox | rejected on any other (`"... the automatic_tcu's 'cvt' block is for a cvt gearbox only"`); a cvt without it: `"... a cvt gearbox needs the automatic_tcu's 'cvt' block (its ratio law)"` | the variator's ratio law |

**`cvt` ratio law** (CVT plan C1, `CvtTcuDesc`; unknown keys rejected): `target_rpm_vs_throttle` (Curve1D, **required**,
throttle x within [0, 1], every rpm `> 0`), `sport_target_rpm_vs_throttle` (optional, same rules; empty = the D curve),
`min_rpm_vs_output_rpm` (optional, x = gearbox output rpm `>= 0`), `stepped_sport` (bool, default `true`). Continuous
mode: the TCU aims the input (turbine of a converter CVT, else the engine) at
`n_t = max(target_rpm_vs_throttle(command), min_rpm_vs_output_rpm(output rpm))`, i.e. the ratio `n_t / output rpm`
clamped to `[ratio_high, ratio_low]` (`ratio_low` at a standstill), moved at most `ratio_rate_per_s` in ln r. S
(`transmission.mode >= 0.5`): with `stepped_sport` the variator runs through its virtual `steps` on the
`sport_schedule`'s shift lines with `min_time_in_gear_s` dwell (the step nearest the current ratio first), moving at
`step_rate_per_s`; without it the S target curve. `shift_up_count` / `shift_down_count` always select the steps
(manual mode with its timeout and forced up/down shifts, as for a stepped box).

Snapshot (never hashed): `PowertrainSnapshot::transmission_kind` (0 manual/sequential, 1 automatic, 2 dual_clutch, 3 cvt;
cvt also `cvt_ratio`, `cvt_step` (0 = continuous), `cvt_belt_capacity_nm`, `cvt_clamp_force_n`, `cvt_belt_slip_rad_s`),
`transmission_selector` (-1 R, 0 N/P, k > 0 D limited to gear k), `transmission_manual_mode`, `transmission_sport_mode`,
`converter_speed_ratio`, `lockup_engagement`; `gear` is the gear held. TCU state (phase, timers, preselect candidate) is hashed only for a vehicle with an automatic_tcu,
so every manual car's hash is unchanged.

Limits (A1): a dual_clutch kickdown steps one gear per shift; the dual_clutch launch rpm is the clutch equilibrium
(capacity x engagement = engine torque, about 2200 rpm for the 350 N·m test box), not a target (the assist unit's launch control sets
one, see electronics.md "DCT launch control"); all shipped data [AS].

### The `traction_control` block

Top-level optional object (`ps::vehicle::TractionControlDesc`, `core/src/io/vehicle_io.cpp`); the car's own traction
controller and, with an assist unit, the definition of TC mode **On** (code 1). Uses actual physics contact velocities,
hub speeds and loaded contact state. Unknown keys throw `"unknown traction_control field: <key>"`.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `enabled` | bool | `false` | default of the `assist.traction_control` channel when no unit or drive mode sets one (no shipped vehicle file carries the block, so `false` applies; `car_hyper*` get their default from the Racing unit / drive mode) |
| `fitted` | bool | `true` | `false`: the car has no TC, the channel is ignored |
| `target_slip` | real | `0.08` | positive-slip target; must be `< 1` |
| `reference_floor_m_s` | real | `2.0` | speed floor in the slip denominator |
| `kp`, `ki_per_s` | real | `2.0`, `6.0` | proportional cut gain / excess-slip reduction rate |
| `attack_rate_per_s`, `recovery_rate_per_s` | real | `20.0`, `8.0` | slew rates of the torque fraction (down / up) |

All numeric keys must be finite and `> 0` (`"traction_control: parameters must be finite and positive"`,
`"traction_control: target_slip must be below 1"`). All defaults `[AS]`. The per-mode extras (`min_fraction`,
`ignition_cut`) exist only in an assist unit's `mode_params`; the block itself cannot set them. The assist unit's
Relaxed / Sport / Drift modes override these keys per mode.

Every substep, TC compares each loaded driven wheel's peripheral speed with the chassis velocity at that contact
patch, projected along its physical rolling direction. Slip is the positive excess speed in the direction of travel,
divided by `max(abs(contact_speed), reference_floor_m_s)`. This handles reverse and cornering without estimating speed.
Unloaded or airborne corners do not contribute. If all driven contacts are airborne/unloaded, TC releases its cut and
allows wheel acceleration. Coasting, braking and switching TC off also clear the cut. Landing resumes regulation from
current contact state; there is no retained speed-estimator or integral error.

Excess slip lowers the bounded torque ceiling using `kp` and `ki_per_s`, limited by `attack_rate_per_s` and the mode's
`min_fraction`. Below target, the ceiling recovers at up to `recovery_rate_per_s`; no-spin recovery from zero takes at
most its reciprocal. The legacy snapshot field `traction_estimated_speed_m_s` now reports actual chassis longitudinal
velocity (the name is retained for consumers); per-wheel regulation uses each contact patch's own velocity.

## Control channels a vehicle reads

Not part of the JSON format, but relevant context for anyone editing a
vehicle file and then driving it from a scenario's `control_events` (see
`scenario.md`): `World::create_vehicle` resolves these channel names once,
at vehicle-creation time (`core/src/world/world.cpp`) — `"steer"`,
`"throttle"`, `"brake"`, `"handbrake"`, `"clutch"`, `"shift_up_count"`,
`"shift_down_count"`, `"gear_request"`, `"ignition"`, `"starter"`,
`"assist.auto_clutch"`, `"assist.auto_blip"`, `"assist.auto_shift"`, `"assist.traction_control"`. The
first vehicle created in a `World` reads the bare names; every subsequent
vehicle reads a `"v<index>."`-prefixed version (e.g. `"v1.steer"`).
The three `assist.auto_*` channels (> 0.5 means on) are the runtime on/off
switches for the `manual_tcu` driver aids; each one's value when the
vehicle is created is its controller block's `enabled` flag (0 if the
vehicle has no `manual_tcu`, or if the block is `fitted: false`).

**Assist channels (assists I1-I7).** `assist.traction_control` is registered on every car: codes 0 Off, 1 On (the car's
own `traction_control` block), 2 Relaxed, 3 Sport, 4 Drift (`value > 0.5 ? clamp(round(value), 1, 4) : 0`), default = the
default drive mode's `traction_control_default` (a car with the `drive_modes` part), else the assist unit's `default_mode`,
else the block's `enabled`, 0 when the block is not fitted. Registered **only** on a car that has the corresponding part:
`assist.launch_control` (a P8 `launch_control` part, whose `channel` key may rename it, or an assist unit's
`launch_control` block; default 0 or the block's `default_enabled`, or the default drive mode's `launch_control_default`), `assist.aero_control` (an assist unit's `aero_control`
block AND a car aero surface of that name; default the block's `default_enabled`; on at `>= 0.5`), `assist.drive_mode` (the
`drive_modes` part; 0 eco, 1 normal, 2 sport; default the part's `default_mode`). A N2O kit adds its controller's arm
channel (shipped: `"nitrous_arm"`). On/off is a strict `> 0.5` for all of these except `assist.aero_control`. The one
page that lists every channel with its precedence rules and the snapshot bits is
[electronics.md "Assists overview"](electronics.md#assists-overview).

**Component-declared channels (I4 C3):** beyond the fixed list above, each
powertrain component that needs one declares its own channel name and
default value (`Powertrain::declare_control_channels`, resolved through the
same `channel_prefix` convention) — a `differential`'s `lock.channel`
(default `"diff_lock"`, default value `0`), a `transfer_case`'s `channel`
(default `"awd"`, `0`), a `park_lock`'s `channel` (default `"park_lock"`,
`0`), a `pto`'s `channel`/`speed_channel` (default `"pto"`/`"pto_speed"`,
`0`/`540.0`), an engine `governor`'s `channel` (default `"hand_throttle"`,
`0`), and a `load_dyno`'s optional `scale_channel` (no default name — empty
means no channel is declared at all, and the load scale is fixed at `1.0`).
See each component's own field table above for its exact channel field and
default. `data/vehicles/tractor.json` declares `diff_lock`, `awd`,
`park_lock`, `pto`, `pto_speed` and `hand_throttle`; `car_sedan.json` and its
variants declare none (no component that declares a channel).

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
existed. A vehicle may have any number of `differential` components (see
["Components"](#components) above); only the **snapshot/debug** view of
them is capped — `PowertrainSnapshot::diffs`/`PowertrainDebug::
diff_transfer_torque` hold at most `kMaxPowertrainDiffs` (4,
`ps/drivetrain/powertrain_state.h`) entries, in `powertrain.components[]`
order, so a 5th-or-later differential's `bias`/`locked`/
`diff_transfer_torque` reads are simply not exposed there (silently
truncated, not an error). This is a snapshot/debug-only addition — no new
hashed simulation state, no change to `World::state_hash()`.

## Telemetry channel a fuel tank exposes

PHYS-033 P4 I7a C6: a vehicle with a `fuel_tank` adds
`vehicle<i>.fuel.tank_kg` (the tank content in kg) right after its
`vehicle<i>.fuel.{flow_g_s,energy_in,heat}` channels. A vehicle without a
tank registers no such channel, so every shipped vehicle's CSV header is
unchanged.

## Contents mass

Optional top-level block. With it the chassis body carries the fuel tank and N2O bottle contents on top of the
mass it was created with (its dry mass).

```json
"contents_mass": { "enabled": true, "update_threshold_kg": 0.1, "source": "..." }
```

| key | type | default | meaning |
|---|---|---|---|
| `enabled` | bool | false | off: the vehicle behaves and hashes exactly as without the block |
| `update_threshold_kg` | number in (0, 1000) | 0.1 | the body mass is updated at a tick boundary once the contents moved this much since the last update |
| `source` | string | - | free text, ignored |

- The initial fill (`initial_kg` of the tank, the bottle fill) is applied when the vehicle is created, so the
  first tick already runs with it.
- During the drive the update is semi-live: at the start of `World::step`, before the pipeline, whenever
  |contents - applied| >= the threshold. A refuel through `World::set_fuel_tank_mass` is picked up the same way.
- The inertia is rescaled with the mass (uniform density: the chassis shape's own inertia at the new mass). The
  centre of mass does not move.
- The applied contents mass is folded into the state hash only for vehicles with the block enabled.
- `World::chassis_contents_mass(id)` / `World::chassis_dry_mass(id)` read it back (0 without the block).
- Unknown keys and an out-of-range threshold are rejected (`contents_mass: ...`).

## Aero

`aero` is read by `core/src/io/aero_io.cpp` (`read_aero`) and validated by
`ps::aero::validate` (`core/src/aero/aerodynamics.cpp`, shared with
`World::create_vehicle`). SI throughout; all `*_local` points are relative to the
chassis centre of mass in the vehicle's local axes (+x forward, +y left, +z up);
[`AERODYNAMICS.md`](../../AERODYNAMICS.md) describes the model (quasi-steady
coefficients, authored stall blend, finite drafting wakes - no measured data).
An unknown key at ANY level throws `std::invalid_argument("unknown aero key: <key>")`;
entries that are not objects throw `"aero entries must be objects"`; a vector that is not
three numbers `"aero <key> must be a 3-vector"`; a wrong JSON type
(`"surfaces"` not an array: `"aero surfaces must be an array"`) likewise. Validation
failures throw `std::invalid_argument` with the messages quoted below.

| Field | Type | Default | Rule | Meaning |
|---|---|---|---|---|
| `drag_area_m2` | number | 0 | finite, >= 0 (`"aero drag_area_m2 must be finite and nonnegative"`) | LEGACY isotropic `Cd*A` already combined: `F = -0.5*rho*CdA*|v|*v` at the CoM. Positive together with `body` (non-default position, moments, side force, reference area or drag_area_xyz_m2) or any `surfaces` throws `"aero legacy drag_area_m2 cannot coexist with explicit body/surfaces"` |
| `body` | object | none | below | explicit body drag, side force and intrinsic moments |
| `surfaces` | array | none | at most 32 | lifting surfaces (wings, splitters, flaps) |
| `fans` | array | none | at most 8 | suction fans (`"aero surface/fan count exceeds limits"`) |
| `wake` | object | `length_m` 0 (no wake) | below | the drafting wake this vehicle leaves |

**`body`**: `position_local` [x,y,z] (0), `drag_area_xyz_m2` [x,y,z] CdA per axis (0; >= 0),
`moment_coeff` [roll,pitch,yaw] (0), `angular_damping` [x,y,z] (0.1; >= 0),
`reference_area_m2` (0; >= 0), `reference_length_m` (1; > 0), `side_force_slope` (0; >= 0).
Violations: `"aero body vectors must be finite"`, `"aero drag and damping must be nonnegative"`,
`"invalid aero body dimensions"`.

**`surfaces[]`**: `name` (required, non-empty, unique: `"duplicate aero surface name"`),
`position_local`, `chord_local` ([1,0,0]) and `normal_local` ([0,0,1]) (orthonormal unit vectors, else
`"aero chord/normal must be orthonormal unit vectors"`), `area_m2` (1; > 0), `cl0` (0), `lift_slope_per_rad` (4.5; >= 0),
`stall_angle_rad` (0.3; in (0, 1.5)), `cd0` (0.02), `induced_drag` (0.08), `stalled_drag` (1.2) (all >= 0),
`rest_angle_rad` (0), `min_offset_rad` (0; <= 0), `max_offset_rad` (0; >= 0), `rate_rad_s` (1; > 0) (actuator travel:
command -1..1 goes toward min/max offset), `max_lift_m` (0; >= 0), optional `ground`: `{probe_local` [0,0,-1] unit,
`range_m` 0 (>= 0), `gain` 0 (in [0, 3])`}` (multiplier `1 + gain*(1-clearance/range)^2`).
Messages: `"invalid aero surface name/vectors"`, `"invalid aero lift parameters"`, `"invalid aero drag coefficients"`,
`"aero max_lift_m must be nonnegative"`, `"invalid aero actuator angles/rate"`, `"invalid aero ground effect"`.

**`fans[]`**: `name` (required, unique), `position_local`, `direction_local` ([0,0,-1], unit), `area_m2` (1; > 0),
`pressure_pa` (1000; >= 0), `seal_height_m` (0.1; > 0), `probe_range_m` (1; > 0, >= seal height),
`max_power_w` (10000; > 0), `rate_per_s` (1; > 0). Message `"invalid aero fan pressure/geometry/power/rate"`.
Fan energy is prepaid: the reservoir starts EMPTY and is credited by the caller
(`World::credit_aero_fan_energy`).

**`wake`**: `length_m` (0; >= 0), `radius_m` (1; > 0), `expansion` (0.12; >= 0), `deficit` (0.25; in [0, 0.75]),
`turbulence` (0.1; in [0, 1]); messages `"invalid aero wake geometry"`, `"invalid aero wake deficit/turbulence"`.

Runtime control (not in the file): `World::set_aero_surface_command` (-1..1),
`World::set_aero_fan_command` (0..1), `World::set_aero_environment` (air density, wind),
`WorldConfig::wind_world`. Example: `data/vehicles/car_hyper.json`.

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

## Engine component slots (engine garage P2)

An `engine` component (with a `ref` or inline) may name part files that override the engine file's own slots for this
vehicle. Every path is resolved relative to the VEHICLE file; the full resolution order and the part formats are in
[engine.md](engine.md#part-slots-extends-and-the-user-layer-engine-garage-p2).

| Key | Value | Replaces the engine's |
|---|---|---|
| `ecu` | path to a [`physics_sim.ecu/1`](ecu.md) file | `default_ecu` / inline ECU fields (merged over them) |
| `cam` | path to a [`physics_sim.cam/1`](cam.md) file | `default_cam` / inline lifts |
| `forced_induction` | path to a turbo configuration, or `"none"` | `default_forced_induction` (was `turbo_configuration`, see above) |
| `installation` | path to an installation | `cycle.default_installation` (pipe engines only, unchanged since ps_audio A4) |
| `thermal` | path to an engine thermal file | `default_thermal` (unchanged since engine thermal T1) |

With a `ref`, a component may carry exactly `id`, `type`, `ref` and these five keys. On an inline engine an `ecu`
OBJECT is still the engine's own inline `ecu` block (lambda/spark tables), an `ecu` STRING is the slot. A slot value
must be a non-empty string (`"none"` only for `forced_induction`); the old `turbo_configuration` key is rejected with
the rename message. Absent = bit-identical to before.

## User layer (engine garage P2)

`ps::io::load_vehicle_json(path, const VehicleLoadOptions&)` applies the player's part choices on top of the vehicle
file — the last layer, so the user wins. The older overloads `load_vehicle_json(path)` and
`load_vehicle_json(path, EngineMapOptions)` delegate to it with no overrides (bit-identical).

```cpp
struct PartOverride { std::string ref; std::map<std::string, std::string> slots; };
struct VehicleLoadOptions { EngineMapOptions maps; std::map<std::string, PartOverride> part_overrides; };
```

`part_overrides` is keyed by powertrain component `id`. For that component:

- `ref` (empty = keep) replaces its part file — legal for `engine`, `gearbox`, `n2o_bottle` and `nitrous_kit`
  components. An inline component given a `ref` becomes `{id, type, ref}` (an inline engine keeps its string slots);
  the new file is then read exactly like a `ref` written in the vehicle file.
- `slots` set the component's slots by name: an engine's `ecu`, `cam`, `forced_induction` (path or `"none"`),
  `installation`, `thermal`; a nitrous kit's `controller`. A user slot replaces the vehicle file's value for that slot.

Paths resolve relative to the vehicle file (absolute paths as given), like the vehicle-level keys they replace — a
garage that writes its parts under a user directory passes absolute paths. Rejected: an id with no powertrain component
(`"part_overrides: no powertrain component with id 'x'"`), a component type without a part file (`"a 'clutch'
component has no part file to override"`), an unknown slot name for the type, an engine `ecu` slot on an inline engine
whose `ecu` block is inline.

The user layer only edits the component JSON before it is read, so a user choice equals writing the same keys into the
vehicle file: same descriptor, same `.psmaps`/turbo keys, same state hash.

**Electronics (engine garage P8).** `VehicleLoadOptions::electronics` (`std::map<std::string, std::string>`) is keyed by
a `controllers` entry id: a path to a `physics_sim.electronics/1` file replaces that entry's part or adds a new entry
(grafting launch control onto a car that has none), `"none"` removes the entry. Details and rejections:
[electronics.md](electronics.md). The same map attaches an `assist_unit` (id of your choice, e.g. `"assist"`).

**Injected tyres.** `VehicleLoadOptions::tyres` (`std::map<std::string, vehicle::TyreDesc>`) is keyed by WHEEL NAME
(`WheelDesc::name`): a game that builds its tyres in memory (compound + size) passes the descs instead of writing tyre
files. An injected desc REPLACES the wheel's own `"tyre"` (path or inline object) when both exist — the user wins, like
`part_overrides` — and the file tyre is then not read at all. It gets the same validation as a file tyre (one function,
`validate_tyre_desc`: non-empty `source`, `rolling_resistance_v_eps` > 0, `low_speed_damping_ratio` in [0, 2],
`low_speed_damping_vx` > 0). A key that names no wheel is rejected (`"VehicleLoadOptions::tyres: no wheel named 'x'"`).
Unknown keys inside a wheel object (e.g. a game's own `"tyre_spec"`) are ignored by the loader, as before.
