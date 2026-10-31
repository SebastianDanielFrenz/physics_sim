# Data file formats — index

This is a human-readable reference for every JSON data format `physics_sim`
loads: vehicles, engines, gearboxes, tyres, the surface table and scenarios.

**Source of truth.** These pages describe what the *loaders* actually do,
not what the example files under `data/` happen to contain. The loaders are:

| Format | Loader | Public descriptor headers |
|---|---|---|
| vehicle | `core/src/io/vehicle_io.cpp` (`load_vehicle_json`) | `core/include/ps/vehicle/vehicle_desc.h`, `ps/vehicle/tyre_desc.h`, `ps/vehicle/curve1d.h`, `ps/drivetrain/{powertrain_desc.h,brake_system_desc.h,controller_desc.h}` |
| engine | `core/src/io/vehicle_io.cpp` (`read_engine_fields`, reached from `load_vehicle_json`) | `ps/drivetrain/powertrain_desc.h` (`TorqueMapEngineDesc`) |
| gearbox | `core/src/io/vehicle_io.cpp` (`read_gearbox_fields`) | `ps/drivetrain/powertrain_desc.h` (`GearboxDesc`, `SynchroDesc`) |
| tyre | `core/src/io/vehicle_io.cpp` (`load_tyre_json`, `read_tyre_object`) | `ps/vehicle/tyre_desc.h` (`TyreDesc`) |
| surfaces | `core/src/io/surface_table.cpp` (`SurfaceTable::SurfaceTable`) | `ps/io/surface_table.h` |
| scenario | `core/src/io/scenario.cpp` (`load_scenario`, `build_scenario`) | `ps/io/scenario.h` |
| fuel | `core/src/io/fuel_io.cpp` (`load_fuel_json`) | `ps/drivetrain/fuel_desc.h` (`FuelDesc`) |
| oxidizer | `core/src/io/nitrous_io.cpp` (`load_oxidizer_json`) | `ps/drivetrain/oxidizer_desc.h` (`OxidizerDesc`) |
| engine_thermal / cooling_system | `core/src/io/engine_thermal_io.cpp` (`load_engine_thermal_json`, `load_cooling_system_json`; the vehicle keys `cooling_system`/`engine_thermal_start` and the engine key `default_thermal` via `load_vehicle_json`) | `ps/drivetrain/engine_thermal_desc.h` (`EngineThermalDesc`, `CoolingSystemDesc`) |
| n2o_bottle / nitrous_kit / nitrous_controller | `core/src/io/nitrous_io.cpp` (`load_n2o_bottle_json`, `load_nitrous_kit_json`, `load_nitrous_controller_json`; vehicle components via `load_vehicle_json`) | `ps/drivetrain/nitrous_desc.h` (`N2oBottleDesc`, `NitrousKitDesc`, `NitrousControllerDesc`) |
| electronics (engine garage P8) | `core/src/io/electronics_io.cpp` (`load_electronics_json`; `controllers` entries of type `electronics` and `VehicleLoadOptions::electronics` via `load_vehicle_json`) | `ps/drivetrain/controller_desc.h` (`LaunchControlDesc`; fitment sets the `fitted` flags) ([electronics.md](electronics.md)) |
| hitch_class (attachment A1) | `core/src/io/hitch_io.cpp` (`load_hitch_class_json`; the vehicle keys `hitch_points` and `tow_brake` are read by `load_vehicle_json`) | `ps/machine/hitch_class.h`, `ps/machine/hitch_point.h`, `ps/machine/tow_brake.h` |
| machine (farm M1) | `core/src/io/machine_io.cpp` (`load_machine_json`, `load_tool_power_json`; the vehicle key `machine` is read by `load_vehicle_json`) | `ps/machine/tool.h` |
| ecu / cam (engine garage P2) | `core/src/io/engine_parts.cpp` (`assemble_engine`: merged into the engine JSON before `read_engine_fields`; reached from `load_vehicle_json`, `load_engine_json`, `load_engine_json_from_string`) | none of their own - the merged fields fill `TorqueMapEngineDesc` / `SimulatedEngineDesc` ([ecu.md](ecu.md), [cam.md](cam.md)) |

The powertrain **graph itself** (which ports may connect, what gets
rejected) is additionally validated by `core/src/drivetrain/powertrain.cpp`
(`Powertrain::Powertrain`), which runs *after* `vehicle_io.cpp` has parsed
the JSON into a `PowertrainDesc`. A vehicle file can be syntactically valid
JSON, pass every `vehicle_io.cpp` check, and still be rejected by
`powertrain.cpp` (e.g. an unconnected port, or a duplicate component id) —
see [`vehicle.md`](vehicle.md)'s "Powertrain graph" section.

Where an example file under `data/` disagrees with what's written here, the
code wins; any such discrepancy found while writing these pages is listed at
the end of this file under "Discrepancies found".

## Pages

- [`vehicle.md`](vehicle.md) — `physics_sim.vehicle/2`, `data/vehicles/*.json`. Wheels, suspension, steering, brakes, the powertrain graph, controllers, aero.
- [`engine.md`](engine.md) — `physics_sim.engine/1`, `data/engines/*.json` (also embeddable inline in a vehicle file's `powertrain.components`).
- [`hitch.md`](hitch.md) — `physics_sim.hitch_class/1`, `data/hitch_classes/*.json`. What a hitch holds and leaves free, how a latch runs.
- [`linkage.md`](linkage.md) — the `linkage` block of a towing hitch point. The three-point links, the lift's position, draft and float modes, the PTO shaft across a hitch.
- [`machine.md`](machine.md) — `physics_sim.machine/1` and `physics_sim.tool_power/1`, `data/machines/*.json`. Work tools: the draft law, soil, PTO, work quads, the shop's power figure.
- [`gearbox.md`](gearbox.md) — `physics_sim.gearbox/1`, `data/gearboxes/*.json` (also embeddable inline).
- [`tyre.md`](tyre.md) — `physics_sim.tyre/1`, `data/tyres/*.json` (also embeddable inline in a wheel, or a `.tir` file).
- [`surfaces.md`](surfaces.md) — `physics_sim.surfaces/1`, `data/surfaces/surfaces.json`.
- [`scenario.md`](scenario.md) — `physics_sim.scenario/1`, `data/scenarios/*.json`.
- [`fuel.md`](fuel.md) — `physics_sim.fuel/1`, `data/fuels/*.json`. Lower heating value, stoichiometric AFR, density — cited fuel properties, no combustion model.
- [`engine_thermal.md`](engine_thermal.md) — `physics_sim.engine_thermal/1`, `data/engine_thermal/*.json`. The engine side of the lumped engine thermal model: capacities, heat split, oil viscosity friction, thermostat.
- [`cooling_system.md`](cooling_system.md) — `physics_sim.cooling_system/1`, `data/cooling_systems/*.json`. The vehicle side: radiator, fan, gauge, warnings; naming one in a vehicle opts it in.
- [`electronics.md`](electronics.md) — `physics_sim.electronics/1`, `data/electronics/*.json`. Assist fitment (`fitted` flags) and launch control (two-step); parts grafted onto a car through the user layer.
- [`oxidizer.md`](oxidizer.md) — `physics_sim.oxidizer/1`, `data/oxidizers/*.json`. NIST nitrous-oxide saturation and gas properties, the HEM/Dyer jet-flow tables built from them.
- [`nitrous.md`](nitrous.md) � `physics_sim.n2o_bottle/1`, `physics_sim.nitrous_kit/1`, `physics_sim.nitrous_controller/1` (`data/n2o_bottles/`, `data/nitrous_kits/`, `data/nitrous_controllers/`): the slot-in nitrous hardware parts, their overrides, control channels and the vehicle wiring.
- [`climate.md`](climate.md) — `physics_sim.climate/1`, `data/climates/*.json`. Location, sun, calendar and seasons (W1); per-season weather regimes, transitions, lags, precipitation phase, humidity, diurnal temperature, lightning (W3).
- [`weather_preset.md`](weather_preset.md) — `physics_sim.weather_preset/1`, `data/weather_presets/*.json`. One fixed-value regime: the base in Preset mode or an override target.
- [`surface_conditions.md`](surface_conditions.md) — `physics_sim.surface_conditions/1`, `data/surface_conditions/*.json`. Per surface class: kind, albedo, emissivity and a two-layer thermal column (W5); the road surface temperature model.
- [`weather_classes.md`](weather_classes.md) — `physics_sim.weather_classes/1`, `data/weather_classes/*.json`. WMO intensity, DWD Starkregen/gust warning, Beaufort and TORRO thresholds with ge/gt boundary rules.

## Format versioning

Every standalone file (vehicle, tyre, engine, gearbox, surfaces, scenario)
carries a required top-level `"format"` string, `"physics_sim.<kind>/<major>"`.
The loader checks this **exactly once**, immediately after parsing, and
before reading any other field:

- Missing `"format"`, or not a string: `"<path>: missing required 'format' field (expected \"physics_sim.<kind>/<major>\")"`.
- Wrong value: `"<path>: unsupported 'format' \"<actual>\" (expected \"physics_sim.<kind>/<major>\")"`.

There is **no legacy/compatibility support**: an old major version is a hard
error, never a silent default or an auto-converted read (an explicit project
policy — see `core/src/io/vehicle_io.cpp`'s `require_format`/
`require_vehicle_format` comments, "PHYS-033 owner ruling Q7"). The vehicle
loader gives one extra hint for the one superseded version this project has
actually shipped: a `physics_sim.vehicle/1` file's error message is extended
with a pointer to `data/vehicles/car_sedan.json` for the current `/2` layout.

An object *embedded inline* inside another file (an inline tyre object
inside a wheel, an inline `clutch`/`gear_pair`/`differential` component
inside a vehicle's `powertrain.components`) does **not** carry its own
`"format"` field — it has no independent version and is implicitly the same
format major as its enclosing file. Only a *standalone* file referenced by
path (`"tyre": "../tyres/foo.json"`, or a component's `"ref"`) is
format-checked on its own.

## Units and the "source" field

SI units throughout (metres, kilograms, seconds, radians, Newtons,
Newton-metres) unless a field comment says otherwise (rpm shows up in a
handful of engine/gearbox-adjacent fields because that's how real engine
data sheets are written; the field name always says `_rpm`). Angles are
radians; wherever a data file's comment gives a value "in degrees" for
readability, the JSON number itself is still radians.

Every vehicle, tyre, fuel, engine and gearbox file requires a `"source"`
string (vehicle/tyre/fuel: enforced by the loader, `"'source' is required"`;
engine/gearbox: not currently loader-enforced, see "Discrepancies found"
below, but every shipped file has one and new ones should too). The project
convention
(PLAN.md section 8, restated in every `source` field currently shipped) is:

- Cite a **class** of real hardware ("a plausible 2.0 L naturally aspirated
  I4", "the mid-size tractor class"), not a specific commercial product,
  *unless the numbers have actually been verified against a cited,
  checkable reference* (a textbook table, a published spec sheet).
- Never embed a proprietary `.tir` tyre file's actual coefficients — those
  are commercially sensitive. A `.tir` file loaded via `ps::vehicle::
  parse_tir_file` gets its `source` filled with `"Parsed from <path>"` if
  the file itself supplies none (a `.tir` has no standard "cite your
  source" key).
- Say which formula/table produced a derived number (e.g. "fmep ~= 0.97 +
  0.15(N/1000) + 0.05(N/1000)^2 bar, Heywood ch. 13") so a reviewer can
  recompute it, not just trust it.
- Flag "verify before citing a specific engine/tyre" wherever the numbers
  were chosen to sit inside a plausible published range rather than copied
  from one confirmed source.

## `Curve1D` — the shared curve format

Every curve in every format (suspension spring/damper tables, camber/toe
vs. travel, engine WOT/motoring torque vs. rpm, clutch engagement vs. pedal)
uses the same on-disk shape and the same interpolation rule
(`ps/vehicle/curve1d.h`, `Curve1D`):

```json
{ "x": [0, 10, 20], "y": [0, 100, 250] }
```

- `x`, `y`: arrays of numbers, same length. The loader (`read_curve`,
  `vehicle_io.cpp`) throws `"curve 'x'/'y' arrays must be the same length"`
  if they differ, and `"curve object needs 'x' and 'y' arrays"` if either
  key is missing.
- Many (not all — see each field's own doc) curves additionally require `x`
  to be **strictly ascending** (`require_ascending`, `vehicle_io.cpp`):
  `"<field>: x values must be strictly ascending"` if not, and `"<field>
  must be non-empty"` if `x` is empty. Where a field doesn't call
  `require_ascending`, the loader accepts any order but `Curve1D::evaluate`
  (below) will not make sense of a non-ascending `x`.
- **Interpolation**: piecewise-linear between consecutive points.
- **Extrapolation**: *clamped*, not extrapolated — `evaluate(x)` for `x <=
  x.front()` returns `y.front()`, and for `x >= x.back()` returns
  `y.back()`. A curve that should represent "zero past this point" needs an
  explicit `(x, 0)` point at the edge; it does not happen automatically.
- An empty curve (`x` empty) evaluates to `0.0` everywhere. This is the
  default for every *optional* curve field (e.g. `SuspensionDesc::
  spring_curve`) — "empty curve" is how a field is *left unset*, which then
  falls back to a separate scalar field (e.g. `spring_rate`) instead.

## Cross-file references and path resolution

A vehicle file references other files by relative path:

- Each wheel's `"tyre"` field, if a string, is a path to a standalone tyre
  JSON file (`physics_sim.tyre/1`) **or** a standard MF `.tir` file
  (detected by a literal `.tir` suffix on the resolved path) — see
  `tyre.md`.
- A powertrain `"engine"`/`"gearbox"` component may use `{"ref": "<path>"}`
  to point at a standalone engine/gearbox JSON file instead of inlining its
  fields — see `engine.md`/`gearbox.md`.

**Resolution rule** (`resolve_path`, `vehicle_io.cpp`): if the given path is
already absolute, it is used as-is; otherwise it is resolved relative to the
*referencing file's own directory* (`std::filesystem::path(vehicle_path).
parent_path()`), never relative to the current working directory or to
`data/` itself. This is why every shipped vehicle file writes tyre/engine/
gearbox paths as `"../tyres/..."`, `"../engines/..."`, `"../gearboxes/..."`
— they live in `data/vehicles/`, one level below `data/`. The reference-car
variants that are no longer shipped (owner 2026-10-13: `data/vehicles/` keeps only `car_sedan`, `car_sedan_sim`,
`car_hyper`, `car_hyper_n2o`, `tractor`) are test fixtures in `tests/support/fixtures/vehicles/` and write the same
paths as `"../../../../data/tyres/..."` etc. A scenario file
resolves `vehicle_json_path`, `terrain.spec_path` and `surface_table_path`
the same way, relative to the scenario file's own directory
(`core/src/io/scenario.cpp`).

A referenced standalone file **is** format-checked on its own (its own
`"format"` field must match, e.g. `physics_sim.tyre/1`, `physics_sim.
engine/1`, `physics_sim.gearbox/1`) — see "Format versioning" above.

## Coordinate conventions

Core frame is **ISO 8855**: **X forward, Y left, Z up**, right-handed
(PLAN.md decision D4). This applies to every body pose, every vehicle-local
attachment point, and every axis vector in these formats:

- A wheel's `attachment_local` (vehicle.md) is the wheel centre's position
  in the **chassis body's local frame**, at zero suspension travel (full
  extension). Front wheels have larger (more positive) X than rear wheels;
  the left side has larger (more positive) Y than the right side.
- A suspension `axis` is chassis-local and points from full extension
  toward full bump — normally `[0, 0, -1]` (straight down).
- A powertrain component's `*_axis_local` fields (engine `crank_axis_local`,
  clutch/gearbox/gear_pair/differential axis fields) are **chassis-local**
  unit vectors giving the direction of positive angular velocity for that
  shaft, used only for the optional rotor-reaction torque on the chassis
  (`PowertrainDesc::rotor_reaction`); the zero vector (the default for all
  of them) means "no rotor reaction for this port" and is always valid.
- Only the Godot adapter performs the one basis change to Godot's own
  Y-up/-Z-forward convention; nothing in `core/` or any data file ever
  does.

Positions/orientations elsewhere (scenario body `position`/`orientation`,
articulation link frames) are likewise ISO 8855/world-frame, in metres and
an `[x, y, z, w]` quaternion.

## The powertrain graph, in brief

A vehicle's `powertrain` block describes a small graph of rotating shafts:
named **components** (engine, clutch, gearbox, gear_pair, differential,
transfer_case, park_lock, pto, load_dyno), each exposing one or more
**ports**; **shafts**, each a list of port references that are rigidly
joined into one node; and each wheel implicitly exposes one more port,
`wheel:<name>`. Full details, every component's port names, and the exact
rules the graph builder (`core/src/drivetrain/powertrain.cpp`) enforces
(every non-wheel port except `pto.out` must be connected exactly once; ids
must be unique; at most one engine/clutch/gearbox/transfer_case/park_lock/
pto, but any number of differential/gear_pair/load_dyno components, I4 C4)
are in [`vehicle.md`](vehicle.md#powertrain-graph).

## Discrepancies found

While writing these pages, the following mismatches between the code (source
of truth) and the shipped example files/comments were found. None of them
were changed — see the task that produced this documentation.

1. **`engine`/`gearbox` `"source"` is not loader-enforced.** Unlike vehicle
   and tyre files (`load_vehicle_json`/`read_tyre_object` both throw if
   `source` is empty), `read_engine_fields`/`read_gearbox_fields` never read
   or validate a `"source"` field at all — it is pure convention (every
   shipped `data/engines/*.json` and `data/gearboxes/*.json` happens to
   carry one, described under the file's `source`/top-level free-form keys,
   which the loader simply ignores rather than validates).
2. **`WheelDesc::arb_partner` and `WheelDesc.is_front`'s comment overstate
   loader validation.** `arb_partner` (an index into `VehicleDesc::wheels`)
   is never range-checked by the loader or by `vehicle_instance.cpp` — an
   out-of-range value indexes `wheels_[...]` directly (`vehicle_instance.
   cpp`'s `partner_compression` read) with no bounds check. Likewise,
   `WheelDesc::is_front`'s header comment says it "picks brake_system's
   corner match", but `brake_system.corners[].wheel`/`handbrake.wheels`
   actually match wheels **by name string** (`powertrain.cpp`), not by
   `is_front`; `is_front` only affects Ackermann-axle averaging and the
   rear-steer sign (`vehicle_instance.cpp`).
3. **`data/surfaces/surfaces.json`'s `"fields"` block is documentation-only
   free text**, not schema read by `SurfaceTable` — only `"surfaces"` (and
   `"format"`) are read; `"description"`/`"fields"` are convention, ignored
   by the loader.
4. **`tyre.md` note on `.tir` files:** the project convention (PLAN.md
   section 8, restated in every tyre `source` field) is "never a proprietary
   `.tir` file" for anything *committed to this repo*; `parse_tir_file`
   itself has no such restriction and will happily parse a real `.tir` a
   user supplies locally — the rule is social/repo-hygiene, not enforced by
   the loader.
