# Scenario format — `physics_sim.scenario/1`

Files: `data/scenarios/*.json`. Loader: `ps::io::load_scenario` (parsing)
and `ps::io::build_scenario` (turns a parsed `Scenario` into a live
`World`) — both in `core/src/io/scenario.cpp`. Public descriptor:
`ps::io::Scenario` (`core/include/ps/io/scenario.h`). Consumed by
`tools/ps_run` (headless scenario runner) and `tools/sweep`.

A scenario describes a complete, self-contained simulation setup: world
config (tick rate, gravity, worker count), static/dynamic bodies, joints,
articulations, a terrain source, vehicles built on top of already-described
bodies, a shared surface table, and a timeline of control events.

## Top level

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required | — | must equal `"physics_sim.scenario/1"` | see README |
| `name` | string | optional | `"scenario"` | none | scenario name |
| `duration_s` | real (s) | optional | `1.0` | none | how long `ps_run` simulates |
| `tick_rate_hz` | real (Hz) | optional | `240.0` | none | `World`'s outer tick rate |
| `substep_rate_hz` | real (Hz) | optional | `960.0` | none | articulation/vehicle substep rate inside each tick |
| `gravity` | Vec3 (m/s²) | optional | `[0, 0, -9.81]` | 3-element array if present | world gravity vector |
| `job_workers` | int | optional | `0` | none | `0` = auto-detect worker count |
| `max_bodies` | size_t | optional | `4096` | none | backend body-capacity hint |
| `air_density` | real (kg/m³) | optional | `1.225` | none | used by vehicle aero drag |
| `telemetry_csv` | string | optional | `""` | none | CSV output filename; empty = `ps_run` decides (`"<name>.csv"`) |
| `bodies` | array of [body objects](#bodies) | optional | empty | see below | rigid bodies |
| `joints` | array of [joint objects](#joints) | optional | empty | see below | Jolt fixed/hinge joints between bodies |
| `articulations` | array of [articulation objects](#articulations) | optional | empty | see below | reduced-coordinate articulated chains |
| `terrain` | [terrain object](#terrain) | optional | none | see below | a procedural terrain source + streaming interest points |
| `vehicles` | array of [vehicle objects](#vehicles) | optional | empty | see below | vehicles built on an already-described chassis body |
| `control_events` | array of `{t, channel, value}` | optional | empty | see below | scripted `World::set_control` calls over time |
| `environment` | [environment object](#environment-weather-plan-w2) | optional | none | see below | clock + calendar + sun installed into the `World` (weather plan W2) |
| `surface_table_path` | string | optional | `""` | resolved relative to the scenario file's own directory if relative | shared surface-parameter table (see `surfaces.md`); empty = every surface reads `lambda_mu=1, crr=0` |

## Bodies

Each entry of `bodies[]`:

| Field | Type | Req./opt. | Default | Meaning |
|---|---|---|---|---|
| `name` | string | **required** | — | referenced by joints/vehicles/articulation... below |
| `shape` | [shape object](#shapes) | **required** | — | collision shape |
| `motion` | string | optional | `"dynamic"` | `"static"`, `"kinematic"`, or `"dynamic"`; anything else throws `"unknown body motion type '<s>' (expected static|kinematic|dynamic)"` |
| `mass` | real (kg) | optional | `1.0` | body mass (ignored for static/kinematic) |
| `position` | Vec3 (m) | optional | `[0,0,0]` | world position |
| `orientation` | Quat `[x,y,z,w]` | optional | identity | world orientation |
| `linear_velocity` / `angular_velocity` | Vec3 | optional | `[0,0,0]` | initial velocity |
| `friction` | real | optional | `0.5` | Coulomb friction coefficient |
| `restitution` | real | optional | `0.0` | bounciness |
| `linear_damping` / `angular_damping` | real | optional | `0.0` | velocity damping |
| `gravity_enabled` | bool | optional | `true` | whether gravity acts on this body |
| `allow_sleep` | bool | optional | `false` | whether Jolt may put this body to sleep |

### Shapes

`{"type": "box"|"sphere"|"capsule"|"cylinder", ...}`; any other value
throws `"unknown shape type '<type>' (expected box|sphere|capsule|
cylinder)"`.

| Type | Fields | Defaults |
|---|---|---|
| `box` | `half_extents` (Vec3, m) | `[0.5, 0.5, 0.5]` |
| `sphere` | `radius` (m) | `0.5` |
| `capsule` | `half_height`, `radius` (m) | `0.5`, `0.25` |
| `cylinder` | `half_height`, `radius` (m) | `0.5`, `0.3` |

## Joints

Each entry of `joints[]`:

| Field | Type | Req./opt. | Default | Meaning |
|---|---|---|---|---|
| `name` | string | required | — | joint name |
| `type` | string | optional | `"fixed"` | `"hinge"` or anything else = `"fixed"` (no error on an unrecognised string — it silently becomes `Fixed`) |
| `body_a` | string | **required** | — | must name an existing body, else `"joint '<name>' references unknown body_a '<name>'"` |
| `body_b` | string | optional | `""` | empty = anchored to the world; else must name an existing body, else `"joint '<name>' references unknown body_b '<name>'"` |
| `anchor_a` / `anchor_b` | Vec3 (m) | optional | `[0,0,0]` | joint anchor, local to each body |
| `axis_a` / `axis_b` | Vec3 | optional | `[0,0,1]` | hinge axis, local to each body |
| `friction_torque` | real (N·m) | optional | `0.0` | hinge friction |
| `limit_min` / `limit_max` | real (rad) | optional | `0.0` / `0.0` | hinge limits; **presence of either key** enables limits (`limits_enabled = true`) |

## Articulations

Each entry of `articulations[]` (`ps::articulation::ArticulationDesc`) — a
reduced-coordinate (Featherstone) chain, unrelated to the vehicle system.
Field-by-field detail is out of scope for this vehicle/engine-focused set
of docs (see `core/include/ps/articulation/articulation_desc.h` and
CLAUDE.md's "Articulation solver" section); in brief:

- `name` (required), `root` (`{floating, pose, linear_velocity,
  angular_velocity, mass, com, inertia_diag}`), `links[]` (`{parent,
  joint: "revolute"|"prismatic"|"fixed", joint_frame_in_parent, axis, mass,
  com, inertia_diag, joint_damping, limit_min, limit_max,
  velocity_actuator: {target_speed, max_force}, q0, qd0, contact}`),
  `loop_closures[]` (`{type: "point"|"hinge", link_a, link_b, anchor_a,
  anchor_b, axis_a, axis_b}`), `contact_strategy`
  (`"none"|"compliant"|"proxy_body"`, default `"none"`; an unrecognised
  string throws `"unknown contact_strategy '<s>' (expected
  none|compliant|proxy_body)"`).
- A link's `joint` field: an unrecognised string throws `"unknown
  articulation joint type '<s>' (expected revolute|prismatic|fixed)"`.
- A link's optional `contact` object (present = contact-enabled for this
  link): `{shape, shape_offset, material: {stiffness,
  stiffness_exponent, damping, friction, friction_regularization_speed,
  proxy_mass_scale}}`.

## Terrain

`"terrain"` (`ScenarioTerrain`), optional:

| Field | Type | Req./opt. | Default | Meaning |
|---|---|---|---|---|
| `spec_path` | string | **required** | — | path to a `terrain.py`-shaped procedural terrain spec, resolved relative to the scenario file's own directory if relative |
| `samples_per_tile` | uint32 | optional | `64` | terrain source grid resolution (must be even, per `HeightfieldShape`'s own requirement) |
| `max_resident_tiles` | uint32 | optional | `TerrainConfig`'s own default | tile-pool size |
| `max_tile_fills_per_tick` | uint32 | optional | `TerrainConfig`'s own default | per-tick fill budget |
| `suppress_terrain_under_static_geometry` | bool | optional | `TerrainConfig`'s own default (`true`) | punch holes in the terrain heightfield under coincident static feature meshes |
| `interest_points` | array of `{id, world_xy: [x,y], radius_m}` | optional | empty | fixed streaming interest points (a static stand-in for a moving camera/vehicle) |

## Vehicles

Each entry of `vehicles[]` (`ScenarioVehicle`):

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `name` | string | **required** | — | — | vehicle instance name |
| `chassis_body` | string | **required** | — | must name an existing body from `bodies[]`, else `"vehicle '<name>' references unknown chassis_body '<name>'"` (checked in `build_scenario`, not `load_scenario`) | which body is the chassis |
| `vehicle_json_path` | string | **required** | — | resolved relative to the scenario file's own directory if relative; loaded immediately via `load_vehicle_json` (any vehicle-file error — see `vehicle.md` — surfaces here) | path to the vehicle's own JSON file |

There is no way to inline a vehicle's fields directly in a scenario file —
`vehicle_json_path` is always a reference to a separate file (`desc` in the
`ScenarioVehicle` struct exists for building a scenario in C++ code, e.g.
tests, not from JSON).

## Control events

`"control_events"` is an array of:

```json
{ "t": 0.3, "channel": "throttle", "value": 0.8 }
```

| Field | Type | Meaning |
|---|---|---|
| `t` | real (s, simulation time) | when `World::set_control(channel, value)` is applied |
| `channel` | string | control channel name (see `vehicle.md`'s "Control channels a vehicle reads" for the names a vehicle-driving scenario typically uses: `steer`, `throttle`, `brake`, `handbrake`, `clutch`, `shift_up_count`, `shift_down_count`, `gear_request`, `ignition`, `starter`, `assist.auto_clutch`, `assist.auto_blip`, `assist.auto_shift`; a second vehicle in the same `World` uses `v1.<channel>`, etc.) |
| `value` | real | value to set |

Events are **sorted by `t`** at load time (stable w.r.t. original order for
equal `t`, per `std::sort`'s guarantees — actually `std::sort` is not
required to be stable; ties break in an unspecified order). `ps_run`
applies them in that time order as it steps. A scenario with no
`control_events` behaves exactly as if every channel reads its `World`
default (usually `0`).

## Environment (weather plan W2)

`"environment"` installs a `ps::environment::Environment` (clock, calendar,
sun - the weather, surface condition and tyre-profile parts arrive with W3+)
into the `World` at build time (`World::install_environment`). The climate
file is `physics_sim.climate/1` (see [`climate.md`](climate.md)) and is loaded,
so any error in it surfaces, by `load_scenario`. A scenario without the block
installs nothing and behaves exactly as before: same state hash, same CSV
header, same allocation behaviour. With one installed, the environment's clock
anchor is folded into `World::state_hash()`, `WorldSnapshot::environment` is
filled every tick, and no telemetry channel is added.

```json
"environment": {
  "climate": "../climates/test_fixed_summer.json",
  "seed": 1,
  "time": { "start": { "season": "autumn", "day": 29, "time_of_day_s": 60000 }, "scale": 20000 },
  "season": "dynamic",
  "north_yaw_deg": 15,
  "events": [ { "t": 3.0, "jump_time_s": 40000 }, { "t": 5.0, "set_time_scale": 5000 } ]
}
```

| Field | Type | Req./opt. | Default | Meaning |
|---|---|---|---|---|
| `climate` | string | required | - | climate file, relative to the scenario file's directory if relative |
| `seed` | uint64 | optional | `0` | stored; used by the weather timeline from W3 on |
| `weather` | `"dynamic"` or `{"preset": "<file>"}` | optional | off | W4: switches the weather on - `"dynamic"` follows the climate's regime chain, a preset file (`physics_sim.weather_preset/1`, path relative to the scenario) is held constant. The climate must carry the weather statistics. With weather on, the environment provides every vehicle's ambient air (ISA at its altitude, humid-air density), wind and gusts; `World::set_ambient`/`set_aero_environment` are then rejected. Absent: the World keeps its own values, bit-identical to before |
| `altitude_offset_m` | real | optional | `0` | added to the climate's `location.elevation_m` as the altitude of the World z = 0 plane above sea level (a track above or below the climate's station) |
| `time.start` | `{year, season, day, time_of_day_s}` | optional | the climate's epoch (`t_env = 0`) | the calendar position the clock starts at: `season` empty = the first season, `day` within that season, `time_of_day_s` since that day's start (midnight) in environment seconds; out-of-range values are load errors |
| `time.scale` | real >= 0 | optional | `1.0` | environment seconds per simulation second; `0` freezes weather/sun/surface time |
| `season` | string | optional | `"dynamic"` | `"dynamic"` advances through the seasons; a season name pins every day to that season's mid-season values |
| `fixed_day_of_year` | int >= 0 | optional | off | pins every day to one game day's values; mutually exclusive with a fixed `season` |
| `north_yaw_deg` | real | optional | `0` | rotation of north about the World's Z axis, counter-clockwise from +Y (0 = north is +Y, east is +X); used for `sun_dir_world` |
| `events` | array | optional | empty | timed environment changes, see below |

Environment time is `t_env = anchor_env + scale * (tick - anchor_tick) / tick_hz`,
a closed-form function of the tick (nothing accumulates). Installing at tick 0
anchors `t_env = start` there; a mid-run `World::install_environment` anchors it
at the current tick.

`events` (sorted by `t`, simulation time, applied by
`ps::io::apply_environment_events` - `ps_run` and the determinism tests call it
each tick right before `World::step()`): each has `t` and exactly one of

- `jump_time_s` (real): advance environment time by that many environment
  seconds (negative goes back; the weather and sun are pure functions of time,
  so this is always valid in W2);
- `set_time_scale` (real >= 0): change the scale without a time jump;
- `set_weather` (W3b, owner answer Q12, the manual weather): `{preset, ramp_s}`;
  `preset` is a `physics_sim.weather_preset/1` file (relative to the scenario
  file), `ramp_s` >= 0 (environment seconds, default 0). The preset replaces the
  weather from the event to the END OF THE CURRENT SEASON (smoothstep ramp-in of
  `ramp_s`; the ramp-out finishes at the season boundary, shortened to at most
  half the remaining time), and the next season starts on the natural weather.
  With the season locked (`set_season` or the `season` key) it lasts until
  `end_weather`/`clear_weather`. Needs the environment's `weather` key;
- `end_weather` (`true`): soft end - every override stops now and ramps out
  over its own `ramp_s`; `clear_weather` (`true`): immediate removal of every
  override (an exact reset to the natural weather; trailing 1 h / 6 h
  precipitation sums still contain override rain for up to 6 h). Both need the
  `weather` key;
- `set_season`: a season name locks the calendar to it (every day carries that
  season's mid values; the game day index and the fraction of the day are
  preserved, the env seconds jump when the day lengths differ), `"dynamic"`
  unlocks (a season-bound override re-resolves its end). Works without the
  `weather` key.

Weather events live in the environment hash (the override list and the calendar
mode are description state); a scenario without them hashes exactly as before.

Rejected (load errors, never silently dropped): unknown keys, a missing
`climate`, a climate file error, an unknown `season`/`time.start.season`, a bad
event (an unknown event key, more or fewer than one action key, a negative
`ramp_s`, an unknown `set_season` name, a weather event without the
`weather` key, an unreadable preset file) and keys of later increments
(`surface_conditions`, `default_tyre_profile`).

The published record (`ps/environment/environment_snapshot.h`, in
`WorldSnapshot::environment` when `has_environment`): `t_env_s`, `time_scale`,
`year`, `season` + `season_name`, `day_in_season`, `day_of_year`,
`time_of_day_s`, `day_s`/`night_s`, `day_fraction`, `clock_hours`
(`day_fraction * 24`: a game day of any length maps onto a 24-hour clock),
`is_day`, `sun_elevation_rad`, `sun_azimuth_rad` (from north, clockwise),
`sun_dir_world` (unit vector towards the sun, World frame Z up), `ghi_w_m2`
(clear-sky Haurwitz irradiance; `cloud_cover` is 0 until W3).

## Minimal complete example

A car on flat static ground, driven by a scripted step-throttle event
(loosely modelled on `data/scenarios/vehicle_step_steer.json`):

```json
{
  "format": "physics_sim.scenario/1",
  "name": "doc_example_scenario",
  "duration_s": 4.0,
  "tick_rate_hz": 240.0,
  "substep_rate_hz": 960.0,
  "gravity": [0, 0, -9.81],
  "job_workers": 0,
  "surface_table_path": "../surfaces/surfaces.json",
  "bodies": [
    { "name": "ground", "motion": "static", "shape": {"type": "box", "half_extents": [1000.0, 1000.0, 0.5]},
      "position": [0, 0, -0.5], "friction": 1.0 },
    { "name": "chassis", "motion": "dynamic", "mass": 1500.0,
      "shape": {"type": "box", "half_extents": [2.0, 0.4, 0.15]},
      "position": [0, 0, 0.6], "friction": 0.5, "allow_sleep": false }
  ],
  "vehicles": [
    { "name": "car", "chassis_body": "chassis", "vehicle_json_path": "../vehicles/car_sedan.json" }
  ],
  "control_events": [
    { "t": 0.0, "channel": "throttle", "value": 0.0 },
    { "t": 0.5, "channel": "throttle", "value": 0.6 }
  ],
  "telemetry_csv": "doc_example_scenario.csv"
}
```

## Common mistakes

- **Missing `"format"`.** Same pattern as every other file in this project.
- **A vehicle's `chassis_body` name that doesn't match any `bodies[].name`
  exactly.** Note this is checked in `build_scenario`, so it surfaces only
  once the scenario is actually built (e.g. by `ps_run`), not at
  `load_scenario` parse time.
- **A joint's `body_a`/`body_b` referencing a body name that doesn't
  exist**, or is misspelled — same "checked when building, not when
  parsing" caveat.
- **Forgetting `surface_table_path` on a vehicle scenario.** Not an error —
  every surface silently reads `lambda_mu=1, crr=0` — but almost certainly
  not what you want for a scenario that should show surface-dependent
  grip/rolling-resistance behaviour.
- **A relative `vehicle_json_path`/`spec_path`/`surface_table_path`
  written relative to the repo root instead of the scenario file's own
  directory.** All three resolve relative to the scenario file's directory,
  matching the convention every other cross-file reference in this project
  uses (see README).
