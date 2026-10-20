# ECU format — `physics_sim.ecu/1`

An ECU part file holds an engine's control calibration: idle and start
thresholds, idle control, the limiter, the throttle actuator, the lambda and
spark tables and the cam phaser schedule. It exists so a garage (or a person)
can swap a calibration without copying the whole engine file (engine garage P2,
`out/engine_garage_design.md` sections 4.4-7).

An ECU file is **not** a separate descriptor. The loader (`core/src/io/engine_parts.cpp`,
`assemble_engine`) merges it into the engine JSON (RFC 7396 merge patch) **before**
the engine reader runs, so every field below has exactly the meaning, units,
defaults and rejections it has inline in an engine file — see
[engine.md](engine.md). An engine that names an ECU part therefore loads to the
same descriptor, the same `.psmaps` key and the same state hash as the same
engine written inline.

Shipped: `data/ecus/hyper_v8_one1.json` (the calibration of
`data/engines/hyper_v8_one1_audio.json`, moved there verbatim).

## How an engine gets its ECU

| Level | Key | Paths relative to |
|---|---|---|
| engine file | `"default_ecu": "<path>"` | the file that wrote the key (an `extends` parent's own directory) |
| vehicle engine component | `"ecu": "<path>"` | the vehicle file |
| user layer | `PartOverride::slots["ecu"]` (`load_vehicle_json(path, VehicleLoadOptions)`) | the vehicle file |
| standalone engine | `EngineSlotOverrides{{"ecu", path}}` (`load_engine_json`, `load_engine_json_from_string`) | the engine file / `base_dir` |

The highest level that names an ECU picks **the** file; lower levels' ECU files
are not read. The chosen file is merged over the engine's own (inline) ECU
content. Absent everywhere = the engine's inline content only (bit-identical to
before P2).

## Fields

`format` (required, `"physics_sim.ecu/1"`), `name` and `source` are the part's
own metadata; they are not merged into the engine. Every other key must be one of:

| Key | Engine meaning ([engine.md](engine.md)) |
|---|---|
| `idle_rpm`, `stall_rpm`, `fire_rpm`, `start_complete_rpm` | engine state thresholds |
| `dfco_enable_rpm`, `dfco_reenable_rpm` | deceleration fuel cut-off |
| `idle_control` | the idle PI block |
| `limiter` | the rev limiter block |
| `throttle_actuator_tau_s` | electronic throttle actuator lag |
| `switching` | engine garage P9: the cam-profile switching rule (below) |
| `lift_control` | engine garage P10: the continuous intake-lift strategy (below) |
| `ecu.lambda_target`, `ecu.spark_advance_deg` | the simulated engine's `ecu` block tables |
| `ecu.ignition_strategy`, `ecu.knock_margin_deg` | engine garage P7 knock-limit opt-in (below) |
| `ecu.part_load_boost` | pedal P1: `"efficient"` (default) or `"spooled"` - a turbo engine's part-load boost strategy ([engine.md](engine.md#the-manual-pedal-law-pedal-plan-p1)); read on every engine, used only with forced induction; a change is Runtime |
| `cycle.spark_advance_deg` | the cycle block's spark schedule (copied into `ecu.spark_advance_deg` by the loader, as inline) |
| `cycle.cam` | the cam phaser schedule of the cycle block |
| `*_source` | a source note next to any of the above (top level, inside `ecu` or `cycle`) |

A part may carry any subset; it is merged as given (a partial ECU leaves the
engine's other inline fields alone). `cycle.*` keys need an engine with a
`cycle` block.

## Knock-limit opt-in (engine garage P7)

Inside the `ecu` object (inline in an engine or in an ECU part):

| Key | Type | Default | Meaning |
|---|---|---|---|
| `ignition_strategy` | string | `"schedule"` | `"schedule"`: the spark is the ECU table (every engine before P7). `"knock_limited"`: the runtime spark is `min(table, theta_kl)`; theta_kl is generated from the cycle block for every listed fuel (or taken from `maps.theta_kl_deg`) |
| `knock_margin_deg` | real, deg crank | required with `"knock_limited"` | how far the ECU stays below the model's knock limit; in [0, 10]. The knock model is calibrated on production engines' measured spark taken as the limit (knock model v6), so its limit already carries the OEM's own margin: this key is margin BEYOND that OEM-equivalent limit (shipped engines 0 since 2026-10-20; the hyper 2) |

With `"knock_limited"` the spark table may be omitted (and a cycle block's
`spark_advance_deg` removed with `null` in a child engine): the ECU then tracks
MBT - `ecu.spark_advance_deg` is filled from the generated `maps.theta_mbt_deg`
after map generation - and the knock limit retards it where needed. A
validate-only (pending) load leaves that table empty until the maps exist.
Opting in moves the engine's map cache key (the octane numbers of its fuels and
the margin join it); an engine that does not opt in is bit-identical to before
P7. See engine.md "Knock limit".

Rejections (engine reader, after the merge):

- `ignition_strategy` not a string, or not `"schedule"`/`"knock_limited"`
- `knock_margin_deg` without `ignition_strategy "knock_limited"`
- `"knock_limited"` without `knock_margin_deg`
- `knock_margin_deg` not a number or outside [0, 10]
- `"knock_limited"` on an engine with neither a cycle block nor `maps.theta_kl_deg`
- `"knock_limited"` with a listed fuel that has no `knock` block (fuel.md)

## `switching` — cam-profile switching (engine garage P9)

The rule by which the ECU selects between the two profiles of a profile cam
([cam.md](cam.md#profiles--discrete-cam-profiles-engine-garage-p9)). It must
come with such a cam, and a profile cam needs it.

```json
"switching": { "type": "discrete", "rpm_on": 6000, "rpm_off": 5800,
               "min_p_im_kpa": 40, "min_oil_c": 60, "transition_s": 0.05 }
```

| Key | Meaning | Range |
|---|---|---|
| `type` | `"discrete"` (the only type; continuous lift is P10) | |
| `rpm_on` | go to the high profile at or above this speed | `rpm_off < rpm_on < limiter.rpm` |
| `rpm_off` | back to the low profile below this speed (hysteresis) | `idle_rpm < rpm_off` |
| `min_p_im_kpa` | the high profile is held only at or above this manifold pressure | `>= 0` |
| `min_oil_c` | ... and at or above this oil temperature | `[-40, 150]` |
| `transition_s` | the blend ramp from one profile to the other | `[0, 2]`, 0 = one substep |
| `source`, `*_source` | notes | |

Runtime (`SimulatedEngine`, once per substep after the state machine): with
`hold = Running && p_im >= min_p_im && oil >= min_oil_c`, the ECU goes high when
`hold && rpm >= rpm_on` and low when `!hold || rpm < rpm_off`. A blend weight
`w` follows at `h / transition_s` per substep; every model lookup (manifold
flow, torque, fuel, the steady demand tables and the electronic throttle's
inverse) is `low + w * (high - low)` - the fraction of cylinders already on the
high lobe - and exactly the single profile at `w = 0` and `w = 1`. Decisions
(PROVISIONAL, out/garage_p9_progress.md):

- without an engine thermal model the oil temperature is unknown and the oil
  condition counts as met (a warm engine); with one, `Sensors::oil_temperature_c`
  is the thermal model's oil lump;
- idle control (feed-forward, PI authority cap, boost idle) runs on the low
  profile - hence `idle_rpm < rpm_off`;
- the clutch's torque demand follows the blended steady tables;
- the switching state (`cam_high`, `w`) enters the state hash only on an
  engine with profiles; `reset()` returns to the low profile;
- one spark table serves both profiles (per-profile spark is an open point).

A switching edit is change class Runtime (`classify_engine_change`, pointer
`/ecu/switching`); `check_buildable` grades the window (rule `CamSwitchWindow`,
[engine.md](engine.md)).

## `lift_control` — continuous intake lift (engine garage P10)

The strategy by which the ECU sets the intake lift `u` and the throttle plate
of a cam with an `intake_lift_family`
([cam.md](cam.md#intake_lift_family--continuous-intake-lift-engine-garage-p10)).
It must come with such a cam, and a family needs it.

```json
"lift_control": { "type": "continuous", "actuator_tau_s": 0.05, "max_rate_per_s": 3.333,
                  "min_u": 0.0, "idle_u": 0.0, "plate_open_above_u": 0.5, "vacuum_target_kpa": 5.0 }
```

| Key | Meaning | Range |
|---|---|---|
| `type` | `"continuous"` (the only type) | |
| `actuator_tau_s` | first-order lag of the eccentric shaft toward its target | `[0, 1]` s, 0 = no lag |
| `max_rate_per_s` | the shaft's rate cap (full stroke = `1 / max_rate_per_s` s) | `> 0` |
| `min_u` | the lowest lift the ECU commands | `[0, 1)`, a node `u` |
| `idle_u` | the lift at idle, in DFCO and with the pedal released | `[min_u, 1)`, a node `u` |
| `plate_open_above_u` | from this lift up the plate is fully open and the lift alone sets the load | `(min_u, 1]`, a node `u` |
| `vacuum_target_kpa` | the manifold vacuum the plate holds below `plate_open_above_u` | `[0, 30]` kPa |
| `source`, `*_source` | notes | |

`min_u`, `idle_u` and `plate_open_above_u` must each be one of the cam's node
`u` values (the ECU's tables are exact at the nodes).

Runtime (`SimulatedEngine`, every substep before the throttle actuator), with
the request `r` = the throttle command (after the throttle stage, the TCU limits
and the launch-control limit) and the torque target `T* = mot + r (wot - mot)` of
the **u = 1** node (the same torque a single-lift engine asks for):

- `r >= 1`: `u = 1`, plate open (WOT is exactly the u = 1 node);
- `T*` below the `min_u` node's torque at the vacuum target: `u = min_u`, and the
  plate holds the manifold pressure at which that node makes `T*` (bisection, down
  to 5 % of ambient [AS]);
- the vacuum range `min_u .. plate_open_above_u`: the plate holds
  `p_amb - vacuum_target_kpa` and `u` interpolates linearly between the nodes'
  steady torques at that pressure;
- `T*` between the `plate_open_above_u` node's vacuum and open-plate torque: `u`
  rests on that node and the plate holds the pressure at which it makes `T*`;
- above: plate open, `u` interpolates between the nodes' open-plate torques;
- pedal released (`r < 0.01`; idle, DFCO, an ignition cut): `u = idle_u`, plate
  closed apart from the idle air.

The shaft follows its target as `u += h/(tau+h) * (u* - u)`, capped at
`max_rate_per_s * h`, and snaps onto the target within 1e-6 [AS]. The plate
is solved against the lift the plant has NOW, so the manifold holds its
target while the shaft moves. Between nodes the plant is the linear blend of
the two bracketing node sets (exactly one node at a node). Decisions
(PROVISIONAL, out/garage_p10_progress.md): node values linear in `u`; the
request scale from the u = 1 node; idle on the `idle_u` node with the plate;
lambda follows the engine's `ecu.lambda_target` at every lift.

A `lift_control` edit is change class Runtime (`classify_engine_change`, pointer
`/ecu/lift_control`); an edit to the family (a node's lift) is Maps, a change of
the node `u` values Respawn.

## Rejections

- `format` missing or other than `physics_sim.ecu/1` (`"<path>: unsupported 'format' \"...\" (expected \"physics_sim.ecu/1\")"`)
- any other key: `"'<key>' is not an ECU field (docs/data_formats/ecu.md lists the fields)"`,
  `"'ecu.<key>' is not an ECU field ..."`, `"'cycle.<key>' is not an ECU field ..."`
- `ecu`/`cycle` not an object
- cycle fields on an engine without a `cycle` block
- an engine file with `default_ecu` **and** inline ECU content (any key of the
  table above in the same engine object):
  `"'default_ecu' and the inline ECU field '<key>' - give the ECU either inline or as a part file, not both (a child engine removes an inherited field with null)"`
- `"none"` as an ECU (`"none"` is only for forced induction)
- `default_ecu` / `ecu` not a non-empty string
- `switching`: an unknown key, `type` other than `"discrete"`, a missing or
  non-finite value, `rpm_off >= rpm_on`, the band outside `idle_rpm .. limiter.rpm`,
  `min_p_im_kpa < 0`, `min_oil_c` or `transition_s` out of range; `switching`
  without a profile cam (`"the ECU has a 'switching' block but the cam has no
  profiles ..."`) or on an engine without a `cycle` block
- on a profile engine: a knock-limited ECU without a spark table (MBT tracking
  is not supported with profiles yet)
- `lift_control`: an unknown key, `type` other than `"continuous"`, a missing or
  non-finite value, `actuator_tau_s` outside `[0, 1]`, `max_rate_per_s <= 0`,
  `min_u` outside `[0, 1)`, `idle_u` outside `[min_u, 1)`, `plate_open_above_u`
  outside `(min_u, 1]`, `vacuum_target_kpa` outside `[0, 30]`; a value that is
  not a node `u` (`"'min_u' = 0.1 is not one of the cam's intake_lift_family node
  u values ..."`); `lift_control` without a lift family (`"the ECU has a
  'lift_control' block but the cam has no intake lift family ..."`) or on an
  engine without a `cycle` block

The merged engine is then validated by the engine reader as usual (ranges,
table shapes, a fuel-cut limiter's hysteresis, ...).

## `extends`

An ECU file may `"extends"` another ECU file (RFC 7396, depth <= 8, cycles
rejected, relative paths per file) — e.g. a "stage 1" calibration that changes
only `limiter.rpm`. It has no path-valued keys.

## Example

```json
{
  "format": "physics_sim.ecu/1",
  "name": "my_v8_stage1",
  "source": "[AS] +250 rpm limiter over the stock calibration",
  "extends": "../ecus/hyper_v8_one1.json",
  "limiter": { "rpm": 8500 }
}
```
