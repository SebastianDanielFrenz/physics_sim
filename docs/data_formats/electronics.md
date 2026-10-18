# `physics_sim.electronics/1` — electronics part files (engine garage P8)

`data/electronics/*.json`. Loader: `core/src/io/electronics_io.cpp` (`ps::io::load_electronics_json`, header
`ps/io/electronics_io.h`); applied by `load_vehicle_json` (`core/src/io/vehicle_io.cpp`). Descriptor:
`ps::drivetrain::LaunchControlDesc` (`ps/drivetrain/controller_desc.h`).

A car has only the electronics it was built with (garage decision D11); everything else is grafted on as a part. The
physics refuses an assist the car does not have, so a script or another client cannot switch it on either.

## Assists overview (assists I6, vault PHYS-074) {#assists-overview}

One page for "which assist lives where, which channel switches it, what wins when two want the same thing". Every
statement below was checked against the code on 2026-10-13 (`core/src/world/world.cpp` channel registration,
`core/src/vehicle/vehicle_instance.cpp`, `core/include/ps/vehicle/{assist_unit,drive_modes}_desc.h`); the detail
sections further down are the per-feature reference.

### Where each assist lives and how it is switched

All driver-assist channels are plain `World::set_control` channels (a `"v<index>."` prefix for the second and later
vehicle, see [vehicle.md](vehicle.md#control-channels-a-vehicle-reads)). "On" means a **strict** `> 0.5` for every
channel except `assist.aero_control`, where it is `>= 0.5`.

| Assist | Lives in | Channel | Registered | Default value | Codes |
|---|---|---|---|---|---|
| auto clutch, auto blip, auto shift | the `manual_tcu` controller's `auto_clutch` / `auto_blip` / `auto_shift` blocks ([vehicle.md](vehicle.md#controllers)) | `assist.auto_clutch`, `assist.auto_blip`, `assist.auto_shift` | always, on every car | the block's `enabled` AND `fitted` (0 without a `manual_tcu`) | 0 / 1 |
| traction control | the vehicle's `traction_control` block (= mode On), plus an optional assist unit's mode table | `assist.traction_control` | always, on every car | 0 when `fitted` is false; else the default drive mode's `traction_control_default` if the car has the `drive_modes` part and that mode has one; else the unit's `default_mode` (unit with a TC block); else the block's `enabled` | 0 Off, 1 On (the car's own block), 2 Relaxed, 3 Sport, 4 Drift; `requested = value > 0.5 ? clamp(round(value), 1, 4) : 0` |
| launch control | EITHER the P8 two-step part (`kind: launch_control`) OR the assist unit's `launch_control` block (grades Sport/Racing); never both | `assist.launch_control` (the P8 part may rename it with its `channel` key) | only on a car with one of the two (a P8 part with `fitted: false` still registers it) | 0 for the P8 part; the unit block's `default_enabled` (shipped `false`); in both cases the default drive mode's `launch_control_default` when the car has the `drive_modes` part and that mode sets one | 0 / 1 |
| rear-wing automation | the assist unit's `aero_control` block (grade Racing) | `assist.aero_control` | only when the unit has the block AND the car's `aero.surfaces` has the named surface | the block's `default_enabled` (shipped Racing unit: `true`) | 0 / 1, on at `>= 0.5` |
| drive modes | the car-specific `drive_modes` electronics part | `assist.drive_mode` | only on a car with the part | the part's `default_mode` | 0 eco, 1 normal, 2 sport, 3 racing (rounded and clamped; a mode the part does not list maps to its `default_mode`) |
| ABS (I5 part 2) | the assist unit's `abs` block ([below](#abs-esc)) | `assist.abs` | only when the unit has the block | the block's `default_enabled` (shipped `true`); the default drive mode's `abs_default` when the car has the `drive_modes` part and that mode sets one | 0 / 1, on at `>= 0.5` |
| ESC (I6) | the assist unit's `esc` block ([below](#abs-esc)); needs the `abs` block | `assist.esc` | only when the unit has the block | the block's `default_mode` code; the default drive mode's `esc_default` when set | 0 off, 1 on, 2 sport; `< 0.5` or not finite = off, else `clamp(round(value), 1, 2)`; an unoffered mode maps to the nearest more-intervening one (sport -> on; off -> sport if offered, else on) |
| electronic parking brake (Q3 / I8) | the `epb` electronics part ([below](#epb)) on the car's handbrake rows | `epb` (not an `assist.*` name) | only on a car with the part | 0 | level 0 / 1, on at `>= 0.5`; per drive mode `park_only` (refused at speed) or `handbrake` |
| anti-lag switch | the engine's anti-lag configuration and the vehicle key `anti_lag_switch.fitted` | the configuration's own `channel` (default `anti_lag`; not an `assist.*` name) | by the engine | the configuration's `default_enabled` | 0 / 1 |
| N2O arm (not an assist, but gated by drive modes) | the nitrous controller part's `arm_channel` | shipped kits: `nitrous_arm` | with a nitrous kit | the controller's | 0 / 1 |

Who owns which file: the **assist unit** is generic per grade (`data/electronics/assist_unit_{street,sport,racing}.json`,
one file per grade, any car can take one); **drive modes** are car-specific numbers (they override that car's own
`manual_tcu`); the **P8 parts** (`launch_two_step_generic.json`, `fit_*.json`) are the older, simpler fitment and
launch-limit parts. All three attach as a `controllers` entry of type `electronics` or through
`VehicleLoadOptions::electronics` (a path replaces/adds, `"none"` removes).

### Precedence: what wins when two assists want the same thing

1. **Launch control over traction control.** While the unit's launch control *owns* the car (phase Armed or Launch)
   the powertrain sees TC neutral: fraction 1, no throttle ceiling, no ignition cut. TC's estimator keeps running, so
   at Handover TC resumes seeded from the launch controller's last throttle ceiling (no torque jump). I2's crawl-speed
   clutch cap (item 6) does not act while LC owns the car.
2. **Rear wing: launch control over airbrake over cruise.** The wing law yields `launch_offset_deg` while the unit's LC
   owns the car, else the airbrake blend, else the cruise ramp. The launch request is the LC's state at the END of the
   previous tick, so the wing follows LC by **one tick**; Handover does not ask for the launch offset. Only the *unit's*
   launch control asks for it, never the P8 two-step.
3. **Drive modes gate two assists.** `launch_control_allowed: false` makes `assist.launch_control` read as off (this also
   switches the P8 two-step off, and `assist_active_bits` bit4 with it); `nitrous_arm_allowed: false` forces the kit's arm
   input to 0 whatever `nitrous_arm` says. The gate is the outermost layer: it also overrides a `launch_control_default`
   (rule 4b).
4. **Drive mode writes the driver's TC choice (assists I7c).** When the *effective* drive mode changes and the new mode has a
   `traction_control_default`, the effective TC request for that tick is the default AND the physics **writes the code into the
   `assist.traction_control` channel itself** (applied by `World::step` after the tick, so `get_control` returns it right after
   `step()`). There is no override state: the channel always equals what the car does, and the driver's next write of **any**
   value (including the value the channel held before the mode change) wins on the next tick - one press. A mode without a
   default leaves the channel alone. The snapshot's `traction_mode_requested` is the channel/default code. Same tick: when the
   driver writes the channel in the very tick the mode changes, the mode default wins for that tick and is written over the
   driver's value; the next tick's driver write wins.
4b. **Drive mode writes the driver's launch-control switch (assists I7b, I7c).** The same rule for the launch-control *enable*:
   when the effective drive mode changes and the new mode has a `launch_control_default` (a boolean), the physics writes `1` /
   `0` into `assist.launch_control` (only on a car where the assist is fitted; an unfitted car has no such channel and ignores
   the default). Precedence, outermost first: `fitted` -> `launch_control_allowed` (a mode that does not allow LC never arms,
   whatever the default or the channel) -> the channel (`> 0.5`). Shipped `car_hyper_n2o`: Sport `true`, Eco and Normal `false`
   (owner, 2026-10-13: "turn LC on when Sport mode is activated"). Arming itself still needs the conditions of rule 8. At spawn
   nothing is written: the channel default is the default mode's value.
4c. **Channel writes by the physics (assists I7c).** A vehicle cannot write `World`'s channel map from inside its job, so each
   vehicle queues at most two pending writes (TC code, LC enable) in its step; `World::step` applies them sequentially after the
   pipeline, in vehicle id order, straight into the flat channel array (no allocation). Vehicles with the same global channel
   name (only vehicle 0 uses bare names; vehicle *i* uses `v<i>.`) would overwrite each other in id order - the later vehicle
   wins. A `set_control` between steps therefore wins from the next tick on. `ps_run` scenario `control_events` and the
   determinism harness are ordinary `set_control` calls: a physics-written value persists in the map until something writes it.
   **Adapters that re-send every channel each tick must mirror the physics' writes back** (otherwise the next push puts the stale
   value over them): after each step, for every channel whose World value differs from the value pushed before that step, adopt
   the World value with a compare-exchange from the pushed value (a user write made meanwhile wins). Reference:
   `adapters/godot/src/channel_mirror.h` (`push_channels` / `adopt_world_channel_writes`, Godot-free).
5. **Pedal bend.** A drive mode's bend is applied after the car's own pedal curve; the auto-shift, launch law, kickdown
   and the TC/LC throttle ceilings work on the *bent* command (ceilings are fractions of it), while TC's pedal gating
   and LC arming use the raw pedal (driver intent). Pedal plan P1: the bend applies only with a gearbox that turns the
   pedal into a power request (automatic, DCT, CVT - decided by gearbox kind when the car is built). Manual and
   sequential gearboxes - every shipped car - keep the driver's pedal, so the bend is inert today.
6. **I2 low-speed clutch cap.** Only while the effective TC mode is not Off, the auto-clutch is in Launch/Locked, the
   ground-speed input rpm of the current gear is below 1.25 x `anti_stall_rpm` `[AS]`, and launch control does not own
   the car: the TC fraction caps the *clutch* capacity instead of the engine.
7. **Unavailable TC modes** fall back to the nearest available mode with *more* intervention (On > Relaxed > Sport >
   Drift); Off is never substituted ([below](#assist-unit)).
8. **Launch control needs more than its own channel:** the auto-clutch assist on, the engine Running, gear 1, throttle
   and brake past the arming thresholds ([below](#assist-launch)).

### Snapshot bits and fields (`PowertrainSnapshot`, never hashed)

| Bit | Assist | `assist_fitted_bits` (the car HAS it) | `assist_active_bits` (on now) |
|---|---|---|---|
| 0 | auto clutch | `manual_tcu` present and `fitted` | channel `> 0.5` and fitted |
| 1 | auto blip | same | same |
| 2 | auto shift | same | same |
| 3 | traction control | the `traction_control` block is fitted | the **effective** mode is not Off |
| 4 | launch control | the car has the P8 part or the unit block | channel `> 0.5` AND the effective drive mode allows launch control (not "the car is launching": that is `launch_phase`) |
| 5 | anti-lag switch | `anti_lag_switch.fitted` (default true) | **never set** (the switch is the engine's own channel; fitted-only) |
| 6 | rear-wing automation | the unit's `aero_control` block and a surface of that name | channel `>= 0.5` and fitted |
| 7 | electronic parking brake | the car has the `epb` part | `epb_state` != 0 (latched or applied as handbrake) |
| 8 | ABS | the unit has the `abs` block | ABS scaled a service brake below 0.995 on the last substep of the tick |
| 9 | ESC | the unit has the `esc` block | ESC brakes a wheel or cuts engine torque |

A HUD shows hidden (fitted bit clear), fitted-off, fitted-on. Other fields: `traction_mode_requested` /
`traction_mode_effective` / `traction_modes_available_bits` (bit *n* = code *n* is offered; 0 when TC is not fitted);
`drive_mode_requested` / `drive_mode_effective` / `drive_modes_available_bits` (bit 0 eco, 1 normal, 2 sport, 3 racing; all 0
without the part); `epb_state` (0 released, 1 latched, 2 handbrake) and `epb_refused` ([below](#epb)); `launch_phase` (0 Idle, 1 Armed, 2 Launch, 3 Handover), `launch_slip_target`, `launch_control_active`
(true while the unit's LC is Armed/Launch, or the P8 two-step is latched); `aero_control_active`,
`aero_control_target_deg`; `abs_active`, `esc_active`, `abs_reference_speed_m_s` (the controllers' own speed estimate,
0 without either block), `esc_yaw_rate_target`, `esc_mode_requested` / `esc_mode_effective` / `esc_modes_available_bits`
(bit *n* = code *n* offered).

### UI pitfalls

- **Cycling the TC mode after a drive-mode change.** Since assists I7c the channel holds the mode's code after a change, so a
  button that cycles from the channel value (or from `traction_mode_effective`) and writes the next code always changes the
  channel and takes effect with one press, including a write of the value the driver had chosen before the mode change
  (guard: `tests/unit/test_assist_all_together.cpp` P1, `tests/unit/test_assist_channel_writeback.cpp` W1-W3).
- **Do not write `assist.traction_control` / `assist.launch_control` on a drive-mode change.** The physics writes the mode's
  default itself; a client write of a different value in the same tick is overwritten (the mode default wins that tick).
- **Re-read the channel after a mode change.** The channel value may change during `step()` (physics write); a client that
  keeps its own copy of the channels must adopt it (see rule 4c).
- **A request the part or unit does not offer is not an error.** Offer only the bits in the `*_available_bits` fields.

## Fitment: the `fitted` flag (A15)

Every assist that has a runtime channel carries `fitted` (default `true`, so every file written before P8 is
unchanged, state hash included):

| Assist | Where `fitted` lives | Channel |
|---|---|---|
| auto clutch / auto blip / auto shift | the `fitted` key of the `auto_clutch` / `auto_blip` / `auto_shift` block of the `manual_tcu` controller | `assist.auto_clutch` / `assist.auto_blip` / `assist.auto_shift` |
| traction control | `traction_control.fitted` (vehicle block; default `true`, also for a car that has no block, because every car could switch TC on before P8) | `assist.traction_control` |
| launch control | `LaunchControlDesc::fitted` (the electronics part's `fitted`) | `assist.launch_control` |
| anti-lag switch | vehicle key `"anti_lag_switch": {"fitted": false}` (default `true`) | the engine's anti-lag `channel` (default `anti_lag`) |

**A not-fitted assist ignores its channel and stays off whatever the channel says.** Decision: it still
*registers* its channel (stable channel list for scripts and adapters; the channel's default value is then `0`
instead of the block's `enabled`), but `VehicleInstance` reads it as 0. The one exception is `assist.launch_control`:
a car registers it only when it has a `launch_control` component or an assist unit with a `launch_control` block (every
channel list of a car without one is unchanged); a part with `fitted: false` still registers it. Anti-lag: with `anti_lag_switch.fitted =
false` the engine sees no channel (`Sensors::anti_lag_channel` NaN) and anti-lag runs on its configuration's
`default_enabled` only.

Snapshot: `VehicleSnapshot::powertrain.assist_fitted_bits` has the same layout as `assist_active_bits` (bit0
auto_clutch, bit1 auto_blip, bit2 auto_shift, bit3 traction_control, bit4 launch_control) plus bit5 = anti-lag
switch and bit6 = aero automation (assists I5, [below](#assist-aero)). A HUD shows three states: hidden (bit not set in `assist_fitted_bits`), fitted and off, fitted and on
(`assist_active_bits`; bit4 = the launch-control channel is on and the effective drive mode allows it; since assists I4 bit4 means "launch control" for either
variant, the P8 two-step part or the assist unit's `launch_control` block - a car has one of them, never both). A car without a `manual_tcu` reports the auto_* assists
as not fitted. Never hashed.

## Part file

```json
{ "format": "physics_sim.electronics/1", "kind": "launch_control", "name": "...", "source": "...",
  "rpm": 4000, "max_speed_mps": 2.0, "channel": "assist.launch_control", "fitted": true }
```

```json
{ "format": "physics_sim.electronics/1", "kind": "assist_fitment", "name": "...", "source": "...",
  "fits": ["auto_clutch", "auto_blip", "auto_shift"] }
```

`"extends"` works like for the other part files (`ps_environment_io` merge patch, depth 8, cycles rejected); the
format has no path-valued keys. Unknown keys are rejected (`"unknown electronics field '<key>'"`).

| Key | `launch_control` | Meaning / rejection |
|---|---|---|
| `rpm` | required, finite, `> 0` | the second rev limit. The vehicle loader requires it above the (first) engine's idle rpm (`"launch rpm R must be above the engine's idle rpm I"`). G1: an `rpm` at or above `max_limiter_fraction` x the engine's limiter (a low garage limiter) is no longer a load error: it is held at that cap (or, when the cap is not above idle, at the midpoint of idle and the limiter), with the load warning `"<ctx>: Limiting: launch rpm R is at or above C (F % of the L rpm limiter): held at H rpm"` |
| `max_limiter_fraction` | optional, `(0, 1)`, default 0.8 `[AS]` | G1: the cap above as a fraction of the limiter. Shipped launch rpms sit at 0.55-0.62 of their limiter; 0.8 leaves room for the limiter's own hysteresis below the second limit |
| `max_speed_mps` | required, finite, `> 0` | the control is active only below this speedo speed (m/s) and releases above it |
| `channel` | optional, default `"assist.launch_control"` | runtime arm switch, a plain 0/1 level (on = `> 0.5`); resolved with the vehicle's channel prefix |
| `fitted` | optional bool, default `true` | `false`: the channel is ignored |
| `fits` | (`assist_fitment` only) non-empty list of `auto_clutch`, `auto_blip`, `auto_shift`, `traction_control`, `anti_lag` | sets those assists' `fitted` to `true`; `auto_*` need a `manual_tcu` on the car (`"fits 'auto_shift' but the vehicle has no manual_tcu"`) |

Shipped parts (all `[AS]`): `launch_two_step_generic.json` (4000 rpm, 2 m/s), `fit_auto_assists.json`,
`fit_traction_control.json`, `fit_anti_lag_switch.json`, and the assist units `assist_unit_street.json`,
`assist_unit_sport.json`, `assist_unit_racing.json` ([below](#assist-unit)).

## Adding a part to a car

1. In the vehicle file, as a `controllers` entry of type `electronics` (resolved relative to the vehicle file):
   `{ "id": "launch", "type": "electronics", "ref": "../electronics/launch_two_step_generic.json" }`. At most one
   `launch_control` per vehicle; ids are unique across all controllers. Entries are applied after the `manual_tcu`
   (an `assist_fitment` part marks its assists fitted over the block's `fitted: false`).
2. By the user layer (garage P2): `VehicleLoadOptions::electronics`, a map *entry id -> path* (absolute, or
   relative to the vehicle file) or `"none"`. A path replaces the file's entry of that id or **adds** a new one — this
   is how launch control is grafted onto a car whose file has none; `"none"` removes an entry (`"no electronics
   entry with id 'x' to remove"` if there is none). The id of the `manual_tcu` controller is rejected. The user layer
   edits only what the loader reads, so grafting a part equals writing the same `controllers` entry into the
   vehicle file: same descriptor, same state hash.

## Launch control (two-step), A16

A second rev limit `rpm`, applied through the engine's **own limiter** (`TcuRequest::limiter_rpm`, read by both
`EngineTorqueMap` and `SimulatedEngine`): a fuel-cut engine cuts at it with its own hysteresis, a soft-cut engine
tapers over its own band below it. The control never touches torque itself and never raises the engine's limiter
(`min(rpm, limiter.rpm)`). `core/src/drivetrain/launch_control.h` has the latch:

- **Becomes active** when the arm channel is on, the car is nearly stationary (speedo `< max_speed_mps - 0.25 m/s`
  [AS hysteresis]) and the clutch is in.
- **Stays active** while the channel is on and the speedo speed `<= max_speed_mps`. Releasing the clutch therefore
  does *not* end it: the car launches with the limit still held, and it ends once the speed exceeds `max_speed_mps`
  (or the channel goes off). Re-arming needs the clutch in again below the speed window.
- **"Clutch in"** = the clutch's effective engagement (min of the pedal curve and the auto-clutch, the value the
  clutch itself uses this substep) `< 0.5` [AS]. With a manual clutch that is the pedal pressed; with the
  auto-clutch assist it is the launch law's open clutch at idle (engagement 0 below `bite_rpm`), so the control can be
  armed at a standstill before the throttle is opened; a gearbox in neutral with the clutch up does not arm. A car
  without a clutch component never arms [AS].

The one latch byte is hashed only on a vehicle that has the component. Snapshot: `PowertrainSnapshot::
launch_control_active` and `EngineSoundState::launch_active` (unhashed); the usual `limiter_active` flag also shows
the cut. Interaction with anti-lag (D5): none — anti-lag acts on a closed throttle with a boosted turbo engine and
reads its own channel; a launch limit and anti-lag active together are not specially handled.

## Assist unit (assists I1, vault PHYS-074) {#assist-unit}

`kind: "assist_unit"` is the driver-assist electronics of a car, graded **Street / Sport / Racing** (owner names,
2026-10-12). Descriptor `ps::vehicle::AssistUnitDesc` (`ps/vehicle/assist_unit_desc.h`), stored as
`VehicleDesc::assist_unit`; attached like every other part (a `controllers` entry of type `electronics`, or
`VehicleLoadOptions::electronics`: a path replaces/adds, `"none"` removes). At most one per vehicle
(`"at most one assist_unit per vehicle"`). A unit adds no controller. `fitted: false` on the car's
`traction_control` block still wins over everything: no modes are available and the channel is ignored.

```json
{ "format": "physics_sim.electronics/1", "kind": "assist_unit", "name": "Racing Assists", "grade": "racing",
  "traction_control": { "modes": ["off", "on", "relaxed", "sport", "drift"], "default_mode": "on",
    "mode_params": { "sport": { "target_slip": 0.18, "kp": 4.0, "ki_per_s": 8.0, "attack_rate_per_s": 10.0,
                                "recovery_rate_per_s": 10.0, "min_fraction": 0.15, "ignition_cut": true } } },
  "source": "[AS] ..." }
```

**Grades.** The grade decides which *blocks* a unit may carry; the `modes` list is the unit's own data, so the
grade-to-mode mapping is a data edit. Approved mapping (owner 2026-10-12), shipped as one file per grade
(`data/electronics/assist_unit_{street,sport,racing}.json`, all `[AS]`):

| Grade | Traction-control modes | Launch control block | Aero-control block |
|---|---|---|---|
| `street` | Off, On | not allowed | not allowed |
| `sport` | Off, On, Relaxed, Sport | allowed (assists I4) | not allowed |
| `racing` | Off, On, Relaxed, Sport, Drift | allowed (assists I4) | allowed ([below](#assist-aero), assists I5) |

`launch_control` / `aero_control` keys: rejected `"grade 'street' does not allow 'launch_control'"` when the grade
forbids them. `launch_control` loads since assists I4 ([below](#assist-launch)), `aero_control` since assists I5
([below](#assist-aero)).

**Traction-control channel.** One channel `assist.traction_control`, append-only codes:

| Code | Mode | Parameters |
|---|---|---|
| 0 | Off | no TC |
| 1 | On | the car's own `traction_control` block, never stored in the unit: bit-identical to a car without a unit |
| 2 | Relaxed | `mode_params.relaxed` overrides |
| 3 | Sport | `mode_params.sport` overrides |
| 4 | Drift | `mode_params.drift` overrides |

Each tick: `requested = value > 0.5 ? clamp(round(value), 1, 4) : 0` (the pre-unit rule `> 0.5` is kept, so a car with
no unit behaves as before and codes 2-4 are On there); `effective` = the fallback below. The channel's default value is
the unit's `default_mode`; without a unit with a `traction_control` block, the base block's `enabled` (0/1); 0 when
`fitted` is false.

**Unavailable modes.** The requested code maps, statelessly (every tick, never latched), to the nearest available
mode with *more* intervention, by the rank On > Relaxed > Sport > Drift > Off (the codes are not ordered by
intervention). If no available mode is more intervening, the most intervening available mode is used. **Off is never
substituted**; if Off itself is not offered, Off maps to the least intervening available mode. Not fitted: Off.
The snapshot (`PowertrainSnapshot`, never hashed) carries `traction_mode_requested`, `traction_mode_effective` and
`traction_modes_available_bits` (bit *n* = code *n* is offered; 0 when TC is not fitted) so a UI offers only fitted modes.

**Per-mode parameters.** `mode_params` is an object keyed `relaxed` / `sport` / `drift` (a key for `off` or `on` is
rejected: `"'on' takes no parameters"`; a key for a mode not in `modes` is rejected). Every field is an optional
override of the car's own `traction_control` block (an unset field inherits the car's value):

| Key | Meaning | Default |
|---|---|---|
| `target_slip`, `kp`, `ki_per_s`, `attack_rate_per_s`, `recovery_rate_per_s` | the controller's gains (finite, `> 0`; `target_slip < 1`) | the car's value |
| `min_fraction` | torque floor: the TC never cuts below this fraction, `[0, 1)` | `0` (today) |
| `ignition_cut` | whether a deep intervention may cut ignition; also how TC and launch-control throttle ceilings act on a turbo engine with a pedal envelope: `true` = spark retard at the current boost (plate and wastegate held on the driver's command), `false` = the plate closes to the ceiling (engine.md "Assist ceilings on a turbo engine") | `true` (today) |

Switching modes keeps the torque ceiling, clamped to the new mode's floor; there is no estimator or integral. Plan-B1 starting values (`[AS]`, measured
against the V6 ordering band in `tests/unit/test_assist_unit.cpp`): Relaxed 0.12/2/5/15/8, floor 0, cut; Sport
0.18/1.5/4/10/10, floor 0.15, no cut; Drift 0.35/1/2/6/12, floor 0.35, no cut.

**Rejections** (each asserted in `test_assist_unit.cpp` R1): `'grade' is required`, `unknown grade "<x>"`, unknown key
(`unknown electronics field`), `'modes' is required and must be a non-empty array`, `'modes' entries must be one of
off, on, relaxed, sport, drift`, `duplicate mode '<m>' in 'modes'`, `'modes' must contain at least one mode other than
'off'`, `'default_mode' is required`, `'default_mode' "<m>" is not one of the listed 'modes'`, `mode_params: unknown mode`,
`takes no parameters`, `mode_params for '<m>' but '<m>' is not in 'modes'`, per-field range messages.

### Launch control (assists I4) {#assist-launch}

Block `launch_control` of an assist unit (grades `sport` and `racing`; Street rejects it with the grade message).
`ps::vehicle::LaunchAssistDesc`, controller `core/src/vehicle/launch_assist.h`. Channel `assist.launch_control` (a
plain 0/1 level, on = `> 0.5`; registered on a car that has the block OR a P8 two-step part; default = the block's
`default_enabled`, shipped `false` `[AS]` so a car carrying the unit launches as before until the assist is switched
on). Exactly one launch rpm form is required; every other key is optional (owner Q7, all `[AS]`):

| Key | Default | Range | Meaning |
|---|---|---|---|
| `launch_rpm` | - | > 0 | absolute launch rpm held against the open clutch (Racing unit: 5000 [AS], car_hyper's value tuned by V8 to boost >= 0.4 bar) |
| `launch_rpm_fraction` | - | (0, 1) | the same as a fraction of the engine's rev limiter (Sport unit: 0.55). Exactly one of the two |
| `default_enabled` | `false` | boolean or 0/1 | the channel's default |
| `arm_throttle` | 0.9 | (0.5, 1] | throttle must exceed this to arm |
| `arm_brake` | 0.5 | (0, 1] | brake must exceed this to arm |
| `release_brake` | 0.2 | [0, arm_brake) | brake below this in Armed starts the launch |
| `arm_max_speed_mps` | 0.5 | > 0 | speedo speed must be below this to arm and stay armed |
| `handover_speed_mps` | 6 | > arm_max_speed_mps | hand over to traction control above this speed |
| `max_launch_s` | 3 | > 0 | ... or after this long |
| `clutch_ramp_s` | 0.25 | >= 0 | fastest rise of the clutch capacity fraction, 1 / `clutch_ramp_s` per s (also the soft hand-over close) |
| `clutch_up_per_s` | 6 | > 0 | rise rate of the fraction below the slip target, scaled by `1 - slip/target` (capped by the ramp rate) |
| `clutch_down_per_s` | 12 | > 0 | fall rate of the fraction above the target, scaled by `slip/target - 1` |
| `slip_target` | On 0.10, Relaxed 0.12, Sport 0.14, Drift 0.20 | each (0, 1), keys `off`/`on`/`relaxed`/`sport`/`drift` | rear-slip target of the launch regulator per effective traction-control mode (Off mirrors On) |
| `slip_floor_mps` | 1.0 | > 0 | speed floor in the slip denominator (TC's formula with this floor) |
| `kp`, `ki_per_s`, `attack_rate_per_s`, `recovery_rate_per_s` | 2, 6, 20, 8 | > 0 | the throttle-ceiling slip PI (TC mode On's values) |
| `min_fraction` | 0.05 | [0, 1) | floor of the PI's throttle fraction |
| `max_limiter_fraction` | 0.8 | (0, 1) | G1: a resolved launch rpm at or above this fraction of the engine's limiter (a low garage limiter) is held at it, with a load warning (`"... held at H rpm"`); see below |

Rejections (each asserted in `tests/unit/test_assist_launch.cpp` L2): `grade '<g>' does not allow 'launch_control'`,
`launch_control must be an object`, `launch_control: unknown field '<k>'`, `exactly one of 'launch_rpm' (> 0) and
'launch_rpm_fraction' (in (0, 1)) is required`, per-field range messages, `slip_target: unknown mode`, `slip_target
values must be in (0, 1)`. At vehicle load the resolved launch rpm must lie strictly between the engine's idle rpm and
its own limiter (`launch rpm <x> must lie between the engine's idle rpm <i> and its own limiter rpm <l>`). G1: a resolved rpm at or above `max_limiter_fraction` x the limiter is not an error but is held at that cap (the midpoint of idle and the limiter when the cap is not above idle); the error remains for an rpm at or below idle that the cap does not lift.

**Q5 (owner): either variant, never both.** A car has EITHER the P8 two-step (a `launch_control` electronics part,
above) OR an assist unit with a `launch_control` block. Both on one car is rejected at load, whatever the order of the
entries: `vehicle '<name>': launch control: a car has either a two-step (launch_control) or an assist-unit launch
control, not both` (and again by `VehicleInstance`'s backstop for a code-built descriptor). Anti-lag is **not**
switched on by launch control (owner Q6); a car that wants it sets its own anti-lag channel.

**DCT launch control** (CVT plan C1 follow-up). On a car with an `automatic_tcu` the block arms only behind a
`dual_clutch` gearbox: its TCU owns the launch clutch, so it stands in for the auto-clutch condition of phase 1 (a
torque-converter automatic or a CVT never arms - the converter's stall speed caps the revs). Armed, the TCU holds the
launch clutch open while the engine is held at the launch rpm; in Launch and Handover the clutch capacity is the
regulator's fraction times `max(0, T_demand + T_sync)`, `T_sync = min(0.5 |slip rad/s|, 0.2 C)` [AS], so the clutch
both carries the engine torque and pulls the revs down. The gear-1 / not-shifting condition counts a decided or running
clutch-to-clutch shift as shifting. Measured on `car_sedan_dct.json` with the Racing unit (launch 5000 rpm, TC On target
0.10, `tests/validation/test_cvt_gearbox.cpp` `[launch]`): armed at 4847-4995 rpm with the car still, front slip mean
0.095 over 0.2-1.0 s after the brake release (max 0.101), 0-100 km/h 8.73 s against 8.92 s for the plain DCT launch.

**Car-specific launch rpm.** A generic unit fits any engine through `launch_rpm_fraction` (Sport: 0.55 of the limiter);
the Racing unit carries car_hyper's absolute 5000. A car with another engine attaches a unit file that `extends` the
shipped one and sets its own `launch_rpm` (setting `launch_rpm_fraction` there needs `launch_rpm` cleared: the
merge-patch rule `"launch_rpm": null`), or uses the fraction form.

**Phases** (`launch_phase` in the snapshot: 0 Idle, 1 Armed, 2 Launch, 3 Handover; one hashed byte plus a timer, a
ramp, the PI's ceiling and integral, hashed ONLY on a car that carries the block):

1. *Idle -> Armed*: channel on, the auto-clutch assist on, engine Running, throttle > `arm_throttle`, brake >
   `arm_brake`, speedo speed < `arm_max_speed_mps`, gearbox in gear 1 and not shifting.
2. *Armed*: the engine's second rev limit = the launch rpm through the engine's own limiter (the P8 `TcuRequest::
   limiter_rpm` path: fuel cut with its hysteresis, soft cut over its band), the auto-clutch is held open (capacity
   0), traction control is suspended (its physical slip feedback keeps updating),
   the aero hook `set_aero_launch_request(true)` (set at the end of the previous tick, so the wing follows one tick later) asks for the launch wing offset. Leaves to Idle when the channel,
   auto-clutch or engine stops, throttle < 0.5, speed >= `arm_max_speed_mps`, gear != 1 or a shift starts; to Launch
   when brake < `release_brake`.
3. *Launch*: the launch limit **stays** at the launch rpm (the engine is the held source: boost keeps building under
   load, so the clutch, not the engine, limits the torque). The clutch is the torque limiter: its capacity fraction `c`
   (0..1) multiplies the torque-following capacity of the auto-clutch (`min(c, 1) * max(0, T_demand + T_sync)`, so it
   never exceeds what the engine and the synchronising torque can carry) and is regulated on the driven wheels' worst
   positive slip (TC's formula, `slip_floor_mps`) against `slip_target[effective mode]`: `rel = slip/target - 1`; below
   the target `c` rises at `min(1/clutch_ramp_s, clutch_up_per_s * min(1, -rel))`, above it falls at
   `clutch_down_per_s * rel`. The engine's throttle ceiling comes from TC's one-sided slip PI on the same slip as the
   second, slower line. Leaves to Handover on speed > `handover_speed_mps` or time > `max_launch_s` (*soft*), or on
   throttle < 0.5, a shift, gear != 1, or the channel/auto-clutch/engine stopping (*hard*).
4. *Handover*: traction control starts from the PI's last ceiling (its fraction and integral are seeded, no torque
   jump) and runs again; the wing returns to the aero controller. After a *soft* hand-over whose `c` is still below 1
   the launch limit and `c` stay in force until `c` reaches 1, so the clutch reaches the plain torque-following close
   without a step and the engine is not released against a half-open clutch. `c` keeps regulating the slip there: it
   climbs at the ramp rate only while the slip is at or below the target and falls at `clutch_down_per_s * rel` above
   it (racing_game PR #5: an open-loop ramp dumped the held launch-rpm engine, at full boost, into the wheels: slip
   2.0 on car_hyper in the Racing drive mode); after `max_launch_s` in this overlay the ramp is open-loop again, so
   the hand-over always ends. After a hard one (or `c` already 1) nothing is kept. Then Idle at the next tick's first substep.

**Boost and the launch rpm.** The unloaded engine builds boost against the closed clutch only as fast as the limiter
cycling allows, so boost at release is a function of the launch rpm. Measured on car_hyper (1.5 s armed): 4500 rpm
0.29 bar, 5000 0.50 (0.57 steady), 5500 0.97, 6000 1.4. The Racing unit therefore ships `launch_rpm` 5000 `[AS]`,
tuned by `test_assist_launch.cpp` V8 to boost >= 0.4 bar (V8d records the 4500 miss). The Sport unit's
`launch_rpm_fraction` 0.55 gives 4537 rpm on car_hyper (V8e prints its boost; it is a generic fit-any-engine unit, not
tuned for car_hyper's boost).

**Precedence with the low-speed fix (assists I2, D-LS1..3).** While the launch control owns the car (Armed, Launch)
the powertrain sees traction control neutral (fraction 1, no ceiling, no ignition cut), so TC's crawl-speed clutch cap
and its lock inhibit never act; the launch clutch cap and throttle ceiling are the only interventions. In Idle and
Handover the I2 path is untouched. (Tests: `test_assist_launch.cpp` P1.)

Snapshot (never hashed): `PowertrainSnapshot::launch_phase`, `launch_slip_target` (the effective mode's target, 0
without the block), `launch_control_active` (also true while the unit's LC is Armed/Launch).

### Aero automation (assists I5) {#assist-aero}

Block `aero_control` of an assist unit (grade `racing` only; Street and Sport reject it with the grade message above).
`ps::vehicle::AeroControlDesc`, a port of racing_game's rear-wing law (`core/src/session.cpp`, "automatic_rear_wing";
the defaults are racing_game's `rg/environment.h` values, all `[D]`). Every key is optional:

| Key | Default | Range (racing_game's where it has one) | Meaning |
|---|---|---|---|
| `surface` | `"rear_wing"` | non-empty string | the car's aero surface (by name) the controller owns |
| `default_enabled` | `false` | boolean or 0/1 | the channel's default value; the shipped Racing unit sets `true` `[AS, owner: Racing = aero automation]` |
| `cruise_offset_deg` | 4 | [-8, 55] | wing offset at and above `cruise_ramp_end_m_s` |
| `cruise_ramp_start_m_s` / `cruise_ramp_end_m_s` | 10 / 30 | [0, 150], end > start | the cruise offset ramps linearly with airspeed between them |
| `airbrake_offset_deg` | 45 | [0, 55] | offset at full airbrake demand |
| `airbrake_min_speed_m_s` | 20 | [0, 150] | the airbrake term exists only at or above this airspeed |
| `airbrake_brake_start` / `airbrake_brake_full` | 0.45 / 1.0 | [0, 0.99] / [0.01, 1], full > start | brake channel range mapped to demand 0..1 |
| `launch_offset_deg` | 10 `[AS]` | [-8, 55] | the offset a launch control asks for (the car_hyper maximum-downforce offset is 9-11 deg `[D]`) |

**The law** (stateless, every tick, `ps::vehicle::aero_control_command`): `v` = chassis-frame x of (chassis velocity -
the vehicle's own wind), read at tick start; `target = cruise_offset * clamp((v - ramp_start)/(ramp_end - ramp_start),
0, 1)`; if `v >= airbrake_min_speed`: `demand = clamp((brake - brake_start)/(brake_full - brake_start), 0, 1)` and
`target += (airbrake_offset - target) * demand`; launch hook set: `target = launch_offset` instead (yield order launch
control > airbrake > cruise; the hook is `VehicleInstance::set_aero_launch_request`, true while the assist unit's launch
control is Armed or Launching, [above](#assist-launch); the request is the previous tick's state, so the wing lags LC by one tick); `radians = target * pi / 180`; `limit = radians >= 0 ? max_offset_rad : -min_offset_rad`;
`command = limit > 0 ? clamp(radians / limit, -1, 1) : 0`. The brake input is the `brake` channel (the handbrake is
not read). The command goes through the surface's own `rate_rad_s` slew like any command.

**Fitment.** The controller is fitted iff the unit carries the block AND the car's `aero.surfaces` has a surface of
that name. **A unit naming a surface the car lacks is not a load error**: it is simply not fitted (bit 6 of
`assist_fitted_bits` 0, no channel), so one generic Racing unit attaches to a wingless car. (Alternative: reject at
load. Rejected: it makes a unit car-specific, and a UI that offers one Racing unit to every car would need to know the
car's aero first.)

**Channel** `assist.aero_control`, registered only on a car where the controller is fitted (every other car's channel
list is unchanged); default = `default_enabled`; `>= 0.5` is on. With the channel on, the controller owns the
surface's command each tick and the World command (`World::set_aero_surface_command`) for that surface is ignored; with
it off the World command stands exactly as without the unit (the controller only borrows the command for the tick's
substeps and puts the World's value back, so nothing is latched).

**Snapshot** (`PowertrainSnapshot`, never hashed): `aero_control_active`, `aero_control_target_deg` (0 when not
active), bit 6 of `assist_fitted_bits` and `assist_active_bits`. **Hash:** no new state; the desc block is not folded
(a car with the channel off hashes exactly like the same car with a unit without the block), the surface offset that the
slewed command moves is already hashed. Tests: `tests/unit/test_assist_aero.cpp` ([assist][aero]).

**Hash (the unit's TC mode table).** The resolved mode table is folded into the vehicle's state hash only for a unit whose `modes` go beyond
Off/On (a Street unit hashes exactly like a car without a unit). The effective mode is a pure function of channel and
desc and is not state.

## ABS and ESC (assists I5 part 2 / I6) {#abs-esc}

Two blocks of the assist unit (`kind: assist_unit`, any grade). ABS and ESC remain sensor-based: they read
the wheels' peripheral speeds (steer and yaw compensated), the gyro yaw rate, the IMU longitudinal
acceleration, the brake / handbrake channels and the mean front road-wheel angle - never the ground speed, a tyre slip,
a contact load or a surface. Both act only through the car's own service-brake rows (DirectTorqueBrakes): each corner's
row reads its own command instead of the brake channel. Descriptors `ps/vehicle/abs_esc_desc.h`, law
`core/src/vehicle/abs_esc.h` (per substep, allocation-free). Shipped part: `data/electronics/assist_unit_abs_esc.json`
(grade street, every number [AS]); no shipped car carries it.

```json
"abs": { "default_enabled": true, "target_slip": 0.12, "min_speed_mps": 2.0, "slip_floor_mps": 3.0,
         "kp": 3.0, "ki_per_s": 30.0, "release_rate_per_s": 60.0, "apply_rate_per_s": 15.0,
         "reanchor_rate_per_s": 10.0, "reanchor_max_decel_mps2": 2.0, "reanchor_scale": 0.05, "rear_select_low": true,
         "front_split_max": 0.2, "front_split_rate_per_s": 0.4 },
"esc": { "modes": ["off", "on", "sport"], "default_mode": "on",
         "characteristic_speed_mps": 25.0, "max_lateral_accel_mps2": 8.5, "min_speed_mps": 5.0, "brake_rate_per_s": 10.0,
         "deadband_rad_s": 0.05, "deadband_fraction": 0.10, "brake_gain_per_rad_s": 2.0, "max_brake": 0.8,
         "torque_gain_per_rad_s": 1.0,
         "mode_params": { "sport": { "deadband_rad_s": 0.10, "deadband_fraction": 0.20, "max_brake": 0.6 } } }
```

Every key is optional (defaults as above, except `front_split_max` 1 and `front_split_rate_per_s` 1 = no limit when
absent); unknown keys, `target_slip` outside (0, 1), `reanchor_scale` outside [0, 1), `front_split_max` outside [0, 1],
`max_brake` above 1, `esc` without `abs`, `esc.modes` without `"on"` and a `mode_params` entry for an unlisted mode are
rejected naming the field. A car needs a service brake corner with `max_torque_nm > 0`.

**Reference speed** `v_ref` (shared, signed: forward > 0). Initialised from the wheels (braking: the wheel with the
largest |v|, else the smallest). Each substep it integrates the IMU acceleration, then:
- braking (brake or handbrake > 0.02): lifted to the fastest wheel in the direction of travel; set to the fastest wheel
  if every wheel turns the other way faster (a car sliding backward after a spin); when every regulated wheel's
  applied scale is at or below `reanchor_scale` it relaxes toward the fastest wheel at `reanchor_rate_per_s` (the
  integrated IMU lacks the yaw-rate x lateral-velocity term and runs above the true speed in a slide). That pull-down
  is bounded: it may lower `v_ref` by at most `reanchor_max_decel_mps2` (default 2.0 [AS]) on top of the IMU term. On
  very low grip (ice, lambda_mu 0.1) every wheel locks even at a few percent of the brake, the fastest wheel is then a
  locked wheel, and the unbounded relaxation dragged `v_ref` down with the wheels (27.8 -> 20.2 m/s in 0.25 s while the
  car did 27.5) - the slip error vanished and ABS held the lock;
- not braking: relaxes toward the slowest wheel at `reanchor_rate_per_s`; while only ESC brakes, toward the slowest
  wheel ESC does not brake (it rolls freely, so its sensor speed is the longitudinal speed).

**ABS** (`assist.abs` >= 0.5, `v_ref` >= `min_speed_mps`, wheel command > 0.02): slip = (|v_ref| - dir v) / max(|v_ref|,
`slip_floor_mps`); integral += ki (slip - target) dt in [0, 1]; scale = clamp(1 - kp max(0, slip - target) - integral,
0, 1), rate-limited (`release_rate_per_s` down, `apply_rate_per_s` up). Rear axle select-low (`rear_select_low`). Front
axle: the higher scale may exceed the lower by a split that grows from 0 at `front_split_rate_per_s` up to
`front_split_max` while the front axle is regulated (the split-mu yaw-moment build-up limit), else resets to 0. The
corner command is multiplied by its applied scale.

**ESC** (mode on/sport, `v_ref` >= `min_speed_mps`, so never in reverse): target r_t = v_ref delta / (L (1 + (v_ref /
v_ch)^2)), |r_t| <= `max_lateral_accel_mps2` / v_ref, delta = the mean front road-wheel angle, L = the wheelbase.
err = r - r_t, over = |err| - (deadband + fraction |r_t|). Over > 0: oversteer (|r| > |r_t| or opposite signs) brakes
the FRONT wheel(s) on the side with sign(y) = sign(-err), understeer the REAR wheel on that side; command
clamp(gain over, 0, max_brake), each corner rate-limited at `brake_rate_per_s`, added to the pedal command (sum capped
at 1) and then passed through ABS. Engine torque fraction clamp(1 - torque_gain over, 0, 1), combined with TC's own
fraction by min through the TC throttle ceiling. Mode codes: 0 off, 1 on, 2 sport (see the
[overview](#assists-overview) for the channel mapping).

**Channels, bits, hash.** `assist.abs` / `assist.esc` exist only on a car whose unit has the block (defaults: the
default drive mode's `abs_default` / `esc_default`, else the block's own). Fitted bits 8 (ABS) / 9 (ESC); active bits
8/9 = switched on. Snapshot (never hashed): `abs_active`, `esc_active`, `abs_reference_speed_m_s`,
`esc_yaw_rate_target`, `esc_mode_requested/effective`, `esc_modes_available_bits`; `PowertrainDebug::abs_wheel_scale`,
`esc_wheel_brake`, `wheel_brake_command` (tests only; `wheel_brake_command` is the ABS/ESC output and reads 0 while
both are off). The blocks, the controller state and the IMU memory are hashed only on a car that carries a block, so
every other car hashes as before. Drive modes: `abs_default` (bool) and `esc_default` (mode name) per mode, applied on a
mode change as pending channel writes like `traction_control_default`.

**Measured** (car_sedan + the unit, tests/unit/test_abs_esc.cpp [abs][esc]): 100-0 km/h 37.92 m vs the
load-transfer-corrected estimate 38.37 m (-1.2 %), ABS off 44.32 m; heading change after 1.5 s at steer 0.3 under full
braking 26.6 deg (ABS off -3.1 deg); split-mu 1.0 / 0.1 stop with a heading-holding driver 1.6 deg max (ABS off spins);
FMVSS 126 sine with dwell on a rear-grip x0.75 fixture: ESC on YRR 0.19 / 0.00, lateral 3.16 m (bounds 0.35 / 0.20 /
1.83 m), ESC off spins. `tests/validation/test_vehicle_braking.cpp` (PHYS-019) runs the same stop with the real unit.
On lambda_mu 0.1 (ice, 100 km/h brake step, 10 s): before the bound the wheels sat at slip -0.5..-1.0 for 7.0 s and `v_ref` fell to
0.02 x the true speed (218.1 m, 16.1 m/s left); with it the wheels hold slip -0.15 at a scale of 0.05, `v_ref` stays 0.7 m/s
below the true speed (217.0 m, 15.9 m/s left). Lambda_mu 0.6 (dirt) 58.79 m and tyres x0.35 on asphalt 105.65 m, dry 37.92 m: unchanged
to the digit (tests L3a-L3d).

## Drive modes (assists I7, vault PHYS-074) {#drive-modes}

`kind: "drive_modes"`, `ps::vehicle::DriveModesDesc` (`ps/vehicle/drive_modes_desc.h`), stored as `VehicleDesc::drive_modes`.
A **car-specific** part (it overrides numbers of the car's transmission controller): attached like the other electronics parts, as a
`controllers` entry of type `electronics` or through `VehicleLoadOptions::electronics` (`"none"` removes it). At most one per
car (`"at most one drive_modes per vehicle"`); `extends` works (RFC 7396, so a `null` mode removes it); unknown keys are
rejected at every level. Shipped: `data/electronics/drive_modes_car_hyper_n2o.json` (owner values 2026-10-13, `[AS]`; racing added in Q3 / I8 `[AS]`),
attached to `car_hyper_n2o`, and `data/electronics/drive_modes_car_hyper.json` (Q3 / I8: normal overrides nothing, racing
`[AS]`) attached to `car_hyper`. Alternative (a block inside the assist unit): rejected - the unit files are generic per grade,
the mode numbers belong to one car's gearing and engine.

```json
{ "format": "physics_sim.electronics/1", "kind": "drive_modes", "default_mode": "normal",
  "modes": {
    "eco":    { "pedal_bend": { "x": 0.6, "y": 0.3 },
                "auto_shift": { "up_rpm_at_zero_throttle": 1800, "up_rpm_at_full_throttle": 6500,
                                "down_rpm_at_zero_throttle": 1100, "down_rpm_at_full_throttle": 2800 },
                "auto_clutch": { "close_time_s": 0.15 }, "traction_control_default": "on",
                "launch_control_allowed": false, "nitrous_arm_allowed": false },
    "normal": { "...": "..." }, "sport": { "...": "..." } } }
```

| Key | Meaning / rejection |
|---|---|
| `modes` | required non-empty object keyed `eco` / `normal` / `sport` / `racing` (Q3 / I8); a mode is **offered** iff it is listed |
| `default_mode` | required, one of the listed modes (the channel default and the target of an unoffered request) |
| `pedal_bend` `{x, y}` | one optional point: `x` in `(0, 1)`, `y` in `[0, 1]` (`"pedal_bend x must be in (0, 1)"` / `"... y must be in [0, 1]"`, `"needs both 'x' and 'y'"`); piecewise linear through `(0,0)`, `(x,y)`, `(1,1)`. Absent = identity (bit-identical) |
| `auto_shift` | any of the four `up_rpm_at_*`, `down_rpm_at_*` of the car's `auto_shift` (rpm `> 0`); a missing key keeps the car's. The merged values must keep `down < up` at zero and at full throttle (`"... downshift rpm must stay below the upshift rpm ..."`) |
| `auto_clutch.close_time_s` | `> 0`; overrides `auto_clutch.close_time_s` (the post-shift close rate `1/close_time_s`) |
| `traction_control_default` | `off`/`on`/`relaxed`/`sport`/`drift`: the TC mode the car switches to when this mode becomes effective |
| `launch_control_default` | optional boolean (assists I7b): the launch-control enable the car switches to when this mode becomes effective (see below). `true` next to `launch_control_allowed: false` in the same mode is rejected (`"launch_control_default true contradicts launch_control_allowed false"`: the data would promise an enable the gate forbids). On a car without a fitted launch control it is ignored (like `traction_control_default` on a car without TC: the part is car-specific but may be shared through `extends`) |
| `launch_control_allowed`, `nitrous_arm_allowed` | booleans, default `true` |
| `epb_mode` | optional `park_only` / `handbrake` (Q3 / I8): the EPB law in this mode; absent = the `epb` part's own `epb_mode`. Ignored on a car without the part |
| `abs_default` | optional boolean (I5 part 2): written into `assist.abs` when this mode becomes effective (same rule as `traction_control_default`); ignored on a car without the `abs` block |
| `esc_default` | optional `off` / `on` / `sport` (I6): written into `assist.esc` the same way; ignored on a car without the `esc` block |

Shift and clutch-close overrides require a `manual_tcu` or `automatic_tcu`. On automatic/DCT boxes, the shift points override the active D/S schedule, and the close time controls the shift-clutch engagement ramp. Traction-control, launch-control, nitrous and EPB mode rules remain available with either controller. The pedal plan P1 policy is unchanged: mode-specific pedal bends apply to automatic/DCT/CVT gearboxes; manual and sequential gearboxes keep their unbent pedal mapping.

**Channel** `assist.drive_mode`, registered only on a car with the part: `0` eco, `1` normal, `2` sport, `3` racing (append-only; Q3 / I8 added racing, so a value `>= 3` now means racing), rounded
and clamped; default = the part's `default_mode` code. A request for a mode the part does not offer maps to the part's
`default_mode` `[AS]` (drive modes have no intervention order, so the TC rule "nearest more-intervening" has no analogue; the
default is the one mode the owner approved, and clamping to a neighbour could select Sport on a car that forbids it).

**Pedal bend.** `out = bend(normal(pedal))`, applied in the throttle-command stage after the car's own `pedal_map` /
linkage curve and before the hand lever - only with an automatic, DCT or CVT gearbox (pedal plan P1; none is shipped, so
on every current car the bend is skipped bit for bit and the key is accepted but inert). The TCU (launch law, auto-shift thresholds, kickdown) and the traction/launch throttle
ceilings work on this command; TC's pedal gating and launch-control arming stay on the raw pedal (driver intent).

**Traction-control default per mode.** In physics, not in the game: when the *effective* drive mode changes and the new mode
has `traction_control_default`, the effective TC mode becomes it (through the I1 availability fallback of the car's unit) and
(assists I7c) the physics writes the mode's code into the `assist.traction_control` channel, applied by `World::step` after the
tick (the channel keeps the raw code even where the fallback maps it to another mode). The driver's next write of any value
wins. A mode without a default leaves the channel alone. At spawn nothing is written: the TC channel default is the default
mode's `traction_control_default`.

**Launch-control default per mode (assists I7b, I7c).** Exactly the TC rule, for the on/off enable of `assist.launch_control`:
when the *effective* drive mode changes and the new mode has `launch_control_default`, the physics writes `1` / `0` into the
channel (only where the assist is fitted). At spawn the channel default is the default mode's `launch_control_default` (like the
TC channel). Not visible as a new field: `assist_active_bits` bit 4 is the effective enable (after the allowed gate) and
`launch_phase` shows the arming. Hash: the mode table's `launch_control_default` values are folded **only** on a car with at
least one offered mode that sets one; every other car (including a `drive_modes` part without the key) hashes as before.
Since I7c there is no override state (it was a flag, value and last seen channel value per assist).

**Gates.** `launch_control_allowed false`: the launch-control assist cannot arm (both the unit's and a P8 two-step are driven
through the same `assist.launch_control` gate on the unit). `nitrous_arm_allowed false`: the kit's arm input is forced to 0.

**Kickdown.** `auto_shift.kickdown_throttle` in the car's `manual_tcu` ([vehicle.md](vehicle.md)); spent by a gear change, re-armed when the command drops below it. `car_hyper_n2o` uses `min_time_in_gear_s` 2.0 `[AS]` (measured on an accelerating pull: 1.0 s bounces once in Normal and Sport, 1.5 s still once in Sport, 2.0 s none in any mode).

**Snapshot** (`PowertrainSnapshot`, never hashed): `drive_mode_requested`, `drive_mode_effective`,
`drive_modes_available_bits` (bit 0 eco, 1 normal, 2 sport, 3 racing; all 0 without the part). **Hash:** the desc and the state (initialised flag,
last effective mode) are folded only on a car with the part, so every other car and scenario hashes as before. Tests:
`tests/unit/test_assist_drive_modes.cpp` ([assist][vehicle][drivetrain]), `tests/unit/test_assist_launch_default.cpp` (I7b,
LD1-LD6), `tests/unit/test_assist_channel_writeback.cpp` (I7c, W1-W9).

## Electronic parking brake (Q3 / I8) {#epb}

`kind: "epb"`, `ps::vehicle::EpbDesc` (`ps/vehicle/epb_desc.h`), stored as `VehicleDesc::epb`; at most one per car, attached
like the other parts. Shipped: `data/electronics/epb_generic.json` (`car_hyper`, `car_hyper_n2o`). The car needs
`brake_system.handbrake` rows (wheels and `max_torque_nm > 0`): the EPB acts only through them (`controls.handbrake` raised to
1), it has no rows of its own; the gearbox park pawl (`park_lock`) stays separate.

```json
{ "format": "physics_sim.electronics/1", "kind": "epb", "max_engage_speed_mps": 2.0, "epb_mode": "park_only" }
```

| Key | Meaning / rejection |
|---|---|
| `max_engage_speed_mps` | finite `> 0`, default 2.0 `[AS]`: the `park_only` gate on the chassis speed at the tick start |
| `epb_mode` | `park_only` (default) / `handbrake`: the law when the effective drive mode sets no `epb_mode` |

Owner rule, verbatim: "No, unless special modes are used (racing, drift), the parking brake should refuse to engage while
speed is significant." **Channel** `epb`, a level (on at `>= 0.5`), default 0, registered only with the part. Law per tick
(mode = the effective drive mode's `epb_mode`, else the part's):

- level 0: released, refusal cleared.
- `handbrake`: applied at any speed (`epb_state` 2).
- `park_only`: a NEW request (a rising edge, or a `handbrake` application that meets `park_only` through a mode switch)
  latches (`epb_state` 1) below `max_engage_speed_mps`, else it is **refused** (`epb_refused`, state 0) and dropped - never
  pended: it engages only after the level falls and rises again, even if the car slows or the mode changes. Once latched it
  stays latched whatever the speed until the level falls.

There is no retry latch in physics or in racing_game. The state survives a vehicle reset. **Snapshot** (never hashed):
`epb_state`, `epb_refused`, `assist_fitted_bits` / `assist_active_bits` bit 7. **Hash:** the desc, the per-mode `epb_mode`
values and the state (last level, refused, state) are folded only on a car with the part; no shipped scenario uses one.
Tests: `tests/unit/test_epb.cpp` L1-L2 C1 E1-E5 R1 ([epb], five recorded sabotages).
