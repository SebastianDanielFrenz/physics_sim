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
| launch control | EITHER the P8 two-step part (`kind: launch_control`) OR the assist unit's `launch_control` block (grades Sport/Racing); never both | `assist.launch_control` (the P8 part may rename it with its `channel` key) | only on a car with one of the two (a P8 part with `fitted: false` still registers it) | 0 for the P8 part; the unit block's `default_enabled` (shipped `false`) | 0 / 1 |
| rear-wing automation | the assist unit's `aero_control` block (grade Racing) | `assist.aero_control` | only when the unit has the block AND the car's `aero.surfaces` has the named surface | the block's `default_enabled` (shipped Racing unit: `true`) | 0 / 1, on at `>= 0.5` |
| drive modes | the car-specific `drive_modes` electronics part | `assist.drive_mode` | only on a car with the part | the part's `default_mode` | 0 eco, 1 normal, 2 sport (rounded and clamped; a mode the part does not list maps to its `default_mode`) |
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
   input to 0 whatever `nitrous_arm` says.
4. **Drive mode over the driver's TC choice, until the driver changes it.** When the *effective* drive mode changes and
   the new mode has a `traction_control_default`, the effective TC mode becomes that default and stays so until the
   `assist.traction_control` channel **value changes**. Writing the same value again does not release it (see the UI
   pitfalls below). A mode without a default leaves a running override alone. The snapshot's `traction_mode_requested`
   already includes the override.
5. **Pedal bend.** A drive mode's bend is applied after the car's own pedal curve; the auto-shift, launch law, kickdown
   and the TC/LC throttle ceilings work on the *bent* command (ceilings are fractions of it), while TC's pedal gating
   and LC arming use the raw pedal (driver intent).
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

A HUD shows hidden (fitted bit clear), fitted-off, fitted-on. Other fields: `traction_mode_requested` /
`traction_mode_effective` / `traction_modes_available_bits` (bit *n* = code *n* is offered; 0 when TC is not fitted);
`drive_mode_requested` / `drive_mode_effective` / `drive_modes_available_bits` (bit 0 eco, 1 normal, 2 sport; all 0
without the part); `launch_phase` (0 Idle, 1 Armed, 2 Launch, 3 Handover), `launch_slip_target`, `launch_control_active`
(true while the unit's LC is Armed/Launch, or the P8 two-step is latched); `aero_control_active`,
`aero_control_target_deg`.

### UI pitfalls

- **Cycling the TC mode after a drive-mode change.** A button that cycles from `traction_mode_effective` and writes the
  next code can write exactly the value the channel already holds (e.g. the driver set 3, the drive mode then forced 2,
  the button wants 3 again). The physics reads that as "no driver change" and keeps the drive mode's override. Write a
  value that differs from the channel's last value, or cycle from the channel value rather than from the effective mode
  (guard: `tests/unit/test_assist_all_together.cpp` P1).
- **Do not write `assist.traction_control` on a drive-mode change.** The physics applies the mode's default itself; a
  client write of a different code silently wins over the mode default.
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
| `rpm` | required, finite, `> 0` | the second rev limit. The vehicle loader additionally requires the (first) engine's idle rpm `< rpm <` its own limiter rpm (`"launch rpm R must be below the engine's own limiter rpm L"`) |
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
    "mode_params": { "sport": { "target_slip": 0.18, "kp": 1.5, "ki_per_s": 4.0, "attack_rate_per_s": 10.0,
                                "recovery_rate_per_s": 10.0, "min_fraction": 0.15, "ignition_cut": false } } },
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
| `ignition_cut` | whether a deep intervention may cut ignition | `true` (today) |

Switching modes keeps the controller's integral and fraction (no jump). Plan-B1 starting values (`[AS]`, measured
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

Rejections (each asserted in `tests/unit/test_assist_launch.cpp` L2): `grade '<g>' does not allow 'launch_control'`,
`launch_control must be an object`, `launch_control: unknown field '<k>'`, `exactly one of 'launch_rpm' (> 0) and
'launch_rpm_fraction' (in (0, 1)) is required`, per-field range messages, `slip_target: unknown mode`, `slip_target
values must be in (0, 1)`. At vehicle load the resolved launch rpm must lie strictly between the engine's idle rpm and
its own limiter (`launch rpm <x> must lie between the engine's idle rpm <i> and its own limiter rpm <l>`).

**Q5 (owner): either variant, never both.** A car has EITHER the P8 two-step (a `launch_control` electronics part,
above) OR an assist unit with a `launch_control` block. Both on one car is rejected at load, whatever the order of the
entries: `vehicle '<name>': launch control: a car has either a two-step (launch_control) or an assist-unit launch
control, not both` (and again by `VehicleInstance`'s backstop for a code-built descriptor). Anti-lag is **not**
switched on by launch control (owner Q6); a car that wants it sets its own anti-lag channel.

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
   0), traction control is suspended (its estimator keeps running, so the slip PI has a speed estimate at release),
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
   the launch limit and `c` stay in force (`c` climbing at the ramp rate) until `c` reaches 1, so the clutch reaches
   the plain torque-following close without a step and the engine is not released against a half-open clutch; after
   a hard one (or `c` already 1) nothing is kept. Then Idle at the next tick's first substep.

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

## Drive modes (assists I7, vault PHYS-074) {#drive-modes}

`kind: "drive_modes"`, `ps::vehicle::DriveModesDesc` (`ps/vehicle/drive_modes_desc.h`), stored as `VehicleDesc::drive_modes`.
A **car-specific** part (it overrides numbers of the car's own `manual_tcu`): attached like the other electronics parts, as a
`controllers` entry of type `electronics` or through `VehicleLoadOptions::electronics` (`"none"` removes it). At most one per
car (`"at most one drive_modes per vehicle"`); `extends` works (RFC 7396, so a `null` mode removes it); unknown keys are
rejected at every level. Shipped: `data/electronics/drive_modes_car_hyper_n2o.json` (owner values 2026-10-13, `[AS]`),
attached to `car_hyper_n2o`. Alternative (a block inside the assist unit): rejected - the unit files are generic per grade,
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
| `modes` | required non-empty object keyed `eco` / `normal` / `sport`; a mode is **offered** iff it is listed |
| `default_mode` | required, one of the listed modes (the channel default and the target of an unoffered request) |
| `pedal_bend` `{x, y}` | one optional point: `x` in `(0, 1)`, `y` in `[0, 1]` (`"pedal_bend x must be in (0, 1)"` / `"... y must be in [0, 1]"`, `"needs both 'x' and 'y'"`); piecewise linear through `(0,0)`, `(x,y)`, `(1,1)`. Absent = identity (bit-identical) |
| `auto_shift` | any of the four `up_rpm_at_*`, `down_rpm_at_*` of the car's `auto_shift` (rpm `> 0`); a missing key keeps the car's. The merged values must keep `down < up` at zero and at full throttle (`"... downshift rpm must stay below the upshift rpm ..."`) |
| `auto_clutch.close_time_s` | `> 0`; overrides `auto_clutch.close_time_s` (the post-shift close rate `1/close_time_s`) |
| `traction_control_default` | `off`/`on`/`relaxed`/`sport`/`drift`: the TC mode the car switches to when this mode becomes effective |
| `launch_control_allowed`, `nitrous_arm_allowed` | booleans, default `true` |

An override needs the car's `manual_tcu` (every shipped car has one).

**Channel** `assist.drive_mode`, registered only on a car with the part: `0` eco, `1` normal, `2` sport (append-only), rounded
and clamped; default = the part's `default_mode` code. A request for a mode the part does not offer maps to the part's
`default_mode` `[AS]` (drive modes have no intervention order, so the TC rule "nearest more-intervening" has no analogue; the
default is the one mode the owner approved, and clamping to a neighbour could select Sport on a car that forbids it).

**Pedal bend.** `out = bend(normal(pedal))`, applied in the throttle-command stage after the car's own `pedal_map` /
linkage curve and before the hand lever. The TCU (launch law, auto-shift thresholds, kickdown) and the traction/launch throttle
ceilings work on this command; TC's pedal gating and launch-control arming stay on the raw pedal (driver intent).

**Traction-control default per mode.** In physics, not in the game: when the *effective* drive mode changes and the new mode
has `traction_control_default`, the effective TC mode becomes it (through the I1 availability fallback of the car's unit) and
stays so until the driver changes the `assist.traction_control` channel value (a changed *value*; writing the same value is not
seen). A change of mode re-applies the default over the driver's choice. A mode without a default leaves a running override
alone. At spawn there is no override: the TC channel default is the default mode's `traction_control_default`.

**Gates.** `launch_control_allowed false`: the launch-control assist cannot arm (both the unit's and a P8 two-step are driven
through the same `assist.launch_control` gate on the unit). `nitrous_arm_allowed false`: the kit's arm input is forced to 0.

**Kickdown.** `auto_shift.kickdown_throttle` in the car's `manual_tcu` ([vehicle.md](vehicle.md)); spent by a gear change, re-armed when the command drops below it. `car_hyper_n2o` uses `min_time_in_gear_s` 2.0 `[AS]` (measured on an accelerating pull: 1.0 s bounces once in Normal and Sport, 1.5 s still once in Sport, 2.0 s none in any mode).

**Snapshot** (`PowertrainSnapshot`, never hashed): `drive_mode_requested`, `drive_mode_effective`,
`drive_modes_available_bits` (bit 0 eco, 1 normal, 2 sport; all 0 without the part). **Hash:** the desc and the state (last
effective mode, TC override flag/code, last seen TC channel value) are folded only on a car with the part, so every other
car and scenario hashes as before. Tests: `tests/unit/test_assist_drive_modes.cpp` ([assist][vehicle][drivetrain]).
