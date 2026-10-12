# `physics_sim.electronics/1` — electronics part files (engine garage P8)

`data/electronics/*.json`. Loader: `core/src/io/electronics_io.cpp` (`ps::io::load_electronics_json`, header
`ps/io/electronics_io.h`); applied by `load_vehicle_json` (`core/src/io/vehicle_io.cpp`). Descriptor:
`ps::drivetrain::LaunchControlDesc` (`ps/drivetrain/controller_desc.h`).

A car has only the electronics it was built with (garage decision D11); everything else is grafted on as a part. The
physics refuses an assist the car does not have, so a script or another client cannot switch it on either.

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
(`assist_active_bits`; bit4 = launch_control switched on; since assists I4 bit4 means "launch control" for either
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
| `channel` | optional, default `"assist.launch_control"` | runtime arm switch, a plain 0/1 level (`>= 0.5` = on); resolved with the vehicle's channel prefix |
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
plain 0/1 level, `>= 0.5` = on; registered on a car that has the block OR a P8 two-step part; default = the block's
`default_enabled`, shipped `false` `[AS]` so a car carrying the unit launches as before until the assist is switched
on). Exactly one launch rpm form is required; every other key is optional (owner Q7, all `[AS]`):

| Key | Default | Range | Meaning |
|---|---|---|---|
| `launch_rpm` | - | > 0 | absolute launch rpm held against the open clutch (Racing unit: 4500, car_hyper's value) |
| `launch_rpm_fraction` | - | (0, 1) | the same as a fraction of the engine's rev limiter (Sport unit: 0.55). Exactly one of the two |
| `default_enabled` | `false` | boolean or 0/1 | the channel's default |
| `arm_throttle` | 0.9 | (0.5, 1] | throttle must exceed this to arm |
| `arm_brake` | 0.5 | (0, 1] | brake must exceed this to arm |
| `release_brake` | 0.2 | [0, arm_brake) | brake below this in Armed starts the launch |
| `arm_max_speed_mps` | 0.5 | > 0 | speedo speed must be below this to arm and stay armed |
| `handover_speed_mps` | 6 | > arm_max_speed_mps | hand over to traction control above this speed |
| `max_launch_s` | 3 | > 0 | ... or after this long |
| `clutch_ramp_s` | 0.25 | >= 0 | clutch capacity ramp 0 -> 1 after release |
| `slip_target` | On 0.10, Relaxed 0.12, Sport 0.14, Drift 0.20 | each (0, 1), keys `off`/`on`/`relaxed`/`sport`/`drift` | rear-slip target of the launch PI per effective traction-control mode (Off mirrors On) |
| `slip_floor_mps` | 1.0 | > 0 | speed floor in the slip denominator (TC's formula with this floor) |
| `kp`, `ki_per_s`, `attack_rate_per_s`, `recovery_rate_per_s` | 2, 6, 20, 8 | > 0 | the slip PI (TC mode On's values) |
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
the Racing unit carries car_hyper's absolute 4500. A car with another engine attaches a unit file that `extends` the
shipped one and sets its own `launch_rpm` (setting `launch_rpm_fraction` there needs `launch_rpm` cleared: the
merge-patch rule `"launch_rpm": null`), or uses the fraction form.

**Phases** (`launch_phase` in the snapshot: 0 Idle, 1 Armed, 2 Launch, 3 Handover; one hashed byte plus a timer, a
ramp, the PI's ceiling and integral, hashed ONLY on a car that carries the block):

1. *Idle -> Armed*: channel on, the auto-clutch assist on, engine Running, throttle > `arm_throttle`, brake >
   `arm_brake`, speedo speed < `arm_max_speed_mps`, gearbox in gear 1 and not shifting.
2. *Armed*: the engine's second rev limit = the launch rpm through the engine's own limiter (the P8 `TcuRequest::
   limiter_rpm` path: fuel cut with its hysteresis, soft cut over its band), the auto-clutch is held open (capacity
   0), traction control is suspended (its estimator keeps running, so the slip PI has a speed estimate at release),
   the aero hook `set_aero_launch_request(true)` asks for the launch wing offset. Leaves to Idle when the channel,
   auto-clutch or engine stops, throttle < 0.5, speed >= `arm_max_speed_mps`, gear != 1 or a shift starts; to Launch
   when brake < `release_brake`.
3. *Launch*: the limiter is lifted; the clutch capacity ramps 0 -> 1 over `clutch_ramp_s` on the torque-following law
   of the auto-clutch (`min(ramp, 1) * max(0, T_demand + T_sync)`, so it never exceeds what the engine and the
   synchronising torque can carry: no dump at the launch rpm); the engine's throttle ceiling comes from a slip PI on
   the driven wheels' worst positive slip (TC's formula, `slip_floor_mps`) toward `slip_target[effective mode]`. Leaves
   to Handover on speed > `handover_speed_mps`, time > `max_launch_s`, throttle < 0.5, a shift, gear != 1, or the
   channel/auto-clutch/engine stopping.
4. *Handover*: one tick. Traction control starts from the PI's last ceiling (its fraction and integral are seeded, no
   torque jump) and runs again; the wing returns to the aero controller; then Idle.

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
control is Armed or Launching, [below](#assist-launch)); `radians = target * pi / 180`; `limit = radians >= 0 ? max_offset_rad : -min_offset_rad`;
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

**Hash.** The resolved mode table is folded into the vehicle's state hash only for a unit whose `modes` go beyond
Off/On (a Street unit hashes exactly like a car without a unit). The effective mode is a pure function of channel and
desc and is not state.
