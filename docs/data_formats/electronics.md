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
a car registers it only when it has a `launch_control` component (every channel list of a car without one is
unchanged); a part with `fitted: false` still registers it. Anti-lag: with `anti_lag_switch.fitted =
false` the engine sees no channel (`Sensors::anti_lag_channel` NaN) and anti-lag runs on its configuration's
`default_enabled` only.

Snapshot: `VehicleSnapshot::powertrain.assist_fitted_bits` has the same layout as `assist_active_bits` (bit0
auto_clutch, bit1 auto_blip, bit2 auto_shift, bit3 traction_control, bit4 launch_control) plus bit5 = anti-lag
switch. A HUD shows three states: hidden (bit not set in `assist_fitted_bits`), fitted and off, fitted and on
(`assist_active_bits`; bit4 = launch_control switched on). A car without a `manual_tcu` reports the auto_* assists
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
`fit_traction_control.json`, `fit_anti_lag_switch.json`.

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
