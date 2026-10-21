# Electric drives

Vehicle powertrain components may include any number of `electric_motor` machines and `battery` stores alongside the existing combustion engine, gearbox and shaft components. Motor port `shaft` connects through the usual `shafts`; the motor's `battery` names a battery component. A motor does not occupy the single combustion-engine slot. Motors may share a battery, share an ICE shaft, or drive independent axles. Electrical resources are separate from fuel/fluid links.

`battery`: `capacity_kwh`, `initial_soc` (0..1), `max_discharge_w`, `max_charge_w`, `mass_kg`. Runtime energy uses joules through IEnergyStore. A shared store's discharge budget per substep is min(content, max_discharge_w x h), plus what a fuel cell adds that substep. When the motors that drive this substep ask for more than that, each gets a share proportional to its request (every driving motor registers its request before the motors run; see "Discharge allocation" below). The charge budget (regeneration, fuel-cell supply) is still granted in component-array order. There is no voltage, thermal, cell-resistance or degradation model in this increment.

`electric_motor`: `battery`, `peak_torque_nm`, `max_power_w`, `max_rpm`, `inertia_kgm2`, `efficiency` (0..1, exclusive zero), `regen_torque_nm`, `mass_kg`, `axis_local`. A sampled constant-torque/constant-power envelope controls the shaft torque, with a rotor-inertia prediction for the substep's power budget. Torque is zero beyond max_rpm. Mechanical work uses solved shaft speed; electrical energy accounts for fixed motor/inverter efficiency. Regeneration uses an implicit damping torque to avoid reversing the rotor at a stop. Optional `regen_power_w` (default 0 = `max_power_w`) limits regeneration power at the shaft; negative is rejected. Brake blending: see below.

`ready_channel` defaults to `electric.ready` (registered default 1), independent of combustion ignition. Optional `throttle_channel`, `regen_channel`, `direction_channel` override the vehicle pedal/brake and selected gearbox/EV direction. These seams allow an eventual hybrid supervisor to command machines independently without rewriting the motor. This increment proves a shared ICE shaft with ignition off; a hybrid strategy, engine-start scheduling and torque blending are not implemented here.

Without a gearbox, selector requests latch D (>0), N (0), R (<0); spawn is N. An ordinary gearbox retains its normal controller, and machines follow its selected direction. TC acts on the electric torque ceiling using physical contact slip. There is no idle, starter, clutch or launch-controller requirement for an EV. Reset does not refill the battery.

PowertrainSnapshot adds `electric_fitted`, `electric_ready`, `electric_motor_count`, `motor_rpm` (fastest machine), `motor_torque_nm` (signed sum), `electric_power_w` (signed mechanical sum), `battery_soc` (energy-weighted), `battery_capacity_kwh` and `battery_power_w` (positive discharge). The new cumulative energy terms are `electric_out`, `battery_draw`, `electric_loss`; the mechanical ledger includes electric_out. EV speedo uses driven-hub rotation. Fixed-reduction EVs publish transmission_kind=4 and D/N/R through transmission_selector. Combustion engine state remains Off on an EV; adapters must display READY rather than infer readiness from combustion state.

All fixture ratings are authored test parameters, not production-car measurements. `data/vehicles/car_electric.json` is a two-axle 600 kW sedan testbed with a shared 90 kWh pack.
### Discharge allocation

Decision (2026-10-19): a pack that cannot feed every driving motor splits its substep budget in proportion to the motors' requests.
Before, the budget went in component-array order, so on a battery-limited dual-motor car the first motor took all of it and the
second got nothing (car_electric with a 100 kW pack: the front motor 94 kW, the rear 0 at 3000 rpm and above; now 47/47 kW, same
total). How it works: after the batteries open their substep budgets, the Powertrain asks every motor for the request it will make
this substep (`ElectricMotor::drive_claim_j`, the same inputs and drive law as its `pre_solve`) and registers it with the motor's
pack (`Battery::add_drive_claim`). In `Battery::reserve_discharge`, while the claims still to come are larger than both the budget
left and the request, the request gets `left x request / claims`. Otherwise it gets `min(left, request)`, the old rule. So an
unlimited pack, a sole motor, the last claimant and a request that registered no claim behave as before. Only cars whose pack is
the limit change. Two pins moved: car_electric 100 kW and car_fuel_cell 10 kW + 30 kW stack. Every hybrid pin stayed the same,
since there the generator only drives when the traction motor does not. The claims are not hashed; they are rebuilt every substep.

Alternatives considered:
- Keep component order. Simple and already deterministic. But the last motor drops to zero, which flips a dual-motor car's torque
  split at the battery limit (a FWD car becomes RWD), and the outcome depends on the order in the file.
- Equal shares per motor. Simple, but it wastes budget: a motor that asks for less than its share leaves energy unused unless the
  shares are redistributed in a second pass. It also ignores the motors' different ratings.
- A priority or torque-split setting (per-axle bias). This belongs to a future torque-vectoring controller; nothing calls for it yet.
- Two passes inside the component loop. Not possible: the motors run inside the single component loop, and what the first motor is
  granted must be known before the second motor runs.

The proportional rule needs one extra pass over the motors per substep. It stays deterministic, and it is bit-identical wherever the
pack is not the limit.

## Hydrogen and hybrid development packages

`fuel_cell` is a lumped stack plus independent hydrogen tank feeding a named battery/DC bus. Fields: battery, max_power_w, efficiency, target_soc, hydrogen_capacity_kg, initial_hydrogen_kg, specific_energy_j_kg, mass_kg (dry stack/auxiliaries/tank), ready_channel. Generated electrical energy is limited by stack power, charge acceptance, available fuel and buffer headroom. It contributes to the DC supply budget in addition to the battery discharge limit. Conversion efficiency is fixed; no electrochemical/thermal stack or pressure dynamics are claimed. `fuel_cell_electric`, `fuel_cell_chemical`, `fuel_cell_loss` close the source conversion ledger. Hydrogen fill changes chassis contents mass when enabled. Snapshot fields separately report stack electrical power and remaining hydrogen; an ICE tank is independent.

`hybrid_control` requires an ICE, a disconnecting gearbox, a downstream traction motor and a crank-mounted generator (`electric_motor.crank_assist=true`). ECO uses electric wheel propulsion, with disconnected ICE generation allowed for low SOC. Normal starts combustion for demand beyond electric capacity. Sport/Racing keep the ICE running. Requested system power is a calibrated full-pedal rating. Battery and fuel-cell ratings limit electric potential. The supervisor works on the driver's pedal command (pedal curve and drive-mode bend applied, before any traction-control or launch ceiling): request = command x `requested_power_w`; the ICE command is the part of that request the motors cannot supply, `(request - available) / (requested_power_w - available)`, and the pedal curve and bend are not applied to it a second time. A traction-control ceiling is applied once, downstream: the engine stage takes the minimum of the ICE command and the ceiling, each motor the minimum of its pedal and the ceiling. It is no longer folded into the request, which amplified it through the split (pedal 0.75 gave an ICE command of 0.5) and made `hybrid_requested_power_w` follow the ceiling. Automatic scheduling (auto-shift, kickdown, launch) reads the driver's pedal command, not the ICE share and not a power ratio: a hybrid shifts at the same rpm points as the same gearbox without the electric part (it used to read requested / (motors + engine envelope at the current rpm), about 0.3 at full pedal on the hypercar, i.e. part-throttle upshifts at ~4100-4700 rpm instead of ~8000). At full pedal the ICE command is 1 whenever the request exceeds the electric power; between, the split stays linear in the pedal (a step to full ICE as soon as the request passes the electric power would make the pedal binary above ~half travel). When the ICE connects while the car rolls (previously disconnected, gearbox in neutral), the supervisor engages the FIT gear, not 1st: the lowest forward gear whose gearbox input rpm at the present road speed is within the up-shift rpm of the schedule at the driver's pedal command (drive-mode overrides and the limiter range applied; the top gear if none fits; 1st at standstill). The same rule drives the automatic gearbox's D-from-N choice. Before the clutch closes the connect window revs the engine toward the fit gear's input speed (clutch held open, throttle 0.45 + 0.0006 x rpm error, the charge-mode formula) and closes within the clutch law's lock window or after `hybrid_control.connect_timeout_s` (default 1.0 s [AS], 0 disables the window); auto-shift is deferred while it is open, so there is no downshift on connecting below the down-shift point. At standstill nothing changes (bit-identical). On a manual-TCU gearbox the supervisor selects 1st only from a settled neutral (idle phase, target neutral), not while a shift passes through position 0; it used to re-request 1st mid-shift, so the car never left 1st and the driver's shift_up_count presses were discarded. The supervisor keeps auto-shift on for such a box whatever the assist.auto_shift channel says. Racing boost preconditioning is not implemented.

Low SOC latches ICE charging until charge_target_soc (default0.8, termination tolerance1e-6SOC). The controller selects actual gearbox neutral while disconnected; the user's D/N/R lever remains separately latched for the downstream electric drive. The generator loads the crank and charges the real battery. A failed start is bounded by crank_timeout_s; READY cycling releases the request. No generator/disconnect hardware means the package is rejected. `car_hybrid.json` demonstrates this topology; `car_fuel_cell.json` demonstrates the fuel-cell source. All hardware ratings are authored fixtures.

Hydrogen's120MJ/kg LHV and approximate700bar reference density40kg/m3 are sourced in data/fuels/hydrogen.json from the US DOE overview. CompressedGas is an explicit fuel phase using a fixed stated storage density; this is not a tank thermodynamics model.

## Brake blending

The brake pedal requests a total brake torque (pedal x the `brake_system` corner capacities). On a vehicle whose electric motors sit on the wheel path (a motor shaft reaching the hubs only through gear pairs and differentials), the motors take that request first and the pads supply only the remainder, per corner. Handbrake and park lock are unchanged; a motor with `regen_channel` or `crank_assist` stays outside the scheme and regenerates in parallel as before; a motor behind a gearbox, the main clutch or a transfer case is blended through the elements' current state ("Motors behind a gearbox" below). A motor behind an automatic gearbox (torque converter, shift clutches) stays outside. `brake_system.regen_blend` (vehicle.md) configures it, `enabled: false` gives the parallel behaviour.

- Motor share: per motor, `min(regen_torque_nm, request, regen power / omega)` x charge acceptance x speed fade. Charge acceptance is the pack's `max_charge_w` budget and a SOC ramp from `taper_start_soc` (1) to `full_soc` (0). The speed fade ramps regeneration out between `fade_start_speed_mps` and `fade_end_speed_mps` (the pads hold the car to a stop). The request is converted between motor shaft and rim by the gear ratios, differential split and mesh efficiencies derived once from the shaft graph at build.
- Pads: each corner's service capacity is reduced by the regeneration reflected to that rim (clamped at 0), so a motor reduces only the pads of its own axle and there is no torque hole when regeneration is lost (full pack, power limit, ABS, low speed).
- ABS: sensor-only. When ABS modulates a wheel a motor drives (applied scale < 0.995), that motor's regeneration is withdrawn for `abs_hold_s` and the pads (which ABS controls) carry the request.
- Fix on the way: the motors' regeneration torque used to be given to the shaft network with tau0 = 0 and a damping slope d, which only resisted changes of speed (the network linearises about the old speed), so a steady regeneration braked almost nothing while the ledger booked the energy. tau0 is now -d x omega_old, giving the torque -d x omega_new. Every EV and hybrid run that brakes with regeneration changes.
- Dashboard: PowertrainSnapshot `regen_power_w` (power into the pack, >= 0), `regen_brake_torque_nm` / `pad_brake_torque_nm` (service torque at the rims, from the motors / the pads), `regen_brake_share` / `pad_brake_share` (their fractions, 0 while no service braking). Never hashed. The ledger is unchanged: regenerated mechanical energy is negative `electric_out`, the pad dissipation is `brake_loss`.
- Hashing: the blend configuration, the ABS hold timer and `regen_power_w` are hashed only for a motor that has them (blended or non-zero power limit); a car without electric motors hashes as before.

### Motors behind a gearbox (parallel hybrids)

The reflection from a motor to the rims is a tree over the shaft graph, evaluated every substep over the elements that are closed at that moment; it is the same on every vehicle with electric motors. Fixed edges are gear pairs, differential halves and a transfer case's rear output. The dynamic ones:

- Gearbox (manual or sequential): closed only with a gear fully engaged and no shift pending (gearbox phase Idle, current gear = target, not neutral); then it reflects through that gear's ratio and mesh efficiency. In neutral, with a shift requested (the regeneration must be gone for the mesh torque to fall below the dog-release threshold, otherwise the gear could never be pulled while braking) and while a gear is synchronised the motor reflects nothing and the pads take the full request in the same substep.
- Main clutch: closed at engagement >= `regen_blend.path_min_engagement` (0.95). A motor upstream of the clutch (on the crank or the clutch input) is also limited to `clutch_capacity_margin` (0.9) of the clutch's kinetic capacity, referred to the motor shaft.
- Transfer case: the rear output is rigid and always open; the front output is open at engagement >= `path_min_engagement`. Branches off one node (rear and front, the two sides of a differential) split the motor torque equally among the OPEN branches only (nominal split, as before).
- Ramp: after a path has been closed (no gear, shift, open clutch) the regeneration ramps in over `path_ramp_s` (0.05; 0 = no ramp), so the reflected torque does not step when the gear engages or the clutch closes. A closing path withdraws it at once.
- The motor's request is limited by the tightest open hub; the pads of the corners it brakes are reduced by exactly the reflected torque, so the total is the request in every substep.
- Not covered: a hybrid supervisor that opens the gearbox itself (`hybrid_control` selects neutral when no traction is wanted) removes the regeneration of a motor upstream of the gearbox by this rule; the pads then brake. A motor behind an automatic gearbox stays outside the scheme.
- Hashing: the ramp state and the three configuration values are hashed only for a motor that has a dynamic path; a car without one hashes as before.

## Envelope and dyno

The motor law, the DC supply budget and the hybrid power split are public pure functions. The runtime calls them, so a garage chart
and the car use one implementation. Extracting them changed no state hash; the pins are in tests/unit/test_electric_pins.cpp.

- `ps/drivetrain/electric_envelope.h`:
  - `motor_full_load_point(motor, rpm, electrical_budget_w, h, content_j = inf)` returns `{torque_nm, power_w, electrical_w, limit}`.
    `limit` is one of `PeakTorque`, `RatedPower`, `BatteryDischarge`, `MaxRpm`, `Empty` (`Idle` when the motor does not drive).
    This is the motor's own `pre_solve` law at full command: the rated-power corner uses the substep's rotor-inertia projection
    `omega + h*T/J`, so the power stays just under `max_power_w`. The pack's grant scales the request; near empty the
    residual-energy bound applies.
  - Building blocks: `motor_drive_request`, `motor_drive_torque`, `motor_drives`.
  - `battery_discharge_budget_w(battery, soc, h)` returns `min(content, max_discharge_w*h)/h`.
  - `fuel_cell_supply_w(cell, battery, soc, hydrogen_kg, h)` is the stack's supply: `max_power_w`, scaled by a SOC ramp
    `clamp((target_soc - soc)/0.05, 0, 1)`, and limited by the hydrogen on board and the pack's charge acceptance.
- `ps/drivetrain/hybrid_split.h`:
  - `hybrid_full_pedal_split(control, available_w, pedal, drive_mode, soc, charge_latched)` returns `{requested_w,
    electric_command, engine_command, engine_coupled, engine_wanted, charge_latched}`.
  - The supervisor's own step is `hybrid_power_split(control, HybridSplitInput)`.
  - Rules: requested = pedal x `requested_power_w`. Electric command = `min(1, requested/available)`. The ICE is coupled in Normal
    when the request exceeds the electric potential, and always in Sport and Racing. Its command is the deficit
    `(requested - available)/(requested_power_w - available)`. A low SOC latches an engine start (Eco: decoupled, charging only).
- `ps/drivetrain/powertrain_dyno.h`: `run_powertrain_dyno(vehicle, maps, PowertrainDynoSpec, pool, cancel)` returns one point per
  axis value.
  - Method: a chassis dyno. The car's own Powertrain is built without a chassis. Every hub is held at `v / r_i` by an implicit
    speed hold (`-D (omega_new - target)`, `D = 1e6 N*m*s/rad`). The spec's axis (`EngineRpm` = gearbox input in the held gear,
    `MotorRpm` of motor k, `GearboxOutputRpm`, `RoadSpeed`) sets `v` through the shaft's kinematic ratio at 1 m/s. Everything
    else runs as in the car at 240/960 Hz: throttle stage, hybrid supervisor including start/stop and neutral when decoupled,
    motor law, pack budget and allocation, fuel cell, gear/reduction/diff losses, clutch and engine.
  - What is held: SOC and hydrogen are held at the spec values for the whole point. The gear is held through a dyno-only hook
    (`Powertrain::set_dyno_gear_hold`; auto-shift is off, and a hybrid's disconnect still selects neutral).
  - Gearbox kinds (the spec's `gear` field means the same thing for each: the position that is measured):
    - manual / sequential: the gear is requested every substep, the manual TCU's auto-shift is off.
    - automatic (torque converter) and dual-clutch: the automatic TCU's shift logic is frozen on that gear
      (`AutoTransmission::set_dyno_hold`, a dyno-only flag, never hashed, off in every car): the target is the requested gear, with no
      selector, schedule, kickdown or preselect; the DualClutch's starting clutch is locked whenever a gear is in; the Automatic's
      converter lock-up is locked (the gear is measured as a manual gear is, in direct drive; the converter's fluid torque
      multiplication is therefore NOT part of the dyno curve). A hybrid's disconnect still selects neutral. The EngineRpm axis of
      a DualClutch is the crank, of an Automatic the turbine (equal when locked).
    - CVT: the box has one forward gear (D), so `gear` k > 0 selects the CVT's k-th virtual step (the stepped ratios,
      `forward_ratios`) and the variator is held at that step's ratio; `PowertrainDynoSpec::cvt_ratio` > 0 overrides it with an
      explicit variator ratio (input/output, within the variator's `ratio_high..ratio_low`, else `std::invalid_argument`). The belt's
      clamping force still follows the torque law (it starts at its maximum), so a belt that cannot carry the torque still limits
      the point. Reverse (`gear < 0`) uses the reverse ratio. The reported gear is the gearbox position (1 for a CVT).
    - Why steps and not "the ratio range mapped onto gear": the stepped ratios are the CVT's own authored virtual gears (what the
      S mode and the paddles select), so `gear` k means the same step as in the car and an explicit ratio stays available.
    - A battery EV cannot have an automatic gearbox in this model (`automatic_tcu` needs an engine); hybrids can.
  - Settling: like `run_dyno_sweep`. Window means over 0.5 s; two consecutive passing pairs on the wheel power (0.2 %/s) and, for
    a turbo engine, on boost (0.005 bar/s); at least 1 s (three windows), at most 15 s.
  - Per point: engine rpm, net crank torque (`tau0 - d*dw` plus the engine's Coulomb row, i.e. `run_dyno_sweep`'s `tau0 - c_k`)
    and command; gear; each motor's rpm, torque, power, electrical power and limit; pack and stack power; system power at the hubs
    and its torque at the axis shaft; boost; `battery_limited`; the supervisor's requested and available power, coupled and
    charging flags; `converged` and `settle_s`.
  - Execution: one job per point, each with its own Powertrain, so the result is bit-identical for any worker count. `cancel`
    has `run_dyno_sweep`'s semantics (GenerationCancelled). There is no progress callback, as in `run_dyno_sweep`.
  - Limits: no tyre slip, no aero load, no chassis; the hubs absorb whatever the powertrain gives. Gear-mesh losses are the
    runtime's lagged reconstruction, so each mesh passes `P_out = P_in / (2 - eta)`.

Sample sweeps (pedal 1, release build, 2026-10-19):

| car | axis point | result |
|---|---|---|
| car_electric | 1000 rpm per motor | 2 x 600 N*m (PeakTorque), 123.2 kW at the hubs |
| car_electric | 9000 rpm | 2 x 317.5 N*m (RatedPower), 586.7 kW; pack 629.9 kW |
| car_electric, 100 kW pack | 3000 rpm | 2 x 150 N*m (BatteryDischarge), 92.4 kW, pack 99.2 kW |
| car_fuel_cell | 3000 rpm | 377 kW mechanical, stack 150 kW, pack 246.9 kW net |
| car_fuel_cell, 10 kW pack + 30 kW stack | 3000 rpm | 2 x 60 N*m, 37.7 kW mechanical |
| car_hybrid Normal, 3rd | 3000 rpm | engine 187 N*m (WOT) + motor 286.4 N*m at 4714 rpm (pack-limited 148.8 kW), 190.1 kW at the hubs |
| car_hybrid Eco, 3rd | 3000 rpm | engine off, neutral, motor only: 134.5 kW |
| car_sedan, 3rd | 1500 / 4500 / 6000 rpm | engine 164 / 200 / 182 N*m, the WOT curve exactly (= `run_dyno_sweep`) |
| car_hyper (twin turbo, Sketch maps), 4th | 3000 / 5000 / 7000 rpm | engine 1214.7 / 1369.1 / 1310.4 N*m at 1.78 / 1.76 / 1.71 bar (`run_dyno_sweep` within 4e-7), 359 / 675 / 905 kW at the hubs, settled in 5-6.5 s |

Cost: 29 points for car_electric take 22 ms serial and 4.3 ms on 10 workers. The 12-point car_hybrid Normal sweep takes 30.5 ms
serial and 9.0 ms on 10 workers. Tests: tests/unit/test_powertrain_dyno.cpp `[drivetrain][electric][dyno]`.
