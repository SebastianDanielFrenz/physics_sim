# Nitrous formats — `n2o_bottle/1`, `nitrous_kit/1`, `nitrous_controller/1`

A nitrous-oxide injection system is three slot-in part files plus a vehicle
wiring (N2O plan N1). They are separate files so a game can swap the part a
player replaces (a bigger bottle, a different jet kit, a different "ECU
strategy") without touching the others. The fluid they share is described by
[oxidizer.md](oxidizer.md) (`physics_sim.oxidizer/1`, `data/oxidizers/nitrous_oxide.json`).

| file | `format` | directory | loaded by | descriptor |
|---|---|---|---|---|
| bottle | `physics_sim.n2o_bottle/1` | `data/n2o_bottles/` | `ps::io::load_n2o_bottle_json` | `drivetrain::N2oBottleDesc` |
| kit | `physics_sim.nitrous_kit/1` | `data/nitrous_kits/` | `ps::io::load_nitrous_kit_json` | `drivetrain::NitrousKitDesc` |
| controller | `physics_sim.nitrous_controller/1` | `data/nitrous_controllers/` | `ps::io::load_nitrous_controller_json` | `drivetrain::NitrousControllerDesc` |

(All three in `ps/io/nitrous_io.h`, implemented in `core/src/io/nitrous_io.cpp`; the
descriptors are in `ps/drivetrain/nitrous_desc.h`; the runtime is the internal
`core/src/drivetrain/nitrous_system.h`.) Test fixtures of each live in
`tests/support/fixtures/n2o/`; no shipped example part files exist before the N5
increment.

**Rules shared by all three.** Every key is checked: an unknown key is an error
(`"<ctx>: unknown key '<k>'"`), a missing/other `format` is a hard error, and a
key ending in `_pa` or `_psi` is rejected by name
(`"<ctx>: key '<k>' is not accepted - pressures are in bar (e.g. 'gauge_bar')"`).
Pressures are **gauge bar** read at sea level (ISA 1.01325 bar reference, a
documented choice: the bottle state does not follow the live ambient). Relative
paths inside a file resolve against that file's own directory.

## Bottle — `physics_sim.n2o_bottle/1`

```json
{ "format": "physics_sim.n2o_bottle/1",
  "oxidizer": "../oxidizers/nitrous_oxide.json",
  "capacity_kg": 4.536,
  "pressure": { "mode": "constant", "gauge_bar": 65.5 },
  "heater": { "channel": "nitrous_heater", "default_on": 0, "set_point_gauge_bar": 68.0 },
  "outlet": "siphon",
  "source": "..." }
```

| key | type | rule |
|---|---|---|
| `oxidizer` | path | required; loaded with `load_oxidizer_json` |
| `capacity_kg` | kg | required, `> 0`; the bottle's content at vehicle creation (a vehicle component may lower it) |
| `pressure` | object | required, see below |
| `heater` | object | optional; a heater switch, instantaneous (owner ruling: no thermal ODE) |
| `outlet` | string | optional; only `"siphon"` (liquid draw) |
| `source` | string | required provenance |

`pressure` (the state is a **fill state plus a data-given pressure**, no
thermodynamic model): `{"mode": "constant", "gauge_bar"}`,
`{"mode": "vs_fill", "fill_fraction": [0..1 ascending, spanning exactly 0 and 1], "gauge_bar": [same length]}`
(pressure falls as the bottle empties), or `{"mode": "temperature", "temperature_k"}`
(the pressure is the saturation pressure at that temperature). Every resulting
absolute pressure must lie inside the oxidizer table's usable range (triple
point to 0.98 of the last row, 70.97 bar abs): 1000 psig = 69.96 bar gauge is
allowed, 80 bar is not (`"<ctx>: absolute pressure <p> bar is outside the oxidizer table's usable range [<lo>, <hi>] bar ..."`).

`heater`: `channel` (a declared control channel, a plain 0/1 level, default `default_on`),
and exactly one of `set_point_gauge_bar` / `set_point_k`
(`"<ctx>: exactly one of 'set_point_gauge_bar' and 'set_point_k' is required"`).
While the channel is on (`>= 0.5`) the pressure is the set point, reached
instantly. Without a `heater` block the bottle declares no channel.

Rejections beyond the above: `capacity_kg <= 0`, unknown pressure mode, vs_fill not
spanning 0..1 / not ascending / unequal lengths, temperature outside the table,
`outlet` other than `siphon`, missing `source`.

The vehicle component (`"type": "n2o_bottle"`, see below) overrides, bottom-up:
`initial_kg` (default the capacity) and a `pressure` block.

## Kit — `physics_sim.nitrous_kit/1`

```json
{ "format": "physics_sim.nitrous_kit/1",
  "type": "wet", "injection": "plenum", "nozzle_count": 1,
  "nitrous_jet_diameter_mm": 1.4478, "nitrous_jet_cd": 0.66, "flow_model": "hem",
  "fuel_jet_diameter_mm": 1.4478, "fuel_jet_cd": 0.70,
  "fuel_pressure": { "gauge_bar": 0.448, "reference": "atmosphere" },
  "rating": { "power_kw": 74.57, "bottle_gauge_bar": 65.5, "basis": "crank", "source": "..." },
  "default_controller": "../nitrous_controllers/edelbrock_basic.json",
  "source": "..." }
```

| key | rule |
|---|---|
| `type` | `"wet"` only; `"dry"` is rejected until the dry increment (`type "dry" is not supported yet`) |
| `injection` | `"plenum"` or `"direct_port"`; recorded for sound/docs, physically identical at the mean-value level |
| `nozzle_count` | integer `>= 1`, default 1; flows multiply by it |
| `nitrous_jet_diameter_mm`, `fuel_jet_diameter_mm` | `> 0` |
| `nitrous_jet_cd`, `fuel_jet_cd` | in `(0, 1]` |
| `flow_model` | `"hem"` (default) or `"dyer"` (NHNE, 0.5 SPI + 0.5 HEM) |
| `fuel_pressure` | required object: `gauge_bar >= 0`, `reference` `"atmosphere"` (fuel pressure = ambient + gauge) or `"manifold"` (fuel pressure = manifold + gauge) |
| `rating` | optional metadata (graph-engine add-on and the validation report), never feeds the simulated engine; `power_kw > 0`, `bottle_gauge_bar > 0`, `basis` `"crank"`/`"wheel"`, `source` required |
| `default_controller` | path; a vehicle component may give its own `controller` instead; a kit with neither is rejected (`no controller`) |

Flows per substep: `mdot_n = frac * nozzle_count * Cd * A * G(p_bottle, p_intake)`,
`mdot_kf = frac * nozzle_count * Cd_f * A_f * sqrt(2 * rho_fuel * max(0, p_fuel - p_intake))`
with `rho_fuel` the density of the fuel the engine burns. `G` is the oxidizer
table's HEM critical flux (or the Dyer flux). N2: a simulated engine (`kind`
`"simulated"`) burns both through its wet charge model (engine.md,
[nitrous](engine.md#nitrous-the-wet-charge-model-n2o-plan-n2)); the kit fuel is drawn
from the engine's own fuel feed (the tank, if any). A torque-map engine still
**ignores** them until N6: every delivered kg is booked as unburnt pass-through
(`PowertrainEnergy::n2o_mass == n2o_unburnt`) and no kit fuel is booked.

## Controller — `physics_sim.nitrous_controller/1`

```json
{ "format": "physics_sim.nitrous_controller/1",
  "arm_channel": "nitrous_arm", "arm_default": 0,
  "min_throttle_command": 0.95, "min_rpm": 3000, "max_rpm": 6200,
  "min_gear": 1, "max_gear": 99, "min_speed_mps": 0,
  "progressive": { "start_fraction": 1.0, "delay_s": 0.0, "ramp_s": 0.0 },
  "spark_retard_deg": 4.0,
  "safety": { "min_bottle_gauge_bar": 55.0, "cut_on_fuel_starve": 1, "max_active_s": 15.0 },
  "purge": { "channel": "nitrous_purge", "jet_diameter_mm": 1.0, "cd": 0.66 },
  "source": "..." }
```

The solenoid opens (the "request") while ALL of these hold, evaluated each
substep before the engine's pre_solve:
armed (`arm` channel `>= 0.5`) · the throttle stage's command `>= min_throttle_command`
(after TCU floor and ceiling) · rpm in `[min_rpm, max_rpm]` · gear in
`[min_gear, max_gear]` (neutral is 0 and reverse negative: they **never spray** - `min_gear`
below 1 is rejected by the loader, and the runtime enforces `gear >= max(1, min_gear)` for a
code-built desc too; a wheel-less rig has no gear condition) · road speed
`>= min_speed_mps` · the engine is Running · no TCU upshift ignition cut · the
previous substep had neither limiter nor DFCO active.

* `arm_channel` is a **plain 0/1 level** (default `arm_default`, 0 = disarmed), the
  channel an adapter toggle drives. It is declared (`Powertrain::declare_control_channels`)
  only for a vehicle with a kit.
* `progressive`: with `t_active` the time the solenoid has been continuously open (it restarts from 0
  whenever the solenoid closes, for any reason - disarm, lift, a safety cut), the flow fraction is
  `start_fraction` while `t_active < delay_s`, then
  `clamp(start + (1 - start) * (t_active - delay_s) / ramp_s, 0, 1)`; `ramp_s` 0 jumps to 1 after the
  delay. The nitrous and the kit fuel flow are both multiplied by it (the two solenoids pulse together;
  the pulse-width solenoid is modelled by its mean flow). Without a `progressive` block the fraction is 1.
* `spark_retard_deg`: a constant, or `{"fraction": [...], "deg": [...]}` (strictly ascending
  fractions in `[0, 1]`, `deg >= 0`, piecewise linear, clamped at the ends) evaluated at the
  progressive flow fraction of the substep. A simulated engine applies it while N2O flows:
  spark = min(schedule - retard, knock limit). The retard is 0 while nothing flows.
* `safety`: each cut **latches** until the command drops below `min_throttle_command`:
  `min_bottle_gauge_bar` (bottle-pressure switch, 0/absent = none), `cut_on_fuel_starve`
  (previous substep's fuel supply fraction < 1), `max_active_s` (solenoid duty time, `> 0`; absent = unlimited).
  The cut reason is published as `EngineSoundState::n2o_safety_cut` (0 none, 1 bottle pressure, 2 fuel starve, 3 duty time).
* `purge`: a second channel (default 0) venting liquid N2O to the atmosphere through its own jet
  (HEM flux), independent of the request; its mass goes to `n2o_purged`. The purge channel must differ from the arm channel.

Rejections: missing `arm_channel`/`min_rpm`/`max_rpm`/`source`, `min_throttle_command`
outside `[0, 1]`, `min_rpm < 0`, `max_rpm <= min_rpm`, `min_gear < 1`
(`"'min_gear' must be >= 1 (neutral and reverse never spray)"`), `max_gear < min_gear`,
`min_speed_mps < 0`, progressive `start_fraction` outside `(0, 1]` / negative delay or ramp,
negative or malformed retard (curve length mismatch, `fraction` outside `[0, 1]`),
`max_active_s <= 0`, negative `min_bottle_gauge_bar`, a malformed `cut_on_fuel_starve`,
purge `cd` outside `(0, 1]` / `jet_diameter_mm <= 0`.

## Vehicle wiring

Two `powertrain.components` types (always `"ref"` a part file) and two fluid links
(see [vehicle.md](vehicle.md#fluid-links-i7a-c6)):

```json
{ "id": "n2o_bottle", "type": "n2o_bottle", "ref": "../n2o_bottles/bottle_10lb.json", "initial_kg": 3.0, "pressure": { ... } },
{ "id": "n2o_kit",    "type": "nitrous_kit", "ref": "../nitrous_kits/kit_wet.json", "controller": "../nitrous_controllers/mine.json" }
...
"fluid": [ ["n2o_bottle.out", "n2o_kit.n2o"], ["n2o_kit.out", "engine.nitrous"] ]
```

Component overrides (bottom-up: part file default, then the vehicle component,
then the runtime user later): bottle `initial_kg` (`0 <= initial_kg <= capacity_kg`,
`"<ctx>: 'initial_kg' (<v>) must not exceed the bottle's capacity_kg (<c>)"`) and `pressure`;
kit `controller` (path, relative to the vehicle file; a vehicle-level controller file **wins** over the kit's
`default_controller`: every controller field, arm channel included, comes from the override and the declared
channels follow it; the kit's own jets/flow model are untouched; a missing file, a wrong `format` or any
controller rejection above fails the vehicle load with the file named). The heater channel (bottle), the arm
channel and the purge channel (controller) must be three different names, since declared channels merge by name:
`"vehicle '<v>' ...: the bottle heater channel '<c>' is also the arm channel of nitrous_kit '<id>' controller"`
(same for the purge channel; the Powertrain constructor repeats the check for a code-built desc). Any other key is rejected. Every bottle needs a kit, every kit a
bottle and an engine with a fuel model (the fuel jet needs the fuel's density):
`"vehicle '<v>': powertrain.components['<id>']: n2o_bottle has no consumer (no powertrain.fluid link from \"<id>.out\")"`,
`"... nitrous_kit has no bottle (no powertrain.fluid link to \"<id>.n2o\")"`,
`"... nitrous_kit has no consumer ..."`. A vehicle without a bottle and kit has no new
channels, CSV columns or hashed state (bit-identical to before N1).

**Control channels** (declared only for a vehicle with a kit, plain 0/1 levels):
the controller's `arm_channel` (default `arm_default`), its `purge.channel` (default 0) and
the bottle heater's `channel` (default `default_on`).

**Runtime surface.** `World::n2o_bottle_mass(vehicle)` (kg, 0 without a kit) and
`World::set_n2o_bottle_mass(vehicle, kg)` (applied at the next tick boundary, clamped
to `[0, capacity]`, booked signed into `PowertrainEnergy::n2o_external_added`; throws
without a bottle or for NaN). `PowertrainEnergy` sub-ledger (cumulative kg, unhashed,
outside `balance_error`): `n2o_mass`, `n2o_unburnt`, `n2o_purged`, `n2o_external_added`,
plus `n2o_energy_in` (J, decomposition heat of the burnt N2O) and `kit_fuel_mass` (kg, the kit-jet share
of `fuel_mass`), both 0 from a torque-map engine. `n2o_mass` is the bottle's delivery; `n2o_unburnt` its share
that passed a cut or starving cylinder. `engine_heat = fuel_energy_in + n2o_energy_in - indicated_work`.
Bottle closure: `initial - bottle = n2o_mass + n2o_purged - n2o_external_added`. `EngineSoundState`:
`n2o_active`, `n2o_safety_cut`, `n2o_flow_g_s`, `n2o_purge_flow_g_s`, `n2o_bottle_kg`,
`n2o_bottle_bar`, `n2o_retard_deg`, `n2o_activations` (solenoid rising edges), `n2o_purge_events` (purge
presses, counted once per press; a press with an empty bottle is not an event), and (N4, the controller's
state for sound/HUD, never hashed) `n2o_solenoid_open` (the controller wants to spray, also with an empty
bottle), `n2o_flow_fraction` (the progressive duty of the tick's last substep, 0 while nothing flows),
`n2o_purge_active`, `n2o_injection` (0 plenum, 1 direct_port); all 0 without a kit. And (N2, simulated engine,
0 otherwise) `charge_temperature_k`, `n2o_lambda_combined`, `n2o_kit_fuel_g_s`;
`PowertrainSnapshot::n2o_bottle_kg`. CSV (kit vehicles only):
`vehicle<i>.n2o.{flow_g_s, bottle_kg, bottle_bar, active}`.

## Shipped parts (N2O plan N2, unvalidated until N5)

| file | what | numbers |
|---|---|---|
| `data/n2o_bottles/bottle_10lb_950psi.json` | 10 lb siphon bottle, constant 950 psig | 4.536 kg [D], 65.5 bar gauge [D from the Edelbrock sheets' 950 psi] |
| `data/nitrous_kits/edelbrock_70011_square_50.json` | Edelbrock #70011 Performer, 38/38 jets, 50 hp | jets 0.9652 mm [AS jet-number convention], Cd 0.66 [C Dyer via Waxman], fuel Cd 0.70 [AS], 6.5 psi fuel [C], HEM |
| `data/nitrous_kits/edelbrock_70060_square_150.json` | Edelbrock #70060 Performer RPM plate, 68/68 jets, 150 hp | jets 1.7272 mm; the sheet's 5-7 deg retard is NOT in the shared default controller (PROVISIONAL, N5) |
| `data/nitrous_controllers/edelbrock_basic.json` | arm switch `nitrous_arm`, WOT switch 0.95, 3000-6200 rpm, on/off, 2 deg retard | retard [C 70011 sheet 1-3 deg midpoint], the rest [AS] |

`tests/support/fixtures/vehicles/car_sedan_gen_n2o.json` (a test fixture, not a shipped car) is `car_sedan_gen.json` with the bottle and the 50 hp kit (arm channel
`nitrous_arm`, default off; smoke test `tests/validation/test_car_sedan_gen_n2o.cpp`: a WOT 3rd-gear pull
gains about +41 kW crank power at 4000-5500 rpm, reported only). Every Edelbrock number is as transcribed in `out/n2o_plan.md` section 9.2; the
whole-kit check against the ratings (V2) is N5.

## Dyno use (engine garage P3b-2)

`run_dyno_sweep` (`ps/drivetrain/engine_dyno.h`) can sweep a simulated engine with a bottle and kit
(`DynoSweepSpec::nitrous`; `nitrous_from_powertrain` picks both out of a loaded vehicle). It reuses `NitrousSystem` with the
rules of a wheel-less rig: armed, sprayed at full controller command (the progressive ramp and the `max_active_s` duty cap are
skipped), no gear or road-speed condition, the bottle never committed (its pressure is held at the descriptor fill). The sweep
reports `n2o_g_s`, `kit_fuel_g_s` and the combined lambda and produces the findings `KitFuelLean`, `N2OBottleDuration` and
`N2OBottlePressureDrop` (a `vs_fill` bottle only), with the remedies and `solve_power_target` described in
`engine.md` ("Nitrous in the sweep", "Simulated remedies"). The kit controller's `min_gear` is not applied on a dyno.

## `extends` and the user layer (engine garage P2)

Bottle, kit and controller files may name a parent with `"extends": "<path>"` (RFC 7396 merge patch, the child last; at
most 8 parents, cycles rejected; relative paths per file). A parent's relative paths are re-expressed for the child
before the merge: a bottle's `"oxidizer"`, a kit's `"default_controller"`. Example — a bigger bottle of the same kind:

```json
{ "extends": "../n2o_bottles/bottle_10lb_950psi.json", "capacity_kg": 6.804 }
```

The user layer (`VehicleLoadOptions::part_overrides`, [vehicle.md](vehicle.md#user-layer-engine-garage-p2)) may replace
an `n2o_bottle` or `nitrous_kit` component's `ref` and a kit's `controller` slot (relative to the vehicle file); the
result equals writing the same keys into the vehicle file.
