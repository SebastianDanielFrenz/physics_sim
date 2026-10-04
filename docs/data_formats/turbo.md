# Turbo formats — `physics_sim.turbo/1` and `physics_sim.turbo_configuration/1`

A turbocharged simulated engine takes its turbo from TWO files, both loaded by
`ps::io::load_turbo_configuration_json(path, cache_dir)`
(`core/src/io/vehicle_io.cpp`):

- the **configuration** (`physics_sim.turbo_configuration/1`,
  `data/turbo_configurations/*.json`) is what an engine file names with
  `default_turbo_configuration` (see [engine.md](engine.md#kind-simulated-the-mvem-engine)):
  how many turbos, the boost target, the intercooler and the boost controller;
- the **hardware** (`physics_sim.turbo/1`, `data/turbos/*.json`) is one turbo's
  rotor, flow limit, efficiencies and turbine size, named by the configuration's
  `turbo` field.

The configuration is the part a game swaps at runtime (`World::replace_turbo_configuration`
keeps the rotor speed and replaces the rest); the hardware is the part a
configuration points at. Only simulated (`"kind": "simulated"`) engines have a
turbo; the graph engine ignores the whole feature.

**Units.** Pressures in both files are **bar** (owner rule 2026-10-02: files and
readouts in bar; the descriptor `TurboPairDesc` stays SI/Pa). The old Pa keys are
rejected by name (below). Mass flow kg/s, inertia kg*m^2, area m^2, power W.

**Pair scaling.** The simulated engine holds ONE lumped `TurboPair` standing for
`count` identical turbos sharing the flow symmetrically. The loader multiplies the
extensive hardware quantities by `count`: `rotor_inertia_kgm2`,
`max_mass_flow_kg_s`, `turbine_area_m2` and `shaft_loss_w_at_reference` (a pair of
two turbos has twice the inertia, flow capacity, throat area and bearing loss of
one). Intensive quantities (`reference_rpm`, `reference_head_bar`, the efficiencies,
the intercooler effectiveness, the boost target) are not scaled. `count` is an
integer in [1, 8].

## The model in one paragraph

Not an OEM compressor map. `TurboPair` (`core/src/drivetrain/turbo_pair.h`) is a
lumped mean-value model stepped once per substep by the engine: a quasi-steady
compressor head `head(speed, flow)` from an authored table (generated at load as
`reference_head * s^2 * (1 - f^2)` with `s` = rotor speed over `reference_rpm` and
`f` = compressor flow over `max_mass_flow_kg_s`), charge pressure = ambient + head
followed with a 25 ms lag, an isentropic outlet temperature with the intercooler's
effectiveness, a PID wastegate on the post-cooler charge pressure, turbine power
from the exhaust flow through the open fraction of the wastegate, and a rotor
energy balance (turbine power - compressor power - shaft loss) clamped at 2.25 x
the energy at `reference_rpm`. The engine's throttle is fed from the charge
pressure/temperature and its exhaust back-pressure is the turbine inlet pressure.
The exhaust gas temperature comes from the engine's own energy balance (see
`exhaust_thermal` below): no temperature cap anywhere, only numerical guards.
`engine.md`'s `cycle` section describes how boosted map nodes are generated.

## Hardware file — `physics_sim.turbo/1`

| Field | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `format` | string | yes | `"physics_sim.turbo/1"` | format tag |
| `name` | string | no | ignored by the loader | free text |
| `source` | string | convention | not enforced by this loader | provenance (README's "source" convention): cite or label `[AS]` |
| `rotor_inertia_kgm2` | number | yes | > 0; x `count` | shaft + wheels inertia of ONE turbo |
| `reference_rpm` | number | yes | > 0 | rotor speed at which `reference_head_bar` is reached at zero flow; also the energy cap reference |
| `reference_head_bar` | number (bar) | yes | > 0 | compressor head at `reference_rpm` and zero flow. `reference_head_pa` is rejected: `"<path>: 'reference_head_pa' was replaced by 'reference_head_bar' (bar)"` |
| `max_mass_flow_kg_s` | number | yes | > 0; x `count` | compressor flow that closes the head curve to zero (normalising flow) |
| `compressor_efficiency` | number | yes | > 0, <= 1 | isentropic; sets the outlet temperature table and the compressor power |
| `turbine_efficiency` | number | yes | > 0, <= 1 | turbine power factor |
| `turbine_area_m2` | number | yes | > 0; x `count` | effective turbine throat; sets the exhaust back-pressure `dp = m_turbine^2 / (2 rho A^2)` |
| `shaft_loss_w_at_reference` | number | yes | > 0; x `count` | bearing/friction power at `reference_rpm`, scales with speed^2 |

A missing key throws `"<path>: '<key>' is required"`, a non-number
`"<path>: '<key>' must be a number"`, a non-positive value
`"<path>: '<key>' must be > 0, got <value>"` (path = the hardware file).
Unknown keys are ignored.

### `surge_line` (optional, I7d D3)

| Field | Type | Rule | Meaning |
|---|---|---|---|
| `pressure_ratio` | array of numbers | required, >= 2 points, first >= 1, strictly increasing | compressor outlet pressure over ambient |
| `flow_fraction` | array of numbers | required, same length, each in [0, 1) | surge flow as a fraction of `max_mass_flow_kg_s` (per turbo; the pair is symmetric, so the same fraction of the pair's flow) |
| `source` | string | required, non-empty | provenance (PLAN.md section 8): cite a measured map or label `[AS]` |

Linear interpolation, clamped at both ends (the line is flat beyond its last point).
Rejections (context `"<hardware path>: surge_line"`): `'surge_line' must be an object`,
`'pressure_ratio' must be an array of numbers` (also `'flow_fraction'`), `needs at
least 2 points`, `'pressure_ratio' and 'flow_fraction' must have the same length`,
`'pressure_ratio' must start at >= 1`, `'pressure_ratio' must be strictly increasing`,
`'flow_fraction' must be in [0, 1)`, `'source' is required (PLAN.md section 8)`.

The line drives a FLAG only (ruling N2: no surge dynamics, no pressure collapse, no
flow reversal): `surge_active` is raised when `pr > 1.05` and `flow_fraction < line(pr)`
and cleared when `flow_fraction > 1.05 * line(pr)` or `pr <= 1.05` (hysteresis);
`surge_events` counts entries. Absent = the flag is never raised (no invented line).
Nothing in the model reads the flag, so it is in neither the state hash nor the
derived-map cache key, and an engine with a line is bit-identical to one without.
The shipped hardware file has no line yet (no cited map exists).

## Configuration file — `physics_sim.turbo_configuration/1`

| Field | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `format` | string | yes | `"physics_sim.turbo_configuration/1"` | format tag |
| `name`, `source` | string | no | not enforced | free text / provenance |
| `turbo` | string | yes | path relative to THIS file; must name a `physics_sim.turbo/1` file | the hardware |
| `count` | integer | yes | [1, 8] | number of identical turbos in the pair, see "Pair scaling" (a count over 8 throws `"<path>: turbo count must be <= 8"`) |
| `target_boost_bar` | number (bar) | yes | > 0 | GAUGE boost the wastegate controller holds at full plate. `target_boost_pa` is rejected: `"<path>: 'target_boost_pa' was replaced by 'target_boost_bar' (gauge, bar)"` |
| `intercooler_effectiveness` | number | yes | > 0, <= 1 | fraction of the compressor outlet temperature rise removed; the pressure drop is the optional key below (absent = none) |
| `intercooler_pressure_drop_bar_at_max_flow` | number (bar) | no | >= 0 (`"<path>: 'intercooler_pressure_drop_bar_at_max_flow' must be >= 0"`), < the hardware `reference_head_bar`, a non-number `"... must be a number"`; NOT scaled by `count` | I7d D4: pressure drop of the cooler at the pair's max flow; absent or 0 = no drop, bit-identical to a file without the key |
| `boost_control` | object | no | see below | PID on the post-cooler charge pressure driving the wastegate |
| `blow_off_valve` | object | no | see below | I7d D2 |
| `anti_lag` | object | no | see below | I7d D5 |
| `exhaust_thermal` | object | no | see below | exhaust energy balance and wall thermal mass parameters; absent = the [AS] defaults |

### Intercooler pressure drop (I7d D4)

`dp = k * (compressor flow / max flow)^2` with `k = intercooler_pressure_drop_bar_at_max_flow`
(in Pa inside the desc, `TurboPairDesc::intercooler_dp_pa_at_max_flow`). The compressor
makes head above ambient; the charge pressure the throttle sees is that minus `dp`, i.e.
`charge = ambient + head - dp` (the lag state), and the compressor outlet pressure is
`charge + dp`. The compressor pressure ratio used for the outlet temperature, the
compressor power and the published `compressor_pressure_ratio` is the OUTLET's. The boost
controller keeps controlling the post-cooler pressure (the usual sensor position), so the
steady boost stays on target and the rotor runs faster by the amount that supplies the
drop: at steady WOT `s = sqrt((boost + dp) / (reference_head * (1 - f^2)))` for the
hyper-like fixture (measured: 114734 rpm without, 121693 rpm with 0.4 bar at max flow and
f = 0.75, vs the head-curve inversion 114742 / 121702). The key is not in the derived-map
cache key (it feeds no map); a nonzero value is folded into the engine state hash through
the desc, zero is not, so a file without the key keeps its hash.

Any of the three efficiencies (compressor, turbine, intercooler) above 1 throws
`"<path>: turbo efficiencies must be <= 1"` (path = the configuration file).

### `boost_control`

| Field | Type | Default | Rule | Meaning |
|---|---|---|---|---|
| `kp` | number | 1.0 | > 0 | proportional gain, error per unit of `target_boost_bar` |
| `ki_per_s` | number | 1.0 | > 0 | integral gain, 1/s; conditional anti-windup |
| `kd_s` | number | 0.05 | >= 0 (`"<ctx>: 'kd_s' must be >= 0"`) | derivative gain on the MEASUREMENT (a target step does not kick); 0 limit-cycles |
| `actuator_tau_s` | number | 0.10 | > 0 | wastegate actuator first-order lag |
| `source` | string | | not enforced | provenance |

`boost_control` must be an object (`"<path>: 'boost_control' must be an object"`).
Defaults are from a gain sweep on `hyper_v8_one1_audio.json` (2026-10-02, vault
PHYS-058): every speed settles on target with < 20 Pa ripple. The controller
requests no boost below 10 % plate (wastegate commanded open, integral reset); the
boost target is `ambient + target_boost * plate`.

### `blow_off_valve` (optional, I7d D2)

A pressure-actuated valve on the charge pipe that vents compressor air when the
throttle shuts under boost, so the compressor keeps a flow path instead of
choking against a closed plate. Absent = no valve and
every number is bit-identical to a configuration without the block.

| Field | Type | Rule | Meaning |
|---|---|---|---|
| `vent` | string | required, `"atmosphere"` or `"recirculate"` | atmosphere: the vented air is lost; recirculate: it returns to the compressor inlet (a pure sound label, the lumped model treats both alike - the vent flow is compressor flow either way, no extra mass is booked) |
| `crack_dp_bar` | number | required, >= 0 | charge pressure minus manifold pressure at which the valve starts to open |
| `full_open_dp_bar` | number | required, > `crack_dp_bar` | the difference at which it is fully open |
| `effective_area_mm2` | number | required, > 0 | effective flow area (Cd included) PER TURBO; scaled by `count` |
| `tau_s` | number | required, >= 0 | first-order lag of the valve opening (0 = follows the command at once) |
| `source` | string | required, non-empty | provenance (PLAN.md section 8) |

Rejections (each prefixed `"<path>: blow_off_valve: "` style context):
`'blow_off_valve' must be an object`, `'vent' must be "atmosphere" or "recirculate"`,
`'<key>' is required`, `'crack_dp_bar' must be >= 0`, `'full_open_dp_bar' must be >
'crack_dp_bar'`, `'effective_area_mm2' must be > 0`, `'tau_s' must be >= 0`,
`'source' is required (PLAN.md section 8)`, and the Pa spellings `crack_dp_pa` /
`full_open_dp_pa` ("was replaced by ... (bar)").

Model: per substep the valve command is `clamp((p_charge - p_im - crack)/(full -
crack), 0, 1)`, evaluated at the END of the turbo step; the opening follows it
with a first-order lag `h/(tau + h)`. The vent flow of the NEXT step is the
isentropic orifice flow `opening * A * p_charge / sqrt(R * T_charge) * Psi(p_amb/p_charge)`
(the engine's shared tabulated Psi, so choked above the critical ratio) and is ADDED
to the compressor flow: the compressor moves more air at a lower head (the head
map falls with flow) and its power draw rises, so in this model the rotor of a
lifted engine decelerates FASTER with the valve than without it (pair-level lift
test: 0 rpm vs 33152 rpm after 500 ms). What the valve buys is a compressor that
stays off its surge line (D3's flag), not a spinning rotor. The vent adds no mass
to the intake - it leaves the system. The crack pressure must exceed the idle vacuum
(ambient - idle manifold, ~0.7 bar) or the valve opens at every closed-throttle
point. The valve opening is dynamic state: it is hashed (together with the valve
parameters) only when a valve exists. The block is NOT part of the derived-map
cache key (it changes nothing the maps contain).

Known limit (measured, I7d D2): the lumped model has no plenum between the
compressor and the throttle, and its quasi-steady charge pressure follows the rotor.
When the throttle closes the rotor of the hyper V8 collapses within ~50 ms even without
a valve, and the charge pressure goes with it, so a valve with a crack pressure of
0.9 bar opens only to ~13 % at a lift-off. A valve that opens in time needs a low
crack pressure and a short lag.

### `exhaust_thermal` (optional)

Turbo engines only. The exhaust gas temperature is the engine's own energy balance
per substep (`engine_simulated.cpp`, one-substep hand-off to the turbo step like the
flows), and the turbine sees it after a lumped thermal mass (two manifolds + two
turbine housings as ONE wall). There is no temperature cap; the temperature is
bounded by construction (sensible heat over the actual gas flow, a well-stirred
wall exchange).

Energy balance (W, per substep): `Q_f = booked fuel mass * LHV / h`; combustion
efficiency `eta_c = min(1, lambda)` ([AS] oxygen-limited ideal - rich mixture
leaves its unburnt share as chemical enthalpy, not heat); released `eta_c * Q_f`;
indicated work `W_i = omega * tau_comb` (the ledger's `indicated_work` rate);
in-cylinder wall heat `Q_wall = in_cylinder_wall_heat_fraction * released`;
sensible exhaust enthalpy `H = released - W_i - Q_wall` (clamped at >= 0, a
numerical guard); `Q_f = W_i + Q_wall + H + (1 - eta_c) Q_f` to round-off. Port
gas temperature `T_port = T_charge + H / (1100 * m_exh)`, `m_exh` = the actual
cylinder exhaust flow (air + fuel); no flow -> `T_port = T_charge`. A spark retard
(anti-lag) lowers `W_i` and therefore heats the exhaust with no separate term.

Wall (C heat capacity, K gas-side, Ka outer conductance): `G = 1100 * m_exh`,
`Keff = K G / (K + G)`; backward Euler `T_w' = (C T_w + h (Keff T_port + Ka
T_amb)) / (C + h (Keff + Ka))`; turbine inlet `T_t = (G T_port + K T_w') / (G +
K)` (gas side well stirred). At vanishing flow the gas leaves at the wall
temperature, so a hot wall gives heat back to the small flow after a lift-off.
`C (T_w' - T_w) = h (Keff (T_port - T_w') - Ka (T_w' - T_amb))` holds to
round-off. The wall starts at ambient (a cold start) unless
`initial_wall_temperature_k` is given.

| Field | Type | Rule | Default | Meaning |
|---|---|---|---|---|
| `in_cylinder_wall_heat_fraction` | number | [0, 1) | 0.20 [AS] | share of the released fuel energy lost to the cylinder walls/coolant (Heywood 1988 Table 12.1: coolant 17-26 % of the fuel energy at max power, SI) |
| `wall_heat_capacity_j_k` | number | > 0 | 9200 [AS] | ~20 kg of cast iron/steel x 460 J/(kg K) |
| `wall_gas_conductance_w_k` | number | >= 0 | 120 [AS] | gas-side h*A (~0.4 m^2 x ~300 W/(m^2 K)), constant (no flow dependence) |
| `wall_ambient_conductance_w_k` | number | >= 0 | 30 [AS] | outer convection + linearised radiation (~0.5 m^2 x ~60 W/(m^2 K)) |
| `initial_wall_temperature_k` | number | > 0 | ambient | wall temperature after a reset |
| `source` | string | required, non-empty | | provenance (PLAN.md section 8) |

Installation totals: NOT scaled by `count`. Unknown keys are rejected
(`"<path>: exhaust_thermal: unknown key '<key>'"`), as is every rule above
(`"... must be in [0, 1)"`, `"... must be >= 0"`, `"'source' is required (PLAN.md
section 8)"`). Hashing: the wall temperature, the port temperature handed to the
next substep and the four parameters are folded into the state hash of turbo
engines only; not part of the derived-map cache key. No ECU material-temperature
protection (enrichment, boost cut) is modelled - a follow-up.

### `anti_lag` (optional, I7d D5)

A rally-style anti-lag system (ALS): while the driver is off the throttle the
engine keeps the throttle partly open, keeps fuelling (no DFCO) and retards the
spark; the work the retard takes from the crank stays in the exhaust gas (the
energy balance above) and keeps the turbine spinning. Absent = no anti-lag and every number is bit-identical to a
configuration without the block.

| Field | Type | Rule | Meaning |
|---|---|---|---|
| `channel` | string | optional, non-empty, default `"anti_lag"` | control channel that switches it (>= 0.5 = on); declared by the vehicle like every component channel |
| `default_enabled` | number | optional, 0 or 1, default 0 | the channel's default value, also used when no channel is wired (a direct engine call, `ps::EngineRig`) |
| `arm_below_command` | number | required, in (0, 1] | throttle command below which anti-lag acts; the command reaching it again re-arms it |
| `min_rpm` | number | required, >= 0 | engine speed range in which it acts |
| `max_rpm` | number | required, > `min_rpm` | |
| `retard_deg` | number | required, >= 0 | spark retard from the scheduled advance while active |
| `plate_floor` | number | required, in [0, 1] | minimum throttle plate while active |
| `max_active_s` | number | required, > 0 | longest active time per lift (a thermal limit) |
| `source` | string | required, non-empty | provenance (PLAN.md section 8) |

Rejections (each prefixed `"<path>: anti_lag: "`): `'anti_lag' must be an object`,
`unknown key '<key>'` (unlike the other blocks: a misspelt limit would otherwise
vanish silently), `'channel' must be a non-empty string`, `'default_enabled' must
be 0 or 1`, `'<key>' is required`, `'arm_below_command' must be in (0, 1]`,
`'min_rpm' must be >= 0`, `'max_rpm' must be > 'min_rpm'`, `'retard_deg' must be
>= 0`, `'plate_floor' must be in [0, 1]`, `'max_active_s' must be > 0`, `'source'
is required (PLAN.md section 8)`.

Model (`core/src/drivetrain/engine_simulated.cpp`, once per substep, decided right
after the throttle request and before DFCO):
- A throttle command (after the TCU floor/ceiling, before an ignition cut) at or
  above `arm_below_command` re-arms anti-lag, zeroes its timer and switches it off.
- Below it, anti-lag is active while the channel reads >= 0.5, the engine is
  Running, rpm is in `[min_rpm, max_rpm]`, it is armed and the timer has room for
  this substep. The timer counts active substeps only; once `max_active_s` is used
  up, anti-lag disarms until the next re-arm (at 960 Hz, 1.5 s = exactly 1440
  substeps). A substep that starves the engine of fuel (empty tank) while active
  also disarms it for the rest of the lift.
- While active: DFCO is held off; the plate target (electronic) or the plate itself
  (mechanical, a throttle kicker) is at least `plate_floor`; the spark is the
  scheduled advance minus `retard_deg`, through the same `eta_spark` curve as any
  spark offset (torque falls by the curve, fuel does not).
- The retard needs no exhaust term of its own: it lowers the indicated work, and
  the exhaust energy balance above puts the difference into the gas. The sound
  field `anti_lag_extra_exhaust_w` reports `omega * (tau_comb(scheduled spark) -
  tau_comb(retarded spark))`, the indicated work the retard removes (a diagnostic,
  read by nothing).
- Energy: the exhaust heat is part of `engine_heat` (fuel energy minus indicated
  work) - the ledger books nothing new and `balance_error` is unchanged.
- No wastegate override: the boost PID's target is still `target_boost * plate`, so
  `plate_floor` doubles as the anti-lag boost target.

Hashing: the activity flag, armed flag, timer and every block
parameter except `channel`/`default_enabled`/`source` are folded into the state
hash, appended after the other turbo state and only when the block exists. The
block is NOT part of the derived-map cache key. A hot swap
(`World::replace_turbo_configuration`) to a configuration without `anti_lag`
clears the anti-lag state; channels are declared when the vehicle is built, so a
swapped-in block naming a channel the vehicle did not declare runs on its
`default_enabled`.

Known limits (measured, I7d D5, `tests/unit/test_turbo_anti_lag.cpp`):
- The lumped pair's rotor speed follows airflow: turbine and compressor power
  nearly balance at every flow, so a low `plate_floor` leaves the rotor SLOWER than
  a DFCO coast (fixture engine, 5000 rpm, rotor 1 s after the lift: floor 0.2 ->
  15.5-18.6k rpm vs 39.8k with anti-lag off; 0.35 -> ~40k; 0.5 -> ~53k).
- Torque is cut only by `eta_spark`: on the hyper V8 at 6000 rpm, floor 0.5 and
  30 deg retard the crank still sees +630..640 N*m while anti-lag acts (a car keeps
  pulling). A cylinder cut or deeper retard tables would be needed to hold the
  crank torque near zero.

## Derived-map cache

The head table (61 x 49 nodes over normalised speed 0..1.5 and flow 0..1.2) and
the outlet temperature ratio curve (91 nodes over pressure ratio 1..10) are generated at
load. With a non-empty `cache_dir` they are cached as
`<cache_dir>/turbo/<key:016x>.json` (`{version, key, speed, flow, head, pr,
temperature_ratio}`). The key is FNV-1a 64 over the raw bytes of a version
constant and the 14 numbers `reference_head_pa`, `rotor_inertia_kgm2`,
`reference_rpm`, `max_mass_flow_kg_s`, `compressor_efficiency`, `turbine_efficiency`,
`turbine_area_m2`, `target_boost_pa`, `intercooler_effectiveness`,
`shaft_loss_w_at_reference`, `wastegate_kp`, `wastegate_ki_per_s`, `wastegate_kd_s`,
`wastegate_actuator_tau_s` (after the count scaling). A cached file is accepted only if
its key matches and its tables equal the freshly generated ones bit for bit;
otherwise it is silently regenerated and rewritten; an unwritable cache never
prevents loading. The key is also the maps' `content_hash` that
`SimulatedEngine::fold_state_hash` folds into the state hash.

## Sound/HUD state (never hashed)

The engine publishes the turbo through `EngineSoundState`
(`ps/drivetrain/powertrain_state.h`, in `VehicleSnapshot::powertrain.engines[0]`);
all fields are 0 for an engine without a turbo and are never hashed or read back
for physics:

| Field | Unit | Meaning |
|---|---|---|
| `turbo_rpm` | rpm | rotor speed |
| `boost_bar` | bar | manifold gauge pressure `(p_im - ambient)` |
| `p_exhaust_pa` | Pa | turbine inlet (exhaust back) pressure |
| `p_charge_pa`, `t_charge_k` | Pa, K | compressor supply to the throttle (post-cooler) |
| `wastegate_opening` | 0..1 | wastegate actuator position (1 = fully open) |
| `compressor_flow_kg_s` | kg/s | total compressor mass flow (the pair) |
| `compressor_pressure_ratio` | - | compressor outlet pressure over ambient, >= 1 |
| `bov_opening` | 0..1 | blow-off valve opening (0 without a valve) |
| `bov_flow_kg_s` | kg/s | vent flow of the latest substep (included in `compressor_flow_kg_s`) |
| `bov_vent` | 0/1 | 1 = the valve recirculates, 0 = atmosphere or no valve |
| `surge_active` | 0/1 | 1 while the operating point is left of the hardware file's `surge_line` (0 without a line) |
| `surge_events` | count | entries into surge, cumulative since the engine was built |
| `bov_events` | count | rising edges of the opening (> 0.1, re-armed below 0.05), cumulative since the engine was built (a counter for sound, never hashed) |
| `anti_lag_active` | 0/1 | 1 while anti-lag acts (0 without an `anti_lag` block) |
| `anti_lag_activations` | count | rising edges of `anti_lag_active`, cumulative since the engine was built |
| `anti_lag_extra_exhaust_w` | W | indicated work rate the retard removes this substep (it stays in the exhaust gas through the energy balance; a diagnostic) |
| `exhaust_port_k` | K | exhaust gas temperature at the ports (the energy balance) |
| `turbine_inlet_k` | K | gas temperature at the turbine inlet, after the exhaust wall |
| `exhaust_wall_k` | K | lumped manifold/turbine-housing wall temperature |

## Shipped files

`data/turbos/hyper_turbo_approx.json` ([AS] per-turbo approximation, not a
measured map) and `data/turbo_configurations/hyper_twin_default.json` (count 2,
1.8 bar gauge, intercooler 0.95, the `boost_control` above), reached through
`data/engines/hyper_v8_one1_audio.json`'s `default_turbo_configuration` and used by
`data/vehicles/car_hyper.json`.
