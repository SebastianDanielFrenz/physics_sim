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
`max_mass_flow_kg_s`, `turbine_area_m2`, `wastegate_area_m2` and `shaft_loss_w_at_reference` (a pair of
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
effectiveness, a PID wastegate on the post-cooler charge pressure, a wastegate
port in parallel with the turbine (both see the same manifold pressure; the
exhaust splits by effective area, turbine share `A_t / (A_t + A_wg * opening)`,
manifold pressure `p_amb + m^2 / (2 rho (A_t + A_wg * opening)^2)` from the total
exhaust flow `m`), turbine power from the turbine's share, compressor power
bounded by the head the CURRENT rotor speed makes (`pr <= 1 + head(speed, flow) /
p_amb`; the charge pressure lags the head by 25 ms, and a slowing wheel does not
pay for stored charge), and a rotor
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
| `rotor_inertia_kgm2` | number | yes | > 0; x `count` | shaft + wheels inertia of ONE turbo (hyper_turbo_approx: 2.3e-4, derived from Garrett GTX3582R-class wheel sizes, source in the file's `rotor_inertia_source`) |
| `reference_rpm` | number | yes | > 0 | rotor speed at which `reference_head_bar` is reached at zero flow; also the energy cap reference |
| `reference_head_bar` | number (bar) | yes | > 0 | compressor head at `reference_rpm` and zero flow. `reference_head_pa` is rejected: `"<path>: 'reference_head_pa' was replaced by 'reference_head_bar' (bar)"` |
| `max_mass_flow_kg_s` | number | yes | > 0; x `count` | compressor flow that closes the head curve to zero (normalising flow) |
| `compressor_efficiency` | number | yes | > 0, <= 1 | isentropic; sets the outlet temperature table and the compressor power |
| `turbine_efficiency` | number | yes | > 0, <= 1 | turbine power factor |
| `turbine_area_m2` | number | yes | > 0; x `count` | effective turbine throat; with the wastegate shut it sets the exhaust back-pressure `dp = m^2 / (2 rho A_t^2)` |
| `wastegate_area_m2` | number | no | > 0; x `count` | effective (Cd x geometric) wastegate port area of ONE turbo, in parallel with the turbine. Absent = 2.4 x `turbine_area_m2` [AS] (the hyper ratio below). Too small an area cannot hold the boost target at high exhaust flow (boost creep: the gate saturates open and the rotor runs to its cap) |
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
energy balance above) and keeps the turbine spinning. Optionally (the "extension"
fields) the spark is set to an absolute late angle, a share of the firing events
is spark-cut with the fuel still injected (that charge burns in the exhaust
manifold), the mixture is enriched and the wastegate is held toward a boost
target. Published ALS practice (Wikipedia "Antilag system", CC BY-SA, citing
Markus, MotorTrend 2024-09-09): ignition typically 35-45 deg ATDC, a throttle
bypass/kick of about 12-20 deg of plate angle, slight enrichment, mainly for
cooling. Absent = no anti-lag and every number is bit-identical to a
configuration without the block.

| Field | Type | Rule | Meaning |
|---|---|---|---|
| `channel` | string | optional, non-empty, default `"anti_lag"` | control channel that switches it (>= 0.5 = on); declared by the vehicle like every component channel |
| `default_enabled` | number | optional, 0 or 1, default 0 | the channel's default value, also used when no channel is wired (a direct engine call, `ps::EngineRig`) |
| `arm_below_command` | number | required, in (0, 1] | throttle command below which anti-lag acts; the command reaching it again re-arms it |
| `min_rpm` | number | required, >= 0 | engine speed range in which it acts |
| `max_rpm` | number | required, > `min_rpm` | |
| `retard_deg` | number | required unless `spark_advance_deg` is given, >= 0 | spark retard from the scheduled advance while active |
| `spark_advance_deg` | number | optional, in [-90, 60], excludes `retard_deg` | absolute spark advance while active, deg BTDC (negative = ATDC; ALS practice -35..-45) |
| `cylinder_cut_fraction` | number | optional, in [0, 1), default 0 | share of the firing events whose spark is cut; the fuel is still injected and booked |
| `manifold_burn_fraction` | number | optional, in [0, 1], default 1 | share of the cut charge's (oxygen-limited) fuel energy released in the exhaust manifold; the rest leaves as chemical enthalpy |
| `lambda` | number | optional, in [0.5, 1.5], default = the ECU target | lambda while active (enrichment) |
| `boost_target_fraction` | number | optional, in [0, 1], default 0 | while active the wastegate PID sees `max(plate, boost_target_fraction)` instead of the plate |
| `plate_floor` | number | required, in [0, 1] | minimum throttle plate while active |
| `max_active_s` | number | required, > 0 | longest active time per lift (a thermal limit) |
| `source` | string | required, non-empty | provenance (PLAN.md section 8) |

Rejections (each prefixed `"<path>: anti_lag: "`): `'anti_lag' must be an object`,
`unknown key '<key>'` (unlike the other blocks: a misspelt limit would otherwise
vanish silently), `'channel' must be a non-empty string`, `'default_enabled' must
be 0 or 1`, `'<key>' is required`, `'arm_below_command' must be in (0, 1]`,
`'min_rpm' must be >= 0`, `'max_rpm' must be > 'min_rpm'`, `'retard_deg' must be
>= 0`, `'plate_floor' must be in [0, 1]`, `'max_active_s' must be > 0`, `'source'
is required (PLAN.md section 8)`, `give either 'retard_deg' or 'spark_advance_deg',
not both`, `'spark_advance_deg' must be in [-90, 60]`, `'cylinder_cut_fraction' must
be in [0, 1)`, `'manifold_burn_fraction' must be in [0, 1]`, `'lambda' must be in
[0.5, 1.5]`, `'boost_target_fraction' must be in [0, 1]`.

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
  (mechanical, a throttle kicker) is at least `plate_floor`; the spark is
  `spark_advance_deg` if given, else the scheduled advance minus `retard_deg`,
  through the same `eta_spark` curve as any spark offset (torque falls by the
  curve, fuel does not). Below the curve's most retarded node (the generator's
  cycle points end 40 deg after MBT) `eta_spark` is extrapolated along the first
  segment's slope, floored at 0 - under anti-lag only; elsewhere the curve still
  clamps (PROVISIONAL: a real ALS fires past the table; a linear extrapolation is
  the simplest continuation, the cycle model at those angles is the alternative).
- `lambda` > 0 replaces the ECU lambda target: fuel per charge `m_air / (lambda *
  AFR)` and `f_lambda` follow it.
- Cylinder cut `c`: `tau_comb = (1 - c) * eta_i f_lambda eta_spark LHV/2pi *
  m_fuel_rev`; the whole injected `m_fuel_rev` is booked (ledger, tank). In the
  exhaust balance the released energy `R = min(1, lambda) Q_f` splits into the
  fired share `(1 - c) R` (indicated work, in-cylinder wall heat
  `in_cylinder_wall_heat_fraction * (1 - c) R`, sensible rest) and the cut share
  `c R`, of which `manifold_burn_fraction * c R` is released in the manifold
  (added to the sensible enthalpy) and the rest stays chemical. `Q_f = W_i +
  Q_wall + H + chemical` still holds to round-off. Not modelled: a manifold
  ignition threshold (the cut charge burns whatever the wall temperature), misfire
  or knock limits, a lambda- or temperature-dependent burn fraction, ECU material
  protection.
- The retard needs no exhaust term of its own: it lowers the indicated work, and
  the exhaust energy balance above puts the difference into the gas. The sound
  field `anti_lag_extra_exhaust_w` reports `omega * (tau_comb(scheduled spark) -
  tau_comb(retarded spark))`, the indicated work the retard removes (a diagnostic,
  read by nothing).
- Energy: the exhaust heat is part of `engine_heat` (fuel energy minus indicated
  work) - the ledger books nothing new and `balance_error` is unchanged.
- Wastegate: the boost PID's target is `target_boost * plate`; with
  `boost_target_fraction` > 0 it is `target_boost * max(plate,
  boost_target_fraction)` while active. Without it `plate_floor` doubles as the
  anti-lag boost target, and a floor below 0.10 commands the wastegate open.

Hashing: the activity flag, armed flag, timer and every block
parameter except `channel`/`default_enabled`/`source` are folded into the state
hash, appended after the other turbo state and only when the block exists; the
extension fields only when one of them differs from its default, so a block
without them hashes as before. The
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
- Retard alone (`retard_deg`) cuts torque only through `eta_spark`: on the hyper V8
  at 6000 rpm, floor 0.5 and 30 deg retard the crank still sees +605 N*m while
  anti-lag acts (a car keeps pulling), at 59.6 % of WOT air and 36.3 g/s of fuel.
- Rally ALS with the extension (`tests/unit/test_turbo_als_cut.cpp` AC6, hyper V8,
  6000 rpm, test fixture: spark -40, floor 0.12, cut 0.4, lambda 0.85, burn 1.0,
  boost target 1.0; mean over +0.25..+1 s after the lift; parallel wastegate
  port, derived 2.3e-4 rotor): air 16.3 % of WOT (0.136 vs 0.839 kg/s), fuel
  11.29 g/s = 15.8 % of WOT, crank torque -47 N*m, boost -0.50 bar, wastegate
  0.89, manifold burn 165 kW, rotor 104.2k / 90.5k / 70.7k rpm at +0.25 / +0.5 /
  +1 s (D5 fixture 99.6k / 82.2k / 59.0k; no anti-lag 92.2k / 90.1k / 86.4k).
  With a realistic rotor the plain fuel-cut coast keeps MORE rotor speed over the
  first second than anti-lag: the plate floor loads the compressor while the boost
  PID, seeing the charge above target, opens the gate. Anti-lag holds manifold
  pressure (D5 fixture +0.66 bar vs -0.93 bar at +1 s), not rotor speed, on that
  time scale. The same fixture at floor 0.17 passed 24.9 % of WOT air.
  Before (floor 0.17, 2e-5 rotor): air 14.8 % of WOT (0.121 vs 0.819 kg/s), fuel 10.03 g/s =
  14.5 % of WOT (69.10 g/s), crank torque -64 N*m, boost -0.55 bar, manifold burn
  147 kW, port 2279 K, turbine inlet 1383 K, rotor 60.9k / 37.4k / 58.4k rpm at
  +0.25 / +0.5 / +1 s (the boost-target PID overshoots at the lift - rotor 174k at
  +80 ms - and the small rotor dips while the gate reopens; D5 fixture 62.0k /
  54.9k / 57.5k; no anti-lag 23.1k / 19.8k / 14.6k). Without a boost target the wastegate PID follows the
  plate: floors 0.12-0.15 give 5-7 % of WOT air and 3.6-8.6k rpm at +1 s, floor
  0.20 11-12 % and 15-19k (spark -35..-45, cut 0-0.5); a plate below 0.10 commands
  the wastegate open. The boost target (wastegate held shut) is what keeps the
  rotor up at realistic airflow. The air
  share is steep in the floor near 0.16-0.18 (12-21 %) because the held boost
  raises the manifold pressure.

Known limits of the wastegate split (review fix, `tests/unit/test_turbo_wastegate_split.cpp`):
- The steady idle rotor speed is only ~10 rpm whatever the inertia (60 s held idle
  after a lift). At idle flow (~0.01 kg/s, gate commanded open below 10 % plate) the
  mean-value orifice turbine sees a few Pa of pressure drop, i.e. milliwatts, and
  the equilibrium sits against the compressor power in the head table's lowest
  speed cell (bilinear, so head is linear in speed below 1/40 of `reference_rpm`).
  Real turbos idle much faster on exhaust pulse energy, which a mean-value turbine
  does not carry. A coasting rotor stays far above that until its stored energy is
  spent: the shipped 2.3e-4 rotor is at 46.6k rpm after 2 s of DFCO and 8 s of
  idle, 3.1k after 60 s of idle.
- The compressor-power bound (`pr <= 1 + head(current speed) / p_amb`) decides the
  lift with the 2e-5 rotor: without it the rotor is at 0 rpm 0.25 s after a WOT
  lift at 6000 rpm (with it 23.1k); with a 1e-4 or 2.3e-4 rotor it moves the lift
  by < 0.3 %.

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
| `anti_lag_manifold_burn_w` | W | fuel energy rate of the cut charge released in the exhaust manifold (0 without a cut) |
| `exhaust_port_k` | K | exhaust gas temperature at the ports (the energy balance) |
| `turbine_inlet_k` | K | gas temperature at the turbine inlet, after the exhaust wall |
| `exhaust_wall_k` | K | lumped manifold/turbine-housing wall temperature |

## Shipped files

`data/turbos/hyper_turbo_approx.json` ([AS] per-turbo approximation, not a
measured map) and `data/turbo_configurations/hyper_twin_default.json` (count 2,
1.8 bar gauge, intercooler 0.95, the `boost_control` above), reached through
`data/engines/hyper_v8_one1_audio.json`'s `default_turbo_configuration` and used by
`data/vehicles/car_hyper.json`.
