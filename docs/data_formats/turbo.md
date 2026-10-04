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

## Configuration file — `physics_sim.turbo_configuration/1`

| Field | Type | Required | Rule | Meaning |
|---|---|---|---|---|
| `format` | string | yes | `"physics_sim.turbo_configuration/1"` | format tag |
| `name`, `source` | string | no | not enforced | free text / provenance |
| `turbo` | string | yes | path relative to THIS file; must name a `physics_sim.turbo/1` file | the hardware |
| `count` | integer | yes | [1, 8] | number of identical turbos in the pair, see "Pair scaling" (a count over 8 throws `"<path>: turbo count must be <= 8"`) |
| `target_boost_bar` | number (bar) | yes | > 0 | GAUGE boost the wastegate controller holds at full plate. `target_boost_pa` is rejected: `"<path>: 'target_boost_pa' was replaced by 'target_boost_bar' (gauge, bar)"` |
| `intercooler_effectiveness` | number | yes | > 0, <= 1 | fraction of the compressor outlet temperature rise removed; there is no pressure drop in the baseline model |
| `boost_control` | object | no | see below | PID on the post-cooler charge pressure driving the wastegate |

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

## Shipped files

`data/turbos/hyper_turbo_approx.json` ([AS] per-turbo approximation, not a
measured map) and `data/turbo_configurations/hyper_twin_default.json` (count 2,
1.8 bar gauge, intercooler 0.95, the `boost_control` above), reached through
`data/engines/hyper_v8_one1_audio.json`'s `default_turbo_configuration` and used by
`data/vehicles/car_hyper.json`.
