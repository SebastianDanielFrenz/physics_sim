# Engine thermal format — `physics_sim.engine_thermal/1`

Files: `data/engine_thermal/*.json` (shipped: `class_si_passenger.json`,
`toyota_a25a_fks.json`, `i4_2l_na.json`). Loader:
`ps::io::load_engine_thermal_json` (`core/src/io/engine_thermal_io.cpp`,
declared in `ps/io/engine_thermal_io.h`). Public descriptor:
`ps::drivetrain::EngineThermalDesc`
(`core/include/ps/drivetrain/engine_thermal_desc.h`). Model:
`core/src/drivetrain/engine_thermal.{h,cpp}`. Design: `out/engine_thermal_plan.md`,
vault PHYS-063.

This is the **engine side** of the lumped engine thermal model: the heat
capacities, the heat split, the oil and the thermostat. The **vehicle side**
(radiator, fan, gauge, warnings) is [`cooling_system.md`](cooling_system.md).
A vehicle becomes thermal only by naming a cooling system
([`vehicle.md`](vehicle.md)); an engine's `default_thermal` alone changes
nothing.

Temperatures are in degrees C in the file (the `_c` suffix) and converted to K
at load. Differences (`hysteresis_k`) stay in K. Per-litre forms are expanded
against `displacement_l` at load, so the descriptor is always absolute.

Unknown keys are rejected. The thermal fields are **not** map-generator inputs
and are not part of the `.psmaps` cache key: adding `default_thermal` to an
engine file changes neither its map content hash nor its cache key.

## Where it is named

| Where | Key | Meaning |
|---|---|---|
| engine file | `default_thermal` | path relative to the engine file |
| vehicle `powertrain.components[]` engine entry | `thermal` | per-vehicle override, path relative to the vehicle file (allowed next to `ref`) |

## Fields

| Field | Type | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `format` | string | required | `"physics_sim.engine_thermal/1"` | |
| `extends` | string | optional | path relative to this file | parent file; RFC 7396 merge, parent first, this file last (a `null` member deletes the parent's). Chains up to 8 deep, cycles rejected |
| `source` | string | required | non-empty | provenance with [C]/[D]/[M]/[AS] tags |
| `displacement_l` | real | required with any `*_per_l` form | `> 0` | engine displacement, litres |
| `capacity` **or** `capacity_per_l` | object | exactly one | `engine_j_k`, `oil_j_k` (both required, `> 0`) | lump capacities, J/K (per litre in the second form). Engine lump = block + head + coolant; oil lump = oil + oil-wetted metal |
| `wall_heat_fraction` | real | required | in `(0, 0.5)` | f_wall: share of the RELEASED fuel energy that reaches the engine lump |
| `friction_to_oil_fraction` | real | required | in `[0, 1]` | beta: share of the friction heat that goes into the oil lump (the rest heats the engine lump) |
| `motoring_friction_fraction` | real | optional, default 1 | in `(0, 1]` | phi_f: friction share of a **graph** (torque-map) engine's motoring curve; the rest is pumping. Ignored by simulated engines, so class files can carry it |
| `oil` | object | required | below | |
| `conductance` **or** `conductance_per_l` | object | exactly one | below | |
| `coolant` | object | required | `cp_j_kg_k`, `density_kg_m3` both `> 0` | 50/50 glycol |
| `pump` | object | required | exactly one of `l_min_per_krpm` or `l_min_per_krpm_per_l`, `> 0` | coolant pump flow per 1000 rpm |
| `thermostat` | object | required | below | |
| `water_jacket` | object | optional | below | an explicit metal-to-coolant path; absent = the two-lump model, bit-identical to before |
| `ecu` | | reserved | rejected | reserved for increment T2 |

`oil`: `kv40_mm2_s`, `kv100_mm2_s` (`kv40 > kv100 > 0`), `density_15c_kg_m3`
(`> 0`, density at 15.6 C), `reference_temperature_c` (in `[60, 140]`; the
friction maps apply unchanged at and above it), `hydrodynamic_fraction` (phi_h,
in `(0, 1]`: the hydrodynamic share of the friction map at the reference speed),
`hydrodynamic_reference_rpm` (required, in `[500, 10000]`: the speed at which
the whole friction map is scaled by exactly `k`), `friction_scale_min`
(optional, default 1, must be `>= 1`).
**`friction_scale_max` is rejected** with a message naming the owner ruling of
2026-10-05: there is no friction cap.

`conductance` keys (all required, `>= 0`; the per-litre form multiplies all
but the first by `displacement_l`): `oil_coolant_time_constant_s` (`> 0`, never
scaled: the oil-coolant conductance is `C_oil / tau`), `block_air_w_k`,
`block_air_w_k_per_m_s`, `oil_air_w_k`, `oil_air_w_k_per_m_s` (the `_per_m_s`
ones multiply the chassis ram air speed).

`thermostat`: `open_c < full_open_c`, `hysteresis_k >= 0`,
`time_constant_s > 0` (the wax lag).

## Water jacket

Without `water_jacket` the engine lump is block, head and coolant at one
temperature, and the radiator and thermostat read that temperature. With it the
engine lump splits into two lumps:

- metal: capacity `C_metal = capacity.engine_j_k - C_coolant`;
- coolant: capacity `C_coolant = coolant_volume_l / 1000 * coolant.density_kg_m3 * coolant.cp_j_kg_k`.

They are joined by the jacket conductance

`G_jacket = wetted_area_m2 * h(Q)`, `h(Q) = max(htc_min_w_m2_k, htc_ref_w_m2_k * (Q / reference_flow_l_min)^flow_exponent)`

with `Q` the coolant pump flow (`pump` x rpm / 1000 x the cooling system's
`pump.flow_factor`). The jacket always sees the whole pump flow; the thermostat
only decides how much of it goes through the radiator. `flow_exponent` 0.8 is
the Dittus-Boelter exponent of turbulent forced convection (Nu ~ Re^0.8).

Wall heat, the friction heat share of the engine lump, the block-to-air loss
and the oil exchange act on the metal. The radiator, thermostat, fan, gauge and
warnings act on the coolant. A bigger wetted area, a higher `htc_ref_w_m2_k` or
a faster pump (`pump.flow_factor`) lower the metal temperature (and with it the
oil temperature) at a steady load; the coolant temperature is still set by the
thermostat and radiator.

| Field | Unit | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `wetted_area_m2` **or** `wetted_area_m2_per_l` | m2 (m2 per litre) | exactly one | `(0, 20]` m2 after scaling | coolant-wetted jacket wall area |
| `coolant_volume_l` **or** `coolant_volume_l_per_l` | L (L per litre) | exactly one | `(0, 100]` L after scaling | coolant held in the jacket and the circuit |
| `reference_flow_l_min` **or** `reference_flow_l_min_per_l` | l/min (l/min per litre) | exactly one | `(0, 5000]` l/min after scaling | the pump flow at which the film coefficient equals `htc_ref_w_m2_k` |
| `htc_ref_w_m2_k` | W/(m2 K) | required | `[100, 50000]` | film coefficient at the reference flow |
| `htc_min_w_m2_k` | W/(m2 K) | required | `[0, htc_ref_w_m2_k]` | floor at low flow (natural convection, an engine at rest) |
| `flow_exponent` | - | optional, default 0.8 | `[0.3, 1.0]` | `h ~ Q^n` |

The `_per_l` forms multiply by `displacement_l`. The metal must keep at least
10 % of `capacity.engine_j_k` (a coolant volume that leaves less is rejected).
Unknown keys are rejected.

State: the metal temperature joins the hashed state only when a jacket is
present. A warm or ambient start, and `set_engine_thermal_state`, set the metal
to the coolant temperature. Snapshot fields (never hashed): `metal_c`,
`jacket_heat_w` (metal to coolant), `jacket_conductance_w_k`,
`coolant_flow_l_min`.

Steady state without a time history:
`ps::drivetrain::solve_steady_thermal(EngineThermalConfig, SteadyThermalInput)`
(`ps/drivetrain/engine_thermal_steady.h`) returns the coolant, metal and oil
temperatures, thermostat opening, fan state, conductances and heat flows the
model settles at for a constant rpm, wall heat, friction heat, ambient and ram
speed. It uses the same helpers as the runtime step
(`coolant_pump_flow_l_min`, `water_jacket_conductance_w_k`,
`radiator_conductance_w_k`, `thermal_capacities`) and works with and without a
jacket (without one `metal_c` equals `coolant_c`).

Worked example (test fixture `tests/support/fixtures/thermal/i4_2l_na_jacket.json`,
extends `data/engine_thermal/i4_2l_na.json`, 2.0 L):

```json
{
  "format": "physics_sim.engine_thermal/1",
  "extends": "../../../../data/engine_thermal/i4_2l_na.json",
  "source": "...",
  "water_jacket": {
    "wetted_area_m2_per_l": 0.2,
    "coolant_volume_l_per_l": 3.0,
    "reference_flow_l_min_per_l": 30,
    "htc_ref_w_m2_k": 5000,
    "flow_exponent": 0.8,
    "htc_min_w_m2_k": 500
  }
}
```

This gives 0.4 m2, 6 L (21726 J/K of coolant, 31474 J/K left for the metal)
and h = 5000 W/(m2 K) at 60 l/min. Steady state from `solve_steady_thermal` with the sedan cooling
system at 3000 rpm, 12 kW wall heat, 3 kW friction, 25 C, 25 m/s (the
thermostat holds the coolant in its band, so the coolant barely moves and the
upgrade shows in the metal and the oil):

| Variant | Coolant C | Metal C | Oil C |
|---|---|---|---|
| no jacket (`i4_2l_na.json`) | 88.55 | 88.55 | 85.44 |
| jacket | 88.54 | 93.67 | 89.24 |
| jacket, wetted area x2 (or `htc_ref_w_m2_k` x2) | 88.54 | 91.15 | 87.37 |
| jacket, `pump.flow_factor` 1.5 | 88.36 | 92.11 | 88.09 |

(Measured with the `[.][diag]` case in `tests/unit/test_engine_thermal_jacket.cpp`.)

## Friction law

`k = max(friction_scale_min, 1 + hydrodynamic_fraction * (sqrt(mu / mu_ref) - 1))`
with the Walther (ASTM D341) viscosity through KV40 and KV100 and the density
`rho15 * (1 - 6.5e-4 * (T - 15.6 C))`. `k` is exactly 1.0 at and above
`reference_temperature_c`. The plant engine uses `k(T_oil)`; the engine's ECU
model uses `k` of the oil temperature rounded to 1 K.

`k` does not multiply the friction map (engine thermal T1b). The engine adds a
hydrodynamic increment to the map's speed-dependent part:

`dF(n) = (k - 1) * F(n_ref) * sqrt(n / n_ref)`, `n_ref = hydrodynamic_reference_rpm`

(linear through zero below 50 rpm). Hydrodynamic friction under a fixed load
goes as `sqrt(mu * speed)`, so the same square root that gives the viscosity
exponent gives the speed exponent. `F` is the simulated engine's
`friction_torque_nm`, or `motoring_friction_fraction * |motoring(n)|` for a
graph engine (pumping is never scaled). The Coulomb part (`F(0)`) and the
breakaway torque are boundary friction and are never scaled, so a cold engine
breaks away with the warm starter torque. A graph engine's idle feed-forward and
idle authority rise by `dF(idle) / (wot(idle) - motoring(idle))` with the ECU
`k` (cold fast idle).

## Minimal example

```json
{
  "format": "physics_sim.engine_thermal/1",
  "extends": "class_si_passenger.json",
  "source": "Toyota A25A-FKS: displacement from its own geometry, everything else the class default.",
  "displacement_l": 2.487
}
```

## Common mistakes

- A class file with `*_per_l` forms is not loadable alone (no `displacement_l`);
  an engine-specific file extends it and adds the displacement.
- `capacity` together with `capacity_per_l` (or the same for `conductance` and
  `pump`): exactly one is allowed. Delete the parent's with `null` when
  overriding in the other form.
- `friction_scale_max`: rejected (no cap).
- `water_jacket` with a coolant volume so large that the metal keeps under
  10 % of `capacity.engine_j_k`: rejected. Raise the engine capacity or lower
  the volume.
