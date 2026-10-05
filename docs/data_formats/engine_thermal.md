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
| `ecu` | | reserved | rejected | reserved for increment T2 |

`oil`: `kv40_mm2_s`, `kv100_mm2_s` (`kv40 > kv100 > 0`), `density_15c_kg_m3`
(`> 0`, density at 15.6 C), `reference_temperature_c` (in `[60, 140]`; the
friction maps apply unchanged at and above it), `hydrodynamic_fraction` (phi_h,
in `(0, 1]`), `friction_scale_min` (optional, default 1, must be `>= 1`).
**`friction_scale_max` is rejected** with a message naming the owner ruling of
2026-10-05: there is no friction cap.

`conductance` keys (all required, `>= 0`; the per-litre form multiplies all
but the first by `displacement_l`): `oil_coolant_time_constant_s` (`> 0`, never
scaled: the oil-coolant conductance is `C_oil / tau`), `block_air_w_k`,
`block_air_w_k_per_m_s`, `oil_air_w_k`, `oil_air_w_k_per_m_s` (the `_per_m_s`
ones multiply the chassis ram air speed).

`thermostat`: `open_c < full_open_c`, `hysteresis_k >= 0`,
`time_constant_s > 0` (the wax lag).

## Friction law

`k = max(friction_scale_min, 1 + hydrodynamic_fraction * (sqrt(mu / mu_ref) - 1))`
with the Walther (ASTM D341) viscosity through KV40 and KV100 and the density
`rho15 * (1 - 6.5e-4 * (T - 15.6 C))`. `k` is exactly 1.0 at and above
`reference_temperature_c`. The plant engine uses `k(T_oil)`; the engine's ECU
model uses `k` of the oil temperature rounded to 1 K.

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
