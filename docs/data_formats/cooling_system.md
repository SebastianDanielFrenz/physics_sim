# Cooling system format — `physics_sim.cooling_system/1`

Files: `data/cooling_systems/*.json` (shipped: `class_passenger_car.json`,
`sedan.json`). Loader: `ps::io::load_cooling_system_json`
(`core/src/io/engine_thermal_io.cpp`, declared in `ps/io/engine_thermal_io.h`).
Public descriptor: `ps::drivetrain::CoolingSystemDesc`
(`core/include/ps/drivetrain/engine_thermal_desc.h`). Design:
`out/engine_thermal_plan.md`, vault PHYS-063.

The **vehicle side** of the engine thermal model: radiator, fan, oil cooler,
gauge and warnings. The engine side is
[`engine_thermal.md`](engine_thermal.md). **Naming a cooling system in the
vehicle file (`"cooling_system"`) is what makes a vehicle thermal**; the
loader then requires the engine to have a thermal file (its `default_thermal`
or the component's `thermal`), else it throws.

Temperatures are in degrees C in the file (`_c`), K in the descriptor. Unknown
keys are rejected. `extends` works as in `engine_thermal.md`.

## Fields

| Field | Type | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `format` | string | required | `"physics_sim.cooling_system/1"` | |
| `extends` | string | optional | path relative to this file | RFC 7396 merge, this file last |
| `source` | string | required | non-empty | provenance with [C]/[D]/[M]/[AS] tags |
| `radiator` | object | required | below | cross-flow eps-NTU radiator |
| `fan` | object | required | below | on/off fan with hysteresis |
| `oil_cooler` | object | optional | `air_w_k_per_m_s >= 0` (default 0) | added to the oil-air conductance's ram term |
| `gauge` | object | required | below | needle position vs coolant temperature |
| `warnings` | object | required | `overheat_c`, `cold_c`, `cold_c < overheat_c` | warning bit thresholds on coolant temperature |
| `heater_core` | | reserved | rejected | reserved for increment T7 |

`radiator`: `frontal_area_m2 > 0`, `ram_coefficient` in `[0, 1]` (face velocity
= coefficient x ram airspeed), `ua_ref_w_k > 0` and `air_mass_flow_ref_kg_s > 0`
(the UA at that air flow), `ua_exponent` (optional, default 0.6, in `(0, 1]`:
`UA = UA_ref * (m_air / m_ref)^exponent`).

`fan`: `face_velocity_m_s >= 0`, `on_c`, `off_c` (`off_c < on_c`, the hysteresis
band), `power_w >= 0` (reported only, nothing draws it). Radiator face speed is
`max(ram, fan)` while the fan is on. This fan is unrelated to the aero fans.

`gauge`: `x_c` (strictly ascending, at least 2) and `y` (same length, within
`[0, 1]`, non-decreasing) - a `Curve1D` over coolant temperature. The shipped
default is buffered like a dashboard needle (0.5 from 75 to 105 C).

## Minimal example

```json
{
  "format": "physics_sim.cooling_system/1",
  "extends": "class_passenger_car.json",
  "source": "Sedan: the class defaults."
}
```

A fan failure or a different radiator is a user-level file extending this one
and overriding only the changed keys.
