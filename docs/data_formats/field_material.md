# `physics_sim.field_material/1` and `physics_sim.field_methods/1` - fertiliser materials and application methods (farming F1b)

Loaders: `ps::io::load_material_json` / `parse_material_json`, `load_methods_json` / `parse_methods_json`
(`core/include/ps/field/field_io.h`, `ps_field_io`). Descriptors `MaterialDesc`, `MethodDesc`
(`ps/field/nutrient_desc.h`). Files: `data/field_materials/` (CAN 27, DAP 18-46, MOP 60, UAN 28, cattle slurry 7.5 % DM,
pig slurry 5 % DM, solid cattle manure, liquid digestate, carbonate lime 90, quicklime 90; `methods.json`). A field
config holds a material palette (1-based ids) and a method palette. Every number needs a tag in its block's `source`
object (`[C]`, `[D]`, `[M]`, `[AS]`, see [soil.md](soil.md)).

## Material

| Block / key | Unit | Notes |
|---|---|---|
| `format`, `name`, `kind` | | `kind` informational: mineral, liquid, slurry, manure, digestate, lime |
| `composition.dm_frac` | | dry matter, fraction of fresh matter, in (0, 1] |
| `composition.density_kg_m3` | kg/m3 | converts m3/ha to t/ha |
| `composition.n_total_kg_t`, `nh4_n_kg_t`, `urea_n_kg_t`, `no3_n_kg_t` | kg/t fresh | organic N = total - the three mineral forms (must not be negative) |
| `composition.p2o5_kg_t`, `k2o_kg_t` | kg/t | to the available P / K pool at once |
| `composition.cao_kg_t` | kg CaO/t | neutralising value: > 0 liming (pending pool, reacts over time), < 0 acidifying (booked at once) |
| `ammonia.k_vol_per_day`, `k_inf_per_day` | 1/day | competing first-order volatilisation and infiltration of the surface ammoniacal N (urea counted as ammoniacal) at 15 degC; emission factor = k_vol/(k_vol + k_inf) |
| `ammonia.k_inf_per_mm_rain` | 1/(day mm) | infiltration rate added per mm of rain |
| `ammonia.org_slow_fraction` | | share of the organic N entering the slow humus pool, the rest the fresh pool |

## Methods (`methods.json`, a `methods` list)

| Key | Notes |
|---|---|
| `name` | unique; the scenario names it. Shipped: broadcast, trailing_hose, trailing_shoe, injected_open, injected_closed, incorporated |
| `soil_placed_fraction` | share of the ammoniacal N put into the soil at once (injection) |
| `k_vol_factor` | multiplies the surface volatilisation rate (band spreading) |
| `incorporation_delay_h` | hours until the surface is worked in (the surface pool is then moved into the soil); < 0 = never |
| `incorporated` | lime reacts at the plough-layer rate instead of the surface rate |

Surface ammonia: per day the surface ammoniacal pool TAN loses `tan kv/(kv+ki) (1 - exp(-(kv+ki) t_exposed/24 h))` as
NH3 (kv = mass-weighted `k_vol_per_day x k_vol_factor x nh3_temp_q10^((T-15)/10)`, ki = `k_inf_per_day` + rain term) and
infiltrates the rest into the ammonium pool; an incorporation within the day cuts the exposure.

Rejections: wrong `format`; unknown key; missing key; untagged number; mineral forms above the total N; `dm_frac`
outside (0, 1]; a negative content or rate; a duplicate method name; `soil_placed_fraction` outside [0, 1]; a negative
`k_vol_factor`. `tests/unit/test_field_io.cpp` I8, I9.
