# `physics_sim.field_scenario/1` - headless field scenario (farming F1a, F1b)

Loader: `ps::io::load_field_scenario(path)`; runner:
`ps::io::run_field_scenario(scenario, csv)` (`core/include/ps/field/field_scenario.h`,
library `ps_field_io`); tool: `ps_field_run <scenario.json> [out.csv]`. Files:
`data/field_scenarios/` (deliberately not `data/scenarios/`: those are the
ps_run hash-compare scenarios). The demos are `wheat_10y.json` and (nutrients) `wheat_fertilised_10y.json`.

A scenario is a climate, a seed, soils, crops, a small grid, a number of years
and a list of TEST-ONLY sow/harvest rules. The runner steps a `Field` from game
day 0 using the daily weather record of the climate (a pure function of climate,
seed and day), applying the rules before the step of the matching day. The field
never harvests or kills a crop by itself.

| Key | Required | Notes |
|---|---|---|
| `format` | yes | `"physics_sim.field_scenario/1"` |
| `climate` | yes | path (relative to the scenario) of a `physics_sim.climate/1` file with weather statistics |
| `seed` | no (0) | weather seed |
| `years` | yes | >= 1; game years of `days_per_year` days (365) from game day 0 |
| `weather` | no | `samples_per_day` (48), `wind_height_m` (10), `sleet_snow_fraction` (0.5): `DailyWeatherConfig` |
| `soils` | yes | non-empty list of soil file paths |
| `soil_pattern` | no | soil index per grid column, repeated over the rows; default: soil 0 everywhere |
| `crops` | yes | list of crop file paths; crop ids are 1-based list positions, rules name the crop |
| `grid` | yes | `nx`, `ny`, `cell_size_m` (4), `origin_x_m`, `origin_y_m` |
| `initial_depletion_fraction` | no (0) | of both buckets' TAW |
| `test_irrigate_to_capacity` | no (false) | test only: every morning the profile is brought to field capacity (booked as irrigation) |
| `nutrients` | no (false) | boolean: the field runs the nutrient model (F1b); needs the `nutrients` blocks of every soil and crop |
| `materials` | no | list of `physics_sim.field_material/1` paths ([field_material.md](field_material.md)); ids are 1-based list positions |
| `methods` | no | path of a `physics_sim.field_methods/1` file |
| `operations[]` | no | `{op: "sow"|"harvest"|"apply", doy, first_year (0), last_year, every_years (1), ...}`; `doy` is the real day of year 1..366, years count from game year 0. sow: `crop`, `density_scale` (1). harvest: `straw_removed` in [0, 1] (1: the straw leaves the field, the rest stays as residue). apply: `material`, `method` (names), exactly one of `rate_t_ha`, `rate_m3_ha`, `rate_n_kg_ha` (total N), optional `incorporation_delay_h`, `coverage` (1) |

CSV columns (one row per day): `day,year,doy,tmin_c,tmax_c,rain_mm,snow_mm,rs_mj,
et0_mm,rh_mean,wind2_m_s` (weather), then grid means `swe_mm,depletion_root_mm,
taw_root_mm,zr_m`, cell 0's `crop,stage`, grid means `bbch,gdd_sow,cc,
biomass_t_ha,grain_t_ha,tr_mm,evap_mm,runoff_mm,drain_mm,ks_sto`. With `nutrients` the grid means
`nmin_kg_ha,nh4_kg_ha,tan_surface_kg_ha,ph,p_test,k_test,f_n,f_pk,n_crop_kg_ha` and the cumulative
`n_leached_cum,nh3_cum,n_mineralised_cum,n_denitrified_cum` follow.

`ps_field_run` prints each harvest (field mean grain and straw dry matter, BBCH),
the worst-cell water-balance residual, the cell-0 precipitation total and the
state hash. With `nutrients` it also prints the worst-cell N, P2O5, K2O and CaO balance residuals.

Rejections: wrong `format`; unknown key; an unknown `op`; `doy` outside [1, 366];
`every_years` < 1; a sow naming no crop of the scenario or without one;
`soil_pattern` entries that are not soil indices; `years` < 1; an apply naming no material or method, with none or several rates, or a negative rate; `straw_removed` outside [0, 1].
`tests/unit/test_field_io.cpp` I6, I7.
