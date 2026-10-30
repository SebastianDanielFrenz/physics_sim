# `physics_sim.field_scenario/1` - headless field scenario (farming F1a)

Loader: `ps::io::load_field_scenario(path)`; runner:
`ps::io::run_field_scenario(scenario, csv)` (`core/include/ps/field/field_scenario.h`,
library `ps_field_io`); tool: `ps_field_run <scenario.json> [out.csv]`. Files:
`data/field_scenarios/` (deliberately not `data/scenarios/`: those are the
ps_run hash-compare scenarios). The demo is `wheat_10y.json`.

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
| `operations[]` | no | `{op: "sow"|"harvest", crop (sow only), doy, first_year (0), last_year, every_years (1), density_scale (1)}`; `doy` is the real day of year 1..366, years count from game year 0 |

CSV columns (one row per day): `day,year,doy,tmin_c,tmax_c,rain_mm,snow_mm,rs_mj,
et0_mm,rh_mean,wind2_m_s` (weather), then grid means `swe_mm,depletion_root_mm,
taw_root_mm,zr_m`, cell 0's `crop,stage`, grid means `bbch,gdd_sow,cc,
biomass_t_ha,grain_t_ha,tr_mm,evap_mm,runoff_mm,drain_mm,ks_sto`.

`ps_field_run` prints each harvest (field mean grain and straw dry matter, BBCH),
the worst-cell water-balance residual, the cell-0 precipitation total and the
state hash.

Rejections: wrong `format`; unknown key; an unknown `op`; `doy` outside [1, 366];
`every_years` < 1; a sow naming no crop of the scenario or without one;
`soil_pattern` entries that are not soil indices; `years` < 1.
`tests/unit/test_field_io.cpp` I6, I7.
