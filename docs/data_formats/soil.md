# `physics_sim.soil/1` - soil for the field model (farming F1a)

Loader: `ps::io::load_soil_json(path)` / `parse_soil_json`
(`core/include/ps/field/field_io.h`, library `ps_field_io`). Descriptor:
`ps::field::SoilDesc` (`ps/field/soil_desc.h`). Files: `data/soils/*.json`
(`silt_loam_loess.json`, `sandy_loam.json`). A field's palette holds one or
more soils; `FieldConfig::cell_soil` names a soil per grid cell.

The soil is a two-bucket column (model overview in `ps/field/field.h`): the root-zone
bucket (depth = the rooting depth of the cell, `fallow_root_depth_m` without a
crop) and the layer below it down to `depth_m`.

**Source tags.** Every number (and boolean) in a block must have an entry of the
same name in that block's `source` object, and the text must start with
`[C]` cited, `[D]` derived, `[M]` measured or `[AS]` assumed. The loader
refuses a file with a parameter without a tag. String members (`name`,
`texture`) are exempt.

| Block / key | Unit | Notes |
|---|---|---|
| `format` | | `"physics_sim.soil/1"` |
| `extends` | | merge-patch parent (as for climate files); the override must itself be tagged |
| `name` | | non-empty |
| `texture` | | informational |
| `water.theta_fc` | m3/m3 | field capacity, in (theta_wp, 1) |
| `water.theta_wp` | m3/m3 | permanent wilting point, > 0 |
| `water.depth_m` | m | bottom of the water bucket; the roots cannot go deeper |
| `water.fallow_root_depth_m` | m | depth of the "root zone" bucket without a crop, below `depth_m` |
| `evaporation.depth_m` | m | Ze, the surface layer that dries by evaporation (FAO-56 ch. 7), <= `fallow_root_depth_m` |
| `evaporation.rew_mm` | mm | readily evaporable water (stage 1), below the total evaporable water TEW = 1000 (theta_fc - 0.5 theta_wp) Ze (FAO-56 eq 73) |
| `runoff.curve_number_ii` | | SCS CN for average antecedent moisture, in (30, 100) |
| `runoff.cn_normal_wetness` | | relative root-zone wetness (1 - Dr/TAW) at which CN II applies, in (0, 1); CN I (dry) and CN III (wet) follow CN_I = CN_II/(2.281 - 0.01281 CN_II), CN_III = CN_II/(0.427 + 0.00573 CN_II), interpolated linearly in the wetness |
| `runoff.ia_ratio` | | initial abstraction as a fraction of the retention S = 25.4 (1000/CN - 10) mm, in [0, 1] |
| `snow.melt_mm_per_k_day` | mm/(K day) | degree-day melt factor, >= 0 |
| `snow.melt_base_c` | degC | melt starts above this daily mean |

Derived (not in the file): TAW per metre = 1000 (theta_fc - theta_wp) mm/m,
TEW (above), total TAW = TAW per metre x `depth_m`.

Rejections: wrong `format`; unknown key; a missing required key; a number given
as a string; a parameter without a source tag; theta_fc <= theta_wp;
`fallow_root_depth_m` >= `depth_m`; `evaporation.depth_m` > `fallow_root_depth_m`;
REW >= TEW; curve number outside (30, 100); a negative melt factor.
`tests/unit/test_field_io.cpp` I2, I3, I5.

## Optional `nutrients` block (F1b)

Present on a soil -> `SoilDesc::has_nutrients`; a field with `nutrients_enabled` needs it on every soil (and the crops'
block, [crop.md](crop.md)). Flat numeric keys plus one `source` object (every key tagged). Descriptor
`ps::field::SoilNutrientDesc` (`ps/field/nutrient_desc.h`). Units: kg/ha for pools, mg/100 g for soil tests (VDLUFA CAL),
pH in CaCl2, kg CaO/ha for the calcium-equivalent pool.

| Keys | Meaning |
|---|---|
| `no3_init_kg_ha`, `nh4_init_kg_ha`, `org_n_kg_ha` | initial nitrate (split between the two water layers by root depth), ammonium and slow organic N |
| `min_rate_per_day_20c`, `fresh_rate_per_day_20c`, `min_q10`, `min_moisture_floor` | first-order mineralisation of the slow / fresh pool at 20 degC and field capacity; Q10 shared by all biological rates; moisture factor linear from the floor at a dry root zone to 1 at field capacity |
| `nitrif_rate_per_day_20c`, `denit_rate_per_day_20c`, `denit_wetness` | NH4 to NO3; root-layer nitrate loss above the relative wetness |
| `soil_t_lag_days`, `soil_t_init_c`, `soil_t_ramp_c` | soil temperature = air temperature low-passed; rates ramp from 0 at 0 degC to full at `soil_t_ramp_c` |
| `n_deposition_kg_ha_day` | atmospheric N, booked as ammonium |
| `nh3_temp_q10` | surface volatilisation factor per 10 K around 15 degC |
| `soil_test_kg_per_mg100g` | kg nutrient/ha of the plough layer per mg/100 g |
| `p_*`, `k_*`: `test_init_mg100g`, `reserve_kg_ha`, `eq_mg100g`, `release_per_day`, `fix_per_day`, `rel_b`, `target_mg100g` | plough-layer pools: available (from the test value) and slow reserve exchanging towards the equilibrium test value; Mitscherlich relative yield 1 - exp(-b test) normalised at `target` |
| `ph_init`, `buffer_cao_kg_ha_per_ph`, `cao_loss_kg_ha_per_mm_drain`, `lime_react_per_day`, `lime_surface_factor` | pH = ph_init + CaO pool / buffer, clamped 3..8; unavoidable CaO loss per mm drained; reaction rate of lime pending in the plough layer, and the relative rate of lime left on the surface |
| `ph_avail_full`, `ph_avail_floor`, `avail_floor_factor` | mineralisation and P/K availability factor over pH |

Rejections (`desc_validation.cpp`): a missing or untagged key, an unknown key, a P/K release or fixation rate outside [0, 1] per day, negative pools or rates, ph_init outside (3, 9), Q10 below 1, a
non-positive buffer. `tests/unit/test_field_io.cpp` I10.

## Optional `management` block (F1c)

Needed by every soil of a field with `"management": true` (see field_scenario.md). Every key is required and carries an entry in
`management.source` starting with [C], [D], [M] or [AS]; unknown keys are rejected. The three layers of the management
layer are the surface, the upper layer (`layer_a_m`, 0.10 m) and the lower layer (`layer_b_m`, to 0.30 m in total).

| group | keys |
|---|---|
| layers | `layer_a_m`, `layer_b_m` |
| workability (relative wetness of the root zone, 0 wilting point, 1 field capacity) | `workable_wet_full`, `workable_wet_zero`, `workable_dry_zero`, `workable_dry_full` |
| structure | `damage_gain`, `damage_recovery_per_day`, `damage_cn_increase`, `damage_root_penalty`, `loose_cn_decrease`, `loose_stage2_evap_reduction`, `loose_bd_reduction`, `settle_per_mm_rain`, `settle_per_day`, `tillage_dry_out_mm_per_m` |
| seedbed | `seedbed_rest`, `seedbed_wet_floor`, `seedbed_settle_per_mm`, `seedbed_settle_per_day` |
| residue | `cover_per_t_ha` (soil cover = 1 - exp(-x m), Gregory 1982 form), `mulch_evap_max_reduction`, `res_c_fraction`, `res_k20_per_day`, `res_surface_factor`, `res_stubble_factor`, `res_moisture_floor`, `res_chop_gain`, `res_assim_eff`, `res_cn_bio`, `res_n_floor_kg_ha` |

Residue is four pools (loose straw, stubble, buried upper, buried lower) with carbon, nitrogen and, for the two surface pools,
P2O5 and K2O. Decomposition is first order with a Q10 temperature factor and a moisture factor; the decomposers build
`res_assim_eff` of the carbon into biomass and humus (kept in `soc`, no further decay - a simplification) and take the nitrogen they
need from the mineral pool (immobilisation) or release it. Residue cover multiplies the soil evaporation by
`1 - mulch_evap_max_reduction x cover` (FAO-56 chapter 11 mulch idea).

Soil evaporation (F1c, all fields): when the weather record provides `e_wet_mm`, the Priestley-Taylor energy limit of a wet
surface (alpha 1.26, Rn floored at 0), the dual-coefficient evaporation of a cell is capped by `kr x few x e_wet_mm`. Records
made by hand (tests) leave `e_wet_mm` negative and are not capped.

## Optional `organic_matter` block (F1d part 5)

Needed when the field runs with `soc_enabled` (scenario key `"soc": true`; needs management). The slow organic N pool of the nutrient
block is the humus; its carbon is that pool times `humus_cn`. Every number carries a source tag.

| key | unit | meaning |
|---|---|---|
| `layer_depth_m` | m | the layer the carbon content refers to (plough layer, at most `depth_m`) |
| `bulk_density_t_m3` | t/m3 | dry bulk density of that layer (SOC % = 100 x carbon / (bulk density x layer depth)) |
| `humus_cn` | - | C:N of the humus pool |
| `rate_per_day_20c` | 1/day | first-order decay of the whole humus pool at 20 degC, optimum moisture and pH; replaces the nutrient block's `min_rate_per_day_20c` for the slow pool, for carbon and nitrogen alike |
| `humification` | - | share of the decomposed residue carbon that enters the humus (ICBM h); must not exceed `res_assim_eff`, and `humification / humus_cn` must not exceed `res_assim_eff / res_cn_bio` |
| `awc_per_soc_pct` | m3/m3 per % | change of `theta_fc` per 1 % SOC (Minasny and McBratney 2018: 1.16 mm per 100 mm) |

Processes (core/src/field/soc.cpp and the soc branches of nutrients.cpp, management.cpp): humus decays with the mineralisation factors
(temperature, moisture, pH), the N it releases goes to ammonium and the carbon to CO2, so the C:N of the pool stays; decomposed residue
carbon dC adds `humification x dC` to the humus and respires the rest of what the microbes assimilated; organic material applied to the field
adds the carbon of its slow organic N at `humus_cn`. The plant-available water follows the carbon: `theta_fc` of the soil file is the value
at the initial carbon content, the profile mean of the change is `awc_per_soc_pct x (SOC % - initial SOC %) x layer_depth_m / depth_m`,
the depletions move with the capacity (stored water unchanged), a full profile that loses capacity drains the difference. Carbon ledger:
`CarbonLedger::c_humus_decay`, `c_amendment`, `c_humus_initial`; `c_balance_residual` then checks the humus as a stock.

