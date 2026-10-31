# `physics_sim.crop/1` - crop for the field model (farming F1a)

Loader: `ps::io::load_crop_json(path)` / `parse_crop_json`
(`core/include/ps/field/field_io.h`, library `ps_field_io`). Descriptor:
`ps::field::CropDesc` (`ps/field/crop_desc.h`). File: `data/crops/winter_wheat.json`
(the only crop of F1a). The model is an AquaCrop-style daily crop (Raes et al.
2012; Steduto et al. 2009): thermal time -> BBCH stages, vernalisation (Wang &
Engel 1998), canopy cover, transpiration x water productivity -> biomass,
harvest index -> grain. Nothing in the model reads the date; only the weather
and the thermal time it produces move the crop.

**Source tags** work as in `soil.md`: every number or boolean in a block needs an
entry in that block's `source` object starting with `[C]`, `[D]`, `[M]` or
`[AS]`. In the shipped wheat file the AquaCrop v7.3 Annex I.10 values and the
FAO-56 Table 12/22 values are `[C]` (document and section in each entry), the
stage table is `[D]` from DWD phenology and DWD station data, the rest `[AS]`.

| Block / key | Unit | Notes |
|---|---|---|
| `format` | | `"physics_sim.crop/1"` |
| `extends`, `name`, `family` | | `family` groups crops for the pathogen/rotation hooks of later increments |
| `thermal.t_base_c`, `t_upper_c` | degC | daily GDD = (clamp(Tmax) + clamp(Tmin))/2 - Tbase, both clamped to [Tbase, Tupper] |
| `phenology.stages[]` | | ordered `{name, bbch, gdd}`: development units from sowing at which the stage starts, strictly ascending, first 0. Names the model needs: `emergence` (index >= 1), `flowering`, `senescence`, `maturity` in that order (`flowering <= senescence <= maturity`) |
| `phenology.vernalisation_until` | | stage name; vernalisation accumulates from sowing until it, and the development of the emerged plant is slowed before it. Present iff a `vernalisation` block exists |
| `vernalisation.t_min_c`, `t_opt_c`, `t_max_c` | degC | cardinal temperatures of the Wang & Engel response, rate = (2 a^alpha b^alpha - a^(2 alpha))/b^(2 alpha), 1 effective day per day at `t_opt_c`, 0 at/outside the limits |
| `vernalisation.days_saturation` | day | effective days for the full development rate |
| `vernalisation.min_rate_factor` | | development rate factor with no vernalisation, [0, 1]; the factor rises linearly to 1 |
| `canopy.cc0_cm2_per_plant`, `plant_density_per_ha` | cm2, 1/ha | canopy at emergence = cc0 x density (scaled by the sowing's `density_scale`); must stay below CCx/2 |
| `canopy.ccx` | | maximum canopy cover, (0, 1] |
| `canopy.cgc_per_gdd`, `cdc_per_gdd` | 1/GDD | canopy growth/decline coefficients. Growth: exponential up to CCx/2, then CC = CCx - (CCx - CC) e^(-CGC dt); decline after the senescence stage: CC = peak (1 - 0.05 (e^(CDC t / peak) - 1)) |
| `canopy.senescence_acceleration` | | extra senescence progress per unit (1 - Ks_sen) |
| `water.kc_tr_x` | | transpiration coefficient at full canopy; Kcb = kc_tr_x x CC*, CC* = 1.72 CC - CC^2 + 0.3 CC^3 |
| `water.height_max_m` | m | for the Kc,max wind/humidity term (FAO-56 eq 72) |
| `water.p_depletion` | | FAO-56 Table 22 depletion fraction p at ETc 5 mm/day |
| `water.p_adjust_for_etc` | bool | p = p + 0.04 (5 - ETc), limited to [0.1, 0.8] (Table 22 note) |
| `water.exp_p_upper`, `exp_p_lower`, `exp_shape` | | Ks for canopy expansion (AquaCrop curve) |
| `water.sen_p_upper`, `sen_shape` | | Ks for early senescence |
| `biomass.wp_star_g_m2` | g/m2 | normalised water productivity; biomass per day = WP* x Tr/ET0 x min(1, GDD/gdd_full) x stress factors |
| `biomass.gdd_full` | GDD/day | thermal limit of biomass production (0 = off) |
| `yield.hi0`, `hi_build_gdd` | | reference harvest index, development units after flowering over which it builds (also the yield-formation length) |
| `yield.flowering_length_gdd`, `excess_fruits_pct`, `ppol` | GDD, %, | AquaCrop 3.12.5: share of flowers opening per day from the flowering distribution f(k) = 0.00558 k^0.63 - 0.000969 k - 0.00383 (k in % of the duration); f_pollination = min(1, (1 + excess/100) sum F_j Ks_pol,j) with the not-yet-open share counted as pollinated; Ks_pol,w linear from 1 at depletion `ppol` x TAW to 0 at TAW |
| `yield.cold_t_c`, `cold_width_k`, `heat_t_c`, `heat_width_k`, `logistic_steepness` | degC, K | Ks_pol,c / Ks_pol,h: logistic curves (rescaled to 0..1) over `width` below Tmin = `cold_t_c` / above Tmax = `heat_t_c`; Ks_pol = min of the three |
| `yield.sto_coefficient_b` | | AquaCrop 3.12.6 Eq. 3.12m/n: f_stomata = mean over the yield-formation days of 1 - (1 - Ks_sto^(1/10))/b, b in 1 (strong) .. 20 (small) |
| `roots.zr_min_m`, `zr_max_m`, `gdd_full` | m, GDD | root depth, linear in development units from emergence to `gdd_full` (from sowing) |
| `frost.kill_onset_c`, `kill_full_c` | degC | an emerged stand loses (onset - Tmin)/(onset - full) of its stand and canopy per day |
| `frost.snow_insulation_swe_mm`, `snow_floor_c` | mm, degC | with at least this snow water equivalent the stand sees at most Tmin = floor |
| `ripeness.harvest_window_days`, `shed_rate_per_day` | day, 1/day | days at maturity before the grain starts to shed, then the fraction lost per day |

The stress hook (`StressFactors`, one set per cell, all 1 in F1a) multiplies the
daily biomass by min(n, pk, ph, weed) x rotation. Water stress enters through
the transpiration itself.

Rejections: wrong `format`; unknown key; a missing key; a parameter without a
source tag; `vernalisation_until` naming no stage, or given without a
`vernalisation` block; stage gdd not ascending; a required stage missing or out
of order; CCx outside (0, 1]; a seedling canopy above CCx/2; stress limits out
of order; a frost kill full temperature above the onset; a root `gdd_full` before
emergence; a shed rate outside [0, 1]. `tests/unit/test_field_io.cpp` I2, I4.

## Optional `nutrients` block (F1b)

Flat numeric keys plus a tagged `source` object; descriptor `ps::field::CropNutrientDesc`. Concentrations in % of the
aboveground dry matter, masses kg/ha.

| Keys | Meaning |
|---|---|
| `nc_a`, `nc_b`, `nc_w_min_t_ha`, `nc_const_pct` | critical N curve Nc = a W^-b % (W aboveground dry matter t/ha), constant below `nc_w_min_t_ha` (Justes et al. 1994) |
| `n_min_conc_pct`, `n_stress_start_t_ha` | N factor = clamp((N% - min)/(Nc - min)), applied once W exceeds the start and before the senescence stage |
| `n_uptake_max_kg_ha_day`, `n_uptake_frac_per_day`, `n_residual_kg_ha` | uptake limited by the demand to Nc, a daily maximum, a share of the available mineral N and a residue the roots cannot take |
| `n_root_fraction`, `n_grain_share`, `n_fixation_fraction` | root N per aboveground N; grain share of aboveground N at harvest; share of the demand taken from the air |
| `p2o5_conc_pct_dm`, `p2o5_grain_share`, `k2o_conc_pct_dm`, `k2o_grain_share` | P and K uptake along the biomass and their grain shares |
| `ph_opt_low`, `ph_opt_high`, `ph_floor`, `ph_floor_factor` | yield factor over pH: 1 inside the optimum, linear to `ph_floor_factor` at `ph_floor` |
| `lodging_nmin_start_kg_ha`, `lodging_nmin_full_kg_ha`, `lodging_max_loss`, `lodging_bbch_from`, `lodging_bbch_to` | grain loss fraction = max over the BBCH window of max_loss x clamp((Nmin - start)/(full - start)); applied to the grain as (1 - loss) |

The stress hook is filled when nutrients are enabled: `stress_n` = N factor, `stress_pk` = min of the P and K soil-test
factors (with the pH availability), `stress_ph` = pH response; daily biomass x min(n, pk, ph, weed) x rotation.
Rejections: untagged or unknown key, shares outside [0, 1], `ph_floor < ph_opt_low <= ph_opt_high` violated,
`lodging_nmin_full` <= start. `tests/unit/test_field_io.cpp` I10.
