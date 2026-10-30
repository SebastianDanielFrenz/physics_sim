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
`[AS]`. In the shipped wheat file only the FAO-56 Table 22 values are `[C]`;
the AquaCrop defaults and the phenology are recalled and tagged `[AS]` on
purpose.

| Block / key | Unit | Notes |
|---|---|---|
| `format` | | `"physics_sim.crop/1"` |
| `extends`, `name`, `family` | | `family` groups crops for the pathogen/rotation hooks of later increments |
| `thermal.t_base_c`, `t_upper_c` | degC | daily GDD = (clamp(Tmax) + clamp(Tmin))/2 - Tbase, both clamped to [Tbase, Tupper] |
| `phenology.stages[]` | | ordered `{name, bbch, gdd}`: development units from sowing at which the stage starts, strictly ascending, first 0. Names the model needs: `emergence` (index >= 1), `flowering`, `senescence`, `maturity` in that order (`flowering <= senescence <= maturity`) |
| `phenology.vernalisation_until` | | stage name; development is slowed before it. Present iff a `vernalisation` block exists |
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
| `yield.hi0`, `hi_build_gdd` | | reference harvest index, development units after flowering over which it builds |
| `yield.pollination_window_gdd`, `hi_water_stress_k`, `heat_pollination_lo_c`, `heat_pollination_up_c` | | HI is reduced by the mean (1 - Ks_sto) in the window (times k) and by the mean heat factor (Tmax between lo and up) |
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
