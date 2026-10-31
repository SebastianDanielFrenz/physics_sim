// core/src/field/desc_validation.cpp - structural validation of soil and crop
// descriptors (shared by the loaders and the Field constructor).

#include "ps/field/crop_desc.h"
#include "ps/field/soil_desc.h"

#include <cmath>
#include <string>

namespace ps::field {

namespace {

bool fin(real x) { return std::isfinite(x); }

struct Check {
    std::string prefix;
    std::string err;
    void pos(const char* name, real v) {
        if (err.empty() && !(fin(v) && v > 0.0)) err = prefix + name + " must be a finite number > 0";
    }
    void nonneg(const char* name, real v) {
        if (err.empty() && !(fin(v) && v >= 0.0)) err = prefix + name + " must be a finite number >= 0";
    }
    void frac(const char* name, real v) {
        if (err.empty() && !(fin(v) && v >= 0.0 && v <= 1.0)) err = prefix + name + " must be in [0, 1]";
    }
    void fail_if(bool bad, const std::string& msg) {
        if (err.empty() && bad) err = prefix + msg;
    }
};

} // namespace

std::string soil_validation_error(const SoilDesc& s) {
    Check c{"soil " + s.name + ": ", {}};
    c.fail_if(s.name.empty(), "name must not be empty");
    c.pos("theta_fc", s.theta_fc);
    c.pos("theta_wp", s.theta_wp);
    c.fail_if(!(s.theta_fc > s.theta_wp), "theta_fc must exceed theta_wp");
    c.fail_if(s.theta_fc >= 1.0, "theta_fc must be below 1");
    c.pos("depth_m", s.depth_m);
    c.pos("fallow_root_depth_m", s.fallow_root_depth_m);
    c.fail_if(!(s.fallow_root_depth_m < s.depth_m), "fallow_root_depth_m must be below depth_m");
    c.pos("evaporation_depth_m", s.evaporation_depth_m);
    c.fail_if(!(s.evaporation_depth_m <= s.fallow_root_depth_m), "evaporation_depth_m must not exceed fallow_root_depth_m");
    c.pos("rew_mm", s.rew_mm);
    c.fail_if(fin(s.rew_mm) && !(s.rew_mm < s.tew_mm()), "rew_mm must be below the total evaporable water");
    c.fail_if(!(fin(s.curve_number_ii) && s.curve_number_ii > 30.0 && s.curve_number_ii < 100.0),
              "curve_number_ii must be in (30, 100)");
    c.fail_if(!(fin(s.cn_normal_wetness) && s.cn_normal_wetness > 0.0 && s.cn_normal_wetness < 1.0),
              "cn_normal_wetness must be in (0, 1)");
    c.frac("ia_ratio", s.ia_ratio);
    c.nonneg("snow_melt_mm_per_k_day", s.snow_melt_mm_per_k_day);
    c.fail_if(!fin(s.snow_melt_base_c), "snow_melt_base_c must be finite");
    if (c.err.empty() && s.has_nutrients) c.err = soil_nutrient_validation_error(s.name, s.nutrients);
    if (c.err.empty() && s.has_management) c.err = soil_management_validation_error(s.name, s.management);
    return c.err;
}

std::string crop_validation_error(const CropDesc& d) {
    Check c{"crop " + d.name + ": ", {}};
    c.fail_if(d.name.empty(), "name must not be empty");
    c.fail_if(!fin(d.t_base_c) || !fin(d.t_upper_c) || !(d.t_upper_c > d.t_base_c), "t_upper_c must exceed t_base_c");
    c.fail_if(d.stages.size() < 2, "needs at least two stages");
    for (std::size_t i = 0; c.err.empty() && i < d.stages.size(); ++i) {
        const CropStageDesc& st = d.stages[i];
        c.fail_if(st.name.empty(), "stage names must not be empty");
        c.fail_if(!fin(st.gdd) || !fin(st.bbch), "stage " + st.name + " has a non-finite value");
        if (i == 0) c.fail_if(st.gdd != 0.0, "the first stage must start at gdd 0");
        else c.fail_if(!(st.gdd > d.stages[i - 1].gdd), "stage gdd values must be strictly ascending");
    }
    const int n = d.stage_count();
    for (int idx : {d.stage_emergence, d.stage_flowering, d.stage_senescence, d.stage_maturity})
        c.fail_if(idx < 0 || idx >= n, "emergence/flowering/senescence/maturity stages must exist");
    c.fail_if(!(d.stage_emergence >= 1 && d.stage_emergence < d.stage_flowering && d.stage_flowering <= d.stage_senescence &&
                d.stage_senescence <= d.stage_maturity),
              "stages must be ordered emergence < flowering <= senescence <= maturity (emergence after the first)");
    if (d.vernalises) {
        c.fail_if(d.stage_vernalisation_end < 1 || d.stage_vernalisation_end >= n, "vernalisation needs a valid until_stage");
        c.fail_if(!(d.vern_t_min_c < d.vern_t_opt_c && d.vern_t_opt_c < d.vern_t_max_c), "vernalisation needs t_min < t_opt < t_max");
        c.pos("vern_days_saturation", d.vern_days_saturation);
        c.frac("vern_min_rate_factor", d.vern_min_rate_factor);
    }
    c.pos("cc0_cm2_per_plant", d.cc0_cm2_per_plant);
    c.pos("plant_density_per_ha", d.plant_density_per_ha);
    c.fail_if(!(fin(d.ccx) && d.ccx > 0.0 && d.ccx <= 1.0), "ccx must be in (0, 1]");
    c.fail_if(fin(d.ccx) && !(d.cc0(1.0) < 0.5 * d.ccx), "seedling canopy cc0 must be below half of ccx");
    c.pos("cgc_per_gdd", d.cgc_per_gdd);
    c.pos("cdc_per_gdd", d.cdc_per_gdd);
    c.nonneg("senescence_acceleration", d.senescence_acceleration);
    c.pos("kc_tr_x", d.kc_tr_x);
    c.pos("height_max_m", d.height_max_m);
    c.fail_if(!(fin(d.p_depletion) && d.p_depletion > 0.0 && d.p_depletion < 1.0), "p_depletion must be in (0, 1)");
    c.fail_if(!(fin(d.exp_p_upper) && fin(d.exp_p_lower) && d.exp_p_upper >= 0.0 && d.exp_p_upper < d.exp_p_lower &&
                d.exp_p_lower <= 1.0),
              "canopy expansion needs 0 <= p_upper < p_lower <= 1");
    c.nonneg("exp_shape", d.exp_shape);
    c.fail_if(!(fin(d.sen_p_upper) && d.sen_p_upper >= 0.0 && d.sen_p_upper < 1.0), "sen_p_upper must be in [0, 1)");
    c.nonneg("sen_shape", d.sen_shape);
    c.pos("wp_star_g_m2", d.wp_star_g_m2);
    c.nonneg("biomass_gdd_full", d.biomass_gdd_full);
    c.nonneg("biomass_rs_ref_mj_m2", d.biomass_rs_ref_mj_m2);
    c.nonneg("biomass_rue_g_mj", d.biomass_rue_g_mj);
    c.frac("hi0", d.hi0);
    c.fail_if(!(fin(d.straw_harvest_fraction) && d.straw_harvest_fraction > 0.0 && d.straw_harvest_fraction <= 1.0),
              "straw_harvest_fraction must be in (0, 1]");
    c.pos("hi_build_gdd", d.hi_build_gdd);
    c.pos("flowering_length_gdd", d.flowering_length_gdd);
    c.nonneg("excess_fruits_pct", d.excess_fruits_pct);
    c.fail_if(!(fin(d.ppol) && d.ppol > 0.0 && d.ppol < 1.0), "ppol must be in (0, 1)");
    c.fail_if(!(fin(d.hi_sto_b) && d.hi_sto_b >= 1.0), "hi_sto_b must be >= 1");
    c.pos("pol_cold_width_k", d.pol_cold_width_k);
    c.pos("pol_heat_width_k", d.pol_heat_width_k);
    c.fail_if(!fin(d.pol_cold_t_c) || !fin(d.pol_heat_t_c), "pollination temperatures must be finite");
    c.pos("pol_logistic_steepness", d.pol_logistic_steepness);
    c.pos("zr_min_m", d.zr_min_m);
    c.fail_if(!(fin(d.zr_max_m) && d.zr_max_m >= d.zr_min_m), "zr_max_m must be >= zr_min_m");
    c.fail_if(d.stage_emergence >= 1 && d.stage_emergence < n && fin(d.root_gdd_full) &&
                  !(d.root_gdd_full > d.stages[static_cast<std::size_t>(d.stage_emergence)].gdd),
              "root_gdd_full must exceed the emergence gdd");
    c.fail_if(!(fin(d.frost_kill_onset_c) && fin(d.frost_kill_full_c) && d.frost_kill_full_c < d.frost_kill_onset_c),
              "frost kill needs full < onset (degC)");
    c.nonneg("snow_insulation_swe_mm", d.snow_insulation_swe_mm);
    c.fail_if(!fin(d.frost_snow_floor_c), "frost_snow_floor_c must be finite");
    c.nonneg("harvest_window_days", d.harvest_window_days);
    c.frac("shed_rate_per_day", d.shed_rate_per_day);
    if (c.err.empty() && d.has_nutrients) c.err = crop_nutrient_validation_error(d.name, d.nutrients);
    if (c.err.empty() && d.has_management) c.err = crop_management_validation_error(d.name, d.management);
    return c.err;
}

std::string soil_nutrient_validation_error(const std::string& name, const SoilNutrientDesc& n) {
    Check c{"soil " + name + ": nutrients.", {}};
    c.nonneg("no3_init_kg_ha", n.no3_init_kg_ha);
    c.nonneg("nh4_init_kg_ha", n.nh4_init_kg_ha);
    c.nonneg("org_n_kg_ha", n.org_n_kg_ha);
    c.nonneg("min_rate_per_day_20c", n.min_rate_per_day_20c);
    c.fail_if(!(fin(n.min_q10) && n.min_q10 >= 1.0), "min_q10 must be >= 1");
    c.frac("min_moisture_floor", n.min_moisture_floor);
    c.nonneg("fresh_rate_per_day_20c", n.fresh_rate_per_day_20c);
    c.nonneg("nitrif_rate_per_day_20c", n.nitrif_rate_per_day_20c);
    c.nonneg("denit_rate_per_day_20c", n.denit_rate_per_day_20c);
    c.fail_if(!(fin(n.denit_wetness) && n.denit_wetness >= 0.0 && n.denit_wetness < 1.0), "denit_wetness must be in [0, 1)");
    c.fail_if(!(fin(n.soil_t_lag_days) && n.soil_t_lag_days >= 1.0), "soil_t_lag_days must be >= 1");
    c.fail_if(!fin(n.soil_t_init_c), "soil_t_init_c must be finite");
    c.pos("soil_t_ramp_c", n.soil_t_ramp_c);
    c.nonneg("n_deposition_kg_ha_day", n.n_deposition_kg_ha_day);
    c.fail_if(!(fin(n.nh3_temp_q10) && n.nh3_temp_q10 >= 1.0), "nh3_temp_q10 must be >= 1");
    c.pos("soil_test_kg_per_mg100g", n.soil_test_kg_per_mg100g);
    c.nonneg("p_test_init_mg100g", n.p_test_init_mg100g);
    c.nonneg("p_reserve_kg_ha", n.p_reserve_kg_ha);
    c.nonneg("p_eq_mg100g", n.p_eq_mg100g);
    c.frac("p_release_per_day", n.p_release_per_day);
    c.frac("p_fix_per_day", n.p_fix_per_day);
    c.nonneg("p_rel_b", n.p_rel_b);
    c.fail_if(n.p_rel_b > 0.0 && !(fin(n.p_target_mg100g) && n.p_target_mg100g > 0.0), "p_target_mg100g must be > 0 with p_rel_b > 0");
    c.nonneg("k_test_init_mg100g", n.k_test_init_mg100g);
    c.nonneg("k_reserve_kg_ha", n.k_reserve_kg_ha);
    c.nonneg("k_eq_mg100g", n.k_eq_mg100g);
    c.frac("k_release_per_day", n.k_release_per_day);
    c.frac("k_fix_per_day", n.k_fix_per_day);
    c.nonneg("k_rel_b", n.k_rel_b);
    c.fail_if(n.k_rel_b > 0.0 && !(fin(n.k_target_mg100g) && n.k_target_mg100g > 0.0), "k_target_mg100g must be > 0 with k_rel_b > 0");
    c.fail_if(!(fin(n.ph_init) && n.ph_init > 3.0 && n.ph_init < 9.0), "ph_init must be in (3, 9)");
    c.pos("buffer_cao_kg_ha_per_ph", n.buffer_cao_kg_ha_per_ph);
    c.nonneg("cao_loss_kg_ha_per_mm_drain", n.cao_loss_kg_ha_per_mm_drain);
    c.nonneg("lime_react_per_day", n.lime_react_per_day);
    c.frac("lime_surface_factor", n.lime_surface_factor);
    c.fail_if(!(fin(n.ph_avail_floor) && fin(n.ph_avail_full) && n.ph_avail_floor < n.ph_avail_full),
              "ph_avail_floor must be below ph_avail_full");
    c.frac("avail_floor_factor", n.avail_floor_factor);
    return c.err;
}

std::string crop_nutrient_validation_error(const std::string& name, const CropNutrientDesc& n) {
    Check c{"crop " + name + ": nutrients.", {}};
    c.pos("nc_a", n.nc_a);
    c.nonneg("nc_b", n.nc_b);
    c.pos("nc_w_min_t_ha", n.nc_w_min_t_ha);
    c.pos("nc_const_pct", n.nc_const_pct);
    c.nonneg("n_min_conc_pct", n.n_min_conc_pct);
    c.fail_if(fin(n.nc_const_pct) && !(n.n_min_conc_pct < n.nc_const_pct), "n_min_conc_pct must be below nc_const_pct");
    c.nonneg("n_stress_start_t_ha", n.n_stress_start_t_ha);
    c.nonneg("n_uptake_max_kg_ha_day", n.n_uptake_max_kg_ha_day);
    c.frac("n_uptake_frac_per_day", n.n_uptake_frac_per_day);
    c.nonneg("n_residual_kg_ha", n.n_residual_kg_ha);
    c.nonneg("n_root_fraction", n.n_root_fraction);
    c.frac("n_grain_share", n.n_grain_share);
    c.frac("n_fixation_fraction", n.n_fixation_fraction);
    c.nonneg("p2o5_conc_pct_dm", n.p2o5_conc_pct_dm);
    c.frac("p2o5_grain_share", n.p2o5_grain_share);
    c.nonneg("k2o_conc_pct_dm", n.k2o_conc_pct_dm);
    c.frac("k2o_grain_share", n.k2o_grain_share);
    c.fail_if(!(fin(n.ph_opt_low) && fin(n.ph_opt_high) && fin(n.ph_floor) && n.ph_floor < n.ph_opt_low && n.ph_opt_low <= n.ph_opt_high),
              "pH response needs ph_floor < ph_opt_low <= ph_opt_high");
    c.frac("ph_floor_factor", n.ph_floor_factor);
    c.nonneg("lodging_nmin_start_kg_ha", n.lodging_nmin_start_kg_ha);
    c.fail_if(!(fin(n.lodging_nmin_full_kg_ha) && n.lodging_nmin_full_kg_ha > n.lodging_nmin_start_kg_ha),
              "lodging_nmin_full_kg_ha must exceed lodging_nmin_start_kg_ha");
    c.frac("lodging_max_loss", n.lodging_max_loss);
    c.fail_if(!(fin(n.lodging_bbch_from) && fin(n.lodging_bbch_to) && n.lodging_bbch_from <= n.lodging_bbch_to),
              "lodging_bbch_from must not exceed lodging_bbch_to");
    return c.err;
}

std::string material_validation_error(const MaterialDesc& m) {
    Check c{"material " + m.name + ": ", {}};
    c.fail_if(m.name.empty(), "name must not be empty");
    c.fail_if(!(fin(m.dm_frac) && m.dm_frac > 0.0 && m.dm_frac <= 1.0), "dm_frac must be in (0, 1]");
    c.pos("density_kg_m3", m.density_kg_m3);
    c.nonneg("n_total_kg_t", m.n_total_kg_t);
    c.nonneg("nh4_n_kg_t", m.nh4_n_kg_t);
    c.nonneg("urea_n_kg_t", m.urea_n_kg_t);
    c.nonneg("no3_n_kg_t", m.no3_n_kg_t);
    c.fail_if(fin(m.n_total_kg_t) && fin(m.nh4_n_kg_t) && fin(m.urea_n_kg_t) && fin(m.no3_n_kg_t) &&
                  m.nh4_n_kg_t + m.urea_n_kg_t + m.no3_n_kg_t > m.n_total_kg_t * (1.0 + 1.0e-9),
              "the mineral N forms exceed the total N");
    c.nonneg("p2o5_kg_t", m.p2o5_kg_t);
    c.nonneg("k2o_kg_t", m.k2o_kg_t);
    c.fail_if(!fin(m.cao_kg_t), "cao_kg_t must be finite");
    c.nonneg("k_vol_per_day", m.k_vol_per_day);
    c.nonneg("k_inf_per_day", m.k_inf_per_day);
    c.nonneg("k_inf_per_mm_rain", m.k_inf_per_mm_rain);
    c.fail_if(m.nh4_n_kg_t + m.urea_n_kg_t > 0.0 && !(m.k_inf_per_day > 0.0), "k_inf_per_day must be > 0 for a material with ammoniacal N");
    c.frac("org_slow_fraction", m.org_slow_fraction);
    return c.err;
}

std::string method_validation_error(const MethodDesc& m) {
    Check c{"method " + m.name + ": ", {}};
    c.fail_if(m.name.empty(), "name must not be empty");
    c.frac("soil_placed_fraction", m.soil_placed_fraction);
    c.nonneg("k_vol_factor", m.k_vol_factor);
    c.fail_if(!fin(m.incorporation_delay_h), "incorporation_delay_h must be finite");
    return c.err;
}

} // namespace ps::field
