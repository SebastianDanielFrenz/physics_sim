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
    c.frac("hi0", d.hi0);
    c.pos("hi_build_gdd", d.hi_build_gdd);
    c.pos("pollination_window_gdd", d.pollination_window_gdd);
    c.nonneg("hi_water_stress_k", d.hi_water_stress_k);
    c.fail_if(!(fin(d.heat_pollination_lo_c) && fin(d.heat_pollination_up_c) && d.heat_pollination_lo_c < d.heat_pollination_up_c),
              "heat pollination needs lo < up");
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
    return c.err;
}

} // namespace ps::field
