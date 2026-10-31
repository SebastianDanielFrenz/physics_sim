// core/src/field/management_validation.cpp - structural validation of the management descriptors (F1c).

#include "ps/field/management_desc.h"

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

const char* const kKindNames[kWeedKinds] = {"annual_grass", "annual_broadleaf", "perennial", "volunteer"};

} // namespace

const char* weed_kind_name(int kind) { return kind >= 0 && kind < kWeedKinds ? kKindNames[kind] : "?"; }

int weed_kind_from_name(const std::string& name) {
    for (int i = 0; i < kWeedKinds; ++i)
        if (name == kKindNames[i]) return i;
    return -1;
}

std::string soil_management_validation_error(const std::string& name, const SoilManagementDesc& m) {
    Check c{"soil " + name + ": management.", {}};
    c.pos("layer_a_m", m.layer_a_m);
    c.pos("layer_b_m", m.layer_b_m);
    c.fail_if(fin(m.layer_a_m) && fin(m.layer_b_m) && !(m.layer_a_m + m.layer_b_m <= 0.5), "layers must not total more than 0.5 m");
    c.frac("workable_wet_full", m.workable_wet_full);
    c.fail_if(!(fin(m.workable_wet_zero) && m.workable_wet_zero > m.workable_wet_full && m.workable_wet_zero <= 1.5),
              "workable_wet_zero must exceed workable_wet_full");
    c.frac("workable_dry_zero", m.workable_dry_zero);
    c.fail_if(!(fin(m.workable_dry_full) && m.workable_dry_full > m.workable_dry_zero && m.workable_dry_full <= 1.0),
              "workable_dry_full must exceed workable_dry_zero");
    c.frac("damage_gain", m.damage_gain);
    c.frac("damage_recovery_per_day", m.damage_recovery_per_day);
    c.nonneg("damage_cn_increase", m.damage_cn_increase);
    c.frac("damage_root_penalty", m.damage_root_penalty);
    c.nonneg("loose_cn_decrease", m.loose_cn_decrease);
    c.frac("loose_stage2_evap_reduction", m.loose_stage2_evap_reduction);
    c.frac("loose_bd_reduction", m.loose_bd_reduction);
    c.nonneg("settle_per_mm_rain", m.settle_per_mm_rain);
    c.frac("settle_per_day", m.settle_per_day);
    c.nonneg("tillage_dry_out_mm_per_m", m.tillage_dry_out_mm_per_m);
    c.frac("seedbed_rest", m.seedbed_rest);
    c.frac("seedbed_wet_floor", m.seedbed_wet_floor);
    c.nonneg("seedbed_settle_per_mm", m.seedbed_settle_per_mm);
    c.frac("seedbed_settle_per_day", m.seedbed_settle_per_day);
    c.nonneg("cover_per_t_ha", m.cover_per_t_ha);
    c.frac("mulch_evap_max_reduction", m.mulch_evap_max_reduction);
    c.fail_if(!(fin(m.res_c_fraction) && m.res_c_fraction > 0.2 && m.res_c_fraction < 0.6), "res_c_fraction must be in (0.2, 0.6)");
    c.frac("res_k20_per_day", m.res_k20_per_day);
    c.frac("res_surface_factor", m.res_surface_factor);
    c.frac("res_stubble_factor", m.res_stubble_factor);
    c.frac("res_moisture_floor", m.res_moisture_floor);
    c.nonneg("res_chop_gain", m.res_chop_gain);
    c.fail_if(!(fin(m.res_assim_eff) && m.res_assim_eff > 0.0 && m.res_assim_eff < 1.0), "res_assim_eff must be in (0, 1)");
    c.pos("res_cn_bio", m.res_cn_bio);
    c.nonneg("res_n_floor_kg_ha", m.res_n_floor_kg_ha);
    return c.err;
}

std::string crop_management_validation_error(const std::string& name, const CropManagementDesc& m) {
    Check c{"crop " + name + ": management.", {}};
    c.frac("root_dm_fraction", m.root_dm_fraction);
    c.frac("root_upper_share", m.root_upper_share);
    c.pos("seed_mass_g", m.seed_mass_g);
    c.fail_if(!(fin(m.seed_viability) && m.seed_viability > 0.0 && m.seed_viability <= 1.0), "seed_viability must be in (0, 1]");
    c.fail_if(!(fin(m.depth_opt_min_m) && fin(m.depth_opt_max_m) && fin(m.depth_zero_m) && m.depth_opt_min_m > 0.0 &&
                m.depth_opt_min_m <= m.depth_opt_max_m && m.depth_opt_max_m < m.depth_zero_m),
              "sowing depths need 0 < depth_opt_min_m <= depth_opt_max_m < depth_zero_m");
    c.frac("depth_surface_factor", m.depth_surface_factor);
    c.frac("seedbed_floor", m.seedbed_floor);
    c.frac("residue_emergence_penalty", m.residue_emergence_penalty);
    c.fail_if(!(fin(m.germ_wet_min) && fin(m.germ_wet_full) && m.germ_wet_min >= 0.0 && m.germ_wet_min < m.germ_wet_full && m.germ_wet_full <= 1.0),
              "germ_wet_min must be below germ_wet_full");
    c.nonneg("dry_hazard_per_day", m.dry_hazard_per_day);
    c.pos("compensation_density_m2", m.compensation_density_m2);
    return c.err;
}

std::string weed_group_validation_error(const WeedGroupDesc& w) {
    Check c{"weed group " + w.name + ": ", {}};
    c.fail_if(w.name.empty(), "name must not be empty");
    c.fail_if(w.kind < 0 || w.kind >= kWeedKinds, "kind must be one of annual_grass, annual_broadleaf, perennial, volunteer");
    c.fail_if(w.perennial != (w.kind == 2), "perennial must be true exactly for the kind perennial");
    c.nonneg("seeds_init_m2", w.seeds_init_m2);
    c.nonneg("clumps_init_m2", w.clumps_init_m2);
    c.fail_if(w.clumps_init_m2 > 0.0 && !w.perennial, "clumps_init_m2 is only for perennials");
    c.frac("init_surface", w.init_surface);
    c.frac("init_upper", w.init_upper);
    c.frac("init_lower", w.init_lower);
    c.fail_if(fin(w.init_surface + w.init_upper + w.init_lower) && std::fabs(w.init_surface + w.init_upper + w.init_lower - 1.0) > 1.0e-9,
              "init_surface + init_upper + init_lower must be 1");
    c.frac("decay_surface_per_day", w.decay_surface_per_day);
    c.frac("decay_upper_per_day", w.decay_upper_per_day);
    c.frac("decay_lower_per_day", w.decay_lower_per_day);
    c.fail_if(!(fin(w.germ_t_min_c) && fin(w.germ_t_opt_c) && fin(w.germ_t_max_c) && w.germ_t_min_c < w.germ_t_opt_c && w.germ_t_opt_c < w.germ_t_max_c),
              "germination temperatures need germ_t_min_c < germ_t_opt_c < germ_t_max_c");
    c.frac("germ_rate_per_day", w.germ_rate_per_day);
    c.fail_if(!(fin(w.germ_wet_min) && fin(w.germ_wet_full) && w.germ_wet_min >= 0.0 && w.germ_wet_min < w.germ_wet_full && w.germ_wet_full <= 1.0),
              "germ_wet_min must be below germ_wet_full");
    c.frac("emerge_surface", w.emerge_surface);
    c.frac("emerge_upper", w.emerge_upper);
    c.frac("emerge_lower", w.emerge_lower);
    c.pos("seedling_g", w.seedling_g);
    c.fail_if(!fin(w.t_base_c), "t_base_c must be finite");
    c.pos("growth_per_gdd", w.growth_per_gdd);
    c.pos("w_max_g", w.w_max_g);
    c.fail_if(!(fin(w.seedling_g) && fin(w.w_max_g) && w.seedling_g < w.w_max_g), "seedling_g must be below w_max_g");
    c.pos("area_max_g_m2", w.area_max_g_m2);
    c.frac("self_thin_per_day", w.self_thin_per_day);
    c.frac("shade", w.shade);
    c.pos("n_conc_pct", w.n_conc_pct);
    c.fail_if(!(fin(w.c_frac) && w.c_frac > 0.2 && w.c_frac < 0.6), "c_frac must be in (0.2, 0.6)");
    c.fail_if(!(fin(w.cold_kill_onset_c) && fin(w.cold_kill_full_c) && w.cold_kill_full_c < w.cold_kill_onset_c),
              "cold_kill_full_c must be below cold_kill_onset_c");
    c.pos("gdd_flower", w.gdd_flower);
    c.fail_if(!(fin(w.gdd_ripe) && w.gdd_ripe > w.gdd_flower), "gdd_ripe must exceed gdd_flower");
    c.fail_if(!w.perennial && !(fin(w.gdd_death) && w.gdd_death > w.gdd_ripe), "gdd_death must exceed gdd_ripe");
    c.nonneg("seeds_per_g", w.seeds_per_g);
    c.frac("shed_per_day", w.shed_per_day);
    c.fail_if(!fin(w.dormancy_t_c), "dormancy_t_c must be finite");
    c.frac("senesce_per_day", w.senesce_per_day);
    c.nonneg("regrow_g", w.regrow_g);
    c.nonneg("cousens_i_pct", w.cousens_i_pct);
    c.fail_if(!(fin(w.cousens_a_pct) && w.cousens_a_pct >= 0.0 && w.cousens_a_pct <= 100.0), "cousens_a_pct must be in [0, 100]");
    c.nonneg("rel_emerge_decay", w.rel_emerge_decay);
    c.frac("resistance_init", w.resistance_init);
    c.fail_if(!w.volunteer_crop.empty() && w.kind != 3, "volunteer_crop is only for the kind volunteer");
    return c.err;
}

std::string herbicide_validation_error(const HerbicideDesc& h) {
    Check c{"herbicide " + h.name + ": ", {}};
    c.fail_if(h.name.empty(), "name must not be empty");
    c.fail_if(h.moa.empty(), "moa must not be empty");
    c.pos("label_rate", h.label_rate);
    for (int k = 0; k < kWeedKinds; ++k) {
        const HerbicideKindResponse& r = h.kind[k];
        const std::string n = std::string("targets.") + weed_kind_name(k);
        c.nonneg((n + ".ed50_rel").c_str(), r.ed50_rel);
        if (r.ed50_rel > 0.0) {
            c.pos((n + ".slope").c_str(), r.slope);
            c.nonneg((n + ".stage_gdd_full").c_str(), r.stage_gdd_full);
            c.fail_if(!(fin(r.stage_gdd_zero) && r.stage_gdd_zero > r.stage_gdd_full), n + ".stage_gdd_zero must exceed stage_gdd_full");
        }
    }
    c.fail_if(!(fin(h.resistance_ed50_factor) && h.resistance_ed50_factor >= 1.0), "resistance_ed50_factor must be >= 1");
    c.nonneg("residual_half_life_days", h.residual_half_life_days);
    c.frac("residual_germ_kill", h.residual_germ_kill);
    c.pos("damage_dose_exponent", h.damage_dose_exponent);
    c.frac("default_damage_in", h.default_damage_in);
    c.frac("default_damage_out", h.default_damage_out);
    c.nonneg("default_max_bbch", h.default_max_bbch);
    for (const HerbicideCropResponse& r : h.crops) {
        c.fail_if(r.crop.empty(), "crop entries need a crop name");
        c.frac("crops.damage_in", r.damage_in);
        c.frac("crops.damage_out", r.damage_out);
        c.nonneg("crops.max_bbch", r.max_bbch);
    }
    return c.err;
}

std::string operation_validation_error(const std::string& name, const OperationEffect& e) {
    Check c{"operation " + name + ": ", {}};
    c.nonneg("coverage", e.coverage);
    const TillageEffect& t = e.tillage;
    c.fail_if(!(fin(t.depth_m) && t.depth_m >= 0.0 && t.depth_m <= 1.0), "tillage.depth_m must be in [0, 1]");
    c.frac("tillage.mix_surface", t.mix_surface);
    c.frac("tillage.mix_upper", t.mix_upper);
    c.frac("tillage.mix_lower", t.mix_lower);
    c.frac("tillage.inversion", t.inversion);
    c.frac("tillage.residue_chop", t.residue_chop);
    c.frac("tillage.loosening", t.loosening);
    c.frac("tillage.seedbed_fineness", t.seedbed_fineness);
    c.frac("tillage.crop_kill", t.crop_kill);
    for (int k = 0; k < kWeedKinds; ++k) c.frac("tillage.weed_kill", t.weed_kill[k]);
    c.fail_if((t.mix_surface > 0.0 || t.mix_upper > 0.0 || t.mix_lower > 0.0 || t.inversion > 0.0 || t.loosening > 0.0 ||
               t.seedbed_fineness > 0.0 || t.residue_chop > 0.0) && !(t.depth_m > 0.0),
              "tillage mixing, inversion, loosening, seedbed or chop need tillage.depth_m > 0");
    c.frac("harvest.straw_carried", e.harvest.straw_carried);
    c.frac("harvest.straw_chopped", e.harvest.straw_chopped);
    c.frac("harvest.grain_loss", e.harvest.grain_loss);
    c.frac("straw_pickup", e.straw_pickup);
    const SeedPlacement& s = e.seed;
    c.nonneg("seed.seeds_per_m2", s.seeds_per_m2);
    c.nonneg("seed.depth_m", s.depth_m);
    c.fail_if(!(s.quality >= 0.0 && s.quality <= 1.0), "seed.quality must be in [0, 1]");
    c.frac("seed.residue_tolerance", s.residue_tolerance);
    c.fail_if(s.crop == 0 && s.seeds_per_m2 > 0.0, "seed.seeds_per_m2 needs a seed.crop");
    c.nonneg("spray.dose", e.spray.dose);
    return c.err;
}

} // namespace ps::field
