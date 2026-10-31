// core/src/field/io/field_io.cpp - see ps/field/field_io.h.

#include "ps/field/field_io.h"

#include "ps/environment/json_extends.h"
#include "../../environment/io/object_reader.h"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace ps::io {

using nlohmann::json;
using namespace ps::field;
using ps::io::detail::ObjectReader;

namespace {

constexpr const char* kSoilFormat = "physics_sim.soil/1";
constexpr const char* kCropFormat = "physics_sim.crop/1";

bool has_tag(const json& v) {
    if (!v.is_string()) return false;
    const std::string s = v.get<std::string>();
    for (const char* t : {"[C]", "[D]", "[M]", "[AS]"})
        if (s.rfind(t, 0) == 0) return true;
    return false;
}

// Every non-string member of the block needs a tagged entry in block["source"].
void require_tags(const json& block, const std::string& ctx, const std::string& origin) {
    const json* src = block.contains("source") ? &block["source"] : nullptr;
    for (auto it = block.begin(); it != block.end(); ++it) {
        if (it.key() == "source" || it.value().is_string()) continue;
        if (!src || !src->is_object() || !src->contains(it.key()) || !has_tag((*src)[it.key()]))
            throw std::runtime_error(origin + ": " + ctx + ": parameter \"" + it.key() +
                                     "\" has no source tag (its entry in \"source\" must start with [C], [D], [M] or [AS])");
    }
}

const json& block_of(ObjectReader& root, const char* key, const std::string& origin) {
    const json& b = root.object(key);
    require_tags(b, key, origin);
    return b;
}

void check_format(ObjectReader& r, const char* want) {
    const std::string f = r.string("format", true, "");
    if (f != want) r.fail(std::string("\"format\" must be \"") + want + "\" (got \"" + f + "\")");
}

int find_stage(const CropDesc& d, const std::string& name) {
    for (std::size_t i = 0; i < d.stages.size(); ++i)
        if (d.stages[i].name == name) return static_cast<int>(i);
    return -1;
}

} // namespace

SoilDesc parse_soil_json(const json& j, const std::string& origin) {
    ObjectReader r(j, "", origin);
    check_format(r, kSoilFormat);
    r.skip("extends");
    r.skip("source");
    SoilDesc s;
    s.name = r.string("name", true, "");
    s.texture = r.string("texture", false, "");
    {
        const json& b = block_of(r, "water", origin);
        ObjectReader w(b, "water", origin);
        s.theta_fc = w.number("theta_fc", true, 0.0);
        s.theta_wp = w.number("theta_wp", true, 0.0);
        s.depth_m = w.number("depth_m", true, 0.0);
        s.fallow_root_depth_m = w.number("fallow_root_depth_m", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        const json& b = block_of(r, "evaporation", origin);
        ObjectReader w(b, "evaporation", origin);
        s.evaporation_depth_m = w.number("depth_m", true, 0.0);
        s.rew_mm = w.number("rew_mm", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        const json& b = block_of(r, "runoff", origin);
        ObjectReader w(b, "runoff", origin);
        s.curve_number_ii = w.number("curve_number_ii", true, 0.0);
        s.cn_normal_wetness = w.number("cn_normal_wetness", true, 0.0);
        s.ia_ratio = w.number("ia_ratio", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        const json& b = block_of(r, "snow", origin);
        ObjectReader w(b, "snow", origin);
        s.snow_melt_mm_per_k_day = w.number("melt_mm_per_k_day", true, 0.0);
        s.snow_melt_base_c = w.number("melt_base_c", true, 0.0);
        w.skip("source");
        w.finish();
    }
    r.finish();
    const std::string e = soil_validation_error(s);
    if (!e.empty()) throw std::runtime_error(origin + ": " + e);
    return s;
}

SoilDesc load_soil_json(const std::filesystem::path& path) {
    return parse_soil_json(load_json_with_extends(path), path.string());
}

CropDesc parse_crop_json(const json& j, const std::string& origin) {
    ObjectReader r(j, "", origin);
    check_format(r, kCropFormat);
    r.skip("extends");
    r.skip("source");
    CropDesc d;
    d.name = r.string("name", true, "");
    d.family = r.string("family", true, "");
    {
        ObjectReader w(block_of(r, "thermal", origin), "thermal", origin);
        d.t_base_c = w.number("t_base_c", true, 0.0);
        d.t_upper_c = w.number("t_upper_c", true, 0.0);
        w.skip("source");
        w.finish();
    }
    std::string vern_until;
    {
        const json& b = block_of(r, "phenology", origin);
        ObjectReader w(b, "phenology", origin);
        const json* arr = w.child("stages");
        if (!arr || !arr->is_array() || arr->empty()) w.fail("\"stages\" must be a non-empty array");
        for (const json& e : *arr) {
            ObjectReader sr(e, "phenology.stages[]", origin);
            CropStageDesc st;
            st.name = sr.string("name", true, "");
            st.bbch = sr.number("bbch", true, 0.0);
            st.gdd = sr.number("gdd", true, 0.0);
            sr.finish();
            d.stages.push_back(st);
        }
        vern_until = w.string("vernalisation_until", false, "");
        w.skip("source");
        w.finish();
    }
    d.stage_emergence = find_stage(d, "emergence");
    d.stage_flowering = find_stage(d, "flowering");
    d.stage_senescence = find_stage(d, "senescence");
    d.stage_maturity = find_stage(d, "maturity");
    if (r.has("vernalisation")) {
        const json& b = block_of(r, "vernalisation", origin);
        ObjectReader w(b, "vernalisation", origin);
        d.vernalises = true;
        d.vern_t_min_c = w.number("t_min_c", true, 0.0);
        d.vern_t_opt_c = w.number("t_opt_c", true, 0.0);
        d.vern_t_max_c = w.number("t_max_c", true, 0.0);
        d.vern_days_saturation = w.number("days_saturation", true, 0.0);
        d.vern_min_rate_factor = w.number("min_rate_factor", true, 0.0);
        w.skip("source");
        w.finish();
        if (vern_until.empty()) r.fail("phenology: \"vernalisation_until\" (a stage name) is required with a vernalisation block");
        d.stage_vernalisation_end = find_stage(d, vern_until);
        if (d.stage_vernalisation_end < 0) r.fail("phenology: vernalisation_until names no stage: " + vern_until);
    } else {
        r.skip("vernalisation");
        if (!vern_until.empty()) r.fail("phenology: \"vernalisation_until\" without a vernalisation block");
    }
    {
        ObjectReader w(block_of(r, "canopy", origin), "canopy", origin);
        d.cc0_cm2_per_plant = w.number("cc0_cm2_per_plant", true, 0.0);
        d.plant_density_per_ha = w.number("plant_density_per_ha", true, 0.0);
        d.ccx = w.number("ccx", true, 0.0);
        d.cgc_per_gdd = w.number("cgc_per_gdd", true, 0.0);
        d.cdc_per_gdd = w.number("cdc_per_gdd", true, 0.0);
        d.senescence_acceleration = w.number("senescence_acceleration", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        const json& b = block_of(r, "water", origin);
        ObjectReader w(b, "water", origin);
        d.kc_tr_x = w.number("kc_tr_x", true, 0.0);
        d.height_max_m = w.number("height_max_m", true, 0.0);
        d.p_depletion = w.number("p_depletion", true, 0.0);
        const json* adj = w.child("p_adjust_for_etc");
        if (!adj || !adj->is_boolean()) w.fail("\"p_adjust_for_etc\" must be a boolean");
        d.p_adjust_for_etc = adj->get<bool>();
        d.exp_p_upper = w.number("exp_p_upper", true, 0.0);
        d.exp_p_lower = w.number("exp_p_lower", true, 0.0);
        d.exp_shape = w.number("exp_shape", true, 0.0);
        d.sen_p_upper = w.number("sen_p_upper", true, 0.0);
        d.sen_shape = w.number("sen_shape", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        ObjectReader w(block_of(r, "biomass", origin), "biomass", origin);
        d.wp_star_g_m2 = w.number("wp_star_g_m2", true, 0.0);
        d.biomass_gdd_full = w.number("gdd_full", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        ObjectReader w(block_of(r, "yield", origin), "yield", origin);
        d.hi0 = w.number("hi0", true, 0.0);
        d.hi_build_gdd = w.number("hi_build_gdd", true, 0.0);
        d.flowering_length_gdd = w.number("flowering_length_gdd", true, 0.0);
        d.excess_fruits_pct = w.number("excess_fruits_pct", true, 0.0);
        d.ppol = w.number("ppol", true, 0.0);
        d.hi_sto_b = w.number("sto_coefficient_b", true, 0.0);
        d.pol_cold_t_c = w.number("cold_t_c", true, 0.0);
        d.pol_cold_width_k = w.number("cold_width_k", true, 0.0);
        d.pol_heat_t_c = w.number("heat_t_c", true, 0.0);
        d.pol_heat_width_k = w.number("heat_width_k", true, 0.0);
        d.pol_logistic_steepness = w.number("logistic_steepness", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        ObjectReader w(block_of(r, "roots", origin), "roots", origin);
        d.zr_min_m = w.number("zr_min_m", true, 0.0);
        d.zr_max_m = w.number("zr_max_m", true, 0.0);
        d.root_gdd_full = w.number("gdd_full", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        ObjectReader w(block_of(r, "frost", origin), "frost", origin);
        d.frost_kill_onset_c = w.number("kill_onset_c", true, 0.0);
        d.frost_kill_full_c = w.number("kill_full_c", true, 0.0);
        d.snow_insulation_swe_mm = w.number("snow_insulation_swe_mm", true, 0.0);
        d.frost_snow_floor_c = w.number("snow_floor_c", true, 0.0);
        w.skip("source");
        w.finish();
    }
    {
        ObjectReader w(block_of(r, "ripeness", origin), "ripeness", origin);
        d.harvest_window_days = w.number("harvest_window_days", true, 0.0);
        d.shed_rate_per_day = w.number("shed_rate_per_day", true, 0.0);
        w.skip("source");
        w.finish();
    }
    r.finish();
    const std::string e = crop_validation_error(d);
    if (!e.empty()) throw std::runtime_error(origin + ": " + e);
    return d;
}

CropDesc load_crop_json(const std::filesystem::path& path) {
    return parse_crop_json(load_json_with_extends(path), path.string());
}

} // namespace ps::io
