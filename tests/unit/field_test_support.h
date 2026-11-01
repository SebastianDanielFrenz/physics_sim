// tests/unit/field_test_support.h - shared helpers of the [field] tests
// (header-only, no ps_core dependency): the shipped data, synthetic weather
// records and the season runners the validation tests share.
#pragma once

#include "ps/environment/climate_io.h"
#include "ps/field/field.h"
#include "ps/field/field_io.h"
#include "ps/field/weather_record.h"

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace ps::test_field {

inline std::filesystem::path data_dir() { return std::filesystem::path(PS_SOURCE_DIR) / "data"; }

inline const ps::environment::ClimateDesc& farm_climate() {
    static const ps::environment::ClimateDesc c = ps::io::load_climate_json(data_dir() / "climates/farm_de_home.json");
    return c;
}
inline const field::SoilDesc& loess() {
    static const field::SoilDesc s = ps::io::load_soil_json(data_dir() / "soils/silt_loam_loess.json");
    return s;
}
inline const field::SoilDesc& sandy() {
    static const field::SoilDesc s = ps::io::load_soil_json(data_dir() / "soils/sandy_loam.json");
    return s;
}
inline const field::CropDesc& wheat() {
    static const field::CropDesc c = ps::io::load_crop_json(data_dir() / "crops/winter_wheat.json");
    return c;
}

// --- nutrients (F1b) -------------------------------------------------------------------------------
// The shipped palette, in this order: Deposit::material = index + 1.
inline const std::vector<std::string>& material_names() {
    static const std::vector<std::string> n = {"can_27",          "dap_18_46",           "mop_60",         "uan_28",
                                               "cattle_slurry_75", "pig_slurry_5",        "cattle_solid_manure",
                                               "digestate_liquid_75", "lime_carbonate_90", "quicklime_90"};
    return n;
}
inline const std::vector<field::MaterialDesc>& materials() {
    static const std::vector<field::MaterialDesc> m = [] {
        std::vector<field::MaterialDesc> v;
        for (const std::string& n : material_names()) v.push_back(ps::io::load_material_json(data_dir() / "field_materials" / (n + ".json")));
        return v;
    }();
    return m;
}
inline const std::vector<field::MethodDesc>& methods() {
    static const std::vector<field::MethodDesc> m = ps::io::load_methods_json(data_dir() / "field_materials/methods.json");
    return m;
}
inline int mat_id(const std::string& name) {
    const std::vector<std::string>& n = material_names();
    for (std::size_t i = 0; i < n.size(); ++i)
        if (n[i] == name) return static_cast<int>(i) + 1;
    return 0;
}
inline int method_id(const std::string& name) {
    const std::vector<field::MethodDesc>& m = methods();
    for (std::size_t i = 0; i < m.size(); ++i)
        if (m[i].name == name) return static_cast<int>(i) + 1;
    return 0;
}
inline field::Deposit deposit(const std::string& material, const std::string& method, double rate_t_ha, double delay_h = -2.0) {
    field::Deposit d;
    d.material = mat_id(material);
    d.method = method_id(method);
    d.rate_t_ha = rate_t_ha;
    d.incorporation_delay_h = delay_h;
    return d;
}
// Applies `kg_n` of nitrogen as material `name`.
inline field::OperationEffect n_dose(const std::string& name, const std::string& method, double kg_n, double coverage = 1.0) {
    field::OperationEffect e;
    e.coverage = coverage;
    e.deposit = deposit(name, method, kg_n / materials()[static_cast<std::size_t>(mat_id(name) - 1)].n_total_kg_t);
    return e;
}

inline field::FieldConfig one_cell_config(const field::SoilDesc& soil, const field::CropDesc& crop, int nx = 1, int ny = 1,
                                          double depletion = 0.0) {
    field::FieldConfig cfg;
    cfg.nx = nx;
    cfg.ny = ny;
    cfg.soils.push_back(soil);
    cfg.crops.push_back(crop);
    cfg.initial_depletion_fraction = depletion;
    return cfg;
}

inline field::FieldConfig nutrient_config(const field::SoilDesc& soil, const field::CropDesc& crop, int nx = 1, int ny = 1) {
    field::FieldConfig cfg = one_cell_config(soil, crop, nx, ny, 0.2);
    cfg.nutrients_enabled = true;
    cfg.materials = materials();
    cfg.methods = methods();
    return cfg;
}

// A synthetic record: constant weather, no randomness.
inline field::DailyWeather synth(std::int64_t day, double tmin, double tmax, double rain = 0.0, double snow = 0.0,
                                 double et0 = 3.0, double wind2 = 2.0, double rh_min = 0.45) {
    field::DailyWeather w;
    w.day = day;
    w.doy = static_cast<int>(day % 365) + 1;
    w.tmin_c = tmin;
    w.tmax_c = tmax;
    w.tmean_c = 0.5 * (tmin + tmax);
    w.rain_mm = rain;
    w.snow_mm = snow;
    w.et0_mm = et0;
    w.wind2_m_s = wind2;
    w.rh_min = rh_min;
    w.rh_max = 0.9;
    w.rh_mean = 0.7;
    w.rs_mj_m2 = 15.0;
    w.ea_kpa = 1.0;
    w.daylight_h = 12.0;
    return w;
}

// Steps `n` synthetic days starting after the field's last day.
inline void run_synth(field::Field& f, int n, double tmin, double tmax, double rain = 0.0, double et0 = 3.0) {
    for (int i = 0; i < n; ++i) f.step_day(synth(f.last_day() + 1, tmin, tmax, rain, 0.0, et0));
}

// The 30-year record of the shipped farm climate (seed 21), computed once per
// test process. Not thread safe; the tests that use it run on one thread.
inline field::DailyWeatherSource& weather30() {
    static field::DailyWeatherSource src(farm_climate(), 21);
    static const bool filled = (src.prefetch(0, 30 * 365 - 1), true);
    (void)filled;
    return src;
}

// First day >= `from` whose record stands for day-of-year `doy`.
inline std::int64_t find_doy(field::DailyWeatherSource& src, std::int64_t from, int doy) {
    for (std::int64_t d = from; d < from + 400; ++d)
        if (src.record(d).doy == doy) return d;
    return -1;
}

struct SeasonResult {
    double grain_dm_t_ha = 0.0;
    double biomass_g_m2 = 0.0;
    double et_mm = 0.0;   // transpiration + soil evaporation over the season
    double tr_mm = 0.0;
    double precip_mm = 0.0;
    double residual_mm = 0.0;
};

// One winter-wheat season of a one-cell field: drilled on doy 288 of `sow_year`,
// harvested on doy 212 of the next year; rain and snow scaled by `rain_scale`;
// `irrigate` brings the profile to field capacity every morning (the
// "no water stress" reference of the Ky test).
inline SeasonResult run_season(const field::SoilDesc& soil, const field::CropDesc& crop, int sow_year, double rain_scale,
                               bool irrigate, double initial_depletion = 0.0) {
    field::DailyWeatherSource& src = weather30();
    const std::int64_t sow_day = find_doy(src, static_cast<std::int64_t>(sow_year) * 365, 288);
    const std::int64_t harvest_day = find_doy(src, sow_day + 1, 212);
    field::FieldConfig cfg = one_cell_config(soil, crop, 1, 1, initial_depletion);
    cfg.initial_day = sow_day - 1;
    field::Field f(cfg);
    f.set_irrigate_to_capacity(irrigate);
    f.sow_all(1);
    for (std::int64_t d = sow_day; d < harvest_day; ++d) {
        field::DailyWeather w = src.record(d);
        w.rain_mm *= rain_scale;
        w.snow_mm *= rain_scale;
        f.step_day(w);
    }
    SeasonResult r;
    const field::WaterLedger l = f.ledger(0);
    r.et_mm = l.evaporation + l.transpiration;
    r.tr_mm = l.transpiration;
    r.precip_mm = l.precipitation;
    r.residual_mm = f.water_balance_residual_mm(0);
    const field::CellView v = f.view(0);
    r.biomass_g_m2 = v.biomass_g_m2;
    const field::HarvestResult h = f.harvest(0);
    r.grain_dm_t_ha = h.grain_dm_t_ha;
    return r;
}


// A yearly fertiliser plan on the winter-wheat calendar of run_season: lime (doy 250), DAP + MOP (262), drilling
// (288), cattle slurry (62) and CAN splits (n_doy) from year 1 on, harvest (212) from year 1 on.
struct Plan {
    double n_kg = 0.0;
    std::vector<double> n_split{0.3, 0.4, 0.3};
    std::vector<int> n_doy{60, 95, 130};
    double p2o5_kg = 0.0, k2o_kg = 0.0;
    int lime_every = 0;
    double lime_t = 0.0;
    double straw_removed = 1.0;
    double slurry_m3 = 0.0;
    int no_harvest_year = -1;   // that year's crop stays on the field until the next drilling
};

struct YearResult {
    int year = 0;
    double grain_fresh_t_ha = 0.0;   // 86 % dry matter
    double n_removed = 0.0, p_removed = 0.0, k_removed = 0.0;   // grain + the removed share of the straw, kg/ha
    double n_grain = 0.0, p_grain = 0.0, k_grain = 0.0;
    double lodging = 0.0;
};

inline void apply_plan_day(field::Field& f, const field::DailyWeather& w, const Plan& p, std::vector<YearResult>* out) {
    const int year = static_cast<int>(w.day / 365);
    if (w.doy == 250 && p.lime_every > 0 && year % p.lime_every == 0 && p.lime_t > 0.0)
        f.apply_all({1.0, deposit("lime_carbonate_90", "incorporated", p.lime_t)});
    if (w.doy == 262) {
        if (p.p2o5_kg > 0.0) f.apply_all({1.0, deposit("dap_18_46", "broadcast", p.p2o5_kg / 460.0)});
        if (p.k2o_kg > 0.0) f.apply_all({1.0, deposit("mop_60", "broadcast", p.k2o_kg / 600.0)});
    }
    if (w.doy == 288) f.sow_all(1);
    if (year >= 1) {
        if (w.doy == 62 && p.slurry_m3 > 0.0)
            f.apply_all({1.0, deposit("cattle_slurry_75", "trailing_hose", p.slurry_m3 * 1.03)});
        for (std::size_t i = 0; i < p.n_doy.size(); ++i)
            if (w.doy == p.n_doy[i] && p.n_kg > 0.0) f.apply_all(n_dose("can_27", "broadcast", p.n_kg * p.n_split[i]));
        if (w.doy == 212 && year != p.no_harvest_year) {
            const field::HarvestResult h = f.harvest_all(p.straw_removed);
            if (out && h.crop_id != 0) {
                YearResult r;
                r.year = year;
                r.grain_fresh_t_ha = h.grain_dm_t_ha / 0.86;
                r.n_grain = h.n_grain_kg_ha;
                r.p_grain = h.p2o5_grain_kg_ha;
                r.k_grain = h.k2o_grain_kg_ha;
                r.n_removed = h.n_grain_kg_ha + p.straw_removed * h.n_straw_kg_ha;
                r.p_removed = h.p2o5_grain_kg_ha + p.straw_removed * h.p2o5_straw_kg_ha;
                r.k_removed = h.k2o_grain_kg_ha + p.straw_removed * h.k2o_straw_kg_ha;
                r.lodging = h.lodging_loss;
                out->push_back(r);
            }
        }
    }
}

// Runs `years` game years of the 30-year farm record (seed 21) from a fresh field; returns the harvests.
inline std::vector<YearResult> run_plan(field::Field& f, const Plan& p, int years) {
    field::DailyWeatherSource& src = weather30();
    std::vector<YearResult> out;
    for (std::int64_t d = f.last_day() + 1; d < static_cast<std::int64_t>(years) * 365; ++d) {
        const field::DailyWeather& w = src.record(d);
        apply_plan_day(f, w, p, &out);
        f.step_day(w);
    }
    return out;
}

inline double mean_of_range(const std::vector<YearResult>& v, std::size_t from, std::size_t to, double YearResult::*m) {
    double s = 0.0;
    std::size_t n = 0;
    for (std::size_t i = from; i < to && i < v.size(); ++i, ++n) s += v[i].*m;
    return n ? s / static_cast<double>(n) : 0.0;
}


// --- pathogens (F1d part 4) -------------------------------------------------------------------------
// Crop ids of pathogen_config(): 1 winter_wheat, 2 winter_barley, 3 winter_rapeseed, 4 grain_maize, 5 sugar_beet;
// herbicide (fungicide) id 1 = cereal_fungicide. Management and pathogens on, no weeds, deep loess.
inline field::CropDesc crop_named(const std::string& name) { return ps::io::load_crop_json(data_dir() / "crops" / (name + ".json")); }
inline field::FieldConfig pathogen_config(int nx = 1, int ny = 1, double depletion = 0.2) {
    field::FieldConfig cfg = nutrient_config(loess(), wheat(), nx, ny);
    cfg.crops.clear();
    for (const char* n : {"winter_wheat", "winter_barley", "winter_rapeseed", "grain_maize", "sugar_beet"}) cfg.crops.push_back(crop_named(n));
    cfg.herbicides.push_back(ps::io::load_herbicide_json(data_dir() / "herbicides/cereal_fungicide.json"));
    cfg.initial_depletion_fraction = depletion;
    cfg.management_enabled = true;
    cfg.pathogens_enabled = true;
    return cfg;
}
inline field::OperationEffect fungicide_spray(double dose = 1.0) {
    field::OperationEffect e;
    e.spray.product = 1;
    e.spray.dose = dose;
    return e;
}

} // namespace ps::test_field
