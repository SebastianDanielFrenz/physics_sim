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

} // namespace ps::test_field
