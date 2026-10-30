// core/src/field/io/field_scenario.cpp - see ps/field/field_scenario.h.

#include "ps/field/field_scenario.h"

#include "ps/environment/climate_io.h"
#include "ps/environment/json_extends.h"
#include "ps/field/field_io.h"
#include "../../environment/io/object_reader.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string>

namespace ps::io {

using nlohmann::json;
using ps::io::detail::ObjectReader;

FieldScenario load_field_scenario(const std::filesystem::path& path) {
    const std::string origin = path.string();
    const json root = load_json_with_extends(path);
    const std::filesystem::path dir = path.parent_path();
    ObjectReader r(root, "", origin);
    if (r.string("format", true, "") != "physics_sim.field_scenario/1")
        r.fail("\"format\" must be \"physics_sim.field_scenario/1\"");
    r.skip("extends");
    r.skip("source");
    FieldScenario sc;
    sc.climate = load_climate_json(dir / r.string("climate", true, ""));
    sc.seed = static_cast<std::uint64_t>(r.integer("seed", false, 0));
    sc.years = r.integer("years", true, 1);
    if (sc.years < 1) r.fail("\"years\" must be >= 1");
    if (r.has("weather")) {
        ObjectReader w(r.object("weather"), "weather", origin);
        sc.weather.samples_per_day = w.integer("samples_per_day", false, sc.weather.samples_per_day);
        sc.weather.wind_height_m = w.number("wind_height_m", false, sc.weather.wind_height_m);
        sc.weather.sleet_snow_fraction = w.number("sleet_snow_fraction", false, sc.weather.sleet_snow_fraction);
        w.finish();
    } else {
        r.skip("weather");
    }
    const json* soils = r.child("soils");
    if (!soils || !soils->is_array() || soils->empty()) r.fail("\"soils\" must be a non-empty array of file names");
    for (const json& e : *soils) {
        if (!e.is_string()) r.fail("\"soils\" entries must be strings");
        sc.field.soils.push_back(load_soil_json(dir / e.get<std::string>()));
    }
    const json* crops = r.child("crops");
    if (!crops || !crops->is_array()) r.fail("\"crops\" must be an array of file names");
    for (const json& e : *crops) {
        if (!e.is_string()) r.fail("\"crops\" entries must be strings");
        sc.field.crops.push_back(load_crop_json(dir / e.get<std::string>()));
    }
    {
        ObjectReader g(r.object("grid"), "grid", origin);
        sc.field.nx = g.integer("nx", true, 1);
        sc.field.ny = g.integer("ny", true, 1);
        sc.field.cell_size_m = g.number("cell_size_m", false, 4.0);
        sc.field.origin_x_m = g.number("origin_x_m", false, 0.0);
        sc.field.origin_y_m = g.number("origin_y_m", false, 0.0);
        g.finish();
    }
    sc.field.initial_depletion_fraction = r.number("initial_depletion_fraction", false, 0.0);
    sc.field.initial_day = -1;
    if (r.has("soil_pattern")) {
        // soil index per column, repeated across the grid
        const json* p = r.child("soil_pattern");
        if (!p->is_array() || p->empty()) r.fail("\"soil_pattern\" must be a non-empty array of soil indices");
        std::vector<int> pat;
        for (const json& e : *p) {
            if (!e.is_number_integer() || e.get<int>() < 0 || static_cast<std::size_t>(e.get<int>()) >= sc.field.soils.size())
                r.fail("\"soil_pattern\" entries must be indices into \"soils\"");
            pat.push_back(e.get<int>());
        }
        sc.field.cell_soil.resize(static_cast<std::size_t>(sc.field.nx) * static_cast<std::size_t>(sc.field.ny));
        for (int iy = 0; iy < sc.field.ny; ++iy)
            for (int ix = 0; ix < sc.field.nx; ++ix)
                sc.field.cell_soil[static_cast<std::size_t>(iy * sc.field.nx + ix)] =
                    static_cast<std::uint16_t>(pat[static_cast<std::size_t>(ix) % pat.size()]);
    } else {
        r.skip("soil_pattern");
    }
    if (r.has("test_irrigate_to_capacity")) {
        const json* b = r.child("test_irrigate_to_capacity");
        if (!b->is_boolean()) r.fail("\"test_irrigate_to_capacity\" must be a boolean");
        sc.irrigate_to_capacity = b->get<bool>();
    } else {
        r.skip("test_irrigate_to_capacity");
    }
    if (const json* ops = r.child("operations")) {
        if (!ops->is_array()) r.fail("\"operations\" must be an array");
        for (const json& e : *ops) {
            ObjectReader o(e, "operations[]", origin);
            FieldOperationRule rule;
            rule.op = o.string("op", true, "");
            if (rule.op != "sow" && rule.op != "harvest") o.fail("\"op\" must be \"sow\" or \"harvest\"");
            rule.doy = o.integer("doy", true, 1);
            if (rule.doy < 1 || rule.doy > 366) o.fail("\"doy\" must be in [1, 366]");
            rule.first_year = o.integer("first_year", false, 0);
            rule.last_year = o.integer("last_year", false, 1 << 30);
            rule.every_years = o.integer("every_years", false, 1);
            if (rule.every_years < 1) o.fail("\"every_years\" must be >= 1");
            rule.density_scale = o.number("density_scale", false, 1.0);
            const std::string crop = o.string("crop", rule.op == "sow", "");
            if (rule.op == "sow") {
                for (std::size_t i = 0; i < sc.field.crops.size(); ++i)
                    if (sc.field.crops[i].name == crop) rule.crop_id = static_cast<int>(i) + 1;
                if (rule.crop_id == 0) o.fail("\"crop\" names no crop of the scenario: " + crop);
            }
            o.finish();
            sc.rules.push_back(rule);
        }
    }
    r.finish();
    return sc;
}

FieldRunSummary run_field_scenario(const FieldScenario& sc, std::ostream* csv) {
    field::DailyWeatherSource src(sc.climate, sc.seed, sc.weather);
    field::Field f(sc.field);
    f.set_irrigate_to_capacity(sc.irrigate_to_capacity);
    FieldRunSummary sum;
    const std::int64_t n_days = static_cast<std::int64_t>(sc.years) * src.days_per_year();
    if (csv)
        *csv << "day,year,doy,tmin_c,tmax_c,rain_mm,snow_mm,rs_mj,et0_mm,rh_mean,wind2_m_s,swe_mm,depletion_root_mm,taw_root_mm,zr_m,"
                "crop,stage,bbch,gdd_sow,cc,biomass_t_ha,grain_t_ha,tr_mm,evap_mm,runoff_mm,drain_mm,ks_sto\n";
    for (std::int64_t d = 0; d < n_days; ++d) {
        const field::DailyWeather& w = src.record(d);
        const int year = static_cast<int>(d / src.days_per_year());
        for (const FieldOperationRule& rule : sc.rules) {
            if (rule.doy != w.doy || year < rule.first_year || year > rule.last_year ||
                (year - rule.first_year) % rule.every_years != 0)
                continue;
            if (rule.op == "sow") {
                f.sow_all(rule.crop_id, rule.density_scale);
            } else {
                const field::HarvestResult h = f.harvest_all();
                if (h.crop_id != 0) sum.harvests.push_back({d, year, w.doy, h});
            }
        }
        f.step_day(w);
        if (csv) {
            double depl = 0, taw = 0, swe = 0, zr = 0, bbch = 0, gdd = 0, cc = 0, bio = 0, grain = 0, tr = 0, ev = 0, ro = 0, dr = 0, ks = 0;
            const int n = f.cell_count();
            for (int c = 0; c < n; ++c) {
                const field::CellView v = f.view(c);
                depl += v.d_root_mm; taw += v.taw_root_mm; swe += v.swe_mm; zr += v.zr_m; bbch += v.bbch;
                gdd += v.gdd_sow; cc += v.cc; bio += v.biomass_g_m2 * 0.01; grain += v.grain_g_m2 * 0.01;
                tr += v.last_tr_mm; ev += v.last_e_mm; ro += v.last_runoff_mm; dr += v.last_drain_mm; ks += v.last_ks_sto;
            }
            const field::CellView v0 = f.view(0);
            char buf[640];
            std::snprintf(buf, sizeof buf,
                          "%lld,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                          static_cast<long long>(d), year, w.doy, w.tmin_c, w.tmax_c, w.rain_mm, w.snow_mm, w.rs_mj_m2,
                          w.et0_mm, w.rh_mean, w.wind2_m_s, swe / n, depl / n, taw / n, zr / n, v0.crop_id, v0.stage, bbch / n, gdd / n, cc / n,
                          bio / n, grain / n, tr / n, ev / n, ro / n, dr / n, ks / n);
            *csv << buf;
        }
    }
    for (int c = 0; c < f.cell_count(); ++c) {
        const double res = f.water_balance_residual_mm(c);
        sum.max_balance_residual_mm = std::max(sum.max_balance_residual_mm, res < 0 ? -res : res);
    }
    sum.total_precip_mm = f.ledger(0).precipitation;
    sum.state_hash = f.fold_hash(1469598103934665603ull);
    return sum;
}

} // namespace ps::io
