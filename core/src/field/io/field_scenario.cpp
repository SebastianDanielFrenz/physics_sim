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
    sc.field.co2_ppm = r.number("co2_ppm", false, sc.field.co2_ppm);
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
    if (r.has("nutrients")) {
        const json* b = r.child("nutrients");
        if (!b->is_boolean()) r.fail("\"nutrients\" must be a boolean");
        sc.field.nutrients_enabled = b->get<bool>();
    } else {
        r.skip("nutrients");
    }
    if (r.has("management")) {
        const json* b = r.child("management");
        if (!b->is_boolean()) r.fail("\"management\" must be a boolean");
        sc.field.management_enabled = b->get<bool>();
        if (sc.field.management_enabled && !sc.field.nutrients_enabled) r.fail("\"management\" needs \"nutrients\": true");
    } else {
        r.skip("management");
    }
    if (r.has("weeds")) {
        const json* a = r.child("weeds");
        if (!a->is_array()) r.fail("\"weeds\" must be an array of file names");
        for (const json& e : *a) {
            if (!e.is_string()) r.fail("\"weeds\" entries must be strings");
            sc.field.weeds.push_back(load_weed_group_json(dir / e.get<std::string>()));
        }
    } else {
        r.skip("weeds");
    }
    if (r.has("herbicides")) {
        const json* a = r.child("herbicides");
        if (!a->is_array()) r.fail("\"herbicides\" must be an array of file names");
        for (const json& e : *a) {
            if (!e.is_string()) r.fail("\"herbicides\" entries must be strings");
            sc.field.herbicides.push_back(load_herbicide_json(dir / e.get<std::string>()));
        }
    } else {
        r.skip("herbicides");
    }
    if (r.has("materials")) {
        const json* mats = r.child("materials");
        if (!mats->is_array()) r.fail("\"materials\" must be an array of file names");
        for (const json& e : *mats) {
            if (!e.is_string()) r.fail("\"materials\" entries must be strings");
            sc.field.materials.push_back(load_material_json(dir / e.get<std::string>()));
        }
    } else {
        r.skip("materials");
    }
    if (r.has("methods")) sc.field.methods = load_methods_json(dir / r.string("methods", true, ""));
    else r.skip("methods");
    if (const json* ops = r.child("operations")) {
        if (!ops->is_array()) r.fail("\"operations\" must be an array");
        for (const json& e : *ops) {
            ObjectReader o(e, "operations[]", origin);
            FieldOperationRule rule;
            rule.op = o.string("op", true, "");
            if (rule.op != "sow" && rule.op != "harvest" && rule.op != "apply" && rule.op != "operation")
                o.fail("\"op\" must be \"sow\", \"harvest\", \"apply\" or \"operation\"");
            rule.doy = o.integer("doy", true, 1);
            if (rule.doy < 1 || rule.doy > 366) o.fail("\"doy\" must be in [1, 366]");
            rule.first_year = o.integer("first_year", false, 0);
            rule.last_year = o.integer("last_year", false, 1 << 30);
            rule.every_years = o.integer("every_years", false, 1);
            if (rule.every_years < 1) o.fail("\"every_years\" must be >= 1");
            rule.density_scale = o.number("density_scale", false, 1.0);
            rule.straw_removed = o.number("straw_removed", false, 1.0);
            if (!(rule.straw_removed >= 0.0 && rule.straw_removed <= 1.0)) o.fail("\"straw_removed\" must be in [0, 1]");
            if (rule.op == "apply") {
                if (!sc.field.nutrients_enabled) o.fail("\"apply\" needs \"nutrients\": true");
                const std::string mat = o.string("material", true, "");
                const std::string met = o.string("method", true, "");
                for (std::size_t i = 0; i < sc.field.materials.size(); ++i)
                    if (sc.field.materials[i].name == mat) rule.effect.deposit.material = static_cast<int>(i) + 1;
                for (std::size_t i = 0; i < sc.field.methods.size(); ++i)
                    if (sc.field.methods[i].name == met) rule.effect.deposit.method = static_cast<int>(i) + 1;
                if (rule.effect.deposit.material == 0) o.fail("\"material\" names no material of the scenario: " + mat);
                if (rule.effect.deposit.method == 0) o.fail("\"method\" names no method of the scenario: " + met);
                const field::MaterialDesc& md = sc.field.materials[static_cast<std::size_t>(rule.effect.deposit.material - 1)];
                const int n_rates = (o.has("rate_t_ha") ? 1 : 0) + (o.has("rate_m3_ha") ? 1 : 0) + (o.has("rate_n_kg_ha") ? 1 : 0);
                if (n_rates != 1) o.fail("exactly one of \"rate_t_ha\", \"rate_m3_ha\", \"rate_n_kg_ha\" is required");
                if (o.has("rate_t_ha")) rule.effect.deposit.rate_t_ha = o.number("rate_t_ha", true, 0.0);
                if (o.has("rate_m3_ha")) rule.effect.deposit.rate_t_ha = o.number("rate_m3_ha", true, 0.0) * md.density_kg_m3 * 1.0e-3;
                if (o.has("rate_n_kg_ha")) {
                    if (!(md.n_total_kg_t > 0.0)) o.fail("\"rate_n_kg_ha\" needs a material with nitrogen");
                    rule.effect.deposit.rate_t_ha = o.number("rate_n_kg_ha", true, 0.0) / md.n_total_kg_t;
                }
                if (!(rule.effect.deposit.rate_t_ha >= 0.0)) o.fail("the rate must be >= 0");
                if (o.has("incorporation_delay_h")) rule.effect.deposit.incorporation_delay_h = o.number("incorporation_delay_h", true, 0.0);
                rule.effect.coverage = o.number("coverage", false, 1.0);
            }
            if (rule.op == "operation") {
                if (!sc.field.management_enabled) o.fail("\"operation\" needs \"management\": true");
                OperationPalette pal;
                for (const auto& c : sc.field.crops) pal.crops.push_back(c.name);
                for (const auto& h : sc.field.herbicides) pal.herbicides.push_back(h.name);
                for (const auto& m : sc.field.materials) pal.materials.push_back(m.name);
                for (const auto& m : sc.field.methods) pal.methods.push_back(m.name);
                const std::string file = o.string("file", true, "");
                rule.effect = load_operation_json(dir / file, pal);
                // optional overrides of the record, e.g. the seed rate or the spray dose of this application
                if (o.has("coverage")) rule.effect.coverage = o.number("coverage", true, 1.0);
                if (o.has("seeds_per_m2")) rule.effect.seed.seeds_per_m2 = o.number("seeds_per_m2", true, 0.0);
                if (o.has("depth_m")) {
                    if (rule.effect.seed.crop != 0) rule.effect.seed.depth_m = o.number("depth_m", true, 0.0);
                    else rule.effect.tillage.depth_m = o.number("depth_m", true, 0.0);
                }
                if (o.has("dose")) rule.effect.spray.dose = o.number("dose", true, 1.0);
                if (o.has("straw_carried")) rule.effect.harvest.straw_carried = o.number("straw_carried", true, 0.0);
                if (o.has("straw_chopped")) rule.effect.harvest.straw_chopped = o.number("straw_chopped", true, 0.0);
                const std::string err = field::operation_validation_error(file, rule.effect);
                if (!err.empty()) o.fail(err);
            } else {
                o.skip("file");
                o.skip("coverage");
                o.skip("seeds_per_m2");
                o.skip("depth_m");
                o.skip("dose");
                o.skip("straw_carried");
                o.skip("straw_chopped");
            }
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
    sum.drainage_by_year_mm.assign(static_cast<std::size_t>(sc.years), 0.0);
    if (csv)
        *csv << "day,year,doy,tmin_c,tmax_c,rain_mm,snow_mm,rs_mj,et0_mm,rh_mean,wind2_m_s,swe_mm,depletion_root_mm,taw_root_mm,zr_m,"
                "crop,stage,bbch,gdd_sow,cc,biomass_t_ha,grain_t_ha,tr_mm,evap_mm,runoff_mm,drain_mm,ks_sto";
    if (csv) *csv << (sc.field.nutrients_enabled ? ",nmin_kg_ha,nh4_kg_ha,tan_surface_kg_ha,ph,p_test,k_test,f_n,f_pk,n_crop_kg_ha,n_leached_cum,nh3_cum,n_mineralised_cum,n_denitrified_cum" : "");
    if (csv && sc.field.management_enabled)
        *csv << ",residue_t_ha,residue_cover,loosening,seedbed,crop_plants_m2,weed_factor,weed_plants_m2,weed_biomass_g_m2,weed_seeds_m2,humus_c_kg_ha,g0_plants,g1_plants,g2_plants,g3_plants,g0_seeds,g1_seeds,g2_seeds,g3_seeds";
    if (csv) *csv << "\n";
    for (std::int64_t d = 0; d < n_days; ++d) {
        const field::DailyWeather& w = src.record(d);
        const int year = static_cast<int>(d / src.days_per_year());
        for (const FieldOperationRule& rule : sc.rules) {
            if (rule.doy != w.doy || year < rule.first_year || year > rule.last_year ||
                (year - rule.first_year) % rule.every_years != 0)
                continue;
            if (rule.op == "sow") {
                f.sow_all(rule.crop_id, rule.density_scale);
            } else if (rule.op == "apply") {
                f.apply_all(rule.effect);
            } else if (rule.op == "operation") {
                const field::HarvestResult h = f.apply_all(rule.effect);
                if (h.crop_id != 0) sum.harvests.push_back({d, year, w.doy, h});
            } else {
                const field::HarvestResult h = f.harvest_all(rule.straw_removed);
                if (h.crop_id != 0) sum.harvests.push_back({d, year, w.doy, h});
            }
        }
        f.step_day(w);
        {
            double dr = 0.0;
            const int n = f.cell_count();
            for (int c = 0; c < n; ++c) dr += f.view(c).last_drain_mm;
            sum.drainage_by_year_mm[static_cast<std::size_t>(year)] += dr / n;
            if (sc.field.management_enabled && !sc.field.weeds.empty()) {
                double wb = 0.0;
                for (int c = 0; c < n; ++c) {
                    const field::ManagementView mv = f.management_view(c);
                    for (const field::WeedGroupView& g : mv.weeds) wb += g.biomass_g_m2;
                }
                sum.weed_biomass_max_g_m2 = std::max(sum.weed_biomass_max_g_m2, wb / n);
            }
        }
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
            std::string row(buf);
            row.pop_back();
            *csv << row;
            if (sc.field.nutrients_enabled) {
                double nmin = 0, nh4 = 0, tan = 0, ph = 0, pt = 0, kt = 0, fn = 0, fpk = 0, nc = 0, le = 0, vo = 0, mi = 0, de = 0;
                for (int c = 0; c < n; ++c) {
                    const field::NutrientView nv = f.nutrient_view(c);
                    const field::NutrientLedger nl = f.nutrient_ledger(c);
                    nmin += nv.nmin; nh4 += nv.nh4; tan += nv.tan_surface; ph += nv.ph; pt += nv.p_test_mg100g;
                    kt += nv.k_test_mg100g; fn += nv.f_n; fpk += nv.f_pk; nc += nv.n_crop; le += nl.n_leached; vo += nl.n_volatilised;
                    mi += nl.n_mineralised; de += nl.n_denitrified;
                }
                char b2[320];
                std::snprintf(b2, sizeof b2, ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f", nmin / n, nh4 / n,
                              tan / n, ph / n, pt / n, kt / n, fn / n, fpk / n, nc / n, le / n, vo / n, mi / n, de / n);
                *csv << b2;
            }
            if (sc.field.management_enabled) {
                double res = 0, cov = 0, loose = 0, sb = 0, pl = 0, wf = 0, wp = 0, wbm = 0, ws = 0, soc = 0;
                double gp[4] = {0, 0, 0, 0}, gs[4] = {0, 0, 0, 0};
                for (int c = 0; c < n; ++c) {
                    const field::ManagementView mv = f.management_view(c);
                    res += mv.straw_t_ha + mv.stubble_t_ha + mv.buried_upper_t_ha + mv.buried_lower_t_ha;
                    cov += mv.cover; loose += mv.loosening; sb += mv.seedbed; pl += mv.plants_m2; wf += mv.weed_factor;
                    soc += mv.soc_gain_c_kg_ha;
                    for (std::size_t gi = 0; gi < mv.weeds.size(); ++gi) {
                        const field::WeedGroupView& g = mv.weeds[gi];
                        wp += g.plants_m2; wbm += g.biomass_g_m2; ws += g.seeds_total;
                        if (gi < 4) { gp[gi] += g.plants_m2; gs[gi] += g.seeds_total; }
                    }
                }
                char b3[512];
                std::snprintf(b3, sizeof b3, ",%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.3f,%.3f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f", res / n, cov / n, loose / n, sb / n,
                              pl / n, wf / n, wp / n, wbm / n, ws / n, soc / n, gp[0] / n, gp[1] / n, gp[2] / n, gp[3] / n, gs[0] / n, gs[1] / n, gs[2] / n, gs[3] / n);
                *csv << b3;
            }
            *csv << "\n";
        }
    }
    for (int c = 0; c < f.cell_count(); ++c) {
        const double res = f.water_balance_residual_mm(c);
        sum.max_balance_residual_mm = std::max(sum.max_balance_residual_mm, res < 0 ? -res : res);
    }
    if (sc.field.nutrients_enabled) {
        for (int c = 0; c < f.cell_count(); ++c) {
            const auto absmax = [](double a, double b) { return std::max(a, b < 0 ? -b : b); };
            sum.max_n_residual = absmax(sum.max_n_residual, f.n_balance_residual(c));
            sum.max_p_residual = absmax(sum.max_p_residual, f.p_balance_residual(c));
            sum.max_k_residual = absmax(sum.max_k_residual, f.k_balance_residual(c));
            sum.max_cao_residual = absmax(sum.max_cao_residual, f.cao_balance_residual(c));
        }
    }
    if (sc.field.management_enabled) {
        double seeds = 0.0;
        for (int c = 0; c < f.cell_count(); ++c) {
            const double r = f.c_balance_residual(c);
            sum.max_c_residual = std::max(sum.max_c_residual, r < 0 ? -r : r);
            const field::ManagementView mv = f.management_view(c);
            for (const field::WeedGroupView& g : mv.weeds) seeds += g.seeds_total;
        }
        sum.weed_seedbank_end_m2 = seeds / f.cell_count();
    }
    sum.total_precip_mm = f.ledger(0).precipitation;
    sum.state_hash = f.fold_hash(1469598103934665603ull);
    return sum;
}

} // namespace ps::io
