// core/src/environment/io/climate_io.cpp - see ps/environment/climate_io.h and
// docs/data_formats/climate.md.

#include "ps/environment/climate_io.h"

#include "ps/environment/json_extends.h"
#include "object_reader.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps::io {

using nlohmann::json;
using namespace ps::environment;

namespace {
using ps::io::detail::ObjectReader;


SunAuthoredPartial read_authored_partial(const json& j, const std::string& ctx, const std::string& origin) {
    ObjectReader r(j, ctx, origin);
    SunAuthoredPartial p;
    if (r.has("e_max_deg")) p.e_max_deg = r.number("e_max_deg", true, 0.0); else r.skip("e_max_deg");
    if (r.has("e_night_deg")) p.e_night_deg = r.number("e_night_deg", true, 0.0); else r.skip("e_night_deg");
    if (r.has("az_rise_deg")) p.az_rise_deg = r.number("az_rise_deg", true, 0.0); else r.skip("az_rise_deg");
    if (r.has("az_set_deg")) p.az_set_deg = r.number("az_set_deg", true, 0.0); else r.skip("az_set_deg");
    r.finish();
    return p;
}

DayNightDesc read_day_night(const json& j, const std::string& ctx, const std::string& origin) {
    ObjectReader r(j, ctx, origin);
    DayNightDesc d;
    const bool has_day = r.has("day_s"), has_night = r.has("night_s");
    const bool has_cycle = r.has("cycle_s"), has_frac = r.has("day_fraction");
    const bool has_astro = r.has("astronomical");
    const bool explicit_form = has_day || has_night;
    const bool fraction_form = has_cycle || has_frac;
    const int forms = int(explicit_form) + int(fraction_form) + int(has_astro);
    if (forms == 0)
        r.fail("must give one form: {day_s, night_s}, {cycle_s, day_fraction} or {astronomical: true}");
    if (forms > 1) r.fail("gives more than one day/night form (day_s/night_s, cycle_s/day_fraction, astronomical)");
    if (explicit_form) {
        d.kind = DayNightKind::Explicit;
        d.day_s = r.number("day_s", true, 0.0);
        d.night_s = r.number("night_s", true, 0.0);
        r.skip("cycle_s"); r.skip("day_fraction"); r.skip("astronomical");
    } else if (fraction_form) {
        d.kind = DayNightKind::CycleFraction;
        d.cycle_s = r.number("cycle_s", true, 0.0);
        d.day_fraction = r.number("day_fraction", true, 0.0);
        r.skip("day_s"); r.skip("night_s"); r.skip("astronomical");
    } else {
        const json* a = r.child("astronomical");
        if (!a->is_boolean() || !a->get<bool>()) r.fail("\"astronomical\" must be true");
        d.kind = DayNightKind::Astronomical;
        r.skip("day_s"); r.skip("night_s"); r.skip("cycle_s"); r.skip("day_fraction");
    }
    r.finish();
    return d;
}

LogNormalDesc read_lognormal(const json& j, const std::string& ctx, const std::string& origin) {
    ObjectReader r(j, ctx, origin);
    LogNormalDesc l;
    l.median = r.number("median", true, 0.0);
    l.sigma_log = r.number("sigma_log", true, 0.0);
    r.finish();
    return l;
}

ConvectiveDesc read_convective(const json& j, const std::string& ctx, const std::string& origin) {
    ObjectReader r(j, ctx, origin);
    ConvectiveDesc c;
    c.cell_duration_min = read_lognormal(r.object("cell_duration_min"), ctx + ".cell_duration_min", origin);
    c.gap_min = read_lognormal(r.object("gap_min"), ctx + ".gap_min", origin);
    c.peak_mm_h = read_lognormal(r.object("peak_mm_h"), ctx + ".peak_mm_h", origin);
    c.rise_fraction = r.number("rise_fraction", true, 0.0);
    c.gust_factor = r.number("gust_factor", true, 0.0);
    if (const json* h = r.child("hail"); h && !h->is_null()) {
        ObjectReader hr(*h, ctx + ".hail", origin);
        HailDesc hd;
        hd.probability = hr.number("probability", true, 0.0);
        hd.diameter_mm = read_lognormal(hr.object("diameter_mm"), ctx + ".hail.diameter_mm", origin);
        hd.min_shape = hr.number("min_shape", true, 0.0);
        hr.finish();
        c.hail = hd;
    }
    r.finish();
    return c;
}

RegimeDesc read_regime(const json& j, const std::string& ctx, const std::string& origin) {
    ObjectReader r(j, ctx, origin);
    RegimeDesc g;
    {
        ObjectReader d(r.object("duration_h"), ctx + ".duration_h", origin);
        g.duration.min_h = d.number("min", true, 0.0);
        g.duration.median_h = d.number("median", true, 0.0);
        g.duration.sigma_log = d.number("sigma_log", true, 0.0);
        d.finish();
    }
    g.cloud_cover = r.range("cloud_cover");
    {
        const json* p = r.child("precip_mm_h");
        if (!p) r.fail("missing required field \"precip_mm_h\" (an object, or null for no stratiform precipitation)");
        if (!p->is_null()) g.precip_mm_h = read_lognormal(*p, ctx + ".precip_mm_h", origin);
    }
    const std::string form = r.string("precip_form", false, "auto");
    if (form == "auto") g.precip_form = PrecipForm::Auto;
    else if (form == "drizzle") g.precip_form = PrecipForm::Drizzle;
    else if (form == "freezing_rain") g.precip_form = PrecipForm::FreezingRain;
    else r.fail("\"precip_form\" must be \"auto\", \"drizzle\" or \"freezing_rain\" (got \"" + form + "\")");
    if (const json* c = r.child("convective"); c && !c->is_null()) g.convective = read_convective(*c, ctx + ".convective", origin);
    {
        ObjectReader w(r.object("wind_mean_m_s"), ctx + ".wind_mean_m_s", origin);
        g.wind_mean_m_s.k = w.number("weibull_k", true, 0.0);
        g.wind_mean_m_s.c = w.number("weibull_c", true, 0.0);
        w.finish();
    }
    {
        ObjectReader w(r.object("wind_dir_deg"), ctx + ".wind_dir_deg", origin);
        g.wind_dir_deg.mean_deg = w.number("mean", true, 0.0);
        g.wind_dir_deg.spread_deg = w.number("spread", true, 0.0);
        w.finish();
    }
    g.gust_factor = r.number("gust_factor", true, 0.0);
    g.temp_offset_k = r.range("temp_offset_k");
    g.rh = r.range("rh");
    g.pressure_offset_hpa = r.range("pressure_offset_hpa");
    g.lightning_per_min = r.number("lightning_per_min", false, 0.0);
    r.finish();
    return g;
}

SeasonWeatherDesc read_season_weather(ObjectReader& r, const std::string& ctx, const std::string& origin) {
    SeasonWeatherDesc w;
    const json& regs = r.object("regimes");
    for (auto it = regs.begin(); it != regs.end(); ++it)   // nlohmann object: keys in alphabetical order
        w.regimes.push_back({it.key(), read_regime(it.value(), ctx + ".regimes." + it.key(), origin)});
    const json& tr = r.object("transitions");
    for (auto it = tr.begin(); it != tr.end(); ++it) {
        if (!it.value().is_object()) r.fail("\"transitions." + it.key() + "\" must be an object {regime: probability}");
        TransitionRowDesc row;
        row.from = it.key();
        for (auto jt = it.value().begin(); jt != it.value().end(); ++jt) {
            if (!jt.value().is_number()) r.fail("\"transitions." + it.key() + "." + jt.key() + "\" must be a number");
            row.to.push_back({jt.key(), jt.value().get<real>()});
        }
        w.transitions.push_back(row);
    }
    w.initial_regime = r.string("initial_regime", true, "");
    return w;
}

SeasonDesc read_season(const json& j, const std::string& ctx, const std::string& origin) {
    ObjectReader r(j, ctx, origin);
    SeasonDesc s;
    s.name = r.string("name", true, "");
    s.days = r.integer("days", true, 1);

    if (const json* span = r.child("doy_span")) {
        if (!span->is_array() || span->size() != 2 || !(*span)[0].is_number() || !(*span)[1].is_number())
            r.fail("\"doy_span\" must be an array of two integers [first, last]");
        const real a = (*span)[0].get<real>();
        const real b = (*span)[1].get<real>();
        if (a != std::floor(a) || b != std::floor(b) || a < 1 || a > 366 || b < 1 || b > 366)
            r.fail("\"doy_span\" must be two integers in [1, 366]");
        s.has_doy_span = true;
        s.doy_start = static_cast<int>(a);
        s.doy_end = static_cast<int>(b);
    }

    if (const json* dn = r.child("day_night")) {
        s.day_night = read_day_night(*dn, ctx + ".day_night", origin);
    } else {
        s.day_night.kind = DayNightKind::Astronomical;  // default form (owner question Q2)
    }

    const bool climate_any = r.has("temp_mean_c") || r.has("temp_range_k") || r.has("rh_mean") ||
                             r.has("pressure_msl_hpa") || r.has("ground_temp_c");
    if (climate_any) {
        SeasonClimateDesc c;
        c.temp_mean_c = r.number("temp_mean_c", true, 0.0);
        c.temp_range_k = r.number("temp_range_k", true, 0.0);
        c.rh_mean = r.number("rh_mean", true, 0.0);
        c.pressure_msl_hpa = r.number("pressure_msl_hpa", true, 0.0);
        c.ground_temp_c = r.number("ground_temp_c", true, 0.0);
        s.climate = c;
    } else {
        r.skip("temp_mean_c"); r.skip("temp_range_k"); r.skip("rh_mean");
        r.skip("pressure_msl_hpa"); r.skip("ground_temp_c");
    }

    if (const json* sa = r.child("sun_authored")) s.sun_authored = read_authored_partial(*sa, ctx + ".sun_authored", origin);

    if (r.has("regimes") || r.has("transitions") || r.has("initial_regime")) {
        s.weather = read_season_weather(r, ctx, origin);
    } else {
        r.skip("regimes"); r.skip("transitions"); r.skip("initial_regime");
    }
    r.finish();
    return s;
}

} // namespace

ClimateDesc parse_climate_json(const json& root, const std::string& origin) {
    ObjectReader r(root, "", origin);
    const std::string format = r.string("format", true, "");
    if (format != "physics_sim.climate/1")
        r.fail("\"format\" must be \"physics_sim.climate/1\" (got \"" + format + "\")");
    r.skip("extends");  // consumed by load_json_with_extends; harmless if a pre-merged object still names it

    ClimateDesc d;

    if (const json* loc = r.child("location")) {
        ObjectReader lr(*loc, "location", origin);
        d.location.latitude_deg = lr.number("latitude_deg", true, 0.0);
        d.location.longitude_deg = lr.number("longitude_deg", false, 0.0);
        d.location.elevation_m = lr.number("elevation_m", false, 0.0);
        lr.finish();
    } else {
        r.fail("missing required field \"location\"");
    }

    if (const json* sun = r.child("sun")) {
        ObjectReader sr(*sun, "sun", origin);
        const std::string mode = sr.string("mode", false, "astronomical");
        if (mode == "astronomical") d.sun_mode = SunMode::Astronomical;
        else if (mode == "authored") d.sun_mode = SunMode::Authored;
        else sr.fail("\"mode\" must be \"astronomical\" or \"authored\" (got \"" + mode + "\")");
        if (const json* a = sr.child("authored")) {
            ObjectReader ar(*a, "sun.authored", origin);
            SunAuthoredDesc ad;
            ad.e_max_deg = ar.number("e_max_deg", true, 0.0);
            ad.e_night_deg = ar.number("e_night_deg", true, 0.0);
            ad.az_rise_deg = ar.number("az_rise_deg", true, 0.0);
            ad.az_set_deg = ar.number("az_set_deg", true, 0.0);
            ar.finish();
            d.sun_authored = ad;
        }
        sr.finish();
    }

    if (const json* cal = r.child("calendar")) {
        ObjectReader cr(*cal, "calendar", origin);
        d.calendar.cycle_s = cr.number("cycle_s", false, 86400.0);
        if (const json* ep = cr.child("epoch")) {
            ObjectReader er(*ep, "calendar.epoch", origin);
            d.calendar.epoch.year = er.integer("year", false, 0);
            d.calendar.epoch.season = er.string("season", false, "");
            d.calendar.epoch.day = er.integer("day", false, 0);
            d.calendar.epoch.time_of_day_s = er.number("time_of_day_s", false, 0.0);
            er.finish();
        }
        cr.finish();
    }

    const json* seasons = r.child("seasons");
    if (!seasons) r.fail("missing required field \"seasons\"");
    if (!seasons->is_array()) r.fail("\"seasons\" must be an array");
    for (std::size_t i = 0; i < seasons->size(); ++i)
        d.seasons.push_back(read_season((*seasons)[i], "seasons[" + std::to_string(i) + "]", origin));

    if (const json* src = r.child("source")) {
        if (src->is_string()) d.source = src->get<std::string>();
        else d.source = src->dump();
    }

    if (const json* l = r.child("lags_s")) {
        ObjectReader lr(*l, "lags_s", origin);
        WeatherLagsDesc g;
        g.cloud_s = lr.number("cloud", true, 0.0);
        g.precip_on_s = lr.number("precip_on", true, 0.0);
        g.precip_off_s = lr.number("precip_off", true, 0.0);
        g.wind_s = lr.number("wind", true, 0.0);
        g.temp_offset_s = lr.number("temp_offset", true, 0.0);
        g.rh_s = lr.number("rh", true, 0.0);
        g.pressure_s = lr.number("pressure", true, 0.0);
        lr.finish();
        d.lags = g;
    }
    if (const json* ph = r.child("precip_phase")) {
        ObjectReader pr(*ph, "precip_phase", origin);
        PrecipPhaseDesc g;
        g.snow_below_c = pr.number("snow_below_c", true, 0.0);
        g.rain_above_c = pr.number("rain_above_c", true, 0.0);
        g.min_rate_mm_h = pr.number("min_rate_mm_h", true, 0.0);
        pr.finish();
        d.precip_phase = g;
    }
    if (const json* hu = r.child("humidity")) {
        ObjectReader hr(*hu, "humidity", origin);
        HumidityDesc g;
        g.precip_rh_floor = hr.number("precip_rh_floor", true, 0.0);
        g.precip_full_mm_h = hr.number("precip_full_mm_h", true, 0.0);
        hr.finish();
        d.humidity = g;
    }
    if (const json* dt = r.child("diurnal_temperature")) {
        ObjectReader tr(*dt, "diurnal_temperature", origin);
        DiurnalTempDesc g;
        g.a_h = tr.number("a_h", true, 0.0);
        g.b = tr.number("b", true, 0.0);
        g.c_h = tr.number("c_h", true, 0.0);
        g.reference_cycle_s = tr.number("reference_cycle_s", true, 0.0);
        g.cloud_range_reduction = tr.number("cloud_range_reduction", true, 0.0);
        tr.finish();
        d.diurnal_temperature = g;
    }
    if (const json* li = r.child("lightning")) {
        ObjectReader lr(*li, "lightning", origin);
        const RangeDesc dist = lr.range("distance_km");
        lr.finish();
        d.lightning = LightningDesc{dist.lo, dist.hi};
    }
    if (const json* gu = r.child("gusts")) {
        ObjectReader gr(*gu, "gusts", origin);
        GustDesc g;   // every member optional: the defaults are the documented [AS]/[C] values
        g.roughness_length_m = gr.number("roughness_length_m", false, g.roughness_length_m);
        g.reference_height_m = gr.number("reference_height_m", false, g.reference_height_m);
        g.reference_gust_factor = gr.number("reference_gust_factor", false, g.reference_gust_factor);
        g.f_min_hz = gr.number("f_min_hz", false, g.f_min_hz);
        g.bands_per_octave = gr.integer("bands_per_octave", false, g.bands_per_octave);
        g.window_s = gr.number("window_s", false, g.window_s);
        g.blend_s = gr.number("blend_s", false, g.blend_s);
        g.min_advection_speed_m_s = gr.number("min_advection_speed_m_s", false, g.min_advection_speed_m_s);
        gr.finish();
        d.gusts = g;
    }
    r.reserve_for_later("condition", "W5 (surface condition)");
    r.finish();

    // Meaning checks (ranges, references, the epoch). The string form keeps
    // the file name in front without catching an exception object.
    const std::string problem = climate_validation_error(d);
    if (!problem.empty()) throw std::runtime_error(origin + ": " + problem);
    return d;
}

ClimateDesc load_climate_json(const std::filesystem::path& path) {
    const json root = load_json_with_extends(path);
    return parse_climate_json(root, path.string());
}

} // namespace ps::io
