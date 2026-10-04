// core/src/environment/io/climate_io.cpp - see ps/environment/climate_io.h and
// docs/data_formats/climate.md.

#include "ps/environment/climate_io.h"

#include "ps/environment/json_extends.h"

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

// Typed access to one JSON object with unknown-key rejection. Never uses
// json::get<T>() on an unchecked value (no exception is caught anywhere in
// this file - clang-cl ASan catch caveat).
class ObjectReader {
public:
    ObjectReader(const json& obj, std::string context, const std::string& origin)
        : obj_(obj), ctx_(std::move(context)), origin_(origin) {
        if (!obj.is_object()) fail("must be an object");
    }

    [[noreturn]] void fail(const std::string& msg) const {
        throw std::runtime_error(origin_ + ": " + (ctx_.empty() ? "" : ctx_ + ": ") + msg);
    }

    bool has(const char* key) const { return obj_.contains(key); }

    real number(const char* key, bool required, real fallback) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_number()) fail(std::string("\"") + key + "\" must be a number");
        const real v = it->get<real>();
        if (!std::isfinite(v)) fail(std::string("\"") + key + "\" must be finite");
        return v;
    }

    int integer(const char* key, bool required, int fallback) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_number()) fail(std::string("\"") + key + "\" must be an integer");
        const real v = it->get<real>();
        if (!std::isfinite(v) || v != std::floor(v) || std::fabs(v) > 1e9)
            fail(std::string("\"") + key + "\" must be an integer");
        return static_cast<int>(v);
    }

    std::string string(const char* key, bool required, const std::string& fallback) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_string()) fail(std::string("\"") + key + "\" must be a string");
        return it->get<std::string>();
    }

    const json* child(const char* key) {
        used_.insert(key);
        auto it = obj_.find(key);
        return it == obj_.end() ? nullptr : &*it;
    }

    void skip(const char* key) { used_.insert(key); }

    // Keys named by the format but not implemented yet: a clear error rather
    // than a silent drop.
    void reserve_for_later(const char* key, const char* increment) {
        used_.insert(key);
        if (obj_.contains(key))
            fail(std::string("\"") + key + "\" is part of physics_sim.climate/1 but is not implemented before " +
                 increment + "; remove it");
    }

    void finish() const {
        for (auto it = obj_.begin(); it != obj_.end(); ++it)
            if (!used_.count(it.key())) fail("unknown key \"" + it.key() + "\"");
    }

    const std::string& context() const { return ctx_; }

private:
    const json& obj_;
    std::string ctx_;
    const std::string& origin_;
    std::set<std::string> used_;
};

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

    r.reserve_for_later("regimes", "W3 (weather timeline)");
    r.reserve_for_later("transitions", "W3 (weather timeline)");
    r.reserve_for_later("initial_regime", "W3 (weather timeline)");
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

    r.reserve_for_later("lags_s", "W3 (weather timeline)");
    r.reserve_for_later("gusts", "W4 (wind)");
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
