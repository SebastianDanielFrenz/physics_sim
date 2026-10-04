// core/src/environment/io/weather_io.cpp - see ps/environment/weather_io.h,
// docs/data_formats/weather_preset.md and weather_classes.md.

#include "ps/environment/weather_io.h"

#include "ps/environment/json_extends.h"
#include "object_reader.h"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace ps::io {

using nlohmann::json;
using namespace ps::environment;
using ps::io::detail::ObjectReader;

namespace {

std::string source_text(const json* src) {
    if (!src) return {};
    return src->is_string() ? src->get<std::string>() : src->dump();
}

ClassScale read_scale(ObjectReader& parent, const char* key, const std::string& origin) {
    const json& j = parent.object(key);
    ObjectReader r(j, key, origin);
    ClassScale s;
    const json* th = r.child("thresholds");
    if (!th || !th->is_array() || th->empty()) r.fail("\"thresholds\" must be a non-empty array of numbers");
    for (const json& v : *th) {
        if (!v.is_number()) r.fail("\"thresholds\" must contain numbers only");
        s.thresholds.push_back(v.get<real>());
    }
    const json* ru = r.child("rule");
    if (!ru) r.fail("missing required field \"rule\" (\"ge\", \"gt\" or an array of them)");
    auto parse_rule = [&](const json& v) {
        if (!v.is_string()) r.fail("\"rule\" entries must be \"ge\" or \"gt\"");
        const std::string t = v.get<std::string>();
        if (t == "ge") return ThresholdRule::Ge;
        if (t == "gt") return ThresholdRule::Gt;
        r.fail("\"rule\" entries must be \"ge\" or \"gt\" (got \"" + t + "\")");
    };
    if (ru->is_array()) {
        if (ru->size() != s.thresholds.size()) r.fail("\"rule\" must have one entry per threshold");
        for (const json& v : *ru) s.rules.push_back(parse_rule(v));
    } else {
        s.rules.assign(s.thresholds.size(), parse_rule(*ru));
    }
    r.finish();
    return s;
}

} // namespace

WeatherPresetDesc parse_weather_preset_json(const json& root, const std::string& origin) {
    ObjectReader r(root, "", origin);
    const std::string format = r.string("format", true, "");
    if (format != "physics_sim.weather_preset/1")
        r.fail("\"format\" must be \"physics_sim.weather_preset/1\" (got \"" + format + "\")");
    r.skip("extends");
    WeatherPresetDesc p;
    p.regime = r.string("regime", true, "");
    if (p.regime.empty()) r.fail("\"regime\" must not be empty");
    p.cloud_cover = r.number("cloud_cover", true, 0.0);
    p.precip_mm_h = r.number("precip_mm_h", true, 0.0);
    const std::string type = r.string("precip_type", true, "");
    if (type == "auto") p.precip_type = PresetPrecipType::Auto;
    else if (type == "rain") p.precip_type = PresetPrecipType::Rain;
    else if (type == "snow") p.precip_type = PresetPrecipType::Snow;
    else if (type == "sleet") p.precip_type = PresetPrecipType::Sleet;
    else if (type == "freezing_rain") p.precip_type = PresetPrecipType::FreezingRain;
    else if (type == "drizzle") p.precip_type = PresetPrecipType::Drizzle;
    else if (type == "hail") p.precip_type = PresetPrecipType::Hail;
    else r.fail("\"precip_type\" must be auto, rain, snow, sleet, freezing_rain, drizzle or hail (got \"" + type + "\")");
    if (p.precip_type == PresetPrecipType::Hail) p.hail_diameter_mm = r.number("hail_diameter_mm", true, 0.0);
    else if (r.has("hail_diameter_mm")) r.fail("\"hail_diameter_mm\" is only allowed with precip_type hail");
    else r.skip("hail_diameter_mm");
    p.wind_mean_m_s = r.number("wind_mean_m_s", true, 0.0);
    p.wind_dir_deg = r.number("wind_dir_deg", true, 0.0);
    p.gust_factor = r.number("gust_factor", true, 0.0);
    if (r.has("temp_offset_k") && r.has("temp_override_c"))
        r.fail("give at most one of \"temp_offset_k\" and \"temp_override_c\"");
    if (r.has("temp_override_c")) {
        p.temp_is_override = true;
        p.temp_override_c = r.number("temp_override_c", true, 0.0);
        r.skip("temp_offset_k");
    } else {
        p.temp_offset_k = r.number("temp_offset_k", false, 0.0);
        r.skip("temp_override_c");
    }
    p.rh = r.number("rh", true, 0.0);
    p.pressure_offset_hpa = r.number("pressure_offset_hpa", false, 0.0);
    p.lightning_per_min = r.number("lightning_per_min", false, 0.0);
    p.source = source_text(r.child("source"));
    r.finish();
    const std::string problem = weather_preset_validation_error(p);
    if (!problem.empty()) throw std::runtime_error(origin + ": " + problem);
    return p;
}

WeatherPresetDesc load_weather_preset_json(const std::filesystem::path& path) {
    return parse_weather_preset_json(load_json_with_extends(path), path.string());
}

WeatherClassesDesc parse_weather_classes_json(const json& root, const std::string& origin) {
    ObjectReader r(root, "", origin);
    const std::string format = r.string("format", true, "");
    if (format != "physics_sim.weather_classes/1")
        r.fail("\"format\" must be \"physics_sim.weather_classes/1\" (got \"" + format + "\")");
    r.skip("extends");
    WeatherClassesDesc d;
    d.drizzle_mm_h = read_scale(r, "drizzle_mm_h", origin);
    d.rain_mm_h = read_scale(r, "rain_mm_h", origin);
    d.snow_mm_h = read_scale(r, "snow_mm_h", origin);
    d.cloudburst_mm_h = r.number("cloudburst_mm_h", true, 0.0);
    d.dwd_rain_1h_mm = read_scale(r, "dwd_rain_1h_mm", origin);
    d.dwd_rain_6h_mm = read_scale(r, "dwd_rain_6h_mm", origin);
    d.dwd_rain_unwetter_level = r.integer("dwd_rain_unwetter_level", true, 0);
    d.beaufort_m_s = read_scale(r, "beaufort_m_s", origin);
    d.dwd_gust_km_h = read_scale(r, "dwd_gust_km_h", origin);
    d.dwd_gust_unwetter_level = r.integer("dwd_gust_unwetter_level", true, 0);
    d.torro_hail_mm = read_scale(r, "torro_hail_mm", origin);
    d.source = source_text(r.child("source"));
    r.finish();
    const std::string problem = weather_classes_validation_error(d);
    if (!problem.empty()) throw std::runtime_error(origin + ": " + problem);
    return d;
}

WeatherClassesDesc load_weather_classes_json(const std::filesystem::path& path) {
    return parse_weather_classes_json(load_json_with_extends(path), path.string());
}

} // namespace ps::io
