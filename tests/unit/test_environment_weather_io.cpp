// tests/unit/test_environment_weather_io.cpp - weather plan W3 loaders: the
// regime / transition / model blocks of physics_sim.climate/1,
// physics_sim.weather_preset/1 and physics_sim.weather_classes/1 (round trips,
// every rejection message) and the shipped files: data/climates/
// test_weather.json, the 12 data/weather_presets/*.json and data/
// weather_classes/wmo_dwd_torro.json - each preset lands in the class it is
// named after. Tags [environment][io][weather].
//
// RECORDED SABOTAGE (performed by hand, reverted): transition rows not checked
// for their sum (the 1e-9 check removed from weather_desc.cpp): the "row sum"
// rejection row failed (the file loaded).

#include "ps/environment/climate_io.h"
#include "ps/environment/json_extends.h"
#include "ps/environment/weather.h"
#include "ps/environment/weather_classes.h"
#include "ps/environment/weather_io.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <functional>
#include <string>

using namespace ps;
using namespace ps::environment;
using nlohmann::json;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;

namespace {

fs::path data(const std::string& rel) { return fs::path(PS_SOURCE_DIR) / "data" / rel; }

json climate_json() { return io::load_json_with_extends(data("climates/test_weather.json")); }

} // namespace

TEST_CASE("Shipped test_weather climate loads and drives a timeline", "[environment][io][weather]") {
    const ClimateDesc d = io::load_climate_json(data("climates/test_weather.json"));
    REQUIRE(d.has_weather_statistics());
    REQUIRE(d.seasons.size() == 4);
    REQUIRE(d.seasons[1].weather);
    const SeasonWeatherDesc& summer = *d.seasons[1].weather;
    CHECK(summer.regimes.size() == 4);
    CHECK(summer.initial_regime == "clear");
    const RegimeDesc* ts = nullptr;
    for (const NamedRegime& r : summer.regimes)
        if (r.name == "thunderstorm") ts = &r.regime;
    REQUIRE(ts);
    REQUIRE(ts->convective);
    REQUIRE(ts->convective->hail);
    CHECK(ts->convective->hail->probability == 0.2);
    CHECK(ts->lightning_per_min == 3.0);
    CHECK(d.lags->precip_on_s == 300.0);
    CHECK(d.diurnal_temperature->a_h == 1.86);
    CHECK(d.lightning->distance_max_km == 15.0);

    WeatherOptions o;
    o.calendar.mode = CalendarMode::FixedSeason;
    o.calendar.fixed_season = 1;
    const WeatherTimeline tl(d, 2026, o);
    CHECK(tl.regime_count() == 4);
    WeatherCursor c;
    bool saw_storm = false;
    for (real t = 0.0; t < 20.0 * 86400.0; t += 3600.0) {
        const WeatherSample s = tl.evaluate(t, c);
        CHECK(s.cloud_cover >= 0.0);
        CHECK(s.cloud_cover <= 1.0);
        CHECK(s.air_temp_c > -10.0);
        CHECK(s.air_temp_c < 45.0);
        saw_storm = saw_storm || tl.regime_name(s.regime) == "thunderstorm";
    }
    CHECK(saw_storm);
}

TEST_CASE("Climate weather-block rejections", "[environment][io][weather]") {
    struct Row { const char* what; std::function<void(json&)> mutate; const char* message; };
    const Row rows[] = {
        {"row sum", [](json& j) { j["seasons"][0]["transitions"]["clear"]["rain"] = 0.2; }, "must sum to 1"},
        {"unknown regime in a row", [](json& j) { j["seasons"][0]["transitions"]["clear"]["hail"] = 0.0; }, "not a regime of this season"},
        {"row for an unknown regime", [](json& j) { j["seasons"][0]["transitions"]["fog"] = json{{"clear", 1.0}}; }, "\"fog\" is not a regime"},
        {"missing row", [](json& j) { j["seasons"][0]["transitions"].erase("rain"); }, "no row for regime \"rain\""},
        {"probability above 1", [](json& j) {
             j["seasons"][0]["transitions"]["clear"] = json{{"clear", 1.5}, {"rain", -0.5}}; }, "must be in [0, 1]"},
        {"probability not a number", [](json& j) { j["seasons"][0]["transitions"]["clear"]["rain"] = "x"; }, "must be a number"},
        {"initial regime unknown", [](json& j) { j["seasons"][0]["initial_regime"] = "fog"; }, "initial_regime \"fog\""},
        {"initial regime missing", [](json& j) { j["seasons"][0].erase("initial_regime"); }, "\"initial_regime\""},
        {"regimes on one season only", [](json& j) {
             for (const char* k : {"regimes", "transitions", "initial_regime"}) j["seasons"][2].erase(k); },
         "every season or on none"},
        {"empty regimes", [](json& j) {
             j["seasons"][0]["regimes"] = json::object(); j["seasons"][0]["transitions"] = json::object(); },
         "regimes must not be empty"},
        {"duration below 0.1 h", [](json& j) { j["seasons"][0]["regimes"]["clear"]["duration_h"]["min"] = 0.05; }, "duration_h.min must be >= 0.1"},
        {"duration median zero", [](json& j) { j["seasons"][0]["regimes"]["clear"]["duration_h"]["median"] = 0; }, "duration_h.median must be > 0"},
        {"cloud range above 1", [](json& j) { j["seasons"][0]["regimes"]["clear"]["cloud_cover"] = json::array({0.2, 1.2}); }, "cloud_cover must lie in"},
        {"cloud range reversed", [](json& j) { j["seasons"][0]["regimes"]["clear"]["cloud_cover"] = json::array({0.5, 0.2}); }, "lo <= hi"},
        {"range wrong shape", [](json& j) { j["seasons"][0]["regimes"]["clear"]["rh"] = 0.5; }, "array of two numbers"},
        {"precip missing", [](json& j) { j["seasons"][0]["regimes"]["clear"].erase("precip_mm_h"); }, "\"precip_mm_h\""},
        {"precip sigma above 3", [](json& j) {
             j["seasons"][0]["regimes"]["rain"]["precip_mm_h"]["sigma_log"] = 4; }, "sigma_log must be in [0, 3]"},
        {"precip form unknown", [](json& j) { j["seasons"][0]["regimes"]["rain"]["precip_form"] = "hail"; }, "\"precip_form\" must be"},
        {"freezing rain in a warm season", [](json& j) {
             j["seasons"][1]["regimes"]["rain"]["precip_form"] = "freezing_rain"; }, "freezing_rain in a season with temp_mean_c above +5"},
        {"weibull k zero", [](json& j) { j["seasons"][0]["regimes"]["clear"]["wind_mean_m_s"]["weibull_k"] = 0; }, "weibull_k must be > 0"},
        {"wind spread above 180", [](json& j) { j["seasons"][0]["regimes"]["clear"]["wind_dir_deg"]["spread"] = 200; }, "spread must be in [0, 180]"},
        {"gust factor below 1", [](json& j) { j["seasons"][0]["regimes"]["clear"]["gust_factor"] = 0.9; }, "gust_factor must be >= 1"},
        {"lightning above 30", [](json& j) { j["seasons"][1]["regimes"]["thunderstorm"]["lightning_per_min"] = 31; }, "lightning_per_min must be in [0, 30]"},
        {"rise fraction 1", [](json& j) { j["seasons"][1]["regimes"]["thunderstorm"]["convective"]["rise_fraction"] = 1.0; }, "rise_fraction must be in (0, 1)"},
        {"hail probability 2", [](json& j) { j["seasons"][1]["regimes"]["thunderstorm"]["convective"]["hail"]["probability"] = 2; }, "hail.probability"},
        {"unknown regime key", [](json& j) { j["seasons"][0]["regimes"]["clear"]["fog"] = 1; }, "unknown key \"fog\""},
        {"unknown convective key", [](json& j) { j["seasons"][1]["regimes"]["thunderstorm"]["convective"]["x"] = 1; }, "unknown key \"x\""},
        {"lags zero", [](json& j) { j["lags_s"]["cloud"] = 0; }, "every time constant must be > 0"},
        {"lags missing with regimes", [](json& j) { j.erase("lags_s"); }, "lags_s is required"},
        {"precip_phase missing", [](json& j) { j.erase("precip_phase"); }, "precip_phase is required"},
        {"precip_phase reversed", [](json& j) { j["precip_phase"]["rain_above_c"] = -1.0; }, "rain_above_c must be > snow_below_c"},
        {"humidity missing", [](json& j) { j.erase("humidity"); }, "humidity is required"},
        {"humidity floor above 1", [](json& j) { j["humidity"]["precip_rh_floor"] = 1.2; }, "precip_rh_floor must be in [0, 1]"},
        {"diurnal missing", [](json& j) { j.erase("diurnal_temperature"); }, "diurnal_temperature is required"},
        {"diurnal b zero", [](json& j) { j["diurnal_temperature"]["b"] = 0; }, "diurnal_temperature.b must be > 0"},
        {"diurnal a vs c", [](json& j) { j["diurnal_temperature"]["c_h"] = -5.0; }, "a_h must be > -c_h/2"},
        {"lightning block missing", [](json& j) { j.erase("lightning"); }, "lightning is required"},
        {"lightning range reversed", [](json& j) { j["lightning"]["distance_km"] = json::array({5, 1}); }, "max >= min"},
        {"climate scalars missing with regimes", [](json& j) {
             for (auto& s : j["seasons"])
                 for (const char* k : {"temp_mean_c", "temp_range_k", "rh_mean", "pressure_msl_hpa", "ground_temp_c"}) s.erase(k); },
         "no climate scalars"},
    };
    for (const Row& row : rows) {
        json j = climate_json();
        row.mutate(j);
        INFO("rejection row: " << row.what);
        CHECK_THROWS_WITH(io::parse_climate_json(j, "test_weather"), ContainsSubstring(row.message));
    }
    CHECK_NOTHROW(io::parse_climate_json(climate_json(), "test_weather"));
}

TEST_CASE("Shipped presets load and land in their classes", "[environment][io][weather]") {
    const WeatherClassesDesc cls = io::load_weather_classes_json(data("weather_classes/wmo_dwd_torro.json"));
    const ClimateDesc climate = io::load_climate_json(data("climates/test_weather.json"));
    struct Expect {
        const char* name;
        PrecipType type;
        PrecipIntensity intensity;
        bool cloudburst, thunderstorm, unwetter;
        int beaufort;
    };
    const Expect rows[] = {
        {"clear", PrecipType::None, PrecipIntensity::None, false, false, false, 2},
        {"overcast", PrecipType::None, PrecipIntensity::None, false, false, false, 3},
        {"drizzle", PrecipType::Drizzle, PrecipIntensity::Moderate, false, false, false, 2},
        {"light_rain", PrecipType::Rain, PrecipIntensity::Light, false, false, false, 3},
        {"rain", PrecipType::Rain, PrecipIntensity::Moderate, false, false, false, 3},
        {"heavy_rain", PrecipType::Rain, PrecipIntensity::Heavy, false, false, true, 4},
        {"cloudburst", PrecipType::Rain, PrecipIntensity::Violent, true, true, true, 5},
        {"thunderstorm", PrecipType::Rain, PrecipIntensity::Heavy, false, true, true, 5},
        {"storm", PrecipType::Rain, PrecipIntensity::Moderate, false, false, true, 9},
        {"snow", PrecipType::Snow, PrecipIntensity::Moderate, false, false, false, 3},
        {"freezing_rain", PrecipType::FreezingRain, PrecipIntensity::Light, false, false, false, 2},
        {"fog", PrecipType::None, PrecipIntensity::None, false, false, false, 1},
    };
    for (const Expect& e : rows) {
        INFO("preset " << e.name);
        WeatherOptions o;
        o.calendar.mode = CalendarMode::FixedSeason;
        o.calendar.fixed_season = 1;   // summer: Auto-typed presets fall as rain
        o.mode = WeatherMode::Preset;
        o.preset = io::load_weather_preset_json(data(std::string("weather_presets/") + e.name + ".json"));
        CHECK_FALSE(o.preset.source.empty());
        const WeatherTimeline tl(climate, 1, o);
        WeatherCursor c;
        // After 6 h of the preset the 1 h / 6 h accumulations are its own.
        const WeatherSample s = tl.evaluate(10.0 * 86400.0, c);
        const WeatherClasses k = classify(cls, s);
        CHECK(k.type == e.type);
        CHECK(k.intensity == e.intensity);
        CHECK(k.cloudburst == e.cloudburst);
        CHECK(k.thunderstorm == e.thunderstorm);
        CHECK(k.unwetter == e.unwetter);
        CHECK(k.beaufort == e.beaufort);
    }
}

TEST_CASE("Preset and classes rejections", "[environment][io][weather]") {
    const json preset = io::load_json_with_extends(data("weather_presets/rain.json"));
    struct Row { const char* what; std::function<void(json&)> mutate; const char* message; };
    const Row prows[] = {
        {"wrong format", [](json& j) { j["format"] = "physics_sim.weather_preset/2"; }, "\"format\" must be"},
        {"missing regime", [](json& j) { j.erase("regime"); }, "\"regime\""},
        {"cloud above 1", [](json& j) { j["cloud_cover"] = 1.5; }, "cloud_cover must be in [0, 1]"},
        {"negative precip", [](json& j) { j["precip_mm_h"] = -1; }, "precip_mm_h must be >= 0"},
        {"type unknown", [](json& j) { j["precip_type"] = "graupel"; }, "\"precip_type\" must be"},
        {"hail without diameter", [](json& j) { j["precip_type"] = "hail"; }, "\"hail_diameter_mm\""},
        {"diameter without hail", [](json& j) { j["hail_diameter_mm"] = 10; }, "only allowed with precip_type hail"},
        {"both temperatures", [](json& j) { j["temp_offset_k"] = 1; j["temp_override_c"] = 1; }, "at most one of"},
        {"gust below 1", [](json& j) { j["gust_factor"] = 0.5; }, "gust_factor must be >= 1"},
        {"direction above 360", [](json& j) { j["wind_dir_deg"] = 400; }, "wind_dir_deg must be in [0, 360]"},
        {"rh missing", [](json& j) { j.erase("rh"); }, "\"rh\""},
        {"lightning above 30", [](json& j) { j["lightning_per_min"] = 40; }, "lightning_per_min must be in [0, 30]"},
        {"unknown key", [](json& j) { j["visibility_m"] = 100; }, "unknown key \"visibility_m\""},
    };
    for (const Row& row : prows) {
        json j = preset;
        row.mutate(j);
        INFO("preset rejection row: " << row.what);
        CHECK_THROWS_WITH(io::parse_weather_preset_json(j, "rain"), ContainsSubstring(row.message));
    }
    {
        json j = preset;
        j["precip_type"] = "hail";
        j["hail_diameter_mm"] = 20.0;
        const WeatherPresetDesc p = io::parse_weather_preset_json(j, "hail");
        CHECK(p.precip_type == PresetPrecipType::Hail);
        CHECK(p.hail_diameter_mm == 20.0);
        json k = preset;
        k["temp_override_c"] = -4.0;
        const WeatherPresetDesc q = io::parse_weather_preset_json(k, "cold");
        CHECK(q.temp_is_override);
        CHECK(q.temp_override_c == -4.0);
    }

    const json classes = io::load_json_with_extends(data("weather_classes/wmo_dwd_torro.json"));
    const Row crows[] = {
        {"wrong format", [](json& j) { j["format"] = "physics_sim.weather_classes/2"; }, "\"format\" must be"},
        {"threshold count", [](json& j) { j["rain_mm_h"] = json{{"thresholds", json::array({2.5, 10.0})}, {"rule", "ge"}}; }, "rain_mm_h must have 3 thresholds"},
        {"not ascending", [](json& j) { j["rain_mm_h"]["thresholds"] = json::array({2.5, 50.0, 10.0}); }, "strictly ascending"},
        {"rule unknown", [](json& j) { j["snow_mm_h"]["rule"] = "le"; }, "must be \"ge\" or \"gt\""},
        {"rule count", [](json& j) { j["snow_mm_h"]["rule"] = json::array({"ge"}); }, "one entry per threshold"},
        {"rule missing", [](json& j) { j["snow_mm_h"].erase("rule"); }, "\"rule\""},
        {"cloudburst zero", [](json& j) { j["cloudburst_mm_h"] = 0; }, "cloudburst_mm_h must be > 0"},
        {"unwetter level 4", [](json& j) { j["dwd_rain_unwetter_level"] = 4; }, "unwetter level must be in [1, 3]"},
        {"missing scale", [](json& j) { j.erase("torro_hail_mm"); }, "\"torro_hail_mm\""},
        {"unknown key", [](json& j) { j["fog_m"] = 1; }, "unknown key \"fog_m\""},
    };
    for (const Row& row : crows) {
        json j = classes;
        row.mutate(j);
        INFO("classes rejection row: " << row.what);
        CHECK_THROWS_WITH(io::parse_weather_classes_json(j, "classes"), ContainsSubstring(row.message));
    }
    // Q17 is a data change: another cloudburst threshold moves the class.
    json j = classes;
    j["cloudburst_mm_h"] = 80.0;
    const WeatherClassesDesc d80 = io::parse_weather_classes_json(j, "classes80");
    WeatherSample s;
    s.precip_type = PrecipType::Rain;
    s.precip_mm_h = 90.0;
    CHECK(classify(d80, s).cloudburst);
    CHECK_FALSE(classify(io::parse_weather_classes_json(classes, "c"), s).cloudburst);
}
