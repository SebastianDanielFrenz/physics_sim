// tests/unit/test_environment_io.cpp - weather plan W1: the `extends`
// merge-patch helper (RFC 7396 examples, chains, cycles) and the
// physics_sim.climate/1 loader (round trips and rejections). Tags
// [environment][io].
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - merge patch: json_extends.cpp erasing nothing on `null` (storing the null
//    instead): RFC examples 3, 4, 7, 14 and 15 failed ("{\"a\":null}" instead
//    of "{}" etc.) and the extends chain "null deletes" check failed.
//  - extends order: merging the PARENT over the child (user file first): the
//    "user file last" chain test failed on the overridden value.
//  - unknown-key rejection: ObjectReader::finish() made a no-op: the
//    "unknown key" rejection row failed (the file loaded).
//  - cycle detection removed: the cycle test overflowed into the depth limit
//    message instead of "extends cycle" (and a self-extending file looped to the
//    limit).

#include "environment_fixtures.h"

#include "ps/environment/calendar.h"
#include "ps/environment/climate_io.h"
#include "ps/environment/json_extends.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::environment;
using nlohmann::json;
using Catch::Matchers::ContainsSubstring;

namespace {

namespace fs = std::filesystem;

// A unique scratch directory removed at scope exit.
struct TempDir {
    fs::path path;
    TempDir() {
        static int counter = 0;
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("ps_env_io_" + std::to_string(stamp) + "_" + std::to_string(counter++));
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    fs::path write_text(const std::string& name, const std::string& text) const {
        const fs::path p = path / name;
        fs::create_directories(p.parent_path());
        std::ofstream(p, std::ios::binary) << text;
        return p;
    }
    fs::path write(const std::string& name, const json& j) const { return write_text(name, j.dump(2)); }
};

json valid_climate_json() {
    return json::parse(R"({
      "format": "physics_sim.climate/1",
      "location": {"latitude_deg": 50.15, "longitude_deg": 8.45, "elevation_m": 150},
      "sun": {"mode": "astronomical"},
      "calendar": {"epoch": {"year": 2, "season": "b", "day": 3, "time_of_day_s": 3600}, "cycle_s": 86400},
      "seasons": [
        {"name": "a", "days": 31, "doy_span": [60, 151], "day_night": {"astronomical": true},
         "temp_mean_c": 10.5, "temp_range_k": 10.0, "rh_mean": 0.7, "pressure_msl_hpa": 1015, "ground_temp_c": 9.0},
        {"name": "b", "days": 31, "doy_span": [152, 59], "day_night": {"day_s": 50400, "night_s": 36000},
         "temp_mean_c": 20.0, "temp_range_k": 12.0, "rh_mean": 0.6, "pressure_msl_hpa": 1010, "ground_temp_c": 15.0}
      ],
      "source": "test"
    })");
}

} // namespace

TEST_CASE("RFC 7396 appendix A merge-patch examples", "[environment][io]") {
    struct Case { const char* original; const char* patch; const char* result; };
    const Case cases[] = {
        {R"({"a":"b"})", R"({"a":"c"})", R"({"a":"c"})"},
        {R"({"a":"b"})", R"({"b":"c"})", R"({"a":"b","b":"c"})"},
        {R"({"a":"b"})", R"({"a":null})", R"({})"},
        {R"({"a":"b","b":"c"})", R"({"a":null})", R"({"b":"c"})"},
        {R"({"a":["b"]})", R"({"a":"c"})", R"({"a":"c"})"},
        {R"({"a":"c"})", R"({"a":["b"]})", R"({"a":["b"]})"},
        {R"({"a":{"b":"c"}})", R"({"a":{"b":"d","c":null}})", R"({"a":{"b":"d"}})"},
        {R"({"a":[{"b":"c"}]})", R"({"a":[1]})", R"({"a":[1]})"},
        {R"(["a","b"])", R"(["c","d"])", R"(["c","d"])"},
        {R"({"a":"b"})", R"(["c"])", R"(["c"])"},
        {R"({"a":"foo"})", "null", "null"},
        {R"({"a":"foo"})", R"("bar")", R"("bar")"},
        {R"({"e":null})", R"({"a":1})", R"({"e":null,"a":1})"},
        {R"([1,2])", R"({"a":"b","c":null})", R"({"a":"b"})"},
        {R"({})", R"({"a":{"bb":{"ccc":null}}})", R"({"a":{"bb":{}}})"},
    };
    int n = 0;
    for (const Case& c : cases) {
        ++n;
        json target = json::parse(c.original);
        io::json_merge_patch(target, json::parse(c.patch));
        INFO("RFC 7396 example " << n << ": " << c.original << " + " << c.patch);
        CHECK(target == json::parse(c.result));
    }
}

TEST_CASE("extends: chains resolve bottom-up, the user file last", "[environment][io]") {
    TempDir dir;
    dir.write("base.json", json::parse(R"({"format": "x", "keep": 1, "override": "base", "gone": 5,
        "nested": {"a": 1, "b": 2, "c": {"d": 1}}, "list": [1, 2, 3]})"));
    dir.write("mid.json", json::parse(R"({"extends": "base.json", "override": "mid", "nested": {"b": 20, "c": null}})"));
    // The user file lives in a subdirectory: its parent path is relative to ITS directory.
    dir.write("user/top.json", json::parse(R"({"extends": "../mid.json", "override": "user", "gone": null,
        "list": [9], "nested": {"e": 5}})"));

    const json r = io::load_json_with_extends(dir.path / "user" / "top.json");
    CHECK(r["keep"] == 1);                       // from base
    CHECK(r["override"] == "user");              // user last wins over mid and base
    CHECK_FALSE(r.contains("gone"));             // null deletes
    CHECK(r["list"] == json::parse("[9]"));      // arrays replace
    CHECK(r["nested"]["a"] == 1);                // untouched member merges
    CHECK(r["nested"]["b"] == 20);               // mid over base
    CHECK_FALSE(r["nested"].contains("c"));      // mid deleted base's nested object
    CHECK(r["nested"]["e"] == 5);                // user added
    CHECK_FALSE(r.contains("extends"));

    // Loading the middle file alone shows the intermediate value (the chain is not mutated by later loads).
    const json m = io::load_json_with_extends(dir.path / "mid.json");
    CHECK(m["override"] == "mid");
    CHECK(m["gone"] == 5);
}

TEST_CASE("extends: cycles, depth limit and malformed files", "[environment][io]") {
    TempDir dir;
    dir.write("a.json", json::parse(R"({"extends": "b.json"})"));
    dir.write("b.json", json::parse(R"({"extends": "c.json"})"));
    dir.write("c.json", json::parse(R"({"extends": "a.json"})"));
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "a.json"), ContainsSubstring("extends cycle"));
    dir.write("self.json", json::parse(R"({"extends": "self.json"})"));
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "self.json"), ContainsSubstring("extends cycle"));

    // A chain of 9 files -> 8 parents is accepted, 10 files -> 9 parents is not (limit 8).
    for (int i = 0; i < 12; ++i) {
        json j = json::object();
        j["v" + std::to_string(i)] = i;
        if (i + 1 < 12) j["extends"] = "chain" + std::to_string(i + 1) + ".json";
        dir.write("chain" + std::to_string(i) + ".json", j);
    }
    const json ok = io::load_json_with_extends(dir.path / "chain3.json");   // chain3 .. chain11: 8 parents
    CHECK(ok["v11"] == 11);
    CHECK(ok["v3"] == 3);
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "chain2.json"), ContainsSubstring("deeper than 8"));
    CHECK(io::load_json_with_extends(dir.path / "chain2.json", 9)["v11"] == 11);

    dir.write("missing_parent.json", json::parse(R"({"extends": "nope.json"})"));
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "missing_parent.json"), ContainsSubstring("cannot open"));
    dir.write_text("bad.json", "{ not json");
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "bad.json"), ContainsSubstring("invalid JSON"));
    dir.write_text("array.json", "[1, 2]");
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "array.json"), ContainsSubstring("JSON object"));
    dir.write("ext_num.json", json::parse(R"({"extends": 3})"));
    CHECK_THROWS_WITH(io::load_json_with_extends(dir.path / "ext_num.json"), ContainsSubstring("non-empty string"));
}

TEST_CASE("Climate loader: the shipped test file and a full round trip", "[environment][io]") {
    const fs::path shipped = fs::path(PS_SOURCE_DIR) / "data" / "climates" / "test_fixed_summer.json";
    const ClimateDesc d = io::load_climate_json(shipped);
    CHECK(d.location.latitude_deg == 50.14988);
    CHECK(d.sun_mode == SunMode::Astronomical);
    REQUIRE(d.seasons.size() == 4);
    CHECK(d.seasons[0].name == "spring");
    CHECK(d.seasons[0].has_doy_span);
    CHECK(d.seasons[3].doy_start == 335);
    CHECK(d.seasons[3].doy_end == 59);            // wraps over the new year
    CHECK(d.seasons[1].day_night.kind == DayNightKind::Astronomical);
    CHECK(d.calendar.epoch.season == "summer");
    CHECK_FALSE(d.seasons[0].climate.has_value());   // no invented scalars
    CHECK_FALSE(d.source.empty());

    // E2 with the shipped file: fixed to summer, every day carries the summer mid values.
    CalendarOptions opt;
    opt.mode = CalendarMode::FixedSeason;
    opt.fixed_season = 1;
    const Calendar cal(d, opt);
    for (int i = 1; i < cal.days_per_year(); ++i) {
        CHECK(cal.day(i).day_s == cal.day(0).day_s);
        CHECK(cal.day(i).y_start == cal.day(0).y_start);
    }
    CHECK(cal.day(0).day_s / 3600.0 > 15.5);      // ~16 h of daylight at 50 N in mid-summer

    // Round trip of every field.
    TempDir dir;
    const fs::path p = dir.write("full.json", valid_climate_json());
    const ClimateDesc f = io::load_climate_json(p);
    CHECK(f.location.elevation_m == 150.0);
    CHECK(f.calendar.epoch.year == 2);
    CHECK(f.calendar.epoch.season == "b");
    CHECK(f.calendar.epoch.day == 3);
    CHECK(f.calendar.epoch.time_of_day_s == 3600.0);
    CHECK(f.seasons[0].climate->temp_mean_c == 10.5);
    CHECK(f.seasons[0].climate->pressure_msl_hpa == 1015.0);
    CHECK(f.seasons[1].day_night.kind == DayNightKind::Explicit);
    CHECK(f.seasons[1].day_night.day_s == 50400.0);
    CHECK(f.seasons[1].doy_start == 152);
    CHECK(f.source == "test");
    CHECK_NOTHROW(Calendar(f));

    // extends: a child file changes one location field and replaces the seasons array.
    dir.write("child.json", json::parse(R"({"extends": "full.json", "location": {"latitude_deg": 70.0}, "source": null})"));
    const ClimateDesc c = io::load_climate_json(dir.path / "child.json");
    CHECK(c.location.latitude_deg == 70.0);
    CHECK(c.location.longitude_deg == 8.45);       // merged, not replaced
    CHECK(c.seasons.size() == 2);
    CHECK(c.source.empty());                       // null deleted it
}

TEST_CASE("Climate loader rejections", "[environment][io]") {
    TempDir dir;
    struct Row { const char* what; std::function<void(json&)> mutate; const char* message; };
    const Row rows[] = {
        {"wrong format", [](json& j) { j["format"] = "physics_sim.climate/2"; }, "\"format\" must be"},
        {"missing format", [](json& j) { j.erase("format"); }, "missing required field \"format\""},
        {"unknown top-level key", [](json& j) { j["wind"] = 1; }, "unknown key \"wind\""},
        {"unknown season key", [](json& j) { j["seasons"][0]["rain"] = 1; }, "unknown key \"rain\""},
        {"unknown location key", [](json& j) { j["location"]["altitude"] = 1; }, "unknown key \"altitude\""},
        {"missing location", [](json& j) { j.erase("location"); }, "\"location\""},
        {"latitude out of range", [](json& j) { j["location"]["latitude_deg"] = 91.0; }, "latitude_deg must be in [-90, 90]"},
        {"longitude out of range", [](json& j) { j["location"]["longitude_deg"] = 181.0; }, "longitude_deg"},
        {"latitude not a number", [](json& j) { j["location"]["latitude_deg"] = "north"; }, "\"latitude_deg\" must be a number"},
        {"no seasons", [](json& j) { j["seasons"] = json::array(); }, "seasons must not be empty"},
        {"seasons not an array", [](json& j) { j["seasons"] = 3; }, "\"seasons\" must be an array"},
        {"season days zero", [](json& j) { j["seasons"][0]["days"] = 0; }, "days must be >= 1"},
        {"season days fractional", [](json& j) { j["seasons"][0]["days"] = 2.5; }, "\"days\" must be an integer"},
        {"duplicate season name", [](json& j) { j["seasons"][1]["name"] = "a"; }, "is not unique"},
        {"empty season name", [](json& j) { j["seasons"][1]["name"] = ""; }, "name must not be empty"},
        {"doy_span zero", [](json& j) { j["seasons"][0]["doy_span"] = json::array({0, 151}); }, "doy_span"},
        {"doy_span 367", [](json& j) { j["seasons"][0]["doy_span"] = json::array({60, 367}); }, "doy_span"},
        {"doy_span wrong shape", [](json& j) { j["seasons"][0]["doy_span"] = json::array({60}); }, "doy_span"},
        {"astronomical sun without doy_span", [](json& j) { j["seasons"][0].erase("doy_span"); }, "no doy_span"},
        {"day_s zero", [](json& j) { j["seasons"][1]["day_night"]["day_s"] = 0; }, "day_s and night_s must be > 0"},
        {"night_s negative", [](json& j) { j["seasons"][1]["day_night"]["night_s"] = -1; }, "day_s and night_s must be > 0"},
        {"both day/night forms", [](json& j) { j["seasons"][1]["day_night"]["cycle_s"] = 86400; }, "more than one day/night form"},
        {"empty day_night", [](json& j) { j["seasons"][1]["day_night"] = json::object(); }, "must give one form"},
        {"astronomical false", [](json& j) { j["seasons"][0]["day_night"]["astronomical"] = false; }, "must be true"},
        {"day_fraction 1", [](json& j) { j["seasons"][1]["day_night"] = json::parse(R"({"cycle_s": 86400, "day_fraction": 1.0})"); }, "day_fraction must be in (0, 1)"},
        {"cycle_s zero", [](json& j) { j["calendar"]["cycle_s"] = 0; }, "cycle_s must be > 0"},
        {"negative temp range", [](json& j) { j["seasons"][0]["temp_range_k"] = -1.0; }, "temp_range_k must be >= 0"},
        {"rh above 1", [](json& j) { j["seasons"][0]["rh_mean"] = 1.5; }, "rh_mean must be in [0, 1]"},
        {"pressure zero", [](json& j) { j["seasons"][0]["pressure_msl_hpa"] = 0; }, "pressure_msl_hpa must be > 0"},
        {"partial climate scalars", [](json& j) { j["seasons"][1].erase("temp_mean_c"); }, "required field \"temp_mean_c\""},
        {"climate on one season only", [](json& j) {
             for (const char* k : {"temp_mean_c", "temp_range_k", "rh_mean", "pressure_msl_hpa", "ground_temp_c"})
                 j["seasons"][1].erase(k); }, "every season or on none"},
        {"epoch season unknown", [](json& j) { j["calendar"]["epoch"]["season"] = "monsoon"; }, "calendar.epoch.season"},
        {"epoch day out of range", [](json& j) { j["calendar"]["epoch"]["day"] = 31; }, "calendar.epoch.day"},
        {"epoch time negative", [](json& j) { j["calendar"]["epoch"]["time_of_day_s"] = -1; }, "time_of_day_s"},
        {"sun mode unknown", [](json& j) { j["sun"]["mode"] = "lunar"; }, "\"mode\" must be"},
        {"authored mode without block", [](json& j) { j["sun"]["mode"] = "authored"; }, "authored sun"},
        {"authored elevation out of range", [](json& j) {
             j["sun"]["mode"] = "authored";
             j["sun"]["authored"] = json::parse(R"({"e_max_deg": 100, "e_night_deg": 20, "az_rise_deg": 90, "az_set_deg": 270})"); },
         "e_max_deg must be in [-90, 90]"},
        // W3 lifted the regimes / lags_s reservation (weather rejections: test_environment_weather_io.cpp).
        {"regimes without transitions", [](json& j) { j["seasons"][0]["regimes"] = json::object(); }, "missing required field \"transitions\""},
        {"lags_s incomplete", [](json& j) { j["lags_s"] = json::object(); }, "missing required field \"cloud\""},
        {"gusts reserved for W4", [](json& j) { j["gusts"] = json::object(); }, "not implemented before W4"},
        {"condition reserved for W5", [](json& j) { j["condition"] = json::object(); }, "not implemented before W5"},
    };
    int n = 0;
    for (const Row& row : rows) {
        json j = valid_climate_json();
        row.mutate(j);
        const fs::path p = dir.write("row" + std::to_string(n++) + ".json", j);
        INFO("rejection row: " << row.what);
        CHECK_THROWS_WITH(io::load_climate_json(p), ContainsSubstring(row.message));
    }

    // The unmutated document loads (the rows above fail for their own reason only).
    CHECK_NOTHROW(io::load_climate_json(dir.write("ok.json", valid_climate_json())));

    // extends cycle through the climate loader.
    dir.write("loop_a.json", json::parse(R"({"extends": "loop_b.json"})"));
    dir.write("loop_b.json", json::parse(R"({"extends": "loop_a.json"})"));
    CHECK_THROWS_WITH(io::load_climate_json(dir.path / "loop_a.json"), ContainsSubstring("extends cycle"));
    // An error message names the file.
    const fs::path bad = dir.write("named.json", [] { json j = valid_climate_json(); j["wind"] = 1; return j; }());
    CHECK_THROWS_WITH(io::load_climate_json(bad), ContainsSubstring("named.json"));
}
