// tests/unit/test_environment_surface_classes.cpp - the shipped data/surface_conditions/default.json (library level,
// no ps_core): every class loads, every number of every non-static class carries a provenance mark, the seven road and
// off-road classes added 2026-10-05 exist with the right kinds, and a short grid run with ALL classes present stays
// finite and inside a plausible range. Tags [environment][surface][io].
//
// What the checks are, and why the data file needs them (docs/data_formats/surface_conditions.md, CLAUDE.md "[C] cited
// and read, [D] derived, [M] from memory, [AS] assumed"):
//  C1 the file loads and holds exactly the 13 classes (asphalt, cobblestone, compacted, concrete, dirt, field, grass,
//     gravel, kerb, low_mu, paving_stones, sand, sett);
//  C2 the seven new classes have the kinds the racing_game request fixed (concrete/paving_stones/sett/cobblestone
//     sealed, gravel/compacted/sand soil) and every field their kind requires;
//  C3 provenance. The source of every non-static class is an object whose strings all start with [C], [D], [M] or
//     [AS]. Old classes: each field GROUP (albedo, emissivity, layer, deep, mtd_mm + drainage | soil [+ canopy],
//     friction) needs a source key (the key or a "key.sub" form). The seven new classes: each numeric LEAF needs one
//     (layer.k_w_mk, soil.theta_fc, friction.wet, ...; a key such as "deep" covers its sub-fields). A source key that
//     names no existing field is an error too (a stale entry after a rename);
//  C4 value plausibility of the data itself (friction.damp >= friction.wet, because ConditionGrip clamps the wet level to
//     damp; theta_fc < theta_sat; the new classes' friction ratios in (0, 2]);
//  C5 a 3-day grid run with all 13 classes mapped (surface_names = the 13 names) and a diurnal forcing with a 3 h rain:
//     surface temperatures finite and between the air and air + 60 K at 14:00 (and above air - 12 K at night), film depth
//     >= 0 and > 0 on every sealed class right after the rain, soil moisture inside [0, theta_sat], and the condition
//     grip factor of every class inside (0, 1.15] (sealed < 1 when wet, sand > 1).
//
// RECORDED SABOTAGES (2026-10-05, performed by hand on data/surface_conditions/default.json, reverted):
//  - S1 delete the "mtd_mm" source entry of "sett": C3 failed with
//    "sett: numeric field "mtd_mm" has no source entry".
//  - S2 mistag one: the "friction.wet" source of "sand" starts with "[X]" instead of "[AS]": C3 failed,
//    CHECK( has_tag(...) ) with "class sand / sand: source key friction.wet" (4 of 5 cases passed, 992 of 993 assertions).
//  - S3 flip a kind: "sand" kind "soil" -> "sealed": the loader itself rejects the file, so C1, C2, C4 and C5 all failed
//    (4 of 5 cases) with 'default.json: surfaces.sand: missing required field "mtd_mm"' (only C3, which reads the JSON
//    directly, still ran). C2's own kind CHECK is therefore shadowed by the loader's rejection for this particular flip.

#include "ps/environment/condition_grip.h"
#include "ps/environment/surface_condition.h"
#include "ps/environment/surface_conditions_io.h"
#include "ps/environment/tyre_profile_desc.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::environment;
using json = nlohmann::json;

namespace {

std::string default_path() { return std::string(PS_SOURCE_DIR) + "/data/surface_conditions/default.json"; }

json load_json() {
    std::ifstream in(default_path());
    REQUIRE(in.good());
    return json::parse(in);
}

const std::vector<std::string> kNewClasses = {"concrete", "paving_stones", "sett", "cobblestone", "gravel", "compacted", "sand"};

const std::vector<std::string> kAllClasses = {"asphalt", "cobblestone", "compacted", "concrete", "dirt",   "field", "grass",
                                              "gravel",  "kerb",        "low_mu",    "paving_stones", "sand", "sett"};

bool is_new_class(const std::string& n) { return std::find(kNewClasses.begin(), kNewClasses.end(), n) != kNewClasses.end(); }

bool has_tag(const std::string& s) {
    for (const char* tag : {"[C]", "[D]", "[M]", "[AS]"})
        if (s.rfind(tag, 0) == 0) return true;
    return false;
}

// A source key covers `path` when it equals it or is a dotted prefix of it ("deep" covers "deep.k_w_mk").
bool leaf_covered(const json& src, const std::string& path) {
    for (auto it = src.begin(); it != src.end(); ++it) {
        const std::string& k = it.key();
        if (path == k || path.rfind(k + ".", 0) == 0) return true;
    }
    return false;
}

// A group is covered by a key equal to it or starting with "group.".
bool group_covered(const json& src, const std::string& group) {
    for (auto it = src.begin(); it != src.end(); ++it) {
        const std::string& k = it.key();
        if (k == group || k.rfind(group + ".", 0) == 0) return true;
    }
    return false;
}

// Dotted paths of every numeric leaf of a class object (everything but "kind" and "source").
void numeric_leaves(const json& o, const std::string& prefix, std::vector<std::string>& out) {
    for (auto it = o.begin(); it != o.end(); ++it) {
        const std::string path = prefix.empty() ? it.key() : prefix + "." + it.key();
        if (prefix.empty() && (it.key() == "kind" || it.key() == "source")) continue;
        if (it->is_object()) numeric_leaves(*it, path, out);
        else if (it->is_number()) out.push_back(path);
    }
}

bool key_names_a_field(const json& cls, const std::string& key) {
    // The key (or its dotted prefix form) must lead to an existing member of the class object.
    const json* cur = &cls;
    std::size_t pos = 0;
    while (true) {
        const std::size_t dot = key.find('.', pos);
        const std::string part = key.substr(pos, dot == std::string::npos ? std::string::npos : dot - pos);
        if (!cur->is_object() || !cur->contains(part)) return false;
        cur = &(*cur)[part];
        if (dot == std::string::npos) return true;
        pos = dot + 1;
    }
}

struct DiurnalRain final : ISurfaceForcingSource {
    // [AS] a clear mid-latitude day with a 3 h rain at 10 mm/h on the third day (hours 1-4).
    real rain_day_start_s = 2.0 * 86400.0;
    SurfaceForcing forcing_at(real t) const override {
        const real h = std::fmod(t, 86400.0) / 3600.0;
        SurfaceForcing f;
        const bool raining = t >= rain_day_start_s + 1.0 * 3600.0 && t < rain_day_start_s + 4.0 * 3600.0;
        f.ghi_w_m2 = (!raining && h > 6.0 && h < 18.0) ? 800.0 * std::sin(3.14159265358979 * (h - 6.0) / 12.0) : 0.0;
        f.air_temp_k = 288.0 + 5.0 * std::sin(2.0 * 3.14159265358979 * (h - 9.0) / 24.0);
        f.wind_m_s = 3.0;
        f.vapour_pressure_hpa = 10.0;
        f.cloud_cover = raining ? 1.0 : 0.0;
        f.bottom_temp_k = 285.0;
        f.precip_mm_h = raining ? 10.0 : 0.0;
        return f;
    }
    InitialProfile initial_at(real) const override { return InitialProfile{288.0, 285.0}; }
    real air_at(real t) const { return forcing_at(t).air_temp_k; }
};

int class_index(const ConditionGrid& g, const std::string& name) {
    for (int i = 0; i < g.class_count(); ++i)
        if (g.class_name(i) == name) return i;
    return -1;
}

} // namespace

TEST_CASE("Surface classes C1: the shipped default.json loads with all 13 classes", "[environment][surface][io]") {
    const SurfaceConditionsDesc d = io::load_surface_conditions_json(default_path());
    REQUIRE(d.classes.size() == kAllClasses.size());
    // sorted by name (the loader sorts)
    for (std::size_t i = 0; i < kAllClasses.size(); ++i) CHECK(d.classes[i].name == kAllClasses[i]);
    CHECK_NOTHROW(validate_surface_conditions(d));
    CHECK(d.classes.size() <= static_cast<std::size_t>(kMaxSurfaceClasses));
}

TEST_CASE("Surface classes C2: the seven new classes exist with the right kinds and fields", "[environment][surface][io]") {
    const SurfaceConditionsDesc d = io::load_surface_conditions_json(default_path());
    const std::map<std::string, SurfaceKind> expect = {
        {"concrete", SurfaceKind::Sealed}, {"paving_stones", SurfaceKind::Sealed}, {"sett", SurfaceKind::Sealed},
        {"cobblestone", SurfaceKind::Sealed}, {"gravel", SurfaceKind::Soil}, {"compacted", SurfaceKind::Soil},
        {"sand", SurfaceKind::Soil}};
    for (const auto& [name, kind] : expect) {
        INFO(name << ": kind");
        const SurfaceClassDesc* c = d.find(name);
        REQUIRE(c != nullptr);
        CHECK(c->kind == kind);
        CHECK(c->albedo > 0.0);
        CHECK(c->albedo < 1.0);
        CHECK(c->emissivity > 0.0);
        CHECK(c->emissivity <= 1.0);
        CHECK(c->layer.depth_m > 0.0);
        CHECK(c->layer.k_w_mk > 0.0);
        CHECK(c->layer.rho_c_j_m3k > 0.0);
        CHECK(c->deep.depth_m > 0.0);
        if (kind == SurfaceKind::Sealed) {
            CHECK(c->mtd_mm > 0.0);
            CHECK(c->drainage_path_m > 0.0);
            CHECK(c->drainage_slope_m_m > 0.0);
        } else {
            CHECK(c->soil.ksat_mm_h > 0.0);
            CHECK(c->soil.theta_fc > 0.0);
            CHECK(c->soil.theta_fc < c->soil.theta_sat);
            CHECK(c->soil.theta_sat <= 1.0);
        }
    }
    // The old classes keep their kinds.
    CHECK(d.find("asphalt")->kind == SurfaceKind::Sealed);
    CHECK(d.find("kerb")->kind == SurfaceKind::Sealed);
    CHECK(d.find("dirt")->kind == SurfaceKind::Soil);
    CHECK(d.find("field")->kind == SurfaceKind::Soil);
    CHECK(d.find("grass")->kind == SurfaceKind::Vegetated);
    CHECK(d.find("low_mu")->kind == SurfaceKind::Static);
}

TEST_CASE("Surface classes C3: every number of every non-static class has a provenance mark", "[environment][surface][io]") {
    const json j = load_json();
    REQUIRE(j.contains("surfaces"));
    for (auto it = j["surfaces"].begin(); it != j["surfaces"].end(); ++it) {
        const std::string name = it.key();
        const json& cls = it.value();
        const std::string kind = cls.at("kind").get<std::string>();
        if (kind == "static") continue;
        INFO("class " << name);
        REQUIRE(cls.contains("source"));
        REQUIRE(cls["source"].is_object());
        const json& src = cls["source"];

        // every source string starts with a tag; every key names a field
        for (auto s = src.begin(); s != src.end(); ++s) {
            INFO(name << ": source key " << s.key());
            REQUIRE(s.value().is_string());
            CHECK(has_tag(s.value().get<std::string>()));   // a source string does not start with [C]/[D]/[M]/[AS]
            CHECK(key_names_a_field(cls, s.key()));
        }

        if (is_new_class(name)) {
            // each numeric leaf has a source entry
            std::vector<std::string> leaves;
            numeric_leaves(cls, "", leaves);
            REQUIRE_FALSE(leaves.empty());
            for (const std::string& leaf : leaves) {
                INFO(name << ": numeric field \"" << leaf << "\" has no source entry");
                CHECK(leaf_covered(src, leaf));
            }
        } else {
            // each field group has a source entry
            std::vector<std::string> groups = {"albedo", "emissivity", "layer", "deep", "friction"};
            if (kind == "sealed") {
                groups.push_back("mtd_mm");
                groups.push_back("drainage");
            } else {
                groups.push_back("soil");
                if (kind == "vegetated") groups.push_back("canopy");
            }
            for (const std::string& g : groups) {
                INFO(name << ": field group \"" << g << "\" has no source entry");
                CHECK(group_covered(src, g));
            }
        }
    }
}

TEST_CASE("Surface classes C4: friction levels and soil bounds of the data itself", "[environment][surface][io]") {
    const SurfaceConditionsDesc d = io::load_surface_conditions_json(default_path());
    for (const SurfaceClassDesc& c : d.classes) {
        if (c.kind == SurfaceKind::Static) continue;
        INFO("class " << c.name);
        CHECK(c.friction.damp > 0.0);
        CHECK(c.friction.damp <= 2.0);
        CHECK(c.friction.wet > 0.0);
        CHECK(c.friction.wet <= 2.0);
        // ConditionGrip clamps the wet level to damp (plan 5.2 step 3): a wet level above damp could never take effect.
        CHECK(c.friction.wet <= c.friction.damp);
        CHECK(c.friction.wet_grass > 0.0);
        CHECK(c.friction.wet_grass <= 2.0);
        if (c.kind != SurfaceKind::Sealed) {
            CHECK(c.soil.theta_fc < c.soil.theta_sat);
            CHECK(c.soil.ksat_mm_h > 0.0);
        }
        CHECK(c.layer.depth_m + c.deep.depth_m <= kMaxColumnDepthM);
    }
    // Sand is the one class meant to grip better damp than dry (ratio above 1); no sealed class is.
    const SurfaceClassDesc* sand = d.find("sand");
    REQUIRE(sand != nullptr);
    CHECK(sand->friction.wet > 1.0);
    for (const SurfaceClassDesc& c : d.classes)
        if (c.kind == SurfaceKind::Sealed) CHECK(c.friction.wet < 1.0);
}

TEST_CASE("Surface classes C5: a grid run with all 13 classes stays finite and plausible", "[environment][surface][io]") {
    const SurfaceConditionsDesc d = io::load_surface_conditions_json(default_path());
    ConditionGridDesc gd;
    gd.dt_env_s = 60.0;
    gd.spinup_s = 2.0 * 86400.0;
    gd.max_fast_forward_s = 30.0 * 86400.0;
    ConditionGrid grid(d, kAllClasses, gd);
    REQUIRE(grid.class_count() == 13);
    CHECK(grid.unmapped_names().empty());
    CHECK(grid.unused_entries().empty());
    const DiurnalRain src;

    // --- right after the 3 h rain: 04:10 on day 3
    const real t_rain = 2.0 * 86400.0 + 4.0 * 3600.0 + 600.0;
    REQUIRE(grid.advance_to(t_rain, src) > 0);
    for (int i = 0; i < grid.class_count(); ++i) {
        const std::string& name = grid.class_name(i);
        if (!grid.has_state(i)) continue;
        INFO("class " << name << " after rain");
        const SurfaceClassState& s = grid.next(i);
        CHECK(std::isfinite(s.water_mm));
        CHECK(s.water_mm >= 0.0);
        CHECK(std::isfinite(s.soil_moisture));
        const ColumnModel& col = grid.column(i);
        if (grid.kind(i) == SurfaceKind::Sealed) {
            CHECK(s.water_mm > 0.0);   // 30 mm of rain fills every sealed class's texture storage
            CHECK(col.w_tex_mm > 0.0);
        } else {
            CHECK(s.soil_moisture >= 0.0);
            CHECK(s.soil_moisture <= col.soil_theta_sat + 1e-12);
        }
        const real ts = grid.surface_temp_k(i, t_rain);
        CHECK(std::isfinite(ts));
        CHECK(ts > 250.0);
        CHECK(ts < 320.0);
    }

    // --- the condition grip right after the rain
    ConditionGrip grip;
    grip.build_static(grid, TyreConditionFactors{});
    grip.update_state(grid, t_rain);
    for (int i = 0; i < grid.class_count(); ++i) {
        if (!grid.has_state(i)) continue;
        INFO("class " << grid.class_name(i) << " grip factor after rain");
        const real f = grip.factor(i, 20.0);
        CHECK(std::isfinite(f));
        CHECK(f > 0.0);
        CHECK(f <= 1.15);
        if (grid.kind(i) == SurfaceKind::Sealed) CHECK(f < 1.0);
    }
    const int sand = class_index(grid, "sand");
    REQUIRE(sand >= 0);
    CHECK(grip.water_weight(sand) > 0.0);
    CHECK(grip.factor(sand, 20.0) > 1.0);

    // --- 14:00 the same day: hot sun, no rain for 10 h
    const real t_noon = 2.0 * 86400.0 + 14.0 * 3600.0;
    REQUIRE(grid.advance_to(t_noon, src) > 0);
    const real air = src.air_at(t_noon);
    for (int i = 0; i < grid.class_count(); ++i) {
        if (!grid.has_state(i)) continue;
        INFO("class " << grid.class_name(i) << " at 14:00");
        const real ts = grid.surface_temp_k(i, t_noon);
        CHECK(std::isfinite(ts));
        CHECK(ts > air);
        CHECK(ts < air + 60.0);
        const SurfaceClassState& s = grid.next(i);
        CHECK(s.water_mm >= 0.0);
        if (grid.kind(i) != SurfaceKind::Sealed) {
            CHECK(s.soil_moisture >= 0.0);
            CHECK(s.soil_moisture <= grid.column(i).soil_theta_sat + 1e-12);
        }
    }

    // --- 23:30: clear night, the surfaces lose heat but stay near the air
    const real t_night = 2.0 * 86400.0 + 23.5 * 3600.0;
    REQUIRE(grid.advance_to(t_night, src) > 0);
    const real air_n = src.air_at(t_night);
    for (int i = 0; i < grid.class_count(); ++i) {
        if (!grid.has_state(i)) continue;
        INFO("class " << grid.class_name(i) << " at 23:30");
        const real ts = grid.surface_temp_k(i, t_night);
        CHECK(std::isfinite(ts));
        CHECK(ts > air_n - 12.0);
        CHECK(ts < air_n + 15.0);
    }
}

TEST_CASE("Surface classes diag: per-class state of the C5 run (printed)", "[.][diag][environment][surface][io]") {
    const SurfaceConditionsDesc d = io::load_surface_conditions_json(default_path());
    ConditionGridDesc gd;
    gd.dt_env_s = 60.0;
    gd.spinup_s = 2.0 * 86400.0;
    ConditionGrid grid(d, kAllClasses, gd);
    const DiurnalRain src;
    const real t_rain = 2.0 * 86400.0 + 4.0 * 3600.0 + 600.0;
    grid.advance_to(t_rain, src);
    ConditionGrip grip;
    grip.build_static(grid, TyreConditionFactors{});
    grip.update_state(grid, t_rain);
    std::printf("%-14s %9s %9s %8s %8s\n", "class", "water_mm", "theta", "ww", "f(20m/s)");
    for (int i = 0; i < grid.class_count(); ++i)
        std::printf("%-14s %9.3f %9.4f %8.3f %8.3f\n", grid.class_name(i).c_str(), grid.next(i).water_mm, grid.next(i).soil_moisture,
                    grip.water_weight(i), grip.factor(i, 20.0));
    for (real h : {14.0, 23.5}) {
        const real t = 2.0 * 86400.0 + h * 3600.0;
        grid.advance_to(t, src);
        std::printf("t = %.1f h, air %.2f K\n", h, src.air_at(t));
        for (int i = 0; i < grid.class_count(); ++i)
            if (grid.has_state(i))
                std::printf("  %-14s Ts %.2f K (%+.2f vs air) water %.3f mm theta %.4f\n", grid.class_name(i).c_str(),
                            grid.surface_temp_k(i, t), grid.surface_temp_k(i, t) - src.air_at(t), grid.next(i).water_mm,
                            grid.next(i).soil_moisture);
    }
}
