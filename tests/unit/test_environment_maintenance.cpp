// tests/unit/test_environment_maintenance.cpp - winter maintenance of the road tiers (road_t1..road_t6): the plough and
// the salt of a surface class's optional "maintenance" block (docs/data_formats/surface_conditions.md "Maintenance
// (road tiers)"). Library level (no ps_core); the cases drive step_column directly with explicit forcing. Tags
// [environment][surface][maintenance].
//
//  M1 a pack that lies above the residual is ploughed when its clock reaches plough_after_s: the pack equals
//     residual_mm exactly, the clock restarts, salt_left_s = duration_s.
//  M2 a class without maintenance (asphalt) is never ploughed and is bit-identical to a copy with the block removed;
//     its hash does not fold the maintenance state; a pinned pack evolution recorded on the code before the
//     maintenance change (2026-11-02, master 88bbc66).
//  M3 salt: the melt of one step equals the predicted m effect ([AS] tolerance 1e-9 relative: it is the same
//     closed expression), is zero without salt below 0 C, equals the no-salt melt where the temperature fade is 0.
//  M4 dt invariance: dt 60 s and dt 1 s clear the pack at the same environment time (within one 60 s step).
//  M5 snow that keeps falling is ploughed again each plough_after_s (two ploughs in 2.2 plough periods).
//  M6 loader: the block is read, rejects unknown keys and bad numbers; the six tiers carry the owner's times.
//
// RECORDED SABOTAGES (performed by hand, reverted, 2026-11-02):
//  - plough removed: M1 and M5 fail;
//  - salt temperature fade removed (eff = 1): M3 fails;
//  - clock not restarted on a plough: M5 fails.

#include "ps/environment/surface_condition.h"
#include "ps/environment/surface_conditions_io.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::environment;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using json = nlohmann::json;

namespace {

std::string data_path() { return std::string(PS_SOURCE_DIR) + "/data/surface_conditions/default.json"; }

SurfaceConditionsDesc shipped() { return io::load_surface_conditions_json(data_path()); }

SurfaceClassDesc cls(const char* name) {
    const SurfaceConditionsDesc d = shipped();
    const SurfaceClassDesc* c = d.find(name);
    REQUIRE(c != nullptr);
    return *c;
}

SurfaceForcing forcing(real air_c, real snow_mm_h = 0.0) {
    SurfaceForcing f;
    f.air_temp_k = air_c + kCelsiusToKelvin;
    f.bottom_temp_k = f.air_temp_k;
    f.wind_m_s = 2.0;
    f.vapour_pressure_hpa = 3.0;
    f.cloud_cover = 0.5;
    f.ghi_w_m2 = 0.0;
    f.snow_mm_h = snow_mm_h;
    return f;
}

// A state with the whole column at `surface_c` and a snow pack of `snow_mm`.
SurfaceClassState state_at(const ColumnModel& col, real surface_c, real snow_mm) {
    SurfaceClassState s;
    initial_column_state(col, InitialProfile{surface_c + kCelsiusToKelvin, surface_c + kCelsiusToKelvin}, s);
    s.snow_mm = snow_mm;
    s.snow_density_kg_m3 = 200.0;
    return s;
}

// The melt (mm) one step of length dt takes from a given state.
real one_step_melt(const ColumnModel& col, const SurfaceForcing& f, real dt, SurfaceClassState s) {
    const real before = s.snow_mm + (f.snow_mm_h > 0.0 ? f.snow_mm_h / 3600.0 * dt : 0.0);
    step_column(col, f, dt, s, s);
    return before - s.snow_mm;
}

constexpr std::uint64_t kFnvBasis = 0xcbf29ce484222325ULL;

std::uint64_t bits_of(double v) {
    std::uint64_t u;
    std::memcpy(&u, &v, sizeof u);
    return u;
}

} // namespace

TEST_CASE("M1 maintenance: the plough leaves the residual after plough_after_s and lays salt", "[environment][surface][maintenance]") {
    const SurfaceClassDesc d = cls("road_t1");
    REQUIRE(d.maintenance.present);
    REQUIRE(d.maintenance.salt.present);
    const ColumnModel col = build_column(d);
    SurfaceClassState s = state_at(col, -5.0, 2.0);
    const SurfaceForcing f = forcing(-5.0);   // frozen: nothing melts, nothing falls
    const real dt = 60.0;
    const int steps = static_cast<int>(d.maintenance.plough_after_s / dt);   // 180
    REQUIRE(steps == 180);
    for (int n = 1; n < steps; ++n) {
        step_column(col, f, dt, s, s);
        REQUIRE_THAT(s.maint_clock_s, WithinAbs(n * dt, 1e-9));   // the clock accumulates the env dt
        REQUIRE(s.snow_mm == 2.0);
        REQUIRE(s.salt_left_s == 0.0);
    }
    step_column(col, f, dt, s, s);   // the plough step
    CHECK(s.snow_mm == d.maintenance.residual_mm);
    CHECK(s.maint_clock_s == 0.0);
    CHECK(s.salt_left_s == d.maintenance.salt.duration_s);
    // The residual does not lie above itself: the clock stays 0 and the salt time runs down.
    step_column(col, f, dt, s, s);
    CHECK(s.snow_mm == d.maintenance.residual_mm);
    CHECK(s.maint_clock_s == 0.0);
    CHECK(s.salt_left_s == d.maintenance.salt.duration_s - dt);
}

TEST_CASE("M2 maintenance: a class without a maintenance block is never ploughed and hashes as before",
          "[environment][surface][maintenance]") {
    const SurfaceClassDesc asphalt = cls("asphalt");
    CHECK_FALSE(asphalt.maintenance.present);
    // road_t1 with the block removed is asphalt's numbers: the pack evolution must be bit-identical, however long it lies.
    SurfaceClassDesc stripped = cls("road_t1");
    stripped.maintenance = MaintenanceDesc{};
    const ColumnModel ca = build_column(asphalt);
    const ColumnModel cs = build_column(stripped);
    SurfaceClassState a = state_at(ca, -5.0, 2.0), b = state_at(cs, -5.0, 2.0);
    SurfaceForcing f = forcing(-5.0, 0.4);
    for (int n = 0; n < 3 * 180; ++n) {
        if (n == 360) f = forcing(3.0);   // then a warm spell melts it
        step_column(ca, f, 60.0, a, a);
        step_column(cs, f, 60.0, b, b);
        REQUIRE(a.snow_mm == b.snow_mm);
        REQUIRE(a.temp_k[0] == b.temp_k[0]);
        REQUIRE(a.maint_clock_s == 0.0);
        REQUIRE(a.salt_left_s == 0.0);
        if (n < 360) REQUIRE(a.snow_mm > 2.0);   // 6 h of snow on 2 mm: nothing removed it
    }
    // Pinned against the code BEFORE the maintenance change (master 88bbc66, 2026-11-02, asphalt: -5 C air with 0.4
    // mm/h of snow on a 2 mm pack for 6 h, then +3 C for 3 h; bit patterns of the final state).
    {
        SurfaceClassState p;
        initial_column_state(ca, InitialProfile{268.15, 268.15}, p);
        p.snow_mm = 2.0;
        p.snow_density_kg_m3 = 200.0;
        SurfaceForcing pf = forcing(-5.0, 0.4);
        for (int n = 0; n < 540; ++n) {
            if (n == 360) pf = forcing(3.0);
            step_column(ca, pf, 60.0, p, p);
        }
        CHECK(bits_of(p.snow_mm) == 0x400733333333338cULL);
        CHECK(bits_of(p.temp_k[0]) == 0x4070c5b3ce468c51ULL);
        CHECK(bits_of(p.water_mm) == 0x3fd999999999999aULL);
        CHECK(bits_of(p.snow_density_kg_m3) == 0x4064bfd8b9f62f68ULL);
    }
    // The grid hash does not fold the maintenance state of a class without the block...
    ConditionGrid g1(shipped(), {"asphalt", "grass"}, ConditionGridDesc{});
    ConditionCheckpoint ck;
    ck.valid = true;
    ck.class_count = 2;
    ck.dt_env_s = ConditionGridDesc{}.dt_env_s;
    ck.grid_index = 5;
    ck.prev[0] = state_at(ca, 4.0, 1.0);
    ck.next[0] = ck.prev[0];
    ck.prev[1] = state_at(build_column(cls("grass")), 4.0, 0.0);
    ck.prev[1].snow_density_kg_m3 = 0.0;   // as the recorded baseline state
    ck.next[1] = ck.prev[1];
    g1.restore(ck);
    const std::uint64_t h0 = g1.fold_hash(kFnvBasis);
    CHECK(h0 == 0xd58e237583b5d4ddULL);   // recorded on master 88bbc66 for the checkpoint above (asphalt + grass)
    ck.prev[0].maint_clock_s = 123.0;
    ck.prev[0].salt_left_s = 456.0;
    g1.restore(ck);
    CHECK(g1.fold_hash(kFnvBasis) == h0);
    // ... while a road tier does.
    ConditionGrid g2(shipped(), {"road_t1"}, ConditionGridDesc{});
    ConditionCheckpoint c2;
    c2.valid = true;
    c2.class_count = 1;
    c2.dt_env_s = ConditionGridDesc{}.dt_env_s;
    c2.prev[0] = state_at(build_column(cls("road_t1")), 4.0, 1.0);
    c2.next[0] = c2.prev[0];
    g2.restore(c2);
    const std::uint64_t h2 = g2.fold_hash(kFnvBasis);
    c2.next[0].maint_clock_s = 7.0;
    g2.restore(c2);
    CHECK(g2.fold_hash(kFnvBasis) != h2);
    c2.next[0].maint_clock_s = 0.0;
    c2.next[0].salt_left_s = 7.0;
    g2.restore(c2);
    CHECK(g2.fold_hash(kFnvBasis) != h2);
}

TEST_CASE("M3 maintenance: salt multiplies the melt with a temperature fade", "[environment][surface][maintenance]") {
    const SurfaceClassDesc d = cls("road_t1");
    const SaltDesc& salt = d.maintenance.salt;
    const ColumnModel col = build_column(d);
    const real dt = 1.0;
    // Closed-form prediction of one step (the melt is evaluated at the OLD surface temperature t0).
    auto predicted = [&](real t_c, real air_c, bool salted) {
        real m = 1.0, dep = 0.0;
        if (salted) {
            const real eff = std::clamp((t_c - salt.ineffective_below_c) / (salt.effective_above_c - salt.ineffective_below_c),
                                        real(0.0), real(1.0));
            m = 1.0 + (salt.melt_factor - 1.0) * eff;
            dep = eff * salt.melt_point_depression_k;
        }
        const real below = (t_c + dep) > 0.0 ? m * kSnowGroundContactWm2K * (t_c + dep) * dt / kLatentHeatFusion : 0.0;
        const real above = air_c > 0.0 ? m * kSnowDegreeDayMmPerKDay / 86400.0 * air_c * dt : 0.0;
        return below + above;
    };
    auto melt = [&](real t_c, real air_c, bool salted) {
        SurfaceClassState s = state_at(col, t_c, 1.0);
        s.salt_left_s = salted ? 1.0e5 : 0.0;
        SurfaceForcing f = forcing(air_c);
        f.bottom_temp_k = t_c + kCelsiusToKelvin;
        return one_step_melt(col, f, dt, s);
    };
    // -4 C surface (fully effective): the melt point is lowered, salted melts, unsalted does not (below 0 C, air at -4).
    const real m_salt = melt(-4.0, -4.0, true);
    CHECK(melt(-4.0, -4.0, false) == 0.0);
    CHECK(m_salt > 0.0);
    CHECK_THAT(m_salt, WithinRel(predicted(-4.0, -4.0, true), 1e-9));
    // Where salt is neutral (the fade is 0 at -12 C) the melt is the unsalted one, to 1e-9 relative: air +2 C gives
    // a degree-day melt.
    const real a_off = melt(-12.0, 2.0, false);
    CHECK(a_off > 0.0);
    CHECK_THAT(melt(-12.0, 2.0, true), WithinRel(a_off, 1e-9));
    // Mid fade (-7.5 C, eff 0.5, m = 2): the degree-day term doubles; no ground melt (T + 0.5*5 < 0).
    CHECK_THAT(melt(-7.5, 2.0, true) / melt(-7.5, 2.0, false), WithinRel(2.0, 1e-9));
    CHECK_THAT(melt(-7.5, 2.0, true), WithinRel(predicted(-7.5, 2.0, true), 1e-9));
    // Above freezing (surface +1 C, fully effective): both melt terms are scaled by m = 3 and the melt point is lowered.
    CHECK_THAT(melt(1.0, 2.0, true), WithinRel(predicted(1.0, 2.0, true), 1e-9));
    // Without the depression key the effect above freezing is exactly the factor m on each term.
    SurfaceClassDesc plain = d;
    plain.maintenance.salt.melt_point_depression_k = 0.0;
    const ColumnModel cp = build_column(plain);
    auto melt_plain = [&](bool salted) {
        SurfaceClassState s = state_at(cp, 1.0, 1.0);
        s.salt_left_s = salted ? 1.0e5 : 0.0;
        SurfaceForcing f = forcing(2.0);
        f.bottom_temp_k = 1.0 + kCelsiusToKelvin;
        return one_step_melt(cp, f, dt, s);
    };
    CHECK_THAT(melt_plain(true) / melt_plain(false), WithinRel(salt.melt_factor, 1e-9));
    // Salt that has run out acts no more.
    SurfaceClassState s = state_at(col, -4.0, 1.0);
    s.salt_left_s = 0.5;
    SurfaceForcing f = forcing(-4.0);
    step_column(col, f, dt, s, s);
    CHECK(s.salt_left_s == 0.0);
}

TEST_CASE("M4 maintenance: the clearing time does not depend on the step", "[environment][surface][maintenance]") {
    const SurfaceClassDesc d = cls("road_t2");
    const ColumnModel col = build_column(d);
    const SurfaceForcing f = forcing(-6.0);
    auto clear_time = [&](real dt) {
        SurfaceClassState s = state_at(col, -6.0, 3.0);
        const int max_steps = static_cast<int>(2.0 * d.maintenance.plough_after_s / dt);
        for (int n = 1; n <= max_steps; ++n) {
            step_column(col, f, dt, s, s);
            if (s.snow_mm <= d.maintenance.residual_mm) return n * dt;
        }
        return real(-1.0);
    };
    const real t60 = clear_time(60.0);
    const real t1 = clear_time(1.0);
    CHECK_THAT(t60, WithinAbs(d.maintenance.plough_after_s, 60.0));
    CHECK_THAT(t1, WithinAbs(d.maintenance.plough_after_s, 1.0));
    CHECK(std::fabs(t60 - t1) <= 60.0);
}

TEST_CASE("M5 maintenance: snow that keeps falling is ploughed again", "[environment][surface][maintenance]") {
    const SurfaceClassDesc d = cls("road_t1");
    const ColumnModel col = build_column(d);
    SurfaceClassState s = state_at(col, -6.0, 0.0);
    s.snow_density_kg_m3 = 0.0;
    const SurfaceForcing f = forcing(-6.0, 2.0);   // 2 mm/h of snow
    const real dt = 60.0;
    const real span = 2.2 * d.maintenance.plough_after_s;
    std::vector<real> plough_times;
    real peak_after_first = 0.0;
    for (int n = 1; n <= static_cast<int>(span / dt); ++n) {
        const real before = s.snow_mm;
        step_column(col, f, dt, s, s);
        if (s.salt_left_s == d.maintenance.salt.duration_s) {   // set only by a plough
            plough_times.push_back(n * dt);
            CHECK(s.snow_mm == d.maintenance.residual_mm);
            CHECK(before > s.snow_mm);
            CHECK(s.maint_clock_s == 0.0);
        }
        if (plough_times.size() == 1) peak_after_first = std::max(peak_after_first, s.snow_mm);
    }
    REQUIRE(plough_times.size() == 2);
    // The pack accumulates 2 mm/h: the first plough comes plough_after_s after it first exceeded the residual (15 min).
    CHECK_THAT(plough_times[0], WithinAbs(d.maintenance.plough_after_s + 900.0, 2.0 * dt));
    CHECK_THAT(plough_times[1] - plough_times[0], WithinAbs(d.maintenance.plough_after_s, dt));
    // Between ploughs the pack grew from the residual by about the fall over plough_after_s.
    CHECK(peak_after_first > d.maintenance.residual_mm + 2.0 * (d.maintenance.plough_after_s / 3600.0) - 0.1);
}

TEST_CASE("M6 maintenance: loader and shipped tiers", "[environment][surface][maintenance][io]") {
    const SurfaceConditionsDesc d = shipped();
    const real plough_s[6] = {10800, 16200, 21600, 28800, 36000, 43200};
    for (int i = 0; i < 6; ++i) {
        const std::string name = "road_t" + std::to_string(i + 1);
        INFO(name);
        const SurfaceClassDesc* c = d.find(name);
        REQUIRE(c != nullptr);
        CHECK(c->kind == SurfaceKind::Sealed);
        REQUIRE(c->maintenance.present);
        CHECK(c->maintenance.plough_after_s == plough_s[i]);
        CHECK(c->maintenance.residual_mm == 0.5);
        REQUIRE(c->maintenance.salt.present);
        CHECK(c->maintenance.salt.melt_factor == 3.0);
        CHECK(c->maintenance.salt.duration_s == 21600.0);
        CHECK(c->maintenance.salt.effective_above_c == -5.0);
        CHECK(c->maintenance.salt.ineffective_below_c == -10.0);
        // an asphalt copy
        CHECK(c->albedo == d.find("asphalt")->albedo);
        CHECK(c->layer.k_w_mk == d.find("asphalt")->layer.k_w_mk);
        CHECK(c->mtd_mm == d.find("asphalt")->mtd_mm);
    }
    for (const char* n : {"asphalt", "kerb", "dirt", "field", "grass", "concrete", "sand"})
        CHECK_FALSE(d.find(n)->maintenance.present);

    // Rejections, on a minimal in-memory class.
    auto base = [] {
        return json::parse(R"({
          "format": "physics_sim.surface_conditions/1",
          "surfaces": { "road_x": {
            "kind": "sealed", "albedo": 0.1, "emissivity": 0.9,
            "layer": {"depth_m": 0.2, "k_w_mk": 1.38, "rho_c_j_m3k": 2.0e6},
            "deep": {"depth_m": 1.8, "k_w_mk": 1.0, "rho_c_j_m3k": 2.2e6},
            "mtd_mm": 0.8, "drainage": {"path_m": 7.3, "slope": 0.025},
            "maintenance": { "plough_after_s": 100,
              "salt": { "melt_factor": 2, "duration_s": 50, "effective_above_c": -2, "ineffective_below_c": -8 } } } } })");
    };
    CHECK_NOTHROW(io::parse_surface_conditions_json(base(), "t"));
    const auto reject = [&](const char* what, auto mutate, const char* msg) {
        json j = base();
        mutate(j["surfaces"]["road_x"]["maintenance"]);
        INFO(what);
        CHECK_THROWS_WITH(io::parse_surface_conditions_json(j, "t"), ContainsSubstring(msg));
    };
    reject("unknown key", [](json& m) { m["frobnicate"] = 1; }, "unknown key");
    reject("unknown salt key", [](json& m) { m["salt"]["x"] = 1; }, "unknown key");
    reject("plough_after_s 0", [](json& m) { m["plough_after_s"] = 0; }, "plough_after_s");
    reject("plough_after_s missing", [](json& m) { m.erase("plough_after_s"); }, "plough_after_s");
    reject("residual negative", [](json& m) { m["residual_mm"] = -1; }, "residual_mm");
    reject("melt_factor < 1", [](json& m) { m["salt"]["melt_factor"] = 0.5; }, "melt_factor");
    reject("duration 0", [](json& m) { m["salt"]["duration_s"] = 0; }, "duration_s");
    reject("thresholds inverted", [](json& m) { m["salt"]["effective_above_c"] = -9; }, "effective_above_c");
    reject("depression negative", [](json& m) { m["salt"]["melt_point_depression_k"] = -1; }, "melt_point_depression_k");
    json st = base();
    st["surfaces"]["road_x"] = {{"kind", "static"}, {"maintenance", json::object()}};
    CHECK_THROWS_WITH(io::parse_surface_conditions_json(st, "t"), ContainsSubstring("static"));
}
