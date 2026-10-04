// tests/unit/test_environment_object.cpp - weather plan W2, library level (no
// ps_core): the Environment object (clock anchor, start position, closed-form
// evaluation, World-frame sun vector, hashed state). The World wiring is in
// test_environment_world.cpp. Tags [environment][clock].
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - W2-S1 (non-accumulating clock): Environment::begin_tick accumulating
//    `t_env_ += scale / tick_hz` per call instead of using the closed form:
//    "evaluation is the closed form at any tick, in any order" failed (a jump to
//    tick 2^40 returned the sum of the calls made so far) and the
//    scale-change continuity check failed.
//  - W2-S3 (east/north swap): evaluate() building sun_dir_world from
//    `east_world_ * dir_north + north_world_ * dir_east`: "sun direction in the
//    World frame" failed at noon (south came out as +/-X instead of -Y) and the
//    morning-is-east check failed.
//  - W2-S2 (hash coverage): fold_state_hash without clock_.scale: "hashed state
//    covers the clock anchor and mode" failed on its scale row (equal hashes for
//    scale 1 vs 30).

#include "environment_fixtures.h"

#include "ps/environment/environment.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>

using namespace ps;
using namespace ps::environment;
using namespace ps::test_env;

namespace {

bool same(const EnvironmentSnapshot& a, const EnvironmentSnapshot& b) {
    return a.t_env_s == b.t_env_s && a.time_scale == b.time_scale && a.year == b.year && a.season == b.season &&
           std::strcmp(a.season_name, b.season_name) == 0 && a.day_in_season == b.day_in_season &&
           a.day_of_year == b.day_of_year && a.time_of_day_s == b.time_of_day_s && a.day_s == b.day_s &&
           a.night_s == b.night_s && a.day_fraction == b.day_fraction && a.clock_hours == b.clock_hours &&
           a.is_day == b.is_day && a.sun_elevation_rad == b.sun_elevation_rad && a.sun_azimuth_rad == b.sun_azimuth_rad &&
           a.sun_dir_world.x == b.sun_dir_world.x && a.sun_dir_world.y == b.sun_dir_world.y &&
           a.sun_dir_world.z == b.sun_dir_world.z && a.ghi_w_m2 == b.ghi_w_m2 && a.cloud_cover == b.cloud_cover;
}

} // namespace

static_assert(std::is_trivially_copyable_v<EnvironmentSnapshot>, "the snapshot is copied into every WorldSnapshot slot allocation-free");

TEST_CASE("Environment: evaluation is the closed form at any tick, in any order", "[environment][clock]") {
    EnvironmentConfig cfg;
    cfg.time_scale = 60.0;
    Environment env(astronomical_four_seasons(), cfg);
    env.attach(1000, 240.0);
    CHECK(env.current().t_env_s == env.start_env_s());

    const std::int64_t ticks[] = {1000, 1240, 5000, std::int64_t(1) << 40, 1001, 999, 1000};
    for (const std::int64_t tick : ticks) {
        env.begin_tick(tick);
        const real expect = env.start_env_s() + 60.0 * (static_cast<real>(tick - 1000) / 240.0);
        INFO("tick " << tick);
        CHECK(env.current().t_env_s == expect); // bit-exact: the closed form of EnvClock
        CHECK(same(env.current(), env.evaluate(expect)));
    }
    // Evaluation order does not matter.
    const EnvironmentSnapshot a = env.evaluate(12345.678);
    (void)env.evaluate(99999.0);
    CHECK(same(a, env.evaluate(12345.678)));
}

TEST_CASE("Environment: scale 0 freezes time and sun; a scale change does not jump", "[environment][clock]") {
    EnvironmentConfig cfg;
    cfg.time_scale = 0.0;
    Environment env(astronomical_four_seasons(), cfg);
    env.attach(0, 240.0);
    const EnvironmentSnapshot first = env.current();
    env.begin_tick(240 * 3600);
    CHECK(same(first, env.current()));
    CHECK(env.current().time_scale == 0.0);

    // set_time_scale re-anchors at the value the OLD scale gave at that tick.
    EnvironmentConfig cfg2;
    cfg2.time_scale = 30.0;
    Environment e2(astronomical_four_seasons(), cfg2);
    e2.attach(0, 240.0);
    e2.begin_tick(2400);
    const real before = e2.current().t_env_s; // 300 s
    e2.set_time_scale(1000.0, 2400);
    CHECK(e2.current().t_env_s == before);
    e2.begin_tick(2400 + 240);
    CHECK_THAT(e2.current().t_env_s, Catch::Matchers::WithinAbs(before + 1000.0, 1e-9));
    CHECK(e2.current().time_scale == 1000.0);
    // A jump is a re-anchor at the tick.
    e2.set_time(5.0e6, 2640);
    CHECK(e2.current().t_env_s == 5.0e6);
    e2.begin_tick(2640 + 24);
    CHECK_THAT(e2.current().t_env_s, Catch::Matchers::WithinAbs(5.0e6 + 100.0, 1e-6));
}

TEST_CASE("Environment: the start position reaches the calendar", "[environment][clock]") {
    EnvironmentConfig cfg;
    cfg.start = EnvironmentStart{0, "autumn", 3, 21600.0};
    Environment env(astronomical_four_seasons(), cfg);
    env.attach(7, 240.0);
    const EnvironmentSnapshot& s = env.current();
    CHECK(std::string(s.season_name) == "autumn");
    CHECK(s.season == 2);
    CHECK(s.day_in_season == 3);
    CHECK(s.time_of_day_s == 21600.0);
    CHECK(s.day_fraction == 0.25);
    CHECK(s.clock_hours == 6.0);

    // Default: the climate epoch (t_env 0).
    Environment def(three_season_explicit());
    def.attach(0, 240.0);
    CHECK(def.current().t_env_s == 0.0);
    CHECK(std::string(def.current().season_name) == "b"); // the fixture's epoch season
    CHECK(def.current().day_in_season == 10);
    CHECK(def.current().time_of_day_s == 20000.0);
}

TEST_CASE("Environment: config rejections", "[environment][clock]") {
    const ClimateDesc d = astronomical_four_seasons();
    EnvironmentConfig c;
    c.time_scale = -1.0;
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.time_scale = std::nan("");
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.fixed_season = "monsoon";
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.fixed_season = "summer";
    c.fixed_day_of_year = 3;
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.start = EnvironmentStart{0, "monsoon", 0, 0.0};
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.start = EnvironmentStart{0, "summer", 0, 1.0e9}; // beyond its day
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.start = EnvironmentStart{0, "summer", 99, 0.0};  // beyond its season
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.north_yaw_deg = std::nan("");
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    c = {};
    c.fixed_day_of_year = 100000;
    CHECK_THROWS_AS(Environment(d, c), std::invalid_argument);
    Environment ok(d, EnvironmentConfig{});
    CHECK_THROWS_AS(ok.attach(0, 0.0), std::invalid_argument);
    CHECK_THROWS_AS(ok.set_time_scale(-2.0, 0), std::invalid_argument);
}

TEST_CASE("Environment: fixed season and fixed day carry the calendar mode", "[environment][clock]") {
    // three_season_explicit: day_s per season a=50400, b=30000, c=43200.
    EnvironmentConfig cfg;
    cfg.fixed_season = "c";
    Environment env(three_season_explicit(), cfg);
    env.attach(0, 240.0);
    const Calendar& cal = env.calendar();
    for (int d = 0; d < cal.days_per_year(); d += 7) {
        const real t = cal.env_time(0, 0, 0, 0.0) + cal.day_start_in_year_s(d) + 10.0;
        INFO("day " << d);
        const EnvironmentSnapshot s = env.evaluate(t);
        CHECK(s.day_s == 43200.0);
        CHECK(s.night_s == 43200.0);
    }
    EnvironmentConfig cfg2;
    cfg2.fixed_day_of_year = 40;
    Environment e2(three_season_explicit(), cfg2);
    Environment dyn(three_season_explicit());
    const real day40_day_s = dyn.calendar().day(40).day_s;  // the dynamic calendar's own day 40
    CHECK(dyn.calendar().day(0).day_s != day40_day_s);      // so the fixed mode is distinguishable
    const real year_start = e2.calendar().env_time(0, 0, 0, 0.0);
    for (int d = 0; d < e2.calendar().days_per_year(); d += 11) {
        INFO("day " << d);
        CHECK(e2.evaluate(year_start + e2.calendar().day_start_in_year_s(d) + 10.0).day_s == day40_day_s);
    }
}

TEST_CASE("Environment: sun direction in the World frame", "[environment][clock][sun]") {
    // Authored sun (three_season_explicit): e_max 60 deg, azimuth 90 (rise) -> 270 (set) through south.
    // Season "b": day 30000 s, night 56400 s; solar noon = night/2 + day/2 = 43200 s into the day.
    const auto noon_of = [](const Environment& e) { return e.calendar().env_time(0, 1, 10, 43200.0); };
    {
        Environment env(three_season_explicit());
        const EnvironmentSnapshot s = env.evaluate(noon_of(env));
        const real e60 = 60.0 * kDegToRad;
        CHECK_THAT(s.sun_elevation_rad, Catch::Matchers::WithinAbs(e60, 1e-12));
        // North = +Y at yaw 0, so the noon sun (due south) is at -Y.
        CHECK_THAT(s.sun_dir_world.x, Catch::Matchers::WithinAbs(0.0, 1e-12));
        CHECK_THAT(s.sun_dir_world.y, Catch::Matchers::WithinAbs(-std::cos(e60), 1e-12));
        CHECK_THAT(s.sun_dir_world.z, Catch::Matchers::WithinAbs(std::sin(e60), 1e-12));
        // Morning sun is in the east (+X), evening in the west.
        const EnvironmentSnapshot am = env.evaluate(env.calendar().env_time(0, 1, 10, 43200.0 - 8000.0));
        const EnvironmentSnapshot pm = env.evaluate(env.calendar().env_time(0, 1, 10, 43200.0 + 8000.0));
        CHECK(am.sun_dir_world.x > 0.1);
        CHECK(pm.sun_dir_world.x < -0.1);
        // Unit length everywhere, below the horizon at night.
        for (real tod : {100.0, 5000.0, 30000.0, 43200.0, 60000.0, 80000.0}) {
            const EnvironmentSnapshot q = env.evaluate(env.calendar().env_time(0, 1, 10, tod));
            CHECK_THAT(q.sun_dir_world.length(), Catch::Matchers::WithinAbs(1.0, 1e-12));
            CHECK_THAT(q.sun_dir_world.z, Catch::Matchers::WithinAbs(std::sin(q.sun_elevation_rad), 1e-12));
        }
        const EnvironmentSnapshot night = env.evaluate(env.calendar().env_time(0, 1, 10, 100.0));
        CHECK(night.sun_dir_world.z < 0.0);
        CHECK(night.ghi_w_m2 == 0.0);
        CHECK_FALSE(night.is_day);
        CHECK(s.ghi_w_m2 > 800.0);
        CHECK(s.ghi_w_m2 == ghi_w_m2(s.sun_elevation_rad, 0.0));
    }
    {
        // north_yaw 90: north = -X (CCW from +Y), so the noon sun (south) is at +X.
        EnvironmentConfig cfg;
        cfg.north_yaw_deg = 90.0;
        Environment env(three_season_explicit(), cfg);
        const EnvironmentSnapshot s = env.evaluate(noon_of(env));
        const real e60 = 60.0 * kDegToRad;
        CHECK_THAT(s.sun_dir_world.x, Catch::Matchers::WithinAbs(std::cos(e60), 1e-12));
        CHECK_THAT(s.sun_dir_world.y, Catch::Matchers::WithinAbs(0.0, 1e-12));
        CHECK_THAT(s.sun_dir_world.z, Catch::Matchers::WithinAbs(std::sin(e60), 1e-12));
    }
}

TEST_CASE("Environment: hashed state covers the clock anchor and mode", "[environment][clock]") {
    const auto hash_of = [](const EnvironmentConfig& cfg, std::int64_t tick, real hz) {
        Environment e(three_season_explicit(), cfg);
        e.attach(tick, hz);
        return e.fold_state_hash(14695981039346656037ULL);
    };
    EnvironmentConfig base;
    base.time_scale = 1.0;
    const std::uint64_t h0 = hash_of(base, 0, 240.0);
    CHECK(h0 == hash_of(base, 0, 240.0));  // deterministic

    EnvironmentConfig scale30 = base;
    scale30.time_scale = 30.0;
    CHECK(hash_of(scale30, 0, 240.0) != h0);
    CHECK(hash_of(base, 5, 240.0) != h0);       // anchor tick
    CHECK(hash_of(base, 0, 120.0) != h0);       // tick rate
    EnvironmentConfig start = base;
    start.start = EnvironmentStart{0, "a", 2, 100.0};
    CHECK(hash_of(start, 0, 240.0) != h0);      // anchor env time
    EnvironmentConfig fs = base;
    fs.fixed_season = "a";
    CHECK(hash_of(fs, 0, 240.0) != h0);         // calendar mode
    EnvironmentConfig fd = base;
    fd.fixed_day_of_year = 3;
    CHECK(hash_of(fd, 0, 240.0) != hash_of(fs, 0, 240.0));

    // The hash is a function of the anchor only: ticking does not move it.
    Environment e(three_season_explicit(), base);
    e.attach(0, 240.0);
    const std::uint64_t before = e.fold_state_hash(1);
    e.begin_tick(123456);
    CHECK(e.fold_state_hash(1) == before);
    // ... while a jump or a scale change does.
    e.set_time(1234.0, 123456);
    CHECK(e.fold_state_hash(1) != before);
}

TEST_CASE("Environment: a long season name is truncated, not overrun", "[environment][clock]") {
    ClimateDesc d = three_season_explicit();
    d.seasons[1].name = std::string(40, 'x');
    d.calendar.epoch.season = d.seasons[1].name;
    Environment env(d);
    env.attach(0, 240.0);
    CHECK(std::strlen(env.current().season_name) == 31);
}
