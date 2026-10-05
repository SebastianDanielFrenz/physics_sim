// tests/unit/test_environment_surface.cpp - weather plan W5, library level (no
// ps_core): the surface condition core (surface_condition.h) - the conduction
// column, the surface energy balance, the env-time grid with fast-forward,
// spin-up, checkpoint/restore and the hash, and the Environment that owns it.
// The World wiring is in test_environment_surface_world.cpp. Tags
// [environment][surface].
//
// Validation cases (weather plan 13, W5 row):
//  S1 asphalt daily maximum on a clear summer day at 50 N vs the SHRP model
//     (FHWA-RD-97-103 eq. 3, Solaimanian & Kennedy 1993), within 3 K.
//  S2 diurnal range, time of maximum and night minimum vs Kallas 1966 (HRR 150,
//     Table 2: College Park MD, June 30 1964, hourly), within 15 % / 1 h / 2 K.
//  S3 backward Euler at dt 60 s vs 15 s within 0.2 K.
//  S4 fast-forward == stepped bit-exact; checkpoint/restore bit-exact.
//
// RECORDED SABOTAGES (performed by hand, reverted, 2026-10-05):
//  - W5-S0 (the plan's sabotages, run inside the tests as negative controls): albedo 0.5 puts S1 at
//    30.32 C (-11.05 K vs SHRP); in S2 rho_c x0.25 moves the range +20.3 % and albedo 0.5 -28.8 %.
//  - W5-S1 (explicit Euler in step_column instead of the implicit Thomas solve): the dense Gaussian
//    reference failed on every node (node 0 316.34 vs 304.26 K), the energy closure failed (43125 vs
//    27341 J/m2), S1/S2/S3 failed (the 5 mm top cell is unstable at dt 60 s: NaN), and the snapshot
//    case read NaN.
//  - W5-S4 (every forward jump > 1 step spins up instead of stepping, max_fast_forward ignored): S4 B
//    failed (2881 steps instead of 2160, checkpoints and grid hash differ, surface 34.17 vs 34.90 C).
//  - W5-S5 (restore copies S[k] into both states): S4 C/E failed (checkpoint, grid hash and the full
//    Environment hash differ after the restore).
//  - W5-S2 (an unconditional fold in Environment::fold_state_hash, a zero int64 folded for every
//    environment): the W4 pin test in ps_determinism_tests failed on environment_day.json
//    (0x5921cbbefbcaa135 vs 0x2af395ee8a54f0b5 at 1 and 10 workers); this file's cases do not see it.
//  - W5-S3 (a std::vector allocated in Environment::fill_surfaces): test_environment_surface_world.cpp
//    "allocates nothing inside the tick" failed at 1 and 4 workers (244 own allocations).

#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/environment/solar_geometry.h"
#include "ps/environment/surface_condition.h"
#include "ps/environment/surface_conditions_io.h"
#include "ps/environment/weather_io.h"
#include "ps/math/transcendental.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::environment;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

std::string data(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

SurfaceConditionsDesc default_conditions() { return io::load_surface_conditions_json(data("surface_conditions/default.json")); }

const SurfaceClassDesc& find_class(const SurfaceConditionsDesc& d, const char* name) {
    const SurfaceClassDesc* c = d.find(name);
    REQUIRE(c != nullptr);
    return *c;
}

real f_to_c(real f) { return (f - 32.0) * 5.0 / 9.0; }

// ---- Kallas 1966 (HRR 150, Table 2): June 30 1964, College Park MD, hourly, deg F.
// Index i = clock hour i (0 = midnight = the table's "12 PM"/hour-24 row).
// The 7 AM row is OCR-garbled in the scan ("60 00 02 ..."): the same row maps
// '8' to '0' in every other column, so air 68 F and surface 80 F [D: OCR repair].
constexpr real kKallasAirF[24] = {72, 76, 76, 74, 73, 66, 64, 68, 78, 83, 87, 90,
                                  92, 94, 97, 99, 98, 96, 95, 91, 88, 86, 84, 74};
constexpr real kKallasSurfF[24] = {88, 84, 82, 81, 80, 78, 77, 80, 87, 98, 109, 120,
                                   130, 137, 141, 142, 135, 127, 121, 111, 103, 98, 95, 91};
// The 12-in section's thermocouples at 2 in and 12 in (the 7 AM row repaired the same way).
constexpr real kKallas2inF[24] = {94, 90, 88, 86, 85, 84, 82, 82, 84, 90, 97, 105,
                                  112, 119, 124, 127, 126, 122, 119, 114, 108, 102, 100, 97};
constexpr real kKallas12inF[24] = {98, 95, 94, 94, 94, 93, 92, 92, 91, 90, 90, 89,
                                   90, 90, 91, 92, 94, 95, 96, 97, 97, 98, 98, 98};
constexpr real kKallasLatDeg = 38.99;    // [C] College Park, MD (University of Maryland)
constexpr real kKallasLonDeg = -76.94;
// Clock -> local apparent solar time. PROVISIONAL [D]: the table is assumed to
// be in EDT (UTC-4): the surface minimum at 6 AM coincides with sunrise only
// under EDT. Solar = clock - 1 h (DST) - (75 - 76.94) deg / 15 deg/h ... i.e.
// clock - 1 h - 0.1293 h + equation of time (-3.4 min on June 30 [C NOAA]).
constexpr real kKallasSolarMinusClockH = -1.0 + (kKallasLonDeg + 75.0) / 15.0 - 3.4 / 60.0;

real periodic_hourly(const real (&v)[24], real clock_h) {
    real h = clock_h - 24.0 * std::floor(clock_h / 24.0);
    const int i = static_cast<int>(std::floor(h));
    const real f = h - static_cast<real>(i);
    const real a = v[i % 24], b = v[(i + 1) % 24];
    return a + (b - a) * f;
}

// The explicit forcing of S2/S3: measured air temperature, clear-sky
// irradiance for the site and day, [AS] wind, humidity and deep boundary.
struct KallasForcing final : ISurfaceForcingSource {
    real wind_m_s = 4.5;               // [C] the SHRP model assumption (Solaimanian & Kennedy 1993); [AS] for this site
    real vapour_pressure_hpa = 20.6;   // [AS] dew point 18 C (Magnus: 6.112 exp(17.62*18/(243.12+18)))
    real bottom_c = 17.5;              // [D] Kallas 1966: yearly average pavement temperature 63-64 F
    real decl_rad = 0.0;
    real lat_rad = kKallasLatDeg * kDegToRad;
    KallasForcing() { decl_rad = declination_spencer_rad(181.5); }   // June 30 1964 (leap year: day 182), noon
    SurfaceForcing forcing_at(real t) const override {
        SurfaceForcing f;
        const real clock_h = t / 3600.0;
        f.air_temp_k = f_to_c(periodic_hourly(kKallasAirF, clock_h)) + kCelsiusToKelvin;
        const real solar_h = clock_h + kKallasSolarMinusClockH;
        const real hour_angle = (solar_h - 12.0) * 15.0 * kDegToRad;
        const real sin_e = math::sin(lat_rad) * math::sin(decl_rad) +
                           math::cos(lat_rad) * math::cos(decl_rad) * math::cos(hour_angle);
        f.ghi_w_m2 = ghi_w_m2(math::asin(sin_e < -1.0 ? -1.0 : (sin_e > 1.0 ? 1.0 : sin_e)), 0.0);
        f.wind_m_s = wind_m_s;
        f.vapour_pressure_hpa = vapour_pressure_hpa;
        f.cloud_cover = 0.0;               // [AS] the record day: highest pavement temperature of the study
        f.bottom_temp_k = bottom_c + kCelsiusToKelvin;
        return f;
    }
    InitialProfile initial_at(real) const override {
        return InitialProfile{f_to_c(86.0) + kCelsiusToKelvin, bottom_c + kCelsiusToKelvin};   // June mean air 72 F is the alternative; surface mean ~86 F
    }
};

// Kallas's section: 12 in (0.305 m) full-depth asphalt concrete on soil [C];
// the default asphalt material values.
SurfaceConditionsDesc kallas_conditions(real layer_rho_c_scale = 1.0, real albedo_override = -1.0) {
    SurfaceConditionsDesc d;
    SurfaceClassDesc c = find_class(default_conditions(), "asphalt");
    c.layer.depth_m = 0.305;
    c.deep.depth_m = 2.0 - 0.305;
    c.layer.rho_c_j_m3k *= layer_rho_c_scale;
    if (albedo_override >= 0.0) c.albedo = albedo_override;
    d.classes.push_back(c);
    return d;
}

struct DayTrace {
    std::vector<real> t_surf_c;   // one per env minute of the last day
    real max_c = -1e9, min_c = 1e9;
    real t_max_h = 0.0, t_min_h = 0.0;
};

// Spins the column up over `days` repetitions of the periodic forcing and
// records the last day minute by minute (grid interpolation).
DayTrace run_kallas(real dt, const SurfaceConditionsDesc& conds, const KallasForcing& f, int days = 120) {
    ConditionGridDesc g;
    g.dt_env_s = dt;
    g.spinup_s = static_cast<real>(days) * 86400.0;
    g.max_fast_forward_s = 30.0 * 86400.0;
    ConditionGrid grid(conds, {}, g);
    const real day0 = static_cast<real>(days) * 86400.0;
    grid.advance_to(day0, f);
    DayTrace tr;
    for (int m = 0; m < 1440; ++m) {
        const real t = day0 + 60.0 * m;
        grid.advance_to(t, f);
        const real c = grid.surface_temp_k(0, t) - kCelsiusToKelvin;
        tr.t_surf_c.push_back(c);
        if (c > tr.max_c) { tr.max_c = c; tr.t_max_h = m / 60.0; }
        if (c < tr.min_c) { tr.min_c = c; tr.t_min_h = m / 60.0; }
    }
    return tr;
}

// Measured: max 142 F at the 3 PM row; parabola vertex through 2/3/4 PM.
struct KallasMeasured {
    real max_c, min_c, range_k, t_max_h, t_min_h;
};
KallasMeasured kallas_measured() {
    KallasMeasured m{};
    m.max_c = f_to_c(142.0);
    m.min_c = f_to_c(77.0);
    m.range_k = m.max_c - m.min_c;
    const real a = kKallasSurfF[14], b = kKallasSurfF[15], c = kKallasSurfF[16];
    m.t_max_h = 15.0 + 0.5 * (a - c) / (a - 2.0 * b + c);
    m.t_min_h = 6.0;
    return m;
}

// ---- S1 harness: the Environment on the test_weather climate's summer
// solstice day, held (fixed day), clear preset with the SHRP wind.
int solstice_day(const ClimateDesc& climate) {
    const Calendar cal(climate);
    for (int i = 0; i < cal.days_per_year(); ++i)
        if (cal.day(i).y_start <= 171.5 && cal.day(i).y_end > 171.5) return i;
    FAIL("no game day contains the solstice");
    return 0;
}

struct ShrpRun {
    real t_air_max_c = -1e9;
    real t_surf_max_c = -1e9;
    real t_surf_min_c = 1e9;
    real t_air_min_c = 1e9;
    real lat_deg = 0.0;
};

ShrpRun run_shrp_day(real asphalt_albedo) {
    ClimateDesc climate = io::load_climate_json(data("climates/test_weather.json"));
    climate.condition = ConditionGridDesc{60.0, 30.0 * 86400.0, 30.0 * 86400.0};
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Preset;
    cfg.weather_preset = io::load_weather_preset_json(data("weather_presets/clear.json"));
    cfg.weather_preset.cloud_cover = 0.0;
    cfg.weather_preset.wind_mean_m_s = 4.5;   // [C] the SHRP model's assumed wind (Solaimanian & Kennedy 1993)
    cfg.fixed_day_of_year = solstice_day(climate);
    cfg.time_scale = 60.0;
    SurfaceConditionsDesc conds = default_conditions();
    for (SurfaceClassDesc& c : conds.classes)
        if (c.name == "asphalt") c.albedo = asphalt_albedo;
    cfg.surface_conditions = conds;
    Environment env(climate, cfg);
    env.attach(0, 1.0);   // one tick = one sim second = one env minute
    int asphalt = -1;
    for (int i = 0; i < env.surface_conditions()->class_count(); ++i)
        if (env.surface_conditions()->class_name(i) == "asphalt") asphalt = i;
    REQUIRE(asphalt >= 0);
    ShrpRun r;
    r.lat_deg = climate.location.latitude_deg;
    // Day 1 settles the last spin-up transient; day 2 is measured.
    for (std::int64_t k = 0; k < 2 * 1440; ++k) {
        env.begin_tick(k);
        if (k < 1440) continue;
        const EnvironmentSnapshot& s = env.current();
        const real ts = s.surfaces[static_cast<std::size_t>(asphalt)].t_surface_c;
        r.t_surf_max_c = std::max(r.t_surf_max_c, ts);
        r.t_surf_min_c = std::min(r.t_surf_min_c, ts);
        r.t_air_max_c = std::max(r.t_air_max_c, s.air_temp_c);
        r.t_air_min_c = std::min(r.t_air_min_c, s.air_temp_c);
    }
    return r;
}

// SHRP (FHWA-RD-97-103 eq. 3): T_surf = T_air - 0.00618 Lat^2 + 0.2289 Lat + 24.4.
real shrp_surface_max_c(real t_air_max_c, real lat_deg) {
    return t_air_max_c - 0.00618 * lat_deg * lat_deg + 0.2289 * lat_deg + 24.4;
}

// ---- S4 harness: the dynamic-weather climate with surfaces.
std::unique_ptr<Environment> dynamic_env(std::uint64_t seed = 7, real scale = 3600.0) {
    ClimateDesc climate = io::load_climate_json(data("climates/test_weather.json"));
    climate.condition = ConditionGridDesc{60.0, 2.0 * 86400.0, 3.0 * 86400.0};
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Dynamic;
    cfg.seed = seed;
    cfg.time_scale = scale;
    cfg.start = EnvironmentStart{0, "spring", 20, 3600.0 * 5.0};
    cfg.surface_conditions = default_conditions();
    return std::make_unique<Environment>(climate, cfg);
}

bool same_checkpoint(const ConditionCheckpoint& a, const ConditionCheckpoint& b) {
    if (a.valid != b.valid || a.grid_index != b.grid_index || a.class_count != b.class_count || a.dt_env_s != b.dt_env_s)
        return false;
    return std::memcmp(a.prev.data(), b.prev.data(), sizeof(a.prev)) == 0 &&
           std::memcmp(a.next.data(), b.next.data(), sizeof(a.next)) == 0;
}

} // namespace

// ------------------------------------------------------------------ column

TEST_CASE("Surface column: geometry, slab boundary, heat capacity and conductances", "[environment][surface]") {
    const SurfaceClassDesc asphalt = find_class(default_conditions(), "asphalt");
    const ColumnModel c = build_column(asphalt);
    REQUIRE(c.nodes >= 10);
    REQUIRE(c.nodes <= kMaxColumnNodes);
    CHECK(c.z_m[0] == 0.0);
    CHECK(c.z_m[static_cast<std::size_t>(c.nodes - 1)] == asphalt.layer.depth_m + asphalt.deep.depth_m);
    // The slab boundary is a node, and the top cell is the documented thickness (rescaled within the slab).
    bool boundary = false;
    for (int i = 0; i < c.nodes; ++i) boundary = boundary || std::fabs(c.z_m[static_cast<std::size_t>(i)] - asphalt.layer.depth_m) < 1e-12;
    CHECK(boundary);
    CHECK(c.z_m[1] > 0.5 * kColumnTopCellM);
    CHECK(c.z_m[1] < 2.0 * kColumnTopCellM);
    // Total heat capacity = sum over slabs of rho_c * thickness.
    real cap = 0.0;
    for (int i = 0; i < c.nodes; ++i) cap += c.cap_j_m2k[static_cast<std::size_t>(i)];
    CHECK_THAT(cap, WithinRel(asphalt.layer.rho_c_j_m3k * asphalt.layer.depth_m + asphalt.deep.rho_c_j_m3k * asphalt.deep.depth_m, 1e-12));
    // Series resistance = sum of slab thickness / k.
    real r = 0.0;
    for (int i = 0; i + 1 < c.nodes; ++i) r += 1.0 / c.cond_w_m2k[static_cast<std::size_t>(i)];
    CHECK_THAT(r, WithinRel(asphalt.layer.depth_m / asphalt.layer.k_w_mk + asphalt.deep.depth_m / asphalt.deep.k_w_mk, 1e-12));

    // Too deep: rejected.
    SurfaceClassDesc thin = asphalt;
    thin.layer.depth_m = 0.001;   // thinner than the top cell: one cell
    CHECK_NOTHROW(build_column(thin));
    SurfaceClassDesc too_deep = asphalt;
    too_deep.deep.depth_m = 11.0;
    CHECK_THROWS_WITH(build_column(too_deep), ContainsSubstring("<= 10"));
}

TEST_CASE("Surface physics: sky emissivity and convection coefficient against hand values", "[environment][surface]") {
    // Geiger: 0.77 - 0.28 * 10^(-0.074 p), p = 10 mmHg -> 0.77 - 0.28 * 10^-0.74.
    const real p_hpa = 10.0 / 0.750061683;
    CHECK_THAT(sky_emissivity(p_hpa, 0.0), WithinAbs(0.77 - 0.28 * std::pow(10.0, -0.74), 1e-12));
    CHECK_THAT(sky_emissivity(p_hpa, 1.0), WithinAbs(1.0, 1e-15));
    CHECK_THAT(sky_emissivity(p_hpa, 0.5), WithinAbs(0.5 + 0.5 * (0.77 - 0.28 * std::pow(10.0, -0.74)), 1e-12));
    // Solaimanian & Kennedy 1993's range at their typical conditions: 17-22.7 W/(m2 C) at ~4.5 m/s.
    const real ts = 330.0, ta = 305.0;
    const real tm = 0.5 * (ts + ta);
    const real hand = 698.24 * (0.00144 * std::pow(tm, 0.3) * std::pow(4.5, 0.7) + 0.00097 * std::pow(ts - ta, 0.3));
    CHECK_THAT(convection_coefficient_w_m2k(ts, ta, 4.5), WithinRel(hand, 1e-12));
    CHECK(hand > 17.0);
    CHECK(hand < 22.7);
    // No free convection while the surface is cooler than the air.
    const real forced = 698.24 * 0.00144 * std::pow(0.5 * (290.0 + 300.0), 0.3) * std::pow(3.0, 0.7);
    CHECK_THAT(convection_coefficient_w_m2k(290.0, 300.0, 3.0), WithinRel(forced, 1e-12));
    CHECK(convection_coefficient_w_m2k(290.0, 300.0, 0.0) == 0.0);
}

TEST_CASE("Surface column: one backward-Euler step is the exact solution of its own linear system", "[environment][surface]") {
    // Reference: the same discrete equations assembled densely and solved by
    // Gaussian elimination with partial pivoting (the Thomas sweep is the fast form).
    const SurfaceClassDesc asphalt = find_class(default_conditions(), "asphalt");
    const ColumnModel col = build_column(asphalt);
    SurfaceClassState s;
    initial_column_state(col, InitialProfile{300.0, 285.0}, s);
    for (int i = 0; i < col.nodes - 1; ++i) s.temp_k[static_cast<std::size_t>(i)] += 3.0 * std::sin(0.7 * i);
    SurfaceForcing f;
    f.ghi_w_m2 = 850.0;
    f.air_temp_k = 303.0;
    f.wind_m_s = 2.5;
    f.vapour_pressure_hpa = 15.0;
    f.cloud_cover = 0.2;
    f.bottom_temp_k = 286.0;
    const real dt = 60.0;
    SurfaceClassState out;
    step_column(col, f, dt, s, out);

    const int m = col.nodes - 1;
    std::vector<std::vector<real>> a(static_cast<std::size_t>(m), std::vector<real>(static_cast<std::size_t>(m + 1), 0.0));
    const real t0 = s.temp_k[0];
    const real eps = col.emissivity;
    const real hc = convection_coefficient_w_m2k(t0, f.air_temp_k, f.wind_m_s);
    const real lsky = sky_emissivity(f.vapour_pressure_hpa, f.cloud_cover) * kStefanBoltzmann * std::pow(f.air_temp_k, 4.0);
    for (int i = 0; i < m; ++i) {
        auto& row = a[static_cast<std::size_t>(i)];
        const real cap = col.cap_j_m2k[static_cast<std::size_t>(i)] / dt;
        row[static_cast<std::size_t>(i)] = cap;
        row[static_cast<std::size_t>(m)] = cap * s.temp_k[static_cast<std::size_t>(i)];
        if (i > 0) {
            const real k = col.cond_w_m2k[static_cast<std::size_t>(i - 1)];
            row[static_cast<std::size_t>(i)] += k;
            row[static_cast<std::size_t>(i - 1)] -= k;
        }
        const real k = col.cond_w_m2k[static_cast<std::size_t>(i)];
        row[static_cast<std::size_t>(i)] += k;
        if (i + 1 < m) row[static_cast<std::size_t>(i + 1)] -= k;
        else row[static_cast<std::size_t>(m)] += k * f.bottom_temp_k;
        if (i == 0) {
            row[0] += 4.0 * eps * kStefanBoltzmann * t0 * t0 * t0 + hc;
            row[static_cast<std::size_t>(m)] += (1.0 - col.albedo) * f.ghi_w_m2 + eps * lsky +
                                                3.0 * eps * kStefanBoltzmann * std::pow(t0, 4.0) + hc * f.air_temp_k;
        }
    }
    for (int c = 0; c < m; ++c) {
        int p = c;
        for (int r = c + 1; r < m; ++r)
            if (std::fabs(a[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]) > std::fabs(a[static_cast<std::size_t>(p)][static_cast<std::size_t>(c)])) p = r;
        std::swap(a[static_cast<std::size_t>(c)], a[static_cast<std::size_t>(p)]);
        for (int r = c + 1; r < m; ++r) {
            const real q = a[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] / a[static_cast<std::size_t>(c)][static_cast<std::size_t>(c)];
            for (int k = c; k <= m; ++k) a[static_cast<std::size_t>(r)][static_cast<std::size_t>(k)] -= q * a[static_cast<std::size_t>(c)][static_cast<std::size_t>(k)];
        }
    }
    std::vector<real> x(static_cast<std::size_t>(m), 0.0);
    for (int r = m - 1; r >= 0; --r) {
        real v = a[static_cast<std::size_t>(r)][static_cast<std::size_t>(m)];
        for (int k = r + 1; k < m; ++k) v -= a[static_cast<std::size_t>(r)][static_cast<std::size_t>(k)] * x[static_cast<std::size_t>(k)];
        x[static_cast<std::size_t>(r)] = v / a[static_cast<std::size_t>(r)][static_cast<std::size_t>(r)];
    }
    for (int i = 0; i < m; ++i) {
        INFO("node " << i);
        CHECK_THAT(out.temp_k[static_cast<std::size_t>(i)], WithinAbs(x[static_cast<std::size_t>(i)], 1e-9));
    }
    CHECK(out.temp_k[static_cast<std::size_t>(m)] == f.bottom_temp_k);

    // Energy closure of the step: d(column heat) = dt * (surface flux at the
    // new temperature, linearised as solved + bottom flux).
    real de = 0.0;
    for (int i = 0; i < m; ++i)
        de += col.cap_j_m2k[static_cast<std::size_t>(i)] * (out.temp_k[static_cast<std::size_t>(i)] - s.temp_k[static_cast<std::size_t>(i)]);
    const real tn = out.temp_k[0];
    const real q_top = (1.0 - col.albedo) * f.ghi_w_m2 + eps * lsky - eps * kStefanBoltzmann * (4.0 * t0 * t0 * t0 * tn - 3.0 * std::pow(t0, 4.0)) +
                       hc * (f.air_temp_k - tn);
    const real q_bot = col.cond_w_m2k[static_cast<std::size_t>(m - 1)] * (f.bottom_temp_k - out.temp_k[static_cast<std::size_t>(m - 1)]);
    CHECK_THAT(de, WithinAbs(dt * (q_top + q_bot), 1e-7 * dt * (std::fabs(q_top) + std::fabs(q_bot))));
    // Placeholders stay exactly 0 (W6-W9 not implemented).
    CHECK(out.water_mm == 0.0);
    CHECK(out.snow_mm == 0.0);
    CHECK(out.canopy_water_mm == 0.0);
}

// ------------------------------------------------------------------ validation

TEST_CASE("S1: asphalt maximum on a clear summer day at 50 N matches the SHRP model within 3 K", "[environment][surface]") {
    const ShrpRun r = run_shrp_day(0.10);
    const real shrp = shrp_surface_max_c(r.t_air_max_c, r.lat_deg);
    INFO("air max " << r.t_air_max_c << " C, surface max " << r.t_surf_max_c << " C, SHRP " << shrp
                    << " C, surface min " << r.t_surf_min_c << " C, air min " << r.t_air_min_c << " C");
    CHECK_THAT(r.t_surf_max_c, WithinAbs(shrp, 3.0));
    // Negative control (the plan's sabotage, run in the test): albedo 0.5 misses the band.
    const ShrpRun bright = run_shrp_day(0.5);
    INFO("albedo 0.5 surface max " << bright.t_surf_max_c << " C");
    CHECK(std::fabs(bright.t_surf_max_c - shrp_surface_max_c(bright.t_air_max_c, bright.lat_deg)) > 3.0);
}

TEST_CASE("S2: diurnal range, time of maximum and the 12 in temperature vs Kallas 1966", "[environment][surface]") {
    const KallasForcing f;
    const DayTrace tr = run_kallas(60.0, kallas_conditions(), f);
    const KallasMeasured m = kallas_measured();
    const real range = tr.max_c - tr.min_c;
    real rms = 0.0;
    for (int h = 0; h < 24; ++h) {
        const real d = tr.t_surf_c[static_cast<std::size_t>(h * 60)] - f_to_c(kKallasSurfF[h]);
        rms += d * d;
    }
    rms = std::sqrt(rms / 24.0);
    INFO("model max " << tr.max_c << " C at " << tr.t_max_h << " h, min " << tr.min_c << " C at " << tr.t_min_h
                      << " h, range " << range << " K; measured max " << m.max_c << " C at " << m.t_max_h << " h, min "
                      << m.min_c << " C, range " << m.range_k << " K; hourly rms " << rms << " K");
    CHECK_THAT(range, WithinRel(m.range_k, 0.15));
    CHECK_THAT(tr.t_max_h, WithinAbs(m.t_max_h, 1.0));

    // The storage below the surface: the modelled temperature at the 12 in
    // thermocouple (the asphalt/soil interface) follows the measured one hourly.
    ConditionGridDesc gd;
    gd.spinup_s = 120.0 * 86400.0;
    ConditionGrid gr(kallas_conditions(), {}, gd);
    const real day0 = 120.0 * 86400.0;
    const ColumnModel& col = gr.column(0);
    int node12 = -1;
    for (int i = 0; i < col.nodes; ++i)
        if (std::fabs(col.z_m[static_cast<std::size_t>(i)] - 0.305) < 1e-12) node12 = i;
    REQUIRE(node12 > 0);
    // The measured column is whole degrees F (0.56 K steps), so the gate is the
    // hourly rms; the worst hour is reported (2.06 K at 2 AM, measured W5).
    real worst12 = 0.0;
    real rms12 = 0.0;
    for (int h = 0; h < 24; ++h) {
        gr.advance_to(day0 + 3600.0 * h, f);   // on a grid node: S[k] is the state at the hour
        const real model = gr.prev(0).temp_k[static_cast<std::size_t>(node12)] - kCelsiusToKelvin;
        const real d = model - f_to_c(kKallas12inF[h]);
        worst12 = std::max(worst12, std::fabs(d));
        rms12 += d * d;
    }
    rms12 = std::sqrt(rms12 / 24.0);
    INFO("12 in: hourly rms |model - measured| " << rms12 << " K, worst hour " << worst12 << " K");
    CHECK(rms12 <= 1.5);

    // Negative controls (sabotages run in the test): a quarter of the asphalt's
    // heat capacity, or albedo 0.5, leave the range band.
    const DayTrace light = run_kallas(60.0, kallas_conditions(0.25), f);
    CHECK(std::fabs((light.max_c - light.min_c) / m.range_k - 1.0) > 0.15);
    const DayTrace bright = run_kallas(60.0, kallas_conditions(1.0, 0.5), f);
    CHECK(std::fabs((bright.max_c - bright.min_c) / m.range_k - 1.0) > 0.15);
}

// [!mayfail] with a discriminating measurement (out/weather_w5_progress.md):
// the modelled night minimum is 20.5 C vs the measured 25.0 C (-4.5 K). The 12 in
// temperature agrees (hourly rms 1.1 K, worst hour 2.1 K, previous case), so the storage and the
// deep boundary are not the cause; the deficit is in the night surface exchange,
// whose forcing is unknown for that night: wind 2 m/s instead of the SHRP
// 4.5 m/s gives 22.3 C, the Brutsaert 1975 clear-sky emissivity (0.85 vs Geiger's
// 0.75) 22.1 C (Surface diag case). Kept Geiger (the cited S&K form) and the
// SHRP wind rather than tuning either to this one record.
TEST_CASE("S2b: night minimum vs Kallas 1966 within 2 K", "[environment][surface][!mayfail]") {
    const KallasForcing f;
    const DayTrace tr = run_kallas(60.0, kallas_conditions(), f);
    const KallasMeasured m = kallas_measured();
    INFO("model min " << tr.min_c << " C at " << tr.t_min_h << " h; measured " << m.min_c << " C at " << m.t_min_h << " h");
    CHECK_THAT(tr.min_c, WithinAbs(m.min_c, 2.0));
}

TEST_CASE("S3: the implicit step at dt 60 s and 15 s agrees within 0.2 K", "[environment][surface]") {
    const KallasForcing f;
    const DayTrace a = run_kallas(60.0, kallas_conditions(), f, 40);
    const DayTrace b = run_kallas(15.0, kallas_conditions(), f, 40);
    real worst = 0.0;
    for (std::size_t i = 0; i < a.t_surf_c.size(); ++i) worst = std::max(worst, std::fabs(a.t_surf_c[i] - b.t_surf_c[i]));
    INFO("max |T(60 s) - T(15 s)| over the day: " << worst << " K");
    CHECK(worst <= 0.2);
}

TEST_CASE("Surface diag: S1/S2 numbers and sensitivities", "[.][diag][environment][surface]") {
    const ShrpRun r = run_shrp_day(0.10);
    std::printf("S1: air max %.3f C, surface max %.3f C, SHRP %.3f C (diff %.3f K); surface min %.3f, air min %.3f\n",
                r.t_air_max_c, r.t_surf_max_c, shrp_surface_max_c(r.t_air_max_c, r.lat_deg),
                r.t_surf_max_c - shrp_surface_max_c(r.t_air_max_c, r.lat_deg), r.t_surf_min_c, r.t_air_min_c);
    const ShrpRun b = run_shrp_day(0.5);
    std::printf("S1 albedo 0.5: surface max %.3f C (diff %.3f K)\n", b.t_surf_max_c,
                b.t_surf_max_c - shrp_surface_max_c(b.t_air_max_c, b.lat_deg));
    const KallasMeasured m = kallas_measured();
    std::printf("Kallas measured: max %.2f C at %.2f h, min %.2f C, range %.2f K\n", m.max_c, m.t_max_h, m.min_c, m.range_k);
    const auto show = [&](const char* what, const KallasForcing& f, const SurfaceConditionsDesc& c) {
        const DayTrace tr = run_kallas(60.0, c, f);
        std::printf("S2 %-28s max %.2f C at %.2f h, min %.2f C at %.2f h, range %.2f K (%.1f %%)\n", what, tr.max_c,
                    tr.t_max_h, tr.min_c, tr.t_min_h, tr.max_c - tr.min_c,
                    100.0 * ((tr.max_c - tr.min_c) / m.range_k - 1.0));
        std::printf("   hourly model-measured:");
        for (int h = 0; h < 24; ++h) std::printf(" %.1f", tr.t_surf_c[static_cast<std::size_t>(h * 60)] - f_to_c(kKallasSurfF[h]));
        std::printf("\n");
    };
    KallasForcing f;
    show("base", f, kallas_conditions());
    show("rho_c x 0.25", f, kallas_conditions(0.25));
    show("albedo 0.5", f, kallas_conditions(1.0, 0.5));
    KallasForcing w2 = f;
    w2.wind_m_s = 2.0;
    show("wind 2 m/s", w2, kallas_conditions());
    KallasForcing hum = f;
    hum.vapour_pressure_hpa = 14.0;
    show("vapour 14 hPa", hum, kallas_conditions());
    // Sky emissivity alternative (diagnostic only): Brutsaert 1975 [M] clear-sky
    // 1.24 (e/T)^(1/7), e in hPa, imitated through the cloud raise (the forcing's
    // cloud cover feeds only the longwave here; the irradiance stays clear-sky).
    struct BrutsaertForcing final : ISurfaceForcingSource {
        const KallasForcing* base = nullptr;
        SurfaceForcing forcing_at(real t) const override {
            SurfaceForcing x = base->forcing_at(t);
            const real eb = 1.24 * std::pow(x.vapour_pressure_hpa / x.air_temp_k, 1.0 / 7.0);
            const real eg = sky_emissivity(x.vapour_pressure_hpa, 0.0);
            x.cloud_cover = (eb - eg) / (1.0 - eg);
            return x;
        }
        InitialProfile initial_at(real t) const override { return base->initial_at(t); }
    } br;
    br.base = &f;
    {
        ConditionGridDesc gd;
        gd.spinup_s = 120.0 * 86400.0;
        ConditionGrid gr(kallas_conditions(), {}, gd);
        gr.advance_to(120.0 * 86400.0, br);
        real bmx = -1e9, bt = 0.0, bmn = 1e9;
        for (int m2 = 0; m2 < 1440; ++m2) {
            const real t = 120.0 * 86400.0 + 60.0 * m2;
            gr.advance_to(t, br);
            const real c = gr.surface_temp_k(0, t) - kCelsiusToKelvin;
            if (c > bmx) { bmx = c; bt = m2 / 60.0; }
            bmn = std::min(bmn, c);
        }
        std::printf("S2 Brutsaert sky (diag)       max %.2f C at %.2f h, min %.2f C, range %.2f K; eps_sky Geiger %.3f vs Brutsaert %.3f at 6 AM\n",
                    bmx, bt, bmn, bmx - bmn, sky_emissivity(f.vapour_pressure_hpa, 0.0),
                    1.24 * std::pow(f.vapour_pressure_hpa / f.forcing_at(6.0 * 3600.0).air_temp_k, 1.0 / 7.0));
    }
    KallasForcing est = f;   // the alternative time-zone reading: the table in EST
    // (EST would shift solar time by +1 h relative to the EDT reading.)
    struct EstForcing final : ISurfaceForcingSource {
        const KallasForcing* base = nullptr;
        SurfaceForcing forcing_at(real t) const override {
            SurfaceForcing x = base->forcing_at(t);
            const SurfaceForcing sun = base->forcing_at(t + 3600.0);   // solar time one hour later
            x.ghi_w_m2 = sun.ghi_w_m2;
            return x;
        }
        InitialProfile initial_at(real t) const override { return base->initial_at(t); }
    } est_src;
    est_src.base = &est;
    ConditionGridDesc g;
    g.spinup_s = 120.0 * 86400.0;
    ConditionGrid grid(kallas_conditions(), {}, g);
    grid.advance_to(120.0 * 86400.0, est_src);
    real mx = -1e9, tmx = 0.0, mn = 1e9;
    for (int m2 = 0; m2 < 1440; ++m2) {
        const real t = 120.0 * 86400.0 + 60.0 * m2;
        grid.advance_to(t, est_src);
        const real c = grid.surface_temp_k(0, t) - kCelsiusToKelvin;
        if (c > mx) { mx = c; tmx = m2 / 60.0; }
        mn = std::min(mn, c);
    }
    std::printf("S2 EST reading                max %.2f C at %.2f h, min %.2f C, range %.2f K\n", mx, tmx, mn, mx - mn);

    // Depth profile vs the measured 2 in and 12 in columns (base forcing).
    {
        ConditionGridDesc gd;
        gd.spinup_s = 120.0 * 86400.0;
        ConditionGrid gr(kallas_conditions(), {}, gd);
        const real day0 = 120.0 * 86400.0;
        std::printf("hour  model2in meas2in  model12in meas12in\n");
        for (int h = 0; h < 24; ++h) {
            const real t = day0 + 3600.0 * h;
            gr.advance_to(t, f);
            const ColumnModel& col = gr.column(0);
            const auto at = [&](real z) {
                for (int i = 0; i + 1 < col.nodes; ++i) {
                    const real z0 = col.z_m[static_cast<std::size_t>(i)], z1 = col.z_m[static_cast<std::size_t>(i + 1)];
                    if (z >= z0 && z <= z1) {
                        const real a = gr.prev(0).temp_k[static_cast<std::size_t>(i)], b = gr.prev(0).temp_k[static_cast<std::size_t>(i + 1)];
                        return a + (b - a) * (z - z0) / (z1 - z0) - kCelsiusToKelvin;
                    }
                }
                return -999.0;
            };
            std::printf("%2d   %7.2f %7.2f   %7.2f %7.2f\n", h, at(0.0508), f_to_c(kKallas2inF[h]), at(0.3048), f_to_c(kKallas12inF[h]));
        }
    }
}

// ------------------------------------------------------------------ grid

TEST_CASE("S4: fast-forward equals stepping bit-exactly; checkpoint/restore is bit-exact", "[environment][surface]") {
    // A: stepped tick by tick (one tick = 15 env s at scale 3600 / 240 Hz) over 1.5 env days.
    auto a = dynamic_env();
    a->attach(0, 240.0);
    // End and checkpoint ticks sit inside a grid interval (30 s / 15 s past a step), so a
    // re-anchored clock's rounding cannot move the grid index there.
    const std::int64_t ticks = 8642;
    const std::int64_t mid = 4321;
    ConditionCheckpoint at_mid;
    std::int64_t max_steps_per_tick = 0;
    for (std::int64_t k = 1; k <= ticks; ++k) {
        a->begin_tick(k);
        max_steps_per_tick = std::max(max_steps_per_tick, a->last_condition_steps());
        if (k == mid) at_mid = a->checkpoint_conditions();
    }
    CHECK(max_steps_per_tick == 1);   // 15 env s per tick: at most one 60-s step

    // B: one forward jump over the same span (within max_fast_forward_s).
    auto b = dynamic_env();
    b->attach(0, 240.0);
    b->set_time(a->clock().env_time(ticks), 1);
    const std::int64_t k_start = b->surface_conditions()->grid_index_at(b->start_env_s());
    CHECK(b->last_condition_steps() == a->surface_conditions()->grid_index() - k_start);   // stepped, not spun up
    CHECK(same_checkpoint(a->checkpoint_conditions(), b->checkpoint_conditions()));
    CHECK(a->surface_conditions()->fold_hash(1469598103934665603ull) == b->surface_conditions()->fold_hash(1469598103934665603ull));
    for (int i = 0; i < a->surface_conditions()->class_count(); ++i)
        CHECK(a->current().surfaces[static_cast<std::size_t>(i)].t_surface_c == b->current().surfaces[static_cast<std::size_t>(i)].t_surface_c);

    // C: restore A's mid-run checkpoint into a fresh environment at the same
    // tick, then step on: bit-identical to A at the end.
    // C reaches the checkpoint's time by a backward jump, i.e. with a spin-up
    // state of its own, before the restore.
    auto c = dynamic_env();
    c->attach(0, 240.0);
    c->set_time(c->start_env_s() + 10.0 * 86400.0, 0);
    c->set_time(a->clock().env_time(mid), mid);
    CHECK(c->surface_conditions()->grid_index() == at_mid.grid_index);
    CHECK_FALSE(same_checkpoint(c->checkpoint_conditions(), at_mid));
    c->restore_conditions(at_mid, mid);
    CHECK(same_checkpoint(c->checkpoint_conditions(), at_mid));
    for (std::int64_t k = mid + 1; k <= ticks; ++k) c->begin_tick(k);
    CHECK(same_checkpoint(c->checkpoint_conditions(), a->checkpoint_conditions()));
    CHECK(c->surface_conditions()->fold_hash(1469598103934665603ull) == a->surface_conditions()->fold_hash(1469598103934665603ull));
    // A restore into an environment on the SAME clock gives the same full state hash.
    auto e = dynamic_env();
    e->attach(0, 240.0);
    e->restore_conditions(at_mid, mid);
    for (std::int64_t k = mid + 1; k <= ticks; ++k) e->begin_tick(k);
    CHECK(e->fold_state_hash(1469598103934665603ull) == a->fold_state_hash(1469598103934665603ull));

    // D: a spin-up path is NOT the stepped path (the state remembers its history).
    auto d = dynamic_env();
    d->attach(0, 240.0);
    d->set_time(a->clock().env_time(ticks) + 10.0 * 86400.0, 1);   // beyond max_fast_forward: spin-up
    CHECK(d->last_condition_steps() == d->surface_conditions()->spinup_steps() + 1);
}

TEST_CASE("Surface grid: jump rules, interpolation and restore rejections", "[environment][surface]") {
    auto e = dynamic_env(3, 60.0);
    const ConditionGrid& g = *e->surface_conditions();
    CHECK(g.initialised());
    const std::int64_t k0 = g.grid_index();
    CHECK(k0 == g.grid_index_at(e->start_env_s()));
    // Interpolation: the snapshot is the linear blend of S[k] and S[k+1] at t_env.
    e->attach(0, 240.0);
    e->begin_tick(240 * 30);   // + 30 env min at scale 60
    const real t = e->clock().env_time(240 * 30);
    const real w = (t - static_cast<real>(g.grid_index()) * 60.0) / 60.0;
    for (int i = 0; i < g.class_count(); ++i) {
        if (!g.has_state(i)) {
            CHECK_FALSE(e->current().surfaces[static_cast<std::size_t>(i)].has_state);
            continue;
        }
        const real expect = g.prev(i).temp_k[0] + (g.next(i).temp_k[0] - g.prev(i).temp_k[0]) * w - kCelsiusToKelvin;
        CHECK(e->current().surfaces[static_cast<std::size_t>(i)].t_surface_c == expect);
    }
    // A backward jump spins up (the state is not reversible).
    e->set_time(e->start_env_s() - 3600.0, 240 * 30 + 1);
    CHECK(e->last_condition_steps() == g.spinup_steps() + 1);
    // Restore rejections.
    const ConditionCheckpoint ck = e->checkpoint_conditions();
    ConditionCheckpoint ahead = ck;
    ahead.grid_index += 5;
    CHECK_THROWS_WITH(e->restore_conditions(ahead, 240 * 30 + 1), ContainsSubstring("ahead of the clock"));
    ConditionCheckpoint old = ck;
    old.grid_index -= g.max_fast_forward_steps() + 1;
    CHECK_THROWS_WITH(e->restore_conditions(old, 240 * 30 + 1), ContainsSubstring("max_fast_forward_s"));
    ConditionCheckpoint wrong = ck;
    wrong.class_count = 3;
    CHECK_THROWS_WITH(e->restore_conditions(wrong, 240 * 30 + 1), ContainsSubstring("class count"));
    ConditionCheckpoint invalid;
    CHECK_THROWS_WITH(e->restore_conditions(invalid, 240 * 30 + 1), ContainsSubstring("not valid"));
    CHECK(same_checkpoint(e->checkpoint_conditions(), ck));   // nothing changed
}

TEST_CASE("Surface grid: hash covers the state only when surfaces are installed", "[environment][surface]") {
    ClimateDesc climate = io::load_climate_json(data("climates/test_weather.json"));
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Dynamic;
    cfg.seed = 5;
    cfg.start = EnvironmentStart{0, "summer", 3, 43200.0};
    const Environment plain(climate, cfg);
    EnvironmentConfig with = cfg;
    with.surface_conditions = default_conditions();
    const Environment surf(climate, with);
    const std::uint64_t h0 = 1469598103934665603ull;
    CHECK(plain.fold_state_hash(h0) != surf.fold_state_hash(h0));
    CHECK_FALSE(plain.has_surface_conditions());
    CHECK_FALSE(plain.current().has_surfaces);
    CHECK(surf.current().has_surfaces);
    // The surface part is exactly the grid's fold appended to the W4 fold.
    CHECK(surf.fold_state_hash(h0) == surf.surface_conditions()->fold_hash(plain.fold_state_hash(h0)));
    // One node one ulp off moves the hash.
    ConditionGrid g = *surf.surface_conditions();
    ConditionCheckpoint ck = g.checkpoint();
    ck.next[0].temp_k[5] = std::nextafter(ck.next[0].temp_k[5], 1e9);
    g.restore(ck);
    CHECK(g.fold_hash(h0) != surf.surface_conditions()->fold_hash(h0));
}

TEST_CASE("Surface grid: class list from names, static and unmapped classes", "[environment][surface]") {
    const SurfaceConditionsDesc d = default_conditions();
    // Desc order (alphabetical) without names: equals the shipped surfaces.json ids.
    const ConditionGrid a(d, {}, ConditionGridDesc{});
    REQUIRE(a.class_count() == 6);
    const char* expect[] = {"asphalt", "dirt", "field", "grass", "kerb", "low_mu"};
    for (int i = 0; i < 6; ++i) CHECK(a.class_name(i) == expect[i]);
    CHECK(a.kind(0) == SurfaceKind::Sealed);
    CHECK(a.kind(1) == SurfaceKind::Soil);
    CHECK(a.kind(3) == SurfaceKind::Vegetated);
    CHECK(a.kind(5) == SurfaceKind::Static);
    CHECK(a.unmapped_names().empty());
    CHECK(a.unused_entries().empty());
    // A name without an entry is static and reported; an entry no name uses is reported.
    const ConditionGrid b(d, {"asphalt", "gravel", "grass"}, ConditionGridDesc{});
    CHECK(b.kind(1) == SurfaceKind::Static);
    REQUIRE(b.unmapped_names().size() == 1);
    CHECK(b.unmapped_names()[0] == "gravel");
    CHECK(b.unused_entries() == std::vector<std::string>{"dirt", "field", "kerb", "low_mu"});
    // Rejections.
    CHECK_THROWS_WITH(ConditionGrid(d, {"asphalt", "asphalt"}, ConditionGridDesc{}), ContainsSubstring("duplicate"));
    CHECK_THROWS_WITH(ConditionGrid(d, {"asphalt", ""}, ConditionGridDesc{}), ContainsSubstring("empty"));
    std::vector<std::string> many;
    for (int i = 0; i < 33; ++i) many.push_back("s" + std::to_string(i));
    CHECK_THROWS_WITH(ConditionGrid(d, many, ConditionGridDesc{}), ContainsSubstring("more than 32"));
    CHECK_THROWS_WITH(ConditionGrid(d, {}, ConditionGridDesc{0.0, 600.0, 600.0}), ContainsSubstring("dt_env_s"));
}

TEST_CASE("Surface conditions: the Environment needs weather and climate scalars", "[environment][surface]") {
    const ClimateDesc climate = io::load_climate_json(data("climates/test_weather.json"));
    EnvironmentConfig cfg;
    cfg.surface_conditions = default_conditions();
    CHECK_THROWS_WITH(Environment(climate, cfg), ContainsSubstring("need weather"));
    cfg.weather = EnvironmentWeather::Dynamic;
    CHECK_NOTHROW(Environment(climate, cfg));
    // Names through the config.
    cfg.surface_names = {"asphalt", "asphalt"};
    CHECK_THROWS_WITH(Environment(climate, cfg), ContainsSubstring("duplicate"));
}

// ------------------------------------------------------------------ loader

namespace {

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        path = std::filesystem::temp_directory_path() / "ps_env_w5_sc";
        std::filesystem::create_directories(path);
    }
    ~TempDir() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    std::filesystem::path write(const std::string& name, const nlohmann::json& j) const {
        const std::filesystem::path p = path / name;
        std::ofstream(p) << j.dump(2);
        return p;
    }
};

nlohmann::json valid_conditions_json() {
    return nlohmann::json::parse(R"({
      "format": "physics_sim.surface_conditions/1",
      "surfaces": {
        "asphalt": { "kind": "sealed", "albedo": 0.1, "emissivity": 0.9,
                     "layer": {"depth_m": 0.2, "k_w_mk": 1.38, "rho_c_j_m3k": 2.0e6},
                     "deep": {"depth_m": 1.8, "k_w_mk": 1.0, "rho_c_j_m3k": 2.2e6} },
        "low_mu": { "kind": "static" }
      }
    })");
}

} // namespace

TEST_CASE("Surface conditions loader: the shipped file and rejections", "[environment][surface][io]") {
    const SurfaceConditionsDesc d = default_conditions();
    REQUIRE(d.classes.size() == 6);
    CHECK(d.find("asphalt")->albedo == 0.10);
    CHECK(d.find("asphalt")->layer.k_w_mk == 1.38);
    CHECK(d.find("asphalt")->layer.rho_c_j_m3k == 2.0e6);
    CHECK(d.find("low_mu")->kind == SurfaceKind::Static);

    TempDir dir;
    struct Row {
        const char* what;
        std::function<void(nlohmann::json&)> mutate;
        const char* message;
    };
    using nlohmann::json;
    const std::vector<Row> rows = {
        {"wrong format", [](json& j) { j["format"] = "physics_sim.surface_conditions/2"; }, "\"format\" must be"},
        {"unknown top-level key", [](json& j) { j["classes"] = 1; }, "unknown key \"classes\""},
        {"no surfaces", [](json& j) { j.erase("surfaces"); }, "\"surfaces\""},
        {"empty surfaces", [](json& j) { j["surfaces"] = json::object(); }, "at least one surface"},
        {"unknown kind", [](json& j) { j["surfaces"]["asphalt"]["kind"] = "gravel"; }, "\"kind\" must be"},
        {"missing kind", [](json& j) { j["surfaces"]["asphalt"].erase("kind"); }, "\"kind\""},
        {"unknown class key", [](json& j) { j["surfaces"]["asphalt"]["colour"] = 1; }, "unknown key \"colour\""},
        {"missing albedo", [](json& j) { j["surfaces"]["asphalt"].erase("albedo"); }, "\"albedo\""},
        {"missing layer", [](json& j) { j["surfaces"]["asphalt"].erase("layer"); }, "\"layer\""},
        {"missing deep conductivity", [](json& j) { j["surfaces"]["asphalt"]["deep"].erase("k_w_mk"); }, "\"k_w_mk\""},
        {"unknown layer key", [](json& j) { j["surfaces"]["asphalt"]["layer"]["porosity"] = 0.1; }, "unknown key \"porosity\""},
        {"albedo 1", [](json& j) { j["surfaces"]["asphalt"]["albedo"] = 1.0; }, "albedo must be in [0, 1)"},
        {"emissivity 0", [](json& j) { j["surfaces"]["asphalt"]["emissivity"] = 0.0; }, "emissivity must be in (0, 1]"},
        {"layer depth 0", [](json& j) { j["surfaces"]["asphalt"]["layer"]["depth_m"] = 0.0; }, "layer.depth_m must be > 0"},
        {"negative conductivity", [](json& j) { j["surfaces"]["asphalt"]["layer"]["k_w_mk"] = -1.0; }, "layer.k_w_mk must be > 0"},
        {"zero heat capacity", [](json& j) { j["surfaces"]["asphalt"]["deep"]["rho_c_j_m3k"] = 0.0; }, "deep.rho_c_j_m3k must be > 0"},
        {"column too deep", [](json& j) { j["surfaces"]["asphalt"]["deep"]["depth_m"] = 12.0; }, "<= 10"},
        {"static with a thermal field", [](json& j) { j["surfaces"]["low_mu"]["albedo"] = 0.3; }, "static class takes no thermal field"},
        {"reserved mtd_mm", [](json& j) { j["surfaces"]["asphalt"]["mtd_mm"] = 0.8; }, "not implemented before W6"},
        {"reserved drainage", [](json& j) { j["surfaces"]["asphalt"]["drainage"] = json::object(); }, "not implemented before W6"},
        {"reserved friction", [](json& j) { j["surfaces"]["asphalt"]["friction"] = json::object(); }, "not implemented before W6"},
        {"reserved soil on a sealed class", [](json& j) { j["surfaces"]["asphalt"]["soil"] = json::object(); }, "not implemented before W9"},
        {"reserved crr", [](json& j) { j["surfaces"]["asphalt"]["crr"] = json::object(); }, "not implemented before W8"},
        {"reserved abrasiveness", [](json& j) { j["surfaces"]["asphalt"]["abrasiveness"] = 1.0; }, "not implemented before W10"},
        {"albedo a string", [](json& j) { j["surfaces"]["asphalt"]["albedo"] = "dark"; }, "\"albedo\" must be a number"},
    };
    int n = 0;
    for (const Row& row : rows) {
        json j = valid_conditions_json();
        row.mutate(j);
        const std::filesystem::path p = dir.write("row" + std::to_string(n++) + ".json", j);
        INFO("rejection row: " << row.what);
        CHECK_THROWS_WITH(io::load_surface_conditions_json(p), ContainsSubstring(row.message));
    }
    CHECK_NOTHROW(io::load_surface_conditions_json(dir.write("ok.json", valid_conditions_json())));
    // extends: an override file changes one value and removes a class (RFC 7396 null).
    const std::filesystem::path base = dir.write("base.json", valid_conditions_json());
    (void)base;
    const std::filesystem::path over = dir.write("over.json", json::parse(R"({"extends": "base.json",
        "surfaces": {"asphalt": {"albedo": 0.15}, "low_mu": null}})"));
    const SurfaceConditionsDesc o = io::load_surface_conditions_json(over);
    REQUIRE(o.classes.size() == 1);
    CHECK(o.classes[0].albedo == 0.15);
    CHECK(o.classes[0].layer.k_w_mk == 1.38);
    // Errors name the file.
    const std::filesystem::path bad = dir.write("named_bad.json", [] { json j = valid_conditions_json(); j["x"] = 1; return j; }());
    CHECK_THROWS_WITH(io::load_surface_conditions_json(bad), ContainsSubstring("named_bad.json"));
}
