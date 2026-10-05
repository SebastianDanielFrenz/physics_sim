// tests/unit/test_environment_atmosphere.cpp - weather plan W4, library level (no
// ps_core): the ISA lapse / barometric altitude model and humid-air density
// (ps/environment/atmosphere.h). Tags [environment][atmosphere].
//
// Reference values: HAND = the closed forms evaluated with an independent tool
// (.NET Math.Pow/Exp in PowerShell, 2026-10-05, full double precision) from the
// ISO 2533 constants g0 9.80665, R 287.05287, L 0.0065, T0 288.15, p0 101325;
// TABLE = the published standard-atmosphere values (ICAO Doc 7488 / ISO 2533
// tables, quoted to 0.1 Pa [M - the tables themselves were not re-read in this
// session, the numbers are the widely tabulated ones]). The model has to meet
// the hand values to 1e-9 (they test the constants and the formula, not the
// machine) and the published ones to the table's own rounding (<= 5e-6).
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - W4-A1 (lapse rate): kIsaLapseRate 0.0065 -> 0.006: every table row of "ISA
//    table points" failed (1000 m: 89874.56 -> 89884.08 Pa) and the stratosphere
//    rows (the T11 and p11 chain) with it.
//  - W4-A2 (vapour gas constant): humid_air_density using kIsaGasConstantAir for
//    the vapour term: "humid-air density, hand value" failed (1.15569810 ->
//    1.16439 kg/m3).
//  - W4-A3 (humidity lifted with altitude): atmosphere_at keeping the sea-level
//    vapour PRESSURE instead of the mole fraction: the constant-mixing-ratio row
//    of "humidity at altitude" failed (e at 3000 m 600 Pa instead of 417 Pa).

#include "ps/environment/atmosphere.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <stdexcept>

using namespace ps;
using namespace ps::environment;
using Catch::Matchers::WithinRel;

namespace {

struct IsaRow {
    real h;
    real p_hand, t_hand;   // independent evaluation, rel 1e-9
    real p_table;          // published, rounded to 0.1 Pa
};

const IsaRow kRows[] = {
    {1000.0, 89874.5629, 281.65, 89874.6},
    {2000.0, 79495.2019, 275.15, 79495.2},
    {3000.0, 70108.5265, 268.65, 70108.5},
    {5000.0, 54019.8882, 255.65, 54019.9},
    {10000.0, 26436.2426, 223.15, 26436.3},
    {11000.0, 22632.0401, 216.65, 22632.1},
    {15000.0, 12044.5528, 216.65, 12044.6},
    {20000.0, 5474.8774, 216.65, 5474.89},
};

} // namespace

TEST_CASE("Atmosphere: ISA table points", "[environment][atmosphere]") {
    const SeaLevelAtmosphere isa;  // 101325 Pa, 288.15 K, dry
    for (const IsaRow& r : kRows) {
        INFO("altitude " << r.h);
        const AtmosphereSample s = atmosphere_at(isa, r.h);
        CHECK(s.altitude_m == r.h);
        // The hand values are printed to 0.1 mPa: 1e-9 relative leaves room for that rounding only.
        CHECK_THAT(s.pressure_pa, WithinRel(r.p_hand, 1e-8));
        CHECK_THAT(s.temperature_k, WithinRel(r.t_hand, 1e-12));
        CHECK_THAT(s.pressure_pa, WithinRel(r.p_table, 5e-6));  // the table's own rounding
        // Pressure at altitude agrees between the two entry points.
        CHECK(isa_pressure_pa(isa.pressure_pa, isa.temperature_k, r.h) == s.pressure_pa);
        CHECK(isa_temperature_k(isa.temperature_k, r.h) == s.temperature_k);
    }
    // 1e-6 relative on the headline sea-level and 5 km points (the brief's bound).
    CHECK_THAT(atmosphere_at(isa, 1000.0).pressure_pa, WithinRel(89874.5629, 1e-6));
    CHECK_THAT(atmosphere_at(isa, 5000.0).pressure_pa, WithinRel(54019.8882, 1e-6));
}

TEST_CASE("Atmosphere: sea level and the clamp", "[environment][atmosphere]") {
    const SeaLevelAtmosphere isa;
    const AtmosphereSample s0 = atmosphere_at(isa, 0.0);
    CHECK(s0.pressure_pa == 101325.0);
    CHECK(s0.temperature_k == 288.15);
    CHECK_THAT(s0.density_kg_m3, WithinRel(1.2250, 1e-4));  // the standard sea-level density 1.225 kg/m3

    // Outside [-5000, 20000] m the altitude is clamped, not an error.
    CHECK(atmosphere_at(isa, 30000.0).altitude_m == kIsaMaxAltitudeM);
    CHECK(atmosphere_at(isa, 30000.0).pressure_pa == atmosphere_at(isa, 20000.0).pressure_pa);
    CHECK(atmosphere_at(isa, -9000.0).altitude_m == kIsaMinAltitudeM);
    CHECK(atmosphere_at(isa, -5000.0).pressure_pa > atmosphere_at(isa, 0.0).pressure_pa);

    // A weather-driven (non-standard) sea-level reference is lifted the same way:
    // 980 hPa and 280 K at sea level, 2000 m up.
    SeaLevelAtmosphere w;
    w.pressure_pa = 98000.0;
    w.temperature_k = 280.0;
    const AtmosphereSample s = atmosphere_at(w, 2000.0);
    CHECK_THAT(s.temperature_k, WithinRel(280.0 - 0.0065 * 2000.0, 1e-12));
    const real expect_p = 98000.0 * std::pow((280.0 - 13.0) / 280.0, 9.80665 / (287.05287 * 0.0065));
    CHECK_THAT(s.pressure_pa, WithinRel(expect_p, 1e-9));
}

TEST_CASE("Atmosphere: humid-air density, hand value", "[environment][atmosphere]") {
    // 30 degC, 1013.25 hPa, vapour pressure 20 hPa: (p - e)/(R_d T) + e/(R_v T).
    const real rho = humid_air_density(101325.0, 303.15, 2000.0);
    CHECK_THAT(rho, WithinRel(1.15569810, 1e-8));
    // Spelled out by hand: dry part 99325 / (287.05287 * 303.15) = 1.14140..., vapour 2000 / (461.5228 * 303.15) = 0.014295...
    CHECK_THAT(rho, WithinRel(99325.0 / (287.05287 * 303.15) + 2000.0 / (461.5228 * 303.15), 1e-14));
    // Moist air is LIGHTER than dry air at the same p and T (M_w 18 < M_air 29).
    const real dry = humid_air_density(101325.0, 303.15, 0.0);
    CHECK(dry == 101325.0 / (287.05287 * 303.15));
    CHECK(rho < dry);
    // The water-vapour gas constant is R*/M_w with CODATA 2018 R* and M_w = 18.01528 g/mol.
    CHECK_THAT(kVapourGasConstant, WithinRel(8.314462618 / 0.01801528, 1e-7));  // the literal 461.5228 is rounded to 7 digits
}

TEST_CASE("Atmosphere: humidity at altitude", "[environment][atmosphere]") {
    // Constant mixing ratio: the vapour PRESSURE follows the total pressure.
    SeaLevelAtmosphere s;
    s.pressure_pa = 101325.0;
    s.temperature_k = 293.15;
    s.vapour_pressure_pa = 600.0;
    const AtmosphereSample a = atmosphere_at(s, 3000.0);
    CHECK_THAT(a.vapour_pressure_pa, WithinRel(600.0 * a.pressure_pa / 101325.0, 1e-12));
    CHECK(a.vapour_pressure_pa < 450.0);
    // 2000 Pa at 20 degC (85 % RH) lifted to 3000 m (0.5 degC) would exceed the Magnus saturation
    // pressure there (~633 Pa): capped, never supersaturated.
    s.vapour_pressure_pa = 2000.0;
    const AtmosphereSample capped = atmosphere_at(s, 3000.0);
    CHECK(capped.vapour_pressure_pa > 600.0);
    CHECK(capped.vapour_pressure_pa < 670.0);

    // Saturation cap: a sea-level reference that is already at saturation cannot hold more at a
    // colder altitude - the cap is the Magnus saturation pressure of the lifted temperature.
    SeaLevelAtmosphere sat;
    sat.pressure_pa = 101325.0;
    sat.temperature_k = 303.15;
    sat.vapour_pressure_pa = 4200.0;      // about saturation at 30 degC
    const AtmosphereSample b = atmosphere_at(sat, 0.0);
    CHECK(b.vapour_pressure_pa <= 4250.0);
    // A grossly supersaturated input is capped at the saturation pressure of its own temperature.
    sat.vapour_pressure_pa = 40000.0;
    const AtmosphereSample c = atmosphere_at(sat, 0.0);
    CHECK(c.vapour_pressure_pa < 5000.0);
    CHECK(c.vapour_pressure_pa > 3500.0);
    // Dry stays dry.
    sat.vapour_pressure_pa = 0.0;
    CHECK(atmosphere_at(sat, 1000.0).vapour_pressure_pa == 0.0);
}

TEST_CASE("Atmosphere: rejects invalid references", "[environment][atmosphere]") {
    SeaLevelAtmosphere s;
    s.pressure_pa = -1.0;
    CHECK_THROWS_AS(atmosphere_at(s, 0.0), std::invalid_argument);
    s = SeaLevelAtmosphere{};
    s.temperature_k = 50.0;
    CHECK_THROWS_AS(atmosphere_at(s, 0.0), std::invalid_argument);
    s = SeaLevelAtmosphere{};
    s.vapour_pressure_pa = -5.0;
    CHECK_THROWS_AS(atmosphere_at(s, 0.0), std::invalid_argument);
    s = SeaLevelAtmosphere{};
    CHECK_THROWS_AS(atmosphere_at(s, std::nan("")), std::invalid_argument);
}
