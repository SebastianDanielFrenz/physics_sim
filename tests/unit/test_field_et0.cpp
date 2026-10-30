// tests/unit/test_field_et0.cpp - FAO-56 Penman-Monteith (ps/field/et0.h) against
// the worked examples printed in Allen et al. 1998, FAO Irrigation and Drainage
// Paper 56, chapter 4: Example 18 (daily, Uccle, 6 July) and Example 17
// (monthly, Bangkok, April). Every intermediate value FAO prints is compared to
// its printed precision (half a unit of the last printed digit, plus the one
// unit FAO's own rounding of chained inputs costs, noted per line).
//
// Recorded sabotage (F1a, 2026-10-31): the aerodynamic term's wind coefficient
// 0.34 changed to 0.40 in fao56_et0 -> "1 + 0.34 u2", the denominator ratios and
// ET0 of both examples fail; the Rnl clear-sky ratio 1.35 changed to 1.30 ->
// Rnl and ET0 of both fail; both reverted.

#include "ps/field/et0.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace ps::field;
using Catch::Matchers::WithinAbs;

TEST_CASE("FAO-56 example 18: daily ET0, Uccle 6 July", "[field][et0]") {
    const double lat = 50.0 + 48.0 / 60.0;
    const double z = 100.0;
    const int J = 187;

    const double u2 = wind_speed_2m(10.0 / 3.6, 10.0);          // FAO: 10 km/h at 10 m, printed 2.78 m/s
    CHECK_THAT(u2, WithinAbs(2.078, 0.0006));                       // printed 2.078

    const double ea = ea_from_rh_extremes_kpa(12.3, 21.5, 0.84, 0.63);
    CHECK_THAT(saturation_vapour_pressure_kpa(21.5), WithinAbs(2.564, 0.0006));   // e0(Tmax)
    CHECK_THAT(saturation_vapour_pressure_kpa(12.3), WithinAbs(1.431, 0.0006));   // e0(Tmin)
    CHECK_THAT(ea, WithinAbs(1.409, 0.0006));

    const double ra = extraterrestrial_radiation(lat, J);
    const double n_day = daylight_hours(lat, J);
    CHECK_THAT(ra, WithinAbs(41.09, 0.006));
    CHECK_THAT(n_day, WithinAbs(16.1, 0.05));                       // printed 16.1 (and once 16.3)

    const double rs = rs_from_sunshine(9.25, n_day, ra);
    CHECK_THAT(rs, WithinAbs(22.07, 0.02));                         // FAO rounds n/N to 0.57 before multiplying

    Et0Input in;
    in.tmin_c = 12.3;
    in.tmax_c = 21.5;
    in.ea_kpa = ea;
    in.u2_m_s = u2;
    in.rs_mj_m2 = rs;
    in.latitude_deg = lat;
    in.elevation_m = z;
    in.day_of_year = J;
    const Et0Result r = fao56_et0(in);

    CHECK_THAT(r.tmean_c, WithinAbs(16.9, 0.0001));
    CHECK_THAT(r.pressure_kpa, WithinAbs(100.1, 0.05));
    CHECK_THAT(r.gamma, WithinAbs(0.0666, 0.00006));
    CHECK_THAT(r.delta, WithinAbs(0.122, 0.0006));
    CHECK_THAT(r.es_kpa, WithinAbs(1.998, 0.002));                  // FAO prints the sum without halving; 1.997/1.998
    CHECK_THAT(r.es_kpa - r.ea_kpa, WithinAbs(0.589, 0.002));
    CHECK_THAT(r.rso, WithinAbs(30.90, 0.01));
    CHECK_THAT(r.rns, WithinAbs(17.00, 0.02));
    CHECK_THAT(r.rnl, WithinAbs(3.71, 0.02));
    CHECK_THAT(r.rn, WithinAbs(13.28, 0.03));
    CHECK_THAT(r.radiation_term, WithinAbs(2.81, 0.015));
    CHECK_THAT(r.aero_term, WithinAbs(1.07, 0.01));
    CHECK_THAT(r.et0_mm, WithinAbs(3.88, 0.02));                    // printed 3.88, reported as 3.9
}

TEST_CASE("FAO-56 example 17: monthly ET0, Bangkok April", "[field][et0]") {
    const double lat = 13.73;
    const int J = 105;
    const double ra = extraterrestrial_radiation(lat, J);
    const double n_day = daylight_hours(lat, J);
    CHECK_THAT(ra, WithinAbs(38.06, 0.006));
    CHECK_THAT(n_day, WithinAbs(12.31, 0.006));
    const double rs = rs_from_sunshine(8.5, n_day, ra);
    CHECK_THAT(rs, WithinAbs(22.65, 0.006));

    Et0Input in;
    in.tmin_c = 25.6;
    in.tmax_c = 34.8;
    in.ea_kpa = 2.85;
    in.u2_m_s = 2.0;
    in.rs_mj_m2 = rs;
    in.latitude_deg = lat;
    in.elevation_m = 2.0;
    in.day_of_year = J;
    in.g_mj_m2 = 0.14 * (30.2 - 29.2);       // eq 43 monthly soil heat flux, printed 0.14
    const Et0Result r = fao56_et0(in);

    CHECK_THAT(r.pressure_kpa, WithinAbs(101.3, 0.05));
    CHECK_THAT(r.gamma, WithinAbs(0.0674, 0.00006));
    CHECK_THAT(r.delta, WithinAbs(0.246, 0.0006));
    CHECK_THAT(r.es_kpa, WithinAbs(4.42, 0.006));
    CHECK_THAT(r.rso, WithinAbs(28.54, 0.006));
    CHECK_THAT(r.rns, WithinAbs(17.44, 0.006));
    CHECK_THAT(r.rnl, WithinAbs(3.11, 0.01));
    CHECK_THAT(r.rn, WithinAbs(14.33, 0.01));
    CHECK_THAT(r.radiation_term, WithinAbs(3.97, 0.01));
    CHECK_THAT(r.aero_term, WithinAbs(1.75, 0.01));
    CHECK_THAT(r.et0_mm, WithinAbs(5.72, 0.01));                    // printed 5.72, reported as 5.7
}

TEST_CASE("FAO-56 ET0 is never negative and has no hidden state", "[field][et0]") {
    Et0Input in;
    in.tmin_c = -15.0;
    in.tmax_c = -12.0;
    in.ea_kpa = 0.18;
    in.u2_m_s = 0.5;
    in.rs_mj_m2 = 0.5;
    in.latitude_deg = 55.0;
    in.elevation_m = 100.0;
    in.day_of_year = 355;
    const Et0Result a = fao56_et0(in);
    const Et0Result b = fao56_et0(in);
    CHECK(a.et0_mm >= 0.0);
    CHECK(a.et0_mm == b.et0_mm);
}
