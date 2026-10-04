// tests/unit/test_environment_sun.cpp - weather plan W1: E3 (astronomical sun
// vs NOAA), E5 (polar / southern / equatorial cases), the warp, the authored
// curve and the clear-sky irradiance. Tags [environment][sun].
//
// REFERENCE for E3: NOAA GML's own solar calculator code (main.js of
// https://gml.noaa.gov/grad/solcalc/, Meeus algorithms, fetched 2026-10-05, run
// unmodified under Deno with a small harness that additionally returns the
// geometric - no refraction - elevation, the hour angle and the declination).
// Inputs: latitude 50.15 N, longitude 8.4547 E, zone 0 (UTC), five UTC times on
// each of 2026-03-20, 06-21, 09-23, 12-21. The literals below are the
// harness output (6 decimals). The model under test is compared AT NOAA's own
// hour angle (the game clock is mean solar time; the equation of time, up to
// +-16 min, is not modelled), so the check covers declination, elevation and
// azimuth geometry, not the equation of time.
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - E3: sun.cpp `st = from_hour_angle(h, decl)` fed -decl (declination sign):
//    "E3 declination" failed at every point (|dDecl| ~ 2*23 deg at the
//    solstices), elevation failed at 15 of 20 points.
//  - E5: solar_geometry.cpp sunrise_hour_angle_rad without the polar clamps
//    (acos of an out-of-range argument): "E5 polar day" produced a NaN day
//    length and failed its finiteness/zero-night checks.

#include "environment_fixtures.h"

#include "ps/environment/calendar.h"
#include "ps/environment/solar_geometry.h"
#include "ps/environment/sun.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ps;
using namespace ps::environment;

namespace {

struct NoaaPoint { const char* date; int doy; double utc_min, ha_deg, decl_deg, az_deg, elev_deg; };
constexpr NoaaPoint kNoaaPoints[] = {
    {"2026-3-20", 79, 300.0, -98.425113, -0.158596, 83.614115, -5.509433},
    {"2026-3-20", 79, 540.0, -38.412795, -0.092709, 134.123171, 30.055803},
    {"2026-3-20", 79, 690.0, -0.905090, -0.051533, 178.822028, 39.792505},
    {"2026-3-20", 79, 840.0, 36.602622, -0.010359, 224.046841, 30.948875},
    {"2026-3-20", 79, 1020.0, 81.611882, 0.039047, 263.566298, 5.393697},
    {"2026-6-21", 172, 300.0, -96.985508, 23.438065, 69.497171, 13.524886},
    {"2026-6-21", 172, 540.0, -36.994574, 23.438129, 119.133096, 50.798482},
    {"2026-6-21", 172, 690.0, 0.499763, 23.438072, 181.020012, 63.285222},
    {"2026-6-21", 172, 840.0, 37.994100, 23.437940, 242.005811, 50.235810},
    {"2026-6-21", 172, 1020.0, 82.987308, 23.437683, 280.505988, 22.156712},
    {"2026-9-23", 266, 300.0, -94.667697, -0.078461, 86.463487, -3.049332},
    {"2026-9-23", 266, 540.0, -34.653047, -0.143335, 138.073916, 31.681028},
    {"2026-9-23", 266, 690.0, 2.856102, -0.183883, 183.708155, 39.606898},
    {"2026-9-23", 266, 840.0, 40.365245, -0.224434, 227.790137, 29.026840},
    {"2026-9-23", 266, 1020.0, 85.376210, -0.273097, 266.272177, 2.750978},
    {"2026-12-21", 355, 300.0, -96.028437, -23.436135, 101.212851, -21.536193},
    {"2026-12-21", 355, 540.0, -36.049075, -23.436887, 146.776839, 9.786837},
    {"2026-12-21", 355, 690.0, 1.438023, -23.437247, 181.375337, 16.401694},
    {"2026-12-21", 355, 840.0, 38.925118, -23.437522, 215.679025, 8.743716},
    {"2026-12-21", 355, 1020.0, 83.909631, -23.437739, 250.136030, -14.063003},
};
struct NoaaDayLength { const char* date; int doy; double minutes; };
constexpr NoaaDayLength kNoaaDayLengths[] = {
    {"2026-3-20", 79, 729.896249},
    {"2026-6-21", 172, 983.728589},
    {"2026-9-23", 266, 728.647415},
    {"2026-12-21", 355, 482.818763},
};

double angle_diff_deg(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

bool all_finite(const SunState& s) {
    return std::isfinite(s.elevation_rad) && std::isfinite(s.azimuth_rad) && std::isfinite(s.dir_east) &&
           std::isfinite(s.dir_north) && std::isfinite(s.dir_up) && std::isfinite(s.declination_rad) &&
           std::isfinite(s.hour_angle_rad);
}

} // namespace

TEST_CASE("E3 astronomical sun equals NOAA at 50.15 N on four dates", "[environment][sun]") {
    const ClimateDesc desc = test_env::real_year(50.15);
    const Calendar cal(desc);
    const Sun sun(desc);

    double worst_decl = 0.0, worst_elev = 0.0, worst_az = 0.0;
    double worst_decl_eq = 0.0, worst_elev_eq = 0.0, worst_az_eq = 0.0;
    for (const NoaaPoint& p : kNoaaPoints) {
        // Game time with the hour angle NOAA reports: H = 0 at 12:00 of the day.
        const double tod = 43200.0 + p.ha_deg / 360.0 * 86400.0;
        const double t = cal.env_time(0, 0, p.doy - 1, tod);
        const SunState s = sun.at(cal, t);
        const double decl_deg = s.declination_rad * kRadToDeg;
        const double elev_deg = s.elevation_rad * kRadToDeg;
        const double az_deg = s.azimuth_rad * kRadToDeg;
        INFO(p.date << " utc " << p.utc_min << " min: decl " << decl_deg << " (NOAA " << p.decl_deg << "), elev "
             << elev_deg << " (" << p.elev_deg << "), az " << az_deg << " (" << p.az_deg << ")");
        // Two tiers (see the header: the Spencer series carries a ~1 day phase
        // error, harmless at the solstices, 0.4 deg of declination at the equinoxes).
        const bool equinox = p.doy == 79 || p.doy == 266;
        const double tol_decl = equinox ? 0.5 : 0.1;
        const double tol_angle = equinox ? 0.5 : 0.2;
        CHECK(std::fabs(decl_deg - p.decl_deg) < tol_decl);                 // E3 declination
        CHECK(std::fabs(elev_deg - p.elev_deg) < tol_angle);                // E3 elevation
        CHECK(std::fabs(angle_diff_deg(az_deg, p.az_deg)) < tol_angle);     // E3 azimuth
        double& wd = equinox ? worst_decl_eq : worst_decl;
        double& we = equinox ? worst_elev_eq : worst_elev;
        double& wa = equinox ? worst_az_eq : worst_az;
        wd = std::max(wd, std::fabs(decl_deg - p.decl_deg));
        we = std::max(we, std::fabs(elev_deg - p.elev_deg));
        wa = std::max(wa, std::fabs(angle_diff_deg(az_deg, p.az_deg)));
        CHECK(s.is_day == (elev_deg > -0.9));   // sunrise at -0.833 deg geometric, within the day-fraction rounding
    }
    WARN("E3 worst deviation vs NOAA, solstices: declination " << worst_decl << " deg, elevation " << worst_elev
         << " deg, azimuth " << worst_az << " deg; equinoxes: declination " << worst_decl_eq << " deg, elevation "
         << worst_elev_eq << " deg, azimuth " << worst_az_eq << " deg");

    // Day length from the sunrise hour angle at 90.833 deg zenith: 2 minutes at the
    // solstices (plan section 13), 5 minutes at the equinoxes (day length changes
    // ~3.5 min per day there, the series is ~1 day late).
    double worst_len = 0.0, worst_len_eq = 0.0;
    for (const NoaaDayLength& d : kNoaaDayLengths) {
        const bool equinox = d.doy == 79 || d.doy == 266;
        const double mine = cal.day(d.doy - 1).day_s / 60.0;
        INFO(d.date << ": " << mine << " min vs NOAA " << d.minutes);
        CHECK(std::fabs(mine - d.minutes) < (equinox ? 5.0 : 2.0));
        double& w = equinox ? worst_len_eq : worst_len;
        w = std::max(w, std::fabs(mine - d.minutes));
    }
    WARN("E3 worst day-length deviation vs NOAA: solstices " << worst_len << " min, equinoxes " << worst_len_eq << " min");
}

namespace {
// NOAA (Meeus) declination at 12:00 UTC, harness_years.mjs, for 5 years x 6 dates;
// y = days since 1 Jan 00:00 of that year at that instant (leap years counted).
struct DeclPoint { int year, month, day; double y; double noaa_decl_deg; bool solstice; };
constexpr DeclPoint kDeclYears[] = {
    {2024, 3, 20, 79.5, 0.147, false},  {2024, 6, 21, 172.5, 23.437, true},  {2024, 9, 22, 265.5, 0.010, false},
    {2024, 12, 21, 355.5, -23.439, true}, {2024, 1, 15, 14.5, -21.163, false}, {2024, 5, 1, 121.5, 15.314, false},
    {2025, 3, 20, 78.5, 0.052, false},  {2025, 6, 21, 171.5, 23.438, true},  {2025, 9, 22, 264.5, 0.104, false},
    {2025, 12, 21, 354.5, -23.438, true}, {2025, 1, 15, 14.5, -21.023, false}, {2025, 5, 1, 120.5, 15.242, false},
    {2026, 3, 20, 78.5, -0.043, false}, {2026, 6, 21, 171.5, 23.438, true},  {2026, 9, 22, 264.5, 0.197, false},
    {2026, 12, 21, 354.5, -23.437, true}, {2026, 1, 15, 14.5, -21.068, false}, {2026, 5, 1, 120.5, 15.169, false},
    {2027, 3, 20, 78.5, -0.139, false}, {2027, 6, 21, 171.5, 23.437, true},  {2027, 9, 22, 264.5, 0.291, false},
    {2027, 12, 21, 354.5, -23.436, true}, {2027, 1, 15, 14.5, -21.112, false}, {2027, 5, 1, 120.5, 15.096, false},
    {2028, 3, 20, 79.5, 0.161, false},  {2028, 6, 21, 172.5, 23.435, true},  {2028, 9, 22, 265.5, -0.004, false},
    {2028, 12, 21, 355.5, -23.436, true}, {2028, 1, 15, 14.5, -21.156, false}, {2028, 5, 1, 121.5, 15.324, false},
};

// Independent evaluation of the Spencer series with the platform libm (test code
// only): the implementation under test must agree to rounding.
double spencer_reference_deg(double y) {
    const double g = 2.0 * 3.14159265358979323846 / 365.0 * (y - 0.5);
    const double d = 0.006918 - 0.399912 * std::cos(g) + 0.070257 * std::sin(g) - 0.006758 * std::cos(2 * g) +
                     0.000907 * std::sin(2 * g) - 0.002697 * std::cos(3 * g) + 0.00148 * std::sin(3 * g);
    return d * 180.0 / 3.14159265358979323846;
}
} // namespace

TEST_CASE("E3 Spencer declination: implementation identity and the leap-cycle error vs NOAA", "[environment][sun]") {
    double worst_equinox = 0.0, worst_solstice = 0.0;
    for (const DeclPoint& p : kDeclYears) {
        const double mine = declination_spencer_rad(p.y) * kRadToDeg;
        INFO(p.year << "-" << p.month << "-" << p.day << " y " << p.y);
        CHECK(std::fabs(mine - spencer_reference_deg(p.y)) < 1e-9);   // implementation = the published series
        const double err = std::fabs(mine - p.noaa_decl_deg);
        if (p.solstice) {
            CHECK(err < 0.05);
            worst_solstice = std::max(worst_solstice, err);
        } else {
            // Spencer's series is a mean-year fit that cannot follow the 4-year leap
            // phase (+-0.3 deg) and is itself ~1 day late at the equinoxes: measured
            // up to 0.53 deg at dDecl/dt = 0.4 deg/day. A sign flip or a one-week
            // phase error would exceed this by far.
            CHECK(err < 0.6);
            worst_equinox = std::max(worst_equinox, err);
        }
    }
    WARN("Spencer vs NOAA over 2024-2028: worst at solstices " << worst_solstice << " deg, elsewhere " << worst_equinox
         << " deg");
    CHECK(worst_equinox > 0.1);   // the documented limitation is real: if it vanished, the tiers above could be tightened
}

TEST_CASE("E3 the warp keeps real culmination and puts sunrise at the authored time", "[environment][sun]") {
    ClimateDesc desc = test_env::real_year(50.15);
    desc.seasons[0].day_night.kind = DayNightKind::Explicit;     // authored 14 h day / 10 h night all year
    desc.seasons[0].day_night.day_s = 50400.0;
    desc.seasons[0].day_night.night_s = 36000.0;
    const Calendar cal(desc);
    const Sun sun(desc);
    const int doy = 172;   // a summer day
    const double lat = 50.15 * kDegToRad;
    const double decl_noon = declination_spencer_rad(doy - 1 + 0.5);

    // Noon is the middle of the authored day and the sun is at its real culmination.
    const double t_noon = cal.env_time(0, 0, doy - 1, 36000.0 / 2.0 + 50400.0 / 2.0);
    const SunState noon = sun.at(cal, t_noon);
    CHECK(std::fabs(noon.elevation_rad - (0.5 * kPi - std::fabs(lat - decl_noon))) < 1e-9);
    CHECK(std::fabs(std::fabs(noon.azimuth_rad - kPi)) < 1e-9);   // due south

    // At the authored sunrise/sunset the sun is on the (refraction) horizon: -0.833 deg
    // geometric, within the small declination drift across the day.
    const double t_rise = cal.env_time(0, 0, doy - 1, 18000.0 + 1e-3);
    const double t_set = cal.env_time(0, 0, doy - 1, 18000.0 + 50400.0 - 1e-3);
    CHECK(std::fabs(sun.at(cal, t_rise).elevation_rad * kRadToDeg + 0.833) < 0.3);
    CHECK(std::fabs(sun.at(cal, t_set).elevation_rad * kRadToDeg + 0.833) < 0.3);
    CHECK(sun.at(cal, t_rise).is_day);
    // A shorter authored day moves the sun faster along the SAME path: the real
    // 16 h day here is longer than 14 h, so at 1 h after sunrise the warped sun is
    // higher than the real one at 1 h after the real sunrise.
    const double e1 = sun.at(cal, cal.env_time(0, 0, doy - 1, 18000.0 + 3600.0)).elevation_rad;
    const double e0 = sun.at(cal, cal.env_time(0, 0, doy - 1, 18000.0)).elevation_rad;
    CHECK(e1 > e0);
}

TEST_CASE("E3 elevation is continuous across midnight and season boundaries", "[environment][sun]") {
    const ClimateDesc desc = test_env::astronomical_four_seasons();
    const Calendar cal(desc);
    const Sun sun(desc);
    // 124 game days; walk every day boundary.
    for (int d = 0; d < cal.days_per_year(); ++d) {
        const DayValues& dv = cal.day(d);
        const double t = cal.env_time(0, dv.season, dv.day_in_season, 0.0);
        const SunState a = sun.at(cal, t - 1e-3);
        const SunState b = sun.at(cal, t + 1e-3);
        INFO("day boundary " << d << " (season " << dv.season << ")");
        CHECK(std::fabs(a.elevation_rad - b.elevation_rad) * kRadToDeg < 0.05);
        CHECK(all_finite(a));
        CHECK(all_finite(b));
    }
    // Over a day the sun moves smoothly: no step larger than 1.5 deg per minute of game time.
    double prev = sun.at(cal, 0.0).elevation_rad;
    for (double t = 60.0; t < 3.0 * 86400.0; t += 60.0) {
        const double e = sun.at(cal, t).elevation_rad;
        CHECK(std::fabs(e - prev) * kRadToDeg < 1.5);
        prev = e;
    }
}

TEST_CASE("E5 polar day and polar night (70 N)", "[environment][sun]") {
    const ClimateDesc desc = test_env::real_year(70.0);
    const Calendar cal(desc);
    const Sun sun(desc);
    const double lat = 70.0 * kDegToRad;

    // 21 June: the sun never sets; the astronomical day is the whole cycle, no night.
    const int june = 171;
    REQUIRE(std::isfinite(cal.day(june).day_s));
    CHECK(cal.day(june).day_s == 86400.0);
    CHECK(cal.day(june).night_s == 0.0);                          // E5 polar day
    double emin = 1e9, emax = -1e9;
    for (int i = 0; i < 288; ++i) {
        const SunState s = sun.at(cal, cal.env_time(0, 0, june, i * 300.0));
        REQUIRE(all_finite(s));
        CHECK(s.elevation_rad > 0.0);
        emin = std::min(emin, s.elevation_rad);
        emax = std::max(emax, s.elevation_rad);
    }
    const double decl = declination_spencer_rad(june + 0.5);
    CHECK(std::fabs(emax - (0.5 * kPi - std::fabs(lat - decl))) * kRadToDeg < 0.2);    // real upper culmination (sampled every 5 min)
    CHECK(std::fabs(emin - (std::fabs(lat + decl) - 0.5 * kPi)) * kRadToDeg < 0.2);    // real lower culmination

    // 21 December: the sun never rises: zero-length day, the whole cycle is night.
    const int dec = 354;
    CHECK(cal.day(dec).day_s == 0.0);
    CHECK(cal.day(dec).night_s == 86400.0);                       // E5 polar night
    for (int i = 0; i < 288; ++i) {
        const SunState s = sun.at(cal, cal.env_time(0, 0, dec, i * 300.0));
        REQUIRE(all_finite(s));
        CHECK(s.elevation_rad < 0.0);
        CHECK_FALSE(s.is_day);
    }
}

TEST_CASE("E5 authored day at a polar latitude falls back to the authored curve", "[environment][sun]") {
    ClimateDesc desc = test_env::real_year(70.0);
    desc.seasons[0].day_night.kind = DayNightKind::Explicit;     // an authored 14 h day, but the real sun never sets in June
    desc.seasons[0].day_night.day_s = 50400.0;
    desc.seasons[0].day_night.night_s = 36000.0;
    const Calendar cal(desc);
    const Sun sun(desc);
    const int june = 171;
    const double lat = 70.0 * kDegToRad;
    const double decl = declination_spencer_rad(june + 0.5);

    const SunState noon = sun.at(cal, cal.env_time(0, 0, june, 18000.0 + 25200.0));
    REQUIRE(all_finite(noon));
    CHECK(noon.authored_curve);
    // Parameters derived from the real culminations: the curve peaks at the real upper culmination.
    CHECK(std::fabs(noon.elevation_rad - (0.5 * kPi - std::fabs(lat - decl))) < 1e-9);
    // Sunrise and sunset instants are on the horizon, the night stays above it (polar day).
    const SunState rise = sun.at(cal, cal.env_time(0, 0, june, 18000.0 + 1e-3));
    CHECK(std::fabs(rise.elevation_rad) < 1e-6);
    const SunState midnight = sun.at(cal, cal.env_time(0, 0, june, 1.0));
    CHECK(midnight.elevation_rad > 0.0);
    CHECK(std::fabs(midnight.elevation_rad - (std::fabs(lat + decl) - 0.5 * kPi)) < 1e-3);
}

TEST_CASE("E5 southern hemisphere, equator and pole", "[environment][sun]") {
    // Sydney-like -33.87: the sun culminates in the NORTH.
    {
        const ClimateDesc desc = test_env::real_year(-33.87);
        const Calendar cal(desc);
        const Sun sun(desc);
        const double lat = -33.87 * kDegToRad;
        for (int doy : {172, 355}) {
            const double decl = declination_spencer_rad(doy - 1 + 0.5);
            const SunState noon = sun.at(cal, cal.env_time(0, 0, doy - 1, 43200.0));
            INFO("doy " << doy);
            CHECK(std::fabs(noon.elevation_rad - (0.5 * kPi - std::fabs(lat - decl))) < 1e-9);
            CHECK(std::fabs(angle_diff_deg(noon.azimuth_rad * kRadToDeg, 0.0)) < 1e-6);     // north
            CHECK(noon.dir_north > 0.0);
        }
        // Morning sun in the east, afternoon in the west.
        const SunState am = sun.at(cal, cal.env_time(0, 0, 171, 36000.0));
        const SunState pm = sun.at(cal, cal.env_time(0, 0, 171, 50000.0));
        CHECK(am.dir_east > 0.0);
        CHECK(pm.dir_east < 0.0);
        // December at -33.87: ~14.4 h day.
        CHECK(cal.day(354).day_s / 3600.0 > 14.0);
        CHECK(cal.day(354).day_s / 3600.0 < 14.8);
    }
    // Equator: ~12.1 h day all year, sun nearly overhead at an equinox noon.
    {
        const ClimateDesc desc = test_env::real_year(0.0);
        const Calendar cal(desc);
        const Sun sun(desc);
        for (int d = 0; d < 365; d += 30) {
            CHECK(cal.day(d).day_s / 3600.0 > 11.9);
            CHECK(cal.day(d).day_s / 3600.0 < 12.4);
        }
        const SunState noon = sun.at(cal, cal.env_time(0, 0, 79, 43200.0));
        CHECK(noon.elevation_rad * kRadToDeg > 88.0);
        for (int i = 0; i < 100; ++i) REQUIRE(all_finite(sun.at(cal, cal.env_time(0, 0, 79, i * 800.0))));
    }
    // Poles: elevation is the declination all day, no NaN.
    for (double lat_deg : {90.0, -90.0}) {
        const ClimateDesc desc = test_env::real_year(lat_deg);
        const Calendar cal(desc);
        const Sun sun(desc);
        const double decl = declination_spencer_rad(171 + 0.5);
        for (int i = 0; i < 24; ++i) {
            const SunState s = sun.at(cal, cal.env_time(0, 0, 171, i * 3600.0));
            INFO("lat " << lat_deg << " hour " << i);
            REQUIRE(all_finite(s));
            CHECK(std::fabs(s.elevation_rad - (lat_deg > 0 ? decl : -decl)) < 1e-3);   // declination drifts ~0.4 deg/day only
        }
    }
}

TEST_CASE("Authored sun curve (owner question Q1, alternative)", "[environment][sun]") {
    ClimateDesc desc = test_env::three_season_explicit();   // authored 60 / 30 / 90 / 270 on every season
    desc.seasons[0].sun_authored.e_max_deg = 40.0;          // season "a" overrides the peak
    const Calendar cal(desc);
    const Sun sun(desc);
    // Season a's midpoint day: 15.
    const DayValues& dv = cal.day(15);
    REQUIRE(dv.has_authored);
    CHECK(dv.authored.e_max_deg == 40.0);
    CHECK(dv.authored.e_night_deg == 30.0);
    const double D = dv.day_s, N = dv.night_s;
    auto at = [&](double tod) { return sun.at(cal, cal.env_time(0, 0, 15, tod)); };

    const SunState rise = at(N / 2.0 + 1e-6);
    const SunState noon = at(N / 2.0 + D / 2.0);
    const SunState set = at(N / 2.0 + D - 1e-6);
    const SunState midnight = at(1e-6);
    CHECK(rise.authored_curve);
    CHECK(std::fabs(rise.elevation_rad) < 1e-6);
    CHECK(std::fabs(noon.elevation_rad * kRadToDeg - 40.0) < 1e-9);
    CHECK(std::fabs(set.elevation_rad) < 1e-6);
    CHECK(std::fabs(midnight.elevation_rad * kRadToDeg + 30.0) < 1e-3);
    CHECK(std::fabs(rise.azimuth_rad * kRadToDeg - 90.0) < 1e-3);
    CHECK(std::fabs(noon.azimuth_rad * kRadToDeg - 180.0) < 1e-9);      // through south
    CHECK(std::fabs(set.azimuth_rad * kRadToDeg - 270.0) < 1e-3);
    CHECK(std::fabs(angle_diff_deg(midnight.azimuth_rad * kRadToDeg, 0.0)) < 0.01);    // night passes through north (may read 360)
    CHECK(noon.is_day);
    CHECK_FALSE(midnight.is_day);
    const double len = std::sqrt(noon.dir_east * noon.dir_east + noon.dir_north * noon.dir_north + noon.dir_up * noon.dir_up);
    CHECK(std::fabs(len - 1.0) < 1e-12);

    // Southern latitude: the day path runs through north.
    desc.location.latitude_deg = -40.0;
    const Calendar cal_s(desc);
    const Sun sun_s(desc);
    const SunState noon_s = sun_s.at(cal_s, cal_s.env_time(0, 0, 15, N / 2.0 + D / 2.0));
    CHECK(std::fabs(angle_diff_deg(noon_s.azimuth_rad * kRadToDeg, 0.0)) < 1e-9);
    const SunState mid_s = sun_s.at(cal_s, cal_s.env_time(0, 0, 15, 1e-6));
    CHECK(std::fabs(mid_s.azimuth_rad * kRadToDeg - 180.0) < 0.01);     // the night passes through south
}

TEST_CASE("Climate descriptor rejections for the sun", "[environment][sun]") {
    ClimateDesc d = test_env::real_year(50.0);
    d.seasons[0].has_doy_span = false;
    CHECK(!climate_validation_error(d).empty());
    d = test_env::three_season_explicit();
    d.sun_authored.reset();
    CHECK(!climate_validation_error(d).empty());   // authored mode without any authored block
    d.sun_authored = SunAuthoredDesc{120.0, 30.0, 90.0, 270.0};
    CHECK(!climate_validation_error(d).empty());   // e_max out of range
    d = test_env::three_season_explicit();
    d.location.latitude_deg = 91.0;
    CHECK(!climate_validation_error(d).empty());
}

TEST_CASE("Clear-sky irradiance (Haurwitz) and cloud reduction (Kasten-Czeplak)", "[environment][sun]") {
    // Independent evaluation with the platform libm (a test, not core/).
    for (double e_deg : {5.0, 15.0, 30.0, 60.0, 90.0}) {
        const double cz = std::sin(e_deg * kDegToRad);
        const double expected = 1098.0 * cz * std::exp(-0.059 / cz);
        CHECK(std::fabs(clear_sky_ghi_w_m2(e_deg * kDegToRad) - expected) < 1e-9 * (1.0 + expected));
    }
    CHECK(clear_sky_ghi_w_m2(0.0) == 0.0);
    CHECK(clear_sky_ghi_w_m2(-0.3) == 0.0);
    CHECK(clear_sky_ghi_w_m2(90.0 * kDegToRad) > 1000.0);
    CHECK(clear_sky_ghi_w_m2(90.0 * kDegToRad) < 1098.0);
    const double e = 40.0 * kDegToRad;
    CHECK(ghi_w_m2(e, 0.0) == clear_sky_ghi_w_m2(e));
    for (double n : {0.25, 0.5, 1.0}) {
        const double expected = clear_sky_ghi_w_m2(e) * (1.0 - 0.75 * std::pow(n, 3.4));
        CHECK(std::fabs(ghi_w_m2(e, n) - expected) < 1e-9 * expected);
    }
    CHECK(std::fabs(ghi_w_m2(e, 1.0) / clear_sky_ghi_w_m2(e) - 0.25) < 1e-12);
    CHECK(ghi_w_m2(e, 2.0) == ghi_w_m2(e, 1.0));   // cloud fraction clamped
}
