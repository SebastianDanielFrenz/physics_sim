// tests/unit/test_cited_engine_data.cpp — PHYS-033 P4 I7a C7: independent
// verification that data/engines/toyota_a25a_fks.json and
// data/fuels/tier3_e10_cert.json's numbers are what their own "source"
// fields claim - literal transcription checks against the underlying xlsx
// citations (own words: sheet/row), an independent re-derivation of
// afr_stoich from the fuel file's own measured elemental mass fractions,
// a demonstration that a single corrupted digit is caught by this file's
// own literal-comparison style, and a whole-vehicle WOT-launch smoke case
// (tests/support/fixtures/vehicles/car_sedan_fuel.json) mirroring test_reference_cars.cpp's
// pattern to prove the new engine+fuel+tank combination actually drives.
//
// This file loads BOTH data files through the production loaders
// (ps::io::load_vehicle_json / ps::io::load_fuel_json) - never hand-copies
// the JSON - so a future edit to either file is caught here if it drifts
// from its own cited source.

#include "vehicle_test_helpers.h"

#include "ps/io/fuel_io.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using Catch::Approx;

namespace {

std::string fuel_path() { return std::string(PS_SOURCE_DIR) + "/data/fuels/tier3_e10_cert.json"; }
std::string car_sedan_fuel_path() { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/car_sedan_fuel.json"; }

// toyota_a25a_fks.json is referenced only through car_sedan_fuel.json's
// powertrain (no standalone engine loader exists, per vehicle_io.h's own
// contract) - extract the one TorqueMapEngineDesc component.
const drivetrain::TorqueMapEngineDesc& engine_of(const vehicle::VehicleDesc& desc) {
    for (const auto& c : desc.powertrain.components) {
        if (const auto* e = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) return *e;
    }
    FAIL("car_sedan_fuel.json has no torque_map engine component");
    static drivetrain::TorqueMapEngineDesc dummy;
    return dummy;
}

} // namespace

// ---------------------------------------------------------------------------
// F11a: literal WOT torque knots (15 points), rebuilt PHYS-033 P4 I7a C7
// review round 1 B1/F1: EACH knot is the MAXIMUM measured brake torque over
// EVERY pedal position on that xlsx speed line (not the pedal==100.00 row -
// at 1326/1498 rpm the ECU's knock/preignition protection enrichment makes
// the pedal-70 rows torque-DOMINANT over pedal-80/100, see B1's own note on
// the fuel-map source field), citing the row of each max. Extended to
// 807-5761 rpm (every measured speed line, F1) plus a DERIVED 6600 rpm knot
// (linear extension of the 5414.5/5761.2 knots - Toyota's own 151 kW rating
// at 6600 rpm cross-checks to 218.59 N*m vs. this knot's 218.48 N*m implied
// by that rating alone, cited only as a cross-check, never as the knot's
// source). The 807 rpm line's max (23.40 N*m) is NOT a WOT knot (F1): it is
// far below any plausible full-throttle torque for this engine - the "High
// Load" sheets never reached fuel-cut/limiter conditions at 807 rpm, so this
// speed line's max is just its highest-pedal steady-state point, not the
// engine's actual WOT capability there. The curve's flat hold below the
// lowest real knot (1003.0 rpm) down to rpm=0 is ASSUMED (F1), not measured.
// ---------------------------------------------------------------------------
TEST_CASE("toyota_a25a_fks.json: WOT torque curve matches the cited xlsx rows exactly",
          "[io][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = ps::io::load_vehicle_json(car_sedan_fuel_path());
    const auto& engine = engine_of(desc);
    const vehicle::Curve1D& wot = engine.wot_torque_nm_vs_rpm;

    // (rpm, Nm) [xlsx row, sheet, pedal] - the MAX measured torque on that
    // speed line, any pedal/sheet (see header comment above).
    struct Point {
        real rpm, nm;
        const char* row;
    };
    const std::vector<Point> kCited = {
        {1003.0, 159.04, "HF row8, pedal 80"},   {1153.2, 176.65, "HF row11, pedal 100"},
        {1325.9, 179.95, "HF row14, pedal 70"},  {1498.5, 189.62, "HF row17, pedal 70"},
        {1728.6, 196.62, "HF row20, pedal 100"}, {2017.1, 216.32, "HF row23, pedal 100"},
        {2534.9, 225.13, "HF row26, pedal 100"}, {2995.6, 217.81, "HF row29, pedal 100"},
        {3571.2, 228.40, "HF row32, pedal 100"}, {4032.2, 228.35, "HF row34, pedal 100"},
        {4493.1, 232.04, "HF row37, pedal 100"}, {4954.0, 231.44, "HF row39, pedal 80"},
        {5414.5, 229.40, "HF row42, pedal 100"}, {5761.2, 227.31, "HF row45, pedal 80"},
        {6600.0, 222.25, "derived (linear extension of the 5414.5/5761.2 knots)"},
    };
    REQUIRE(wot.x.size() == kCited.size());
    REQUIRE(wot.y.size() == kCited.size());
    for (std::size_t i = 0; i < kCited.size(); ++i) {
        INFO("point " << i << " xlsx " << kCited[i].row);
        CHECK(wot.x[i] == Approx(kCited[i].rpm).epsilon(1e-9));
        CHECK(wot.y[i] == Approx(kCited[i].nm).epsilon(1e-9));
    }

    // F1 cross-check ONLY (not the knot's source): Toyota's own rated power
    // (SAE Table 1: 151 kW at 6600 rpm) implies a torque of
    // P/(omega) = 151000 / (6600*2*pi/60) = 218.59 N*m at 6600 rpm - close to
    // (not equal to) this file's own linearly-extended knot, reported so a
    // future reader can judge plausibility, never asserted as authoritative.
    const real p_w = 151000.0;
    const real omega_6600 = 6600.0 * 2.0 * 3.14159265358979323846 / 60.0;
    const real rated_torque_cross_check = p_w / omega_6600;
    std::printf("[cited engine data] F1: 6600 rpm WOT knot=%.2f N*m vs. 151kW-rating cross-check=%.2f N*m\n",
                static_cast<double>(wot.y.back()), static_cast<double>(rated_torque_cross_check));
}

// ---------------------------------------------------------------------------
// F11b: literal motoring torque knots (10 measured + 1 derived x=0 point),
// from the "Steady State" sheet's Accelerator Pedal == 0 AND Torque < 0
// rows (closed-throttle, zero-fuel).
// ---------------------------------------------------------------------------
TEST_CASE("toyota_a25a_fks.json: motoring torque curve matches the cited xlsx rows exactly",
          "[io][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = ps::io::load_vehicle_json(car_sedan_fuel_path());
    const auto& engine = engine_of(desc);
    const vehicle::Curve1D& m = engine.motoring_torque_nm_vs_rpm;

    struct Point {
        real rpm, nm;
        const char* row; // "derived" for the x=0 extrapolated point
    };
    const std::vector<Point> kCited = {
        {0.0, -17.5, "derived (linear extrapolation of the two lowest measured points)"},
        {1326.0, -22.90, "row42"}, {1498.0, -23.60, "row56"}, {1728.0, -23.30, "row70"},  {2017.0, -24.30, "row83"},
        {2535.0, -25.90, "row96"}, {2996.0, -28.40, "row111"}, {3571.0, -30.60, "row124"}, {4032.0, -33.90, "row137"},
        {4493.0, -37.40, "row145"}, {4954.0, -42.30, "row153"},
        {6600.0, -59.80, "derived (linear extension of the two highest measured knots, 4493/4954 - F1)"},
    };
    REQUIRE(m.x.size() == kCited.size());
    REQUIRE(m.y.size() == kCited.size());
    for (std::size_t i = 0; i < kCited.size(); ++i) {
        INFO("point " << i << " " << kCited[i].row);
        CHECK(m.x[i] == Approx(kCited[i].rpm).epsilon(1e-9));
        CHECK(m.y[i] == Approx(kCited[i].nm).epsilon(1e-9));
    }

    // The x=0 point is DERIVED, not measured (engine file's own source
    // field): linear extrapolation of the two lowest measured points
    // (1326, -22.90) and (1498, -23.60) back to rpm=0. Re-derive it here
    // independently rather than just repeating the literal -17.5 above.
    const real slope = (-23.60 - -22.90) / (1498.0 - 1326.0);
    const real y0 = -22.90 - slope * 1326.0;
    CHECK(y0 == Approx(-17.5).margin(0.005)); // engine file rounds to 2dp
}

// ---------------------------------------------------------------------------
// F11c: PHYS-033 P4 I7a C7 review round 1, B2 rewrite. The pre-review
// version of this test was CIRCULAR: 9 of its 11 "fired" points were the
// map's own theta_c==1 WOT-node cells (rounded), so a corrupted INTERIOR
// cell (any load column strictly between 0 and 1) could pass every check
// here undetected - confirmed by a manual +10% corruption of
// fuel_flow_g_s[rpm=2535.0][load=0.5] (see the corruption-experiment
// comment on the deleted F11f test case below), which the old test did not
// catch.
//
// This version has two independent, non-overlapping checks:
//
//  (A) A literal, full-grid transcription check: every one of the map's
//      16 (rpm) x 13 (load) = 208 cells, hand-transcribed from
//      data/engines/toyota_a25a_fks.json itself below (kFuelMapRpm/
//      kFuelMapLoad/kFuelMapValues), is compared via Table2D::value() AT
//      the exact grid node (where value() degenerates to the literal
//      stored cell, no interpolation) - this is the actual, robust
//      corruption detector: hand calculation shows the reviewer's own
//      example corruption (rpm=2535, load=0.5, cell 1.7634 -> +10% ->
//      1.9397) shifts the error at the NEAREST real fired point there
//      (theta_c=0.5693, mostly weighted toward the neighbouring
//      load=0.6 cell) from +0.18% to only about +2.9% - nowhere near
//      failing a fired-row error bound loosened enough to tolerate the
//      known ~41% ECU-enrichment outliers below, whereas the grid literal
//      check catches ANY single-cell change exactly.
//
//  (B) The brief's own requirement: compare the map against every
//      measured FIRED row inside the map's rpm range [807, 6600] that the
//      B1/F1 fit used (xlsx literals below, sheet+row+pedal cited per the
//      out/c7/dump_f11c.py re-extraction), n=182 (SteadyState + HighLoadFinal,
//      HighLoadInitial excluded per the fit's own methodology). theta_c is
//      derived per-point from THIS FILE'S OWN loaded WOT/motoring curves
//      (never a separate hand-copy of them). Aggregate error stats
//      (max/median/>5%/>10%) are asserted against an HONEST measured-with-
//      margin bound: measured max=41.10% at 1498.8 rpm (HF row15, pedal 80,
//      lambda=0.635 - ECU enrichment, see the engine file's own B1 anomaly
//      paragraph, NOT a fit defect), median=0.50%, 23 points over 5%, 11
//      over 10%. The map is never re-tuned to hide these known ECU-enriched
//      outliers.
//
//  Separately (info-only, not asserted): the 26 rows the B1 Pareto-dominance
//  filter DROPPED from the fit (another row on the same speed line makes
//  >= torque on < fuel) are also evaluated against the shipped map and
//  printed, so a reviewer can see the fit did not have to fight those rows.
// ---------------------------------------------------------------------------
TEST_CASE("toyota_a25a_fks.json: fuel map matches its own literal grid and every measured fired row",
          "[io][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = ps::io::load_vehicle_json(car_sedan_fuel_path());
    const auto& engine = engine_of(desc);
    REQUIRE(engine.has_fuel_map);
    const drivetrain::Table2D& map = engine.fuel_map;
    const vehicle::Curve1D& wot = engine.wot_torque_nm_vs_rpm;
    const vehicle::Curve1D& mot = engine.motoring_torque_nm_vs_rpm;

    // --- (A) literal full-grid transcription, hand-copied from
    // data/engines/toyota_a25a_fks.json's fuel_map block (16 rpm rows x 13
    // load columns = 208 cells). ---
    const std::vector<real> kFuelMapRpm = {807.0,  1003.0, 1153.0, 1326.0, 1498.0, 1728.0, 2017.0,
                                            2535.0, 2996.0, 3571.0, 4032.0, 4493.0, 4954.0, 5414.0,
                                            5761.0, 6600.0};
    const std::vector<real> kFuelMapLoad = {0.0, 0.02, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const real kFuelMapValues[16][13] = {
        {0.0000, 0.0204, 0.0510, 0.1020, 0.1992, 0.2822, 0.3641, 0.4473, 0.5350, 0.6339, 0.7328, 0.8317, 0.9306},
        {0.0000, 0.0226, 0.0566, 0.1131, 0.2257, 0.3206, 0.4161, 0.5154, 0.6159, 0.7264, 0.8544, 1.0029, 1.3886},
        {0.0000, 0.0289, 0.0722, 0.1443, 0.2828, 0.3955, 0.5210, 0.6458, 0.7689, 0.9354, 1.1244, 1.3206, 1.9293},
        {0.0000, 0.0343, 0.0856, 0.1713, 0.3301, 0.4622, 0.6101, 0.7479, 0.8829, 1.0412, 1.2341, 1.5421, 2.0220},
        {0.0000, 0.0405, 0.1013, 0.2026, 0.3823, 0.5454, 0.7074, 0.8739, 1.0493, 1.2252, 1.5064, 1.8073, 2.3271},
        {0.0000, 0.0473, 0.1183, 0.2366, 0.4593, 0.6495, 0.8445, 1.0417, 1.2449, 1.4539, 1.7378, 2.2488, 3.8556},
        {0.0000, 0.0587, 0.1468, 0.2937, 0.5626, 0.8135, 1.0702, 1.3292, 1.5814, 1.8620, 2.3529, 2.8546, 3.8120},
        {0.0000, 0.0744, 0.1860, 0.3719, 0.7405, 1.0722, 1.4324, 1.7634, 2.0958, 2.4398, 2.9538, 3.6383, 4.4661},
        {0.0000, 0.0845, 0.2112, 0.4225, 0.8408, 1.2434, 1.6343, 2.0180, 2.4142, 2.8096, 3.2883, 3.9466, 4.8312},
        {0.0000, 0.1081, 0.2701, 0.5401, 1.0802, 1.5919, 2.0825, 2.5768, 3.0572, 3.5588, 4.1587, 4.8987, 6.0946},
        {0.0000, 0.1227, 0.3067, 0.6135, 1.2270, 1.8161, 2.4293, 3.0027, 3.5636, 4.2243, 4.9149, 5.8239, 7.2859},
        {0.0000, 0.1376, 0.3440, 0.6880, 1.3760, 2.0698, 2.7602, 3.4411, 4.1266, 4.8684, 5.6907, 6.7663, 8.1561},
        {0.0000, 0.1508, 0.3769, 0.7538, 1.5077, 2.2576, 3.0405, 3.8188, 4.6183, 5.5062, 6.5733, 7.8303, 9.0873},
        {0.0000, 0.1653, 0.4130, 0.8258, 1.6515, 2.4640, 3.3215, 4.2154, 5.1597, 6.2522, 7.4491, 8.7095, 9.9861},
        {0.0000, 0.2149, 0.5372, 1.0745, 2.1490, 3.2235, 4.2979, 5.3724, 6.4469, 7.5214, 8.5959, 9.6704, 10.8121},
        {0.0000, 0.3348, 0.8375, 1.6758, 3.3519, 5.0599, 6.6587, 8.1699, 9.5592, 10.5902, 11.3687, 11.9937, 12.8093},
    };
    REQUIRE(map.x.size() == kFuelMapRpm.size());
    REQUIRE(map.y.size() == kFuelMapLoad.size());
    for (std::size_t ix = 0; ix < kFuelMapRpm.size(); ++ix) {
        CHECK(map.x[ix] == Approx(kFuelMapRpm[ix]).epsilon(1e-9));
        for (std::size_t iy = 0; iy < kFuelMapLoad.size(); ++iy) {
            if (ix == 0) CHECK(map.y[iy] == Approx(kFuelMapLoad[iy]).epsilon(1e-9));
            INFO("grid cell rpm=" << kFuelMapRpm[ix] << " load=" << kFuelMapLoad[iy]);
            // At an exact grid node, Table2D::value()'s bracket on each axis
            // degenerates (t==0 or t==1) so bilinear collapses to the
            // literal stored cell - this is a transcription check, not an
            // interpolation check.
            CHECK(map.value(kFuelMapRpm[ix], kFuelMapLoad[iy]) ==
                  Approx(kFuelMapValues[ix][iy]).epsilon(1e-9).margin(1e-9));
        }
    }

    // --- (B) every measured fired row in [807, 6600] rpm, from
    // out/c7/dump_f11c.py's re-extraction of the same xlsx
    // (out/c7/f11c_dump.txt), HighLoadInitial rows excluded (transient,
    // pre-stabilization, excluded from the fit itself per B1). ---
    struct Fired {
        real rpm, torque_nm, fmf_g_s;
        const char* cite;
    };
    const std::vector<Fired> kFired = {
        {807.0, 12.90, 0.1910, "SS row12, pedal 4"},       {807.0, 23.40, 0.2380, "SS row13, pedal 6"},
        {807.0, 24.80, 0.2340, "SS row14, pedal 8"},       {807.0, 54.20, 0.3780, "SS row16, pedal 15"},
        {807.0, 82.20, 0.5080, "SS row17, pedal 20"},      {807.0, 108.20, 0.6510, "SS row18, pedal 25"},
        {808.0, 30.70, 0.2710, "SS row15, pedal 12"},      {1003.0, 12.90, 0.2160, "SS row20, pedal 4"},
        {1003.0, 21.20, 0.2650, "SS row21, pedal 7"},      {1003.0, 29.20, 0.3030, "SS row22, pedal 10"},
        {1003.0, 44.10, 0.3800, "SS row23, pedal 15"},     {1003.0, 71.20, 0.5290, "SS row24, pedal 20"},
        {1003.0, 86.80, 0.6160, "SS row25, pedal 25"},     {1003.0, 114.60, 0.7860, "SS row26, pedal 30"},
        {1003.0, 148.90, 1.0680, "SS row27, pedal 35"},    {1003.0, 159.04, 1.3886, "HF row8, pedal 80"},
        {1003.1, 158.80, 1.3868, "HF row7, pedal 100"},    {1003.2, 153.50, 1.0946, "HF row6, pedal 50"},
        {1153.0, 9.90, 0.2330, "SS row29, pedal 4"},       {1153.0, 16.30, 0.2770, "SS row30, pedal 7"},
        {1153.0, 19.80, 0.2930, "SS row31, pedal 10"},     {1153.0, 40.80, 0.4150, "SS row32, pedal 15"},
        {1153.0, 64.50, 0.5670, "SS row33, pedal 20"},     {1153.0, 80.00, 0.6630, "SS row34, pedal 25"},
        {1153.0, 103.90, 0.8110, "SS row35, pedal 30"},    {1153.0, 144.50, 1.1970, "SS row36, pedal 35"},
        {1153.0, 162.80, 1.4330, "SS row37, pedal 40"},    {1153.0, 165.60, 1.4420, "SS row38, pedal 45"},
        {1153.0, 166.30, 1.4280, "SS row39, pedal 50"},    {1153.0, 168.80, 1.4430, "SS row40, pedal 60"},
        {1153.0, 170.90, 1.4420, "SS row41, pedal 55"},    {1153.2, 173.28, 1.9387, "HF row10, pedal 80"},
        {1153.2, 176.65, 1.9316, "HF row11, pedal 100"},   {1153.2, 167.06, 1.4248, "HF row9, pedal 50"},
        {1325.9, 179.95, 2.0221, "HF row14, pedal 70"},    {1326.0, 9.20, 0.2710, "SS row43, pedal 4"},
        {1326.0, 14.60, 0.3090, "SS row44, pedal 7"},      {1326.0, 20.70, 0.3510, "SS row45, pedal 10"},
        {1326.0, 29.30, 0.3990, "SS row46, pedal 15"},     {1326.0, 56.40, 0.5970, "SS row47, pedal 20"},
        {1326.0, 71.90, 0.7070, "SS row48, pedal 25"},     {1326.0, 92.80, 0.8360, "SS row49, pedal 30"},
        {1326.0, 134.70, 1.1630, "SS row50, pedal 35"},    {1326.0, 171.50, 1.7480, "SS row51, pedal 40"},
        {1326.0, 172.50, 1.7460, "SS row52, pedal 45"},    {1326.0, 172.60, 1.7580, "SS row53, pedal 60"},
        {1326.0, 172.70, 1.7400, "SS row54, pedal 50"},    {1326.0, 172.80, 1.7510, "SS row55, pedal 55"},
        {1326.2, 172.94, 2.4647, "HF row13, pedal 100"},   {1326.2, 171.95, 2.4671, "HF row12, pedal 80"},
        {1498.0, 21.60, 0.4010, "SS row59, pedal 10"},     {1498.0, 27.20, 0.4450, "SS row60, pedal 15"},
        {1498.0, 177.80, 1.9960, "SS row68, pedal 50"},    {1498.5, 189.62, 2.3280, "HF row17, pedal 70"},
        {1498.7, 174.55, 3.2877, "HF row16, pedal 100"},   {1498.8, 173.67, 3.2946, "HF row15, pedal 80"},
        {1499.0, 8.60, 0.3060, "SS row57, pedal 4"},       {1499.0, 15.30, 0.3550, "SS row58, pedal 7"},
        {1499.0, 57.20, 0.6740, "SS row61, pedal 20"},     {1499.0, 72.60, 0.7890, "SS row62, pedal 25"},
        {1499.0, 89.50, 0.9270, "SS row63, pedal 30"},     {1499.0, 129.00, 1.2530, "SS row64, pedal 35"},
        {1499.0, 170.80, 1.8430, "SS row65, pedal 40"},    {1499.0, 174.80, 1.9930, "SS row66, pedal 45"},
        {1499.0, 176.30, 1.9780, "SS row67, pedal 60"},    {1499.0, 178.00, 2.0050, "SS row69, pedal 55"},
        {1728.6, 195.73, 2.7176, "HF row19, pedal 70"},    {1728.6, 196.62, 3.8731, "HF row20, pedal 100"},
        {1728.9, 193.53, 3.8649, "HF row18, pedal 80"},    {1729.0, 9.70, 0.3550, "SS row71, pedal 4"},
        {1729.0, 16.40, 0.4200, "SS row72, pedal 7"},      {1729.0, 22.50, 0.4760, "SS row73, pedal 10"},
        {1729.0, 29.80, 0.5360, "SS row74, pedal 15"},     {1729.0, 56.80, 0.7740, "SS row75, pedal 20"},
        {1729.0, 72.30, 0.9130, "SS row76, pedal 25"},     {1729.0, 88.80, 1.0610, "SS row77, pedal 30"},
        {1729.0, 125.70, 1.4030, "SS row78, pedal 35"},    {1729.0, 148.20, 1.6350, "SS row79, pedal 40"},
        {1729.0, 176.90, 2.3030, "SS row80, pedal 45"},    {1729.0, 176.90, 2.3020, "SS row81, pedal 50"},
        {1729.0, 177.60, 2.3040, "SS row82, pedal 55"},    {2016.7, 205.08, 3.1933, "HF row21, pedal 70"},
        {2016.8, 210.34, 3.8056, "HF row22, pedal 80"},    {2017.0, 10.60, 0.4260, "SS row84, pedal 4"},
        {2017.0, 17.10, 0.4930, "SS row85, pedal 10"},     {2017.0, 31.00, 0.6370, "SS row86, pedal 15"},
        {2017.0, 55.10, 0.8890, "SS row87, pedal 20"},     {2017.0, 68.00, 1.0270, "SS row88, pedal 25"},
        {2017.0, 86.90, 1.2340, "SS row89, pedal 30"},     {2017.0, 119.60, 1.5760, "SS row90, pedal 35"},
        {2017.0, 148.50, 1.9130, "SS row91, pedal 40"},    {2017.0, 183.50, 2.6950, "SS row92, pedal 50"},
        {2017.0, 183.90, 2.6980, "SS row93, pedal 45"},    {2017.0, 185.80, 2.6970, "SS row94, pedal 60"},
        {2017.0, 186.70, 2.7080, "SS row95, pedal 55"},    {2017.1, 216.32, 3.8120, "HF row23, pedal 100"},
        {2534.7, 217.28, 4.2986, "HF row24, pedal 70"},    {2534.9, 225.08, 4.4653, "HF row25, pedal 80"},
        {2534.9, 225.13, 4.4660, "HF row26, pedal 100"},   {2535.0, 12.30, 0.5660, "SS row97, pedal 4"},
        {2535.0, 22.40, 0.7160, "SS row98, pedal 10"},     {2535.0, 34.40, 0.8700, "SS row99, pedal 15"},
        {2535.0, 55.70, 1.1570, "SS row100, pedal 20"},    {2535.0, 69.10, 1.3590, "SS row101, pedal 25"},
        {2535.0, 81.70, 1.5300, "SS row102, pedal 30"},    {2535.0, 117.00, 1.9900, "SS row103, pedal 35"},
        {2535.0, 150.20, 2.4450, "SS row104, pedal 40"},   {2535.0, 150.90, 2.4700, "SS row105, pedal 40"},
        {2535.0, 152.10, 2.4650, "SS row106, pedal 40"},   {2535.0, 193.80, 3.3580, "SS row107, pedal 45"},
        {2535.0, 196.30, 3.5390, "SS row108, pedal 55"},   {2535.0, 196.70, 3.5320, "SS row109, pedal 60"},
        {2535.0, 197.30, 3.5340, "SS row110, pedal 50"},   {2995.6, 213.03, 4.8921, "HF row27, pedal 70"},
        {2995.6, 217.81, 4.8310, "HF row29, pedal 100"},   {2995.7, 217.51, 4.8208, "HF row28, pedal 80"},
        {2996.0, 14.20, 0.7310, "SS row112, pedal 4"},     {2996.0, 26.60, 0.9360, "SS row113, pedal 10"},
        {2996.0, 31.50, 1.0120, "SS row114, pedal 15"},    {2996.0, 52.80, 1.3650, "SS row115, pedal 20"},
        {2996.0, 64.60, 1.5550, "SS row116, pedal 25"},    {2996.0, 79.20, 1.7660, "SS row117, pedal 30"},
        {2996.0, 111.20, 2.2860, "SS row118, pedal 35"},   {2996.0, 139.60, 2.7340, "SS row119, pedal 40"},
        {2996.0, 163.10, 3.1420, "SS row120, pedal 45"},   {2996.0, 196.30, 4.0510, "SS row121, pedal 55"},
        {2996.0, 196.70, 4.0610, "SS row122, pedal 60"},   {2996.0, 196.80, 4.0430, "SS row123, pedal 50"},
        {3571.0, 54.40, 1.7420, "SS row128, pedal 20"},    {3571.0, 79.90, 2.2180, "SS row130, pedal 30"},
        {3571.2, 228.40, 6.0950, "HF row32, pedal 100"},   {3571.4, 225.63, 6.0191, "HF row30, pedal 70"},
        {3571.5, 227.69, 6.0264, "HF row31, pedal 80"},    {3572.0, 29.20, 1.2470, "SS row125, pedal 4"},
        {3572.0, 39.20, 1.4340, "SS row126, pedal 10"},    {3572.0, 41.70, 1.4810, "SS row127, pedal 15"},
        {3572.0, 66.10, 1.9470, "SS row129, pedal 25"},    {3572.0, 111.30, 2.8110, "SS row131, pedal 35"},
        {3572.0, 135.20, 3.2470, "SS row132, pedal 40"},   {3572.0, 160.60, 3.7580, "SS row133, pedal 45"},
        {3572.0, 191.10, 4.5220, "SS row134, pedal 50"},   {3572.0, 207.80, 5.0740, "SS row135, pedal 60"},
        {3572.0, 208.10, 5.0800, "SS row136, pedal 55"},   {4031.9, 227.32, 7.2709, "HF row33, pedal 80"},
        {4032.0, 33.00, 1.5650, "SS row138, pedal 4"},     {4032.0, 41.10, 1.7290, "SS row139, pedal 10"},
        {4032.0, 50.80, 1.9590, "SS row140, pedal 20"},    {4032.0, 77.60, 2.5830, "SS row141, pedal 30"},
        {4032.0, 129.50, 3.6930, "SS row142, pedal 40"},   {4032.0, 184.60, 5.1440, "SS row143, pedal 50"},
        {4032.0, 222.80, 6.6260, "SS row144, pedal 60"},   {4032.2, 228.35, 7.2859, "HF row34, pedal 100"},
        {4493.0, 30.00, 1.7210, "SS row146, pedal 4"},     {4493.0, 38.90, 1.9540, "SS row147, pedal 10"},
        {4493.0, 47.90, 2.1840, "SS row148, pedal 20"},    {4493.0, 73.80, 2.8480, "SS row149, pedal 30"},
        {4493.0, 121.70, 4.0560, "SS row150, pedal 40"},   {4493.0, 170.70, 5.4050, "SS row151, pedal 50"},
        {4493.0, 202.10, 6.6090, "SS row152, pedal 55"},   {4493.1, 232.04, 8.1561, "HF row37, pedal 100"},
        {4493.1, 230.13, 8.2219, "HF row35, pedal 70"},    {4493.4, 231.41, 8.1478, "HF row36, pedal 80"},
        {4953.0, 41.10, 2.2910, "SS row156, pedal 20"},    {4953.8, 230.66, 9.1048, "HF row38, pedal 70"},
        {4954.0, 36.10, 2.1590, "SS row154, pedal 10"},    {4954.0, 38.50, 2.2230, "SS row155, pedal 16"},
        {4954.0, 73.20, 3.2130, "SS row157, pedal 30"},    {4954.0, 116.20, 4.4320, "SS row158, pedal 40"},
        {4954.0, 231.44, 9.0871, "HF row39, pedal 80"},    {4955.0, 163.40, 5.9630, "SS row159, pedal 50"},
        {5414.0, 32.70, 2.3850, "SS row160, pedal 10"},    {5414.0, 112.90, 4.9280, "SS row164, pedal 40"},
        {5414.0, 156.90, 6.6660, "SS row165, pedal 50"},   {5414.5, 229.40, 9.9860, "HF row42, pedal 100"},
        {5414.5, 227.99, 9.9057, "HF row40, pedal 70"},    {5414.6, 228.19, 9.9969, "HF row41, pedal 80"},
        {5415.0, 35.70, 2.4620, "SS row161, pedal 16"},    {5415.0, 36.00, 2.4690, "SS row162, pedal 20"},
        {5415.0, 69.90, 3.5220, "SS row163, pedal 30"},    {5760.8, 225.32, 10.6680, "HF row43, pedal 70"},
        {5761.0, 226.24, 10.9407, "HF row44, pedal 100"},  {5761.2, 227.31, 10.8123, "HF row45, pedal 80"},
    };
    REQUIRE(kFired.size() == 182);

    std::vector<real> abs_errs_pct;
    abs_errs_pct.reserve(kFired.size());
    int over5 = 0, over10 = 0;
    real worst_err_pct = 0.0;
    std::string worst_cite;
    for (const Fired& f : kFired) {
        const real w = wot.evaluate(f.rpm);
        const real m = mot.evaluate(f.rpm);
        REQUIRE(w > m); // denominator below must be well-formed
        const real theta_c = std::max(real(0.0), std::min(real(1.0), (f.torque_nm - m) / (w - m)));
        const real predicted = map.value(f.rpm, theta_c);
        const real err_pct = (predicted - f.fmf_g_s) / f.fmf_g_s * real(100.0);
        const real abs_err_pct = std::abs(err_pct);
        abs_errs_pct.push_back(abs_err_pct);
        if (abs_err_pct > real(5.0)) ++over5;
        if (abs_err_pct > real(10.0)) ++over10;
        if (abs_err_pct > worst_err_pct) {
            worst_err_pct = abs_err_pct;
            worst_cite = f.cite;
        }
        INFO("fired point: " << f.cite << " rpm=" << f.rpm << " T=" << f.torque_nm << " theta_c=" << theta_c
                              << " predicted=" << predicted << " measured=" << f.fmf_g_s << " err%=" << err_pct);
    }
    std::sort(abs_errs_pct.begin(), abs_errs_pct.end());
    const std::size_t n = abs_errs_pct.size();
    const real median = (n % 2 == 1) ? abs_errs_pct[n / 2]
                                      : (abs_errs_pct[n / 2 - 1] + abs_errs_pct[n / 2]) * real(0.5);
    std::printf(
        "[cited engine data] F11c fired-row stats: n=%zu max=%.2f%% (%s) median=%.2f%% >5%%=%d >10%%=%d\n",
        n, static_cast<double>(worst_err_pct), worst_cite.c_str(), static_cast<double>(median), over5, over10);
    // Honest measured-with-margin bound (B2): measured n=182, max=41.10%
    // (HF row15, pedal 80, lambda=0.635 - ECU knock/preignition
    // enrichment at low speed/high load, see the engine file's own B1
    // anomaly paragraph), median=0.50%, >5%=23, >10%=11. A small margin is
    // added above every measured figure; the map is NOT re-tuned to bring
    // the known ECU-enriched outliers under a tighter bound.
    CHECK(n == 182);
    CHECK(worst_err_pct <= real(45.0));
    CHECK(median <= real(1.0));
    CHECK(over5 <= 25);
    CHECK(over10 <= 15);

    // --- Info-only: the 26 rows the B1 Pareto-dominance filter dropped
    // from the fit (another row on the same speed line makes >= torque on
    // < fuel), evaluated against the shipped map - not asserted, just
    // printed so a reviewer can see the fit did not have to fight these.
    const std::vector<Fired> kDropped = {
        {1498.8, 173.67, 3.2946, "HF row15, pedal 80 (dominated by HF row16 at same rpm line)"},
        {1498.7, 174.55, 3.2877, "HF row16, pedal 100 (kept as WOT anchor; also listed as dominator)"},
        {1326.2, 171.95, 2.4671, "HF row12, pedal 80"},
        {1326.2, 172.94, 2.4647, "HF row13, pedal 100"},
        {1153.0, 168.80, 1.4430, "SS row40, pedal 60"},
        {1153.0, 166.30, 1.4280, "SS row39, pedal 50"},
        {1153.0, 165.60, 1.4420, "SS row38, pedal 45"},
        {1728.9, 193.53, 3.8649, "HF row18, pedal 80"},
        {1153.2, 173.28, 1.9387, "HF row10, pedal 80"},
        {1326.0, 172.50, 1.7460, "SS row52, pedal 45"},
        {1326.0, 172.60, 1.7580, "SS row53, pedal 60"},
        {1153.0, 162.80, 1.4330, "SS row37, pedal 40"},
        {1729.0, 176.90, 2.3030, "SS row80, pedal 45"},
        {2995.6, 213.03, 4.8921, "HF row27, pedal 70"},
        {1326.0, 171.50, 1.7480, "SS row51, pedal 40"},
        {4493.1, 230.13, 8.2219, "HF row35, pedal 70"},
        {5761.0, 226.24, 10.9407, "HF row44, pedal 100"},
        {1499.0, 174.80, 1.9930, "SS row66, pedal 45"},
        {5414.6, 228.19, 9.9969, "HF row41, pedal 80"},
        {2017.0, 183.90, 2.6980, "SS row93, pedal 45"},
        {4953.8, 230.66, 9.1048, "HF row38, pedal 70"},
        {807.0, 23.40, 0.2380, "SS row13, pedal 6"},
        {2535.0, 150.90, 2.4700, "SS row105, pedal 40"},
        {2996.0, 196.70, 4.0610, "SS row122, pedal 60"},
        {2996.0, 196.30, 4.0510, "SS row121, pedal 55"},
        {2535.0, 196.30, 3.5390, "SS row108, pedal 55"},
    };
    for (const Fired& f : kDropped) {
        const real w = wot.evaluate(f.rpm);
        const real m = mot.evaluate(f.rpm);
        if (!(w > m)) continue;
        const real theta_c = std::max(real(0.0), std::min(real(1.0), (f.torque_nm - m) / (w - m)));
        const real predicted = map.value(f.rpm, theta_c);
        const real err_pct = (predicted - f.fmf_g_s) / f.fmf_g_s * real(100.0);
        std::printf("[cited engine data] F11c dropped (Pareto-dominated) row, info only: %s rpm=%.1f T=%.2f "
                    "theta_c=%.4f predicted=%.4f measured=%.4f err%%=%+.2f\n",
                    f.cite, static_cast<double>(f.rpm), static_cast<double>(f.torque_nm),
                    static_cast<double>(theta_c), static_cast<double>(predicted), static_cast<double>(f.fmf_g_s),
                    static_cast<double>(err_pct));
    }
}

// ---------------------------------------------------------------------------
// F11d/e: tier3_e10_cert.json's literal fuel-property citations, plus an
// independent re-derivation of afr_stoich from the file's OWN cited
// measured elemental mass fractions (Heywood 1988 section 3.2 method),
// reproduced here rather than copied from the source field's prose.
// ---------------------------------------------------------------------------
TEST_CASE("tier3_e10_cert.json: literal cited fuel properties and an independent AFR re-derivation",
          "[io][fuel]") {
    const drivetrain::FuelDesc fuel = ps::io::load_fuel_json(fuel_path());

    // MEASURED (ASTM D240) 18037.00 BTU/lb -> J/kg via exact BTU(IT)/lb
    // conversion factors (1 BTU_IT = 1055.05585262 J, 1 lb = 0.45359237 kg).
    const real lhv_derived = 18037.00 * (1055.05585262 / 0.45359237);
    CHECK(fuel.lhv_j_kg == Approx(lhv_derived).epsilon(1e-9));
    CHECK(fuel.lhv_j_kg == Approx(41954062.0).margin(1.0)); // stored to the nearest whole J/kg

    // MEASURED (ASTM D4052) 0.74326 g/cm^3 -> kg/m^3.
    CHECK(fuel.density_kg_m3 == Approx(743.26).epsilon(1e-9));

    // Independent re-derivation of afr_stoich from the file's own cited
    // MEASURED elemental mass fractions (ASTM D5291/D5599): C=0.8299,
    // H=0.1343, O=0.0358 (Heywood 1988 section 3.2's stoichiometric method,
    // reproduced here rather than trusting the source field's own prose).
    const real C = 0.8299, H = 0.1343, O = 0.0358;
    REQUIRE(C + H + O == Approx(1.0).margin(1e-9));
    const real n_C = C / 12.011;
    const real n_H = H / 1.008;
    const real n_O = O / 15.999;
    const real n_O2_req = n_C + n_H / 4.0 - n_O / 2.0;
    const real air_per_mol_o2 = 31.998 + 3.773 * 28.16; // M_O2 + 3.773*M_N2(atmospheric)
    const real afr_derived = n_O2_req * air_per_mol_o2;
    CHECK(afr_derived == Approx(14.002173341183884).epsilon(1e-9));
    CHECK(fuel.afr_stoich == Approx(afr_derived).epsilon(1e-9));
}

// ---------------------------------------------------------------------------
// F11f: PHYS-033 P4 I7a C7 review round 1, B2. The previous version of this
// TEST_CASE demonstrated corruption-detection via STRING MANIPULATION of a
// temp copy of tier3_e10_cert.json (never the tracked file) - a real but
// narrow demonstration (one field, one file). The review round asked for
// this to be replaced with a one-time MANUAL experiment performed directly
// against the tracked data files during the C7 review round 1 build/test
// pass (not a permanent runtime TEST_CASE, since a real corruption must be
// applied to the actual tracked file to prove anything about the actual
// tracked file's own tests - a copy proves only that the test file's
// string-matching logic works, not that today's numbers are protected).
//
// Procedure for each row below: save the current (fixed, pre-corruption)
// file content, apply the ONE stated edit directly to the real data file,
// rebuild, run the named test(s), record pass/fail, then restore the saved
// content (never `git checkout`, since these files carry this review
// round's own uncommitted B1/B2/B3/F1/F2/F3 fixes that a checkout would
// discard - restore from the saved copy instead).
//
//   1. WOT knot (data/engines/toyota_a25a_fks.json, wot_torque_nm_vs_rpm.y,
//      the 4493.1 rpm knot 232.04 -> 235.00): caught by F11a's exact
//      per-knot literal comparison (Approx(...).epsilon(1e-9)) - fails at
//      that one index immediately.
//   2. Motoring knot (motoring_torque_nm_vs_rpm.y, the 2535.0 rpm knot
//      -25.9 -> -28.0): caught by F11b's exact per-knot literal comparison
//      the same way.
//   3. Interior fuel-map cell (fuel_map.fuel_flow_g_s[rpm=2535.0]
//      [load=0.5], 1.7634 -> 1.9397, +10% - the reviewer's own example):
//      caught by F11c's NEW full-grid literal check (part A above) at
//      that exact cell - the fired-row aggregate-error check (part B)
//      alone would NOT reliably catch this (hand calculation: the nearest
//      real fired point there, theta_c~=0.5693, is weighted only ~31%
//      toward the corrupted load=0.5 column versus ~69% toward the
//      uncorrupted load=0.6 column, so its own error only moves from
//      +0.18% to about +2.9% - nowhere near the >=45% bound needed to
//      tolerate the map's known ECU-enrichment outliers) - this is exactly
//      why B2 required adding the grid-literal check rather than only
//      tightening the fired-row bound.
//   4. LHV (data/fuels/tier3_e10_cert.json, lhv_j_kg, 41954062.0 ->
//      49954062.0, one digit flipped): caught by F11d's exact literal
//      comparison (Approx(41954062.0).margin(1.0)).
//
// Results of actually performing case 3 (the only one whose detection
// mechanism is new in this review round) are recorded in the C7 review
// round 1 report/commit message, not restated here to avoid this comment
// drifting out of sync with a number that lives in the map data.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// F11g: whole-vehicle smoke - car_sedan_fuel.json (the shipped
// toyota_a25a_fks.json + tier3_e10_cert.json + full 45.02 kg tank) launches
// at WOT from settled rest without stalling and without violating the
// energy-ledger balance, mirroring test_reference_cars.cpp's own pattern.
// ---------------------------------------------------------------------------
TEST_CASE("car_sedan_fuel.json: WOT launch from settled rest does not stall, energy balance holds",
          "[validation][vehicle][drivetrain][fuel]") {
    constexpr real kMass = 1500.0;
    const Vec3 kHalfExtents{2.0, 0.4, 0.15};
    constexpr real kOriginHeight = 0.6;

    auto rig = ps_test::build_vehicle_rig(car_sedan_fuel_path(), kMass, kHalfExtents, kOriginHeight,
                                           /*slope_grade=*/0.0, /*job_workers=*/4);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    const real dt = w.dt();

    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    bool ever_stalled = false;
    real max_balance_ratio = 0.0;
    const std::uint64_t launch_ticks = static_cast<std::uint64_t>(3.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < launch_ticks; ++i) {
        w.step();
        if (w.powertrain_state(rig.vehicle).engine_state == drivetrain::EngineState::Stalled) ever_stalled = true;
        const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real denom = std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) +
                            std::abs(e.brake_loss) + std::abs(e.gear_loss) + std::abs(e.integrator_loss) +
                            std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
                            std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) +
                            std::abs(e.diff_loss);
        if (denom > 1e-9) max_balance_ratio = std::max(max_balance_ratio, std::abs(e.balance_error) / denom);
    }
    const real speed_3s = w.get_motion(rig.chassis).linear.length();
    const drivetrain::PowertrainEnergy& final_e = w.powertrain_energy(rig.vehicle);

    std::printf("[cited engine data] car_sedan_fuel speed@3s=%.4f m/s max_balance_ratio=%.3e "
                "fuel_mass=%.6f kg tank_kg=%.6f ever_stalled=%d\n",
                static_cast<double>(speed_3s), static_cast<double>(max_balance_ratio),
                static_cast<double>(final_e.fuel_mass), static_cast<double>(w.fuel_tank_mass(rig.vehicle)),
                ever_stalled ? 1 : 0);

    CHECK_FALSE(ever_stalled);
    CHECK(speed_3s > 5.0);
    CHECK(max_balance_ratio <= 5e-8);
    CHECK(final_e.fuel_mass > 0.0); // the engine actually burned fuel over the launch
    CHECK(w.fuel_tank_mass(rig.vehicle) < 45.02); // tank has drawn down from the full 45.02 kg
}
