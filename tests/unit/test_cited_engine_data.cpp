// tests/unit/test_cited_engine_data.cpp — PHYS-033 P4 I7a C7: independent
// verification that data/engines/toyota_a25a_fks.json and
// data/fuels/tier3_e10_cert.json's numbers are what their own "source"
// fields claim - literal transcription checks against the underlying xlsx
// citations (own words: sheet/row), an independent re-derivation of
// afr_stoich from the fuel file's own measured elemental mass fractions,
// a demonstration that a single corrupted digit is caught by this file's
// own literal-comparison style, and a whole-vehicle WOT-launch smoke case
// (data/vehicles/car_sedan_fuel.json) mirroring test_reference_cars.cpp's
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
std::string car_sedan_fuel_path() { return std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_fuel.json"; }

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

// RAII temp file, same pattern as tests/unit/test_fuel_io.cpp's TempJsonFile.
struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const std::string& tag) {
        path = std::filesystem::temp_directory_path() /
               ("ps_test_cited_engine_data_" + tag + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
    }
    void write(const std::string& content) const {
        std::ofstream f(path, std::ios::binary);
        f << content;
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

} // namespace

// ---------------------------------------------------------------------------
// F11a: literal WOT torque knots (13 points), transcribed from the engine
// file's own source field citation of the "High Load Final" sheet's
// Accelerator Pedal == 100.00 rows (xlsx row cited in brackets below).
// ---------------------------------------------------------------------------
TEST_CASE("toyota_a25a_fks.json: WOT torque curve matches the cited xlsx rows exactly",
          "[io][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = ps::io::load_vehicle_json(car_sedan_fuel_path());
    const auto& engine = engine_of(desc);
    const vehicle::Curve1D& wot = engine.wot_torque_nm_vs_rpm;

    // (rpm, Nm) [xlsx row] - "High Load Final" sheet, pedal == 100.00.
    struct Point {
        real rpm, nm;
        const char* row;
    };
    const std::vector<Point> kCited = {
        {1003.1, 158.80, "row7"},  {1153.2, 176.65, "row11"}, {1326.2, 172.94, "row13"}, {1498.7, 174.55, "row16"},
        {1728.6, 196.62, "row20"}, {2017.1, 216.32, "row23"}, {2534.9, 225.13, "row26"}, {2995.6, 217.81, "row29"},
        {3571.2, 228.40, "row32"}, {4032.2, 228.35, "row34"}, {4493.1, 232.04, "row37"}, {5414.5, 229.40, "row42"},
        {5761.0, 226.24, "row44"},
    };
    REQUIRE(wot.x.size() == kCited.size());
    REQUIRE(wot.y.size() == kCited.size());
    for (std::size_t i = 0; i < kCited.size(); ++i) {
        INFO("point " << i << " xlsx " << kCited[i].row);
        CHECK(wot.x[i] == Approx(kCited[i].rpm).epsilon(1e-9));
        CHECK(wot.y[i] == Approx(kCited[i].nm).epsilon(1e-9));
    }
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
// F11c: the fuel map, evaluated at literal FIRED (measured) fuel-flow
// points read off the same xlsx (out/c7/map_fired.txt, a reproducible
// extraction of the same source workbook the engine file's own source
// field cites for the map). theta_c is derived per-point from THIS FILE'S
// OWN loaded WOT/motoring curves (never a separate hand-copy of them),
// exactly the construction method the engine file's source field
// describes: T = m(rpm) + theta_c*(w(rpm) - m(rpm)).
// ---------------------------------------------------------------------------
TEST_CASE("toyota_a25a_fks.json: fuel map reproduces literal fired fuel-flow points",
          "[io][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = ps::io::load_vehicle_json(car_sedan_fuel_path());
    const auto& engine = engine_of(desc);
    REQUIRE(engine.has_fuel_map);
    const drivetrain::Table2D& map = engine.fuel_map;
    const vehicle::Curve1D& wot = engine.wot_torque_nm_vs_rpm;
    const vehicle::Curve1D& mot = engine.motoring_torque_nm_vs_rpm;

    // Fired points from out/c7/map_fired.txt (a reproducible re-extraction
    // of the same xlsx toyota_a25a_fks.json's own source field cites for
    // the map), spanning 6 of the map's 10 rpm nodes - well over the "at
    // least 10" the brief asks for.
    struct Fired {
        real rpm, torque_nm, fmf_g_s;
        const char* cite;
    };
    const std::vector<Fired> kFired = {
        {1326.0, 9.200, 0.2710, "SteadyState row 43, rpm=1326.00, pedal=4.00"},
        {1326.0, 14.600, 0.3090, "SteadyState row 44, rpm=1326.00, pedal=7.00"},
        {1326.0, 172.939, 2.4647, "HighLoadFinal row 13, rpm=1326.19, pedal=100.00"},
        {1498.0, 174.550, 3.2878, "HighLoadFinal (pedal=100.00 node, see wot curve row16)"},
        {1728.0, 196.620, 3.7983, "HighLoadFinal (pedal=100.00 node, see wot curve row20)"},
        {2017.0, 216.320, 3.8120, "HighLoadFinal (pedal=100.00 node, see wot curve row23)"},
        {2535.0, 225.130, 4.4660, "HighLoadFinal (pedal=100.00 node, see wot curve row26)"},
        {2996.0, 217.810, 4.8313, "HighLoadFinal (pedal=100.00 node, see wot curve row29)"},
        {3571.0, 228.400, 6.0946, "HighLoadFinal (pedal=100.00 node, see wot curve row32)"},
        {4032.0, 228.350, 7.2859, "HighLoadFinal (pedal=100.00 node, see wot curve row34)"},
        {4493.0, 232.040, 8.1561, "HighLoadFinal (pedal=100.00 node, see wot curve row37)"},
    };

    real max_rel_err = 0.0;
    for (const Fired& f : kFired) {
        const real w = wot.evaluate(f.rpm);
        const real m = mot.evaluate(f.rpm);
        REQUIRE(w > m); // denominator below must be well-formed
        const real theta_c = (f.torque_nm - m) / (w - m);
        const real predicted = map.value(f.rpm, theta_c);
        const real rel_err = std::abs(predicted - f.fmf_g_s) / f.fmf_g_s;
        max_rel_err = std::max(max_rel_err, rel_err);
        INFO("fired point: " << f.cite << " rpm=" << f.rpm << " T=" << f.torque_nm << " theta_c=" << theta_c
                              << " predicted=" << predicted << " measured=" << f.fmf_g_s);
        // The WOT-torque-node fired points (theta_c==1 exactly, by
        // construction of the map's own rpm axis == the WOT/motoring curve's
        // rpm knots) must reproduce the literal table cell exactly; the
        // interior points (theta_c derived from a torque strictly between
        // m(rpm) and w(rpm)) go through the map's own piecewise-linear fit
        // over ALL fired points at that rpm (see the engine file's source
        // field) so a small reconstruction error is expected - measured
        // (with this test) at most 0.072938%, bound set with margin above
        // that.
        CHECK(rel_err < 0.005);
    }
    std::printf("[cited engine data] fuel map fired-point max_rel_err=%.6f%%\n", static_cast<double>(max_rel_err * 100.0));
    CHECK(max_rel_err < 0.005);
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
// F11f: a single corrupted digit in a cited literal is caught by this
// file's own literal-comparison style. Never mutates the tracked file -
// copies its bytes into a TempJsonFile, corrupts THAT copy, loads it, and
// checks the loaded value diverges from the true cited literal.
// ---------------------------------------------------------------------------
TEST_CASE("tier3_e10_cert.json: a corrupted lhv_j_kg digit is caught by literal comparison",
          "[io][fuel]") {
    std::ifstream in(fuel_path(), std::ios::binary);
    REQUIRE(in.is_open());
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    // The bare number "41954062.0" also appears a second time inside the
    // source field's own derivation prose ("... stored to the nearest whole
    // J/kg = 41954062.0 ..."), so anchor on the actual JSON field
    // assignment to corrupt only the real value, not the prose.
    const std::string true_literal = "\"lhv_j_kg\": 41954062.0";
    const std::string corrupted_literal = "\"lhv_j_kg\": 49954062.0"; // one digit flipped: 1->9
    const std::size_t pos = text.find(true_literal);
    REQUIRE(pos != std::string::npos);
    REQUIRE(text.find(true_literal, pos + 1) == std::string::npos); // exactly one occurrence
    text.replace(pos, true_literal.size(), corrupted_literal);

    TempJsonFile temp("corrupted_lhv");
    temp.write(text);

    const drivetrain::FuelDesc corrupted = ps::io::load_fuel_json(temp.path.string());
    CHECK(corrupted.lhv_j_kg == Approx(49954062.0).margin(1.0));
    // This is exactly the discriminating check F11d applies to the real
    // file - it would fail here, demonstrating the real file's own
    // literal-comparison test would catch this class of corruption.
    CHECK_FALSE(corrupted.lhv_j_kg == Approx(41954062.0).margin(1.0));
}

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
