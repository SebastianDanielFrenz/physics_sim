// tests/unit/test_a25a_sim_engine_data.cpp - PHYS-033 P4 I7c C17a (fallback
// M-B): data/engines/toyota_a25a_fks_sim.json, the Toyota A25A-FKS as a
// kind "simulated" engine on maps back-solved from the EPA NVFEL Tier 3
// test-data xlsx by tools/enginedata/a25a_mb_engine.py, and
// data/vehicles/car_sedan_sim.json, the car that carries it.
//
// "measured-map engine (I7c D12 M-B): its dyno comparison is tautological
// and is not a generator validation" - the maps were solved FROM the WOT
// points checked below, so this test proves the round trip (xlsx -> script
// -> JSON -> production loader -> SimulatedEngine's own torque function)
// is lossless and that the file is wired the way its source claims (units,
// axis order, fuel). It is not evidence the cycle model predicts anything.
//
// White-box: SimulatedEngine (core/src/drivetrain/engine_simulated.h) is
// built directly from the loaded desc (written before I7b C12 wired the
// simulated engine into the Powertrain; the whole-vehicle run of
// car_sedan_sim.json is test_simulated_engine_powertrain.cpp's
// "C12/C17a integration" case).
//
// Tolerance for the WOT check: 2e-5 relative. The literals below are the
// script's r6() (6 significant digits) roundings of the xlsx cells, and the
// JSON table values are r6() roundings of the back-solved quantities; the
// script's own replica of the runtime equations measures the residual from
// that rounding at <= 3.0e-6 relative over the 14 anchors
// (out/a25a_mb_report.txt, "WOT reproduction"). 2e-5 is ~7x that and still
// 3-4 orders of magnitude below what a unit or axis error produces (the
// sabotage runs recorded in the C17a progress file: a transposed VE table
// is rejected by the loader, a reversed VE row order misses by tens of
// percent, a kPa axis written in Pa by up to ~100 %).

#include "drivetrain/engine_simulated.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_desc.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <variant>

using ps::real;
using namespace ps::drivetrain;

namespace {

std::string sim_car_path() { return std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_sim.json"; }
std::string sim_engine_path() { return std::string(PS_SOURCE_DIR) + "/data/engines/toyota_a25a_fks_sim.json"; }
std::string c7_car_path() { return std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_fuel.json"; }

const SimulatedEngineDesc* sim_engine_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) return e;
    }
    return nullptr;
}

// One WOT anchor per measured speed line: the xlsx sheet "High Load Final"
// row holding that line's maximum measured brake torque over every pedal
// position (C7's envelope, data/engines/toyota_a25a_fks.json). Values are
// literal xlsx readings rounded to 6 significant digits: engine speed
// [rpm], intake manifold absolute pressure [kPa], test-cell pressure [kPa],
// test-cell temperature [degC], brake torque [N*m].
struct WotAnchor {
    int row;
    real rpm, map_kpa, p_cell_kpa, t_cell_c, torque_nm;
};
constexpr WotAnchor kAnchors[] = {
    { 8, 1003.04, 97.509,  98.1706, 25.7,    159.039},
    {11, 1153.2,  97.4626, 98.1555, 25.3,    176.654},
    {14, 1325.94, 95.431,  98.1772, 25.3,    179.954},
    {17, 1498.52, 95.5144, 98.2396, 25.5001, 189.621},
    {20, 1728.58, 96.987,  98.27,   26.4,    196.616},
    {23, 2017.13, 97.2897, 98.267,  26.8997, 216.321},
    {26, 2534.94, 97.3509, 98.2473, 27.3,    225.125},
    {29, 2995.63, 97.2884, 98.2005, 27.7,    217.813},
    {32, 3571.24, 97.0007, 98.1844, 27.5308, 228.4},
    {34, 4032.22, 96.8321, 98.2054, 27.1,    228.347},
    {37, 4493.11, 96.6618, 98.2557, 27.2,    232.038},
    {39, 4954.03, 96.4381, 98.2724, 27.4004, 231.437},
    {42, 5414.48, 96.1927, 98.2773, 27.9,    229.401},
    {45, 5761.17, 96.0589, 98.291,  28.2004, 227.315},
};

constexpr real kRelTol = 2e-5;

real omega_of(real rpm) { return rpm * 2.0 * 3.14159265358979323846 / 60.0; }

// Brake torque at the anchor's measured MAP and cell ambient: the fired
// torque function pre_solve uses, minus the Coulomb row (tau excludes it).
real brake_torque(SimulatedEngine& eng, const WotAnchor& a) {
    eng.set_ambient(a.p_cell_kpa * 1000.0, a.t_cell_c + 273.15);
    const auto ev = eng.evaluate_torque(omega_of(a.rpm), a.map_kpa * 1000.0, true, 1.0, false);
    return ev.tau - eng.maps().c_k;
}

} // namespace

TEST_CASE("C17a: car_sedan_sim.json and toyota_a25a_fks_sim.json load as labelled", "[drivetrain][io]") {
    const auto car = ps::io::load_vehicle_json(sim_car_path());
    const SimulatedEngineDesc* e = sim_engine_of(car);
    REQUIRE(e != nullptr);

    // fuels[0] is burnt: the EPA Tier 3 E10 certification fuel the data
    // were measured on (data/fuels/tier3_e10_cert.json); e10.json second.
    REQUIRE(e->fuels.size() == 2);
    CHECK(e->fuels[0].lhv_j_kg == 41954062.0);
    CHECK(e->fuels[1].name != e->fuels[0].name);

    // The clutch the source text names (same as C7's car_sedan_fuel.json).
    bool found_clutch = false;
    for (const auto& c : car.powertrain.components) {
        if (const auto* cl = std::get_if<ClutchDesc>(&c.params)) {
            CHECK(cl->capacity_nm == 350.0);
            found_clutch = true;
        }
    }
    CHECK(found_clutch);

    // The engine file's top-level source carries the brief's verbatim label.
    std::ifstream in(sim_engine_path());
    REQUIRE(in.good());
    const auto j = nlohmann::json::parse(in);
    CHECK(j.at("kind").get<std::string>() == "simulated");
    CHECK(j.at("source").get<std::string>().find(
              "measured-map engine (I7c D12 M-B): its dyno comparison is tautological and is not a generator "
              "validation") == 0);
}

TEST_CASE("C17a: SimulatedEngine reproduces every measured WOT brake torque at measured MAP and cell ambient",
          "[drivetrain][io]") {
    const auto car = ps::io::load_vehicle_json(sim_car_path());
    const SimulatedEngineDesc* e = sim_engine_of(car);
    REQUIRE(e != nullptr);
    SimulatedEngine eng(*e);

    real worst = 0.0;
    for (const auto& a : kAnchors) {
        const real t = brake_torque(eng, a);
        const real rel = (t - a.torque_nm) / a.torque_nm;
        std::printf("C17a WOT HLF row %2d: %8.2f rpm MAP %8.4f kPa  measured %8.3f  model %10.5f N*m  rel %+.2e\n",
                    a.row, a.rpm, a.map_kpa, a.torque_nm, t, rel);
        worst = std::fmax(worst, std::fabs(rel));
        CHECK(std::fabs(rel) <= kRelTol);
    }
    std::printf("C17a WOT worst |rel| %.2e (tolerance %.0e)\n", worst, kRelTol);

    // Report only: the ISA steady WOT curve the runtime built from these
    // maps (ISA sits ~3 kPa above the cell's 98.2 kPa, hence more torque).
    for (real rpm : {1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 6500.0}) {
        std::printf("C17a ISA steady WOT %5.0f rpm: %7.2f N*m\n", rpm, eng.maps().steady_wot_nm.evaluate(rpm));
    }
    std::printf("C17a idle_ff %.5f, idle p_im %.2f kPa\n", eng.maps().idle_ff, eng.maps().idle_p_im / 1000.0);
}

TEST_CASE("C17a: the sim engine's WOT anchors equal C7's torque-map WOT knots", "[drivetrain][io]") {
    // C7's data/engines/toyota_a25a_fks.json (loaded through
    // car_sedan_fuel.json) stores the same per-speed-line maximum, x rounded
    // to 0.1 rpm and y to 0.01 N*m. At each anchor rpm the sim engine must
    // agree with C7's curve to that rounding: 0.005 N*m for y plus the
    // curve's slope times <= 0.05 rpm of x rounding (< 0.002 N*m), plus the
    // WOT check's own residual (< 2e-5 * 232 N*m) - bound 0.012 N*m.
    const auto sim_car = ps::io::load_vehicle_json(sim_car_path());
    const auto c7_car = ps::io::load_vehicle_json(c7_car_path());
    const SimulatedEngineDesc* e = sim_engine_of(sim_car);
    REQUIRE(e != nullptr);
    const TorqueMapEngineDesc* c7 = nullptr;
    for (const auto& c : c7_car.powertrain.components) {
        if (const auto* t = std::get_if<TorqueMapEngineDesc>(&c.params)) c7 = t;
    }
    REQUIRE(c7 != nullptr);
    SimulatedEngine eng(*e);
    for (const auto& a : kAnchors) {
        const real t = brake_torque(eng, a);
        const real c7_t = c7->wot_torque_nm_vs_rpm.evaluate(a.rpm);
        std::printf("C17a vs C7 %8.2f rpm: sim %9.4f  C7 %9.4f  diff %+.4f N*m\n", a.rpm, t, c7_t, t - c7_t);
        CHECK(std::fabs(t - c7_t) <= 0.012);
    }
}
