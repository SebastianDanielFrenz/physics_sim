// Electric / fuel-cell / hybrid state-hash pins (powertrain dyno increment, 2026-10-19).
//
// Recorded on origin/codex/hybrid-fuel-cell (68eb13d) BEFORE the envelope / split extraction, so the refactor that makes
// ElectricMotor::pre_solve, Battery and FuelCell call the public envelope functions (ps/drivetrain/electric_envelope.h)
// and the hybrid supervisor call hybrid_power_split (ps/drivetrain/hybrid_split.h) is proven bit-identical: every pin
// below must keep its value through that commit. The battery-discharge allocation change (proportional instead of
// component order) moves only the pins whose pack is the limit; those are re-recorded with a dated comment.
#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdio>
#include <string>

using namespace ps;
using namespace ps::drivetrain;

namespace {

vehicle::VehicleDesc load(const char* name) { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/" + name); }

// WOT from a settled rest in D / 1st for `seconds`; returns the World state hash at the end.
std::uint64_t pull(vehicle::VehicleDesc d, real seconds, real mode = -1, real gear = 1) {
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1850, {2, .4, .15}, .6, 0, 4);
    World& w = *rig.world;
    if (mode >= 0) w.set_control("assist.drive_mode", mode);
    ps_test::settle(w, .25);
    w.set_control("gear_request", gear);
    w.set_control("throttle", 1);
    const int n = static_cast<int>(seconds * 240);
    for (int i = 0; i < n; ++i) w.step();
    return w.state_hash();
}

void set_pack(vehicle::VehicleDesc& d, real discharge_w, real soc = -1) {
    for (auto& c : d.powertrain.components)
        if (auto* b = std::get_if<BatteryDesc>(&c.params)) {
            b->max_discharge_w = discharge_w;
            if (soc >= 0) b->initial_soc = soc;
        }
}

} // namespace

TEST_CASE("Electric pins: state hashes of EV, fuel-cell and hybrid pulls", "[ev][drivetrain][electric][pin]") {
    struct Pin {
        const char* name;
        std::uint64_t hash;
    };
    std::uint64_t got[9];
    got[0] = pull(load("car_electric.json"), 2.0);
    {
        auto d = load("car_electric.json");
        set_pack(d, 100000); // battery-limited dual motor
        got[1] = pull(d, 2.0);
    }
    got[2] = pull(load("car_fuel_cell.json"), 2.0);
    {
        auto d = load("car_fuel_cell.json");
        set_pack(d, 10000, .4);
        for (auto& c : d.powertrain.components)
            if (auto* f = std::get_if<FuelCellDesc>(&c.params)) f->max_power_w = 30000;
        got[3] = pull(d, 2.0); // battery + stack limited dual motor
    }
    for (int mode = 0; mode < 4; ++mode) got[4 + mode] = pull(load("car_hybrid.json"), 3.0, mode, 99);
    {
        auto d = load("car_hybrid.json");
        set_pack(d, 150000, .19); // low SOC: the charge latch, Normal mode
        got[8] = pull(d, 3.0, 1, 99);
    }
    const Pin pins[9] = {
        {"car_electric WOT", 0xa21b507ab32c71c9ull},
        // 2026-10-19 proportional discharge allocation (Battery::reserve_discharge): was 0x10546b0bd04eaa6d (component order).
        {"car_electric 100 kW pack WOT", 0xa38a4372c053b432ull},
        {"car_fuel_cell WOT", 0x0610de97253cfd3full},
        // 2026-10-19 proportional discharge allocation: was 0x8e9e9bc42c01e683 (component order).
        {"car_fuel_cell 10 kW pack + 30 kW stack WOT", 0xa360e3419f34177eull},
        {"car_hybrid Eco WOT", 0x4c23b7286cd6f18full},
        {"car_hybrid Normal WOT", 0x2712a8e54088a401ull},
        {"car_hybrid Sport WOT", 0x9f0ef54b6962910cull},
        {"car_hybrid Racing WOT", 0x9f0ef54b6962910cull},
        // Power-target charging and disconnected-engine traction-cut isolation (2026-10-23).
        {"car_hybrid Normal low SOC WOT", 0xbb6bf7af0f6af1a3ull},
    };
    for (int i = 0; i < 9; ++i) {
        std::printf("pin %-44s 0x%016llxull\n", pins[i].name, static_cast<unsigned long long>(got[i]));
        INFO(pins[i].name);
        CHECK(got[i] == pins[i].hash);
    }
}
