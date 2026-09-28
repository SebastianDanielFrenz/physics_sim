// tests/unit/test_engine_rig_wheelless.cpp - ps_audio A5 task 4: does the
// Powertrain builder accept the wheel-less graph the future EngineRig (A10,
// audio plan D-10) needs - an engine crank shafted to a LoadDyno, no clutch,
// no gearbox, no wheels, no brakes, no TCU? White-box (core/src on the include
// path). EngineRig itself is NOT built here (A10).

#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/vehicle/vehicle_reset.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

TEST_CASE("A5 EngineRig check: a crank-only Powertrain with a LoadDyno builds and runs", "[drivetrain][pipe]") {
    // Catches: a builder rule that silently assumes wheels, a gearbox or a
    // clutch (the unconnected-port check, the speedo walk, the TCU).
    const vehicle::VehicleDesc sedan = ps_test::load_sedan_desc();
    PowertrainDesc pt;
    for (const ComponentDesc& c : sedan.powertrain.components) {
        if (std::holds_alternative<TorqueMapEngineDesc>(c.params)) pt.components.push_back(c);
    }
    REQUIRE(pt.components.size() == 1u);
    const std::string eng_id = pt.components.front().id;
    LoadDynoDesc dyno;
    dyno.mode = LoadDynoMode::Viscous;
    dyno.inertia_kgm2 = 0.05;
    dyno.viscous_nm_s_rad = 0.05;
    pt.components.push_back(ComponentDesc{"dyno", dyno});
    ShaftDesc s;
    s.ports = {PortRef{eng_id, "crank", false}, PortRef{"dyno", "shaft", false}};
    pt.shafts = {s};

    const BrakeSystemDesc no_brakes{};
    const std::vector<vehicle::WheelDesc> no_wheels;
    const std::vector<ControllerDesc> no_controllers;
    Powertrain rig(pt, no_brakes, no_wheels, no_controllers, "engine_rig_check");
    REQUIRE(rig.has_engine());
    CHECK_FALSE(rig.has_clutch());
    CHECK_FALSE(rig.has_gearbox());

    vehicle::VehicleResetOptions ro;
    ro.engine_state = EngineState::Off;
    rig.reset(ro, {});

    // Crank for 1 s on the starter, then 2 s at 30 % throttle.
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.ignition = 1.0;
    real worst_balance = 0.0, scale = 0.0;
    int worst_tick = -1;
    for (int tick = 0; tick < 720; ++tick) {
        ctx.starter = tick < 240 ? 1.0 : 0.0;
        ctx.throttle = tick < 240 ? 0.0 : 0.3;
        rig.begin_tick();
        for (int k = 0; k < 4; ++k) {
            ctx.first_substep_of_tick = k == 0;
            rig.begin_substep();
            rig.pre_solve_components(ctx);
            rig.solve(ctx.h);
            rig.post_solve_components(ctx);
        }
        rig.end_tick();
        // Tick 0 carries the reset's unbooked rotor seed (vault PHYS-049
        // F-T0, the same convention as test_simulated_engine_powertrain's T8).
        if (tick > 0 && std::fabs(rig.ledger().balance_error) > worst_balance) {
            worst_balance = std::fabs(rig.ledger().balance_error);
            worst_tick = tick;
        }
        scale = std::fmax(scale, std::fabs(rig.ledger().engine_out) + std::fabs(rig.ledger().external_absorbed));
    }
    const real rpm = rig.engine_omega() * 60.0 / (2.0 * 3.14159265358979323846);
    std::printf("A5 EngineRig check: engine %.0f rpm after 3 s, dyno absorbed %.1f J, worst |balance_error| %.3e J at tick %d\n",
                rpm, rig.ledger().external_absorbed, worst_balance, worst_tick);
    CHECK(rpm > 1000.0);
    CHECK(rig.ledger().external_absorbed > 0.0);
    CHECK(worst_balance <= 1e-9 * std::fmax(scale, 1.0));
}
