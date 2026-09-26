// tests/unit/test_powertrain_pto.cpp — PHYS-033 P4 I4 C11 (plan section
// 3.5): the pto (power take-off) component (tractor.json's engine/clutch
// shaft -> an external out port) and load_dyno (test/bench load, never
// shipped in a data file). White-box (core/src on this target's include
// path), mirroring test_powertrain_park_lock.cpp's own pattern: raw
// Pto/LoadDyno/NetworkBuilder/Assembly-level tests for each component's own
// engagement/mode/brake-decay/loss formulas, a whole-vehicle row-placement
// check (car_sedan.json with a load_dyno added in code vs. tractor.json with
// "pto" stripped), and the usual D8/D7 whole-World checks (debug_alloc-armed
// zero-alloc stepping, 1-vs-10-worker state_hash equality) with pto toggled
// and a mode change mid-run.

#include "drivetrain/components.h"
#include "drivetrain/network.h"
#include "drivetrain/powertrain.h"

#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {
Powertrain build(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

vehicle::VehicleDesc load_tractor() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json"); }
vehicle::VehicleDesc load_sedan() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json"); }

// tractor.json (I4 C11) with its "pto" component + the shaft port that
// connects it removed - the baseline network used to find where pto's own
// row would land (see Test 8 below). Mirrors test_powertrain_park_lock.cpp's
// own "strip the component and its shaft ports" technique for tractor's
// park_lock row.
vehicle::VehicleDesc load_tractor_without_pto() {
    vehicle::VehicleDesc d = load_tractor();
    auto& comps = d.powertrain.components;
    comps.erase(std::remove_if(comps.begin(), comps.end(), [](const ComponentDesc& c) { return c.id == "pto"; }), comps.end());
    for (auto& shaft : d.powertrain.shafts) {
        auto& ports = shaft.ports;
        ports.erase(std::remove_if(ports.begin(), ports.end(), [](const PortRef& p) { return p.component == "pto"; }), ports.end());
    }
    // Drop any shaft left with fewer than 2 ports (pto.in was unioned into
    // the engine/clutch shaft; pto.out's own private shaft, if any, becomes
    // empty once pto.out is removed too).
    auto& shafts = d.powertrain.shafts;
    shafts.erase(std::remove_if(shafts.begin(), shafts.end(), [](const ShaftDesc& s) { return s.ports.size() < 2; }), shafts.end());
    return d;
}

// tractor.json (I4 C11) with an extra load_dyno component (Viscous, so no
// control channel is needed) shafted onto pto.out - a test/bench harness
// attached IN CODE, never in a shipped data file (LoadDyno is not part of
// any data/vehicles/*.json).
vehicle::VehicleDesc load_tractor_with_pto_dyno() {
    vehicle::VehicleDesc d = load_tractor();
    LoadDynoDesc dyno_desc;
    dyno_desc.mode = LoadDynoMode::Viscous;
    dyno_desc.viscous_nm_s_rad = 5.0;
    dyno_desc.inertia_kgm2 = 0.02;
    ComponentDesc dyno_component;
    dyno_component.id = "dyno";
    dyno_component.params = dyno_desc;
    d.powertrain.components.push_back(dyno_component);
    ShaftDesc shaft;
    shaft.ports = {PortRef{"pto", "out", false}, PortRef{"dyno", "shaft", false}};
    d.powertrain.shafts.push_back(shaft);
    return d;
}

// car_sedan.json (no pto/transfer_case/park_lock at all: clutch, engine,
// synchro = 3 base friction rows, then Phase H's 4 brake rows at [3..6])
// with a load_dyno-Brake component added in code, shafted onto the
// gearbox's own output - exercises the SAME "pto/dyno rows insert before
// Phase H's brakes" placement rule Test 8 checks for tractor's pto, on a
// vehicle that otherwise has zero base friction rows of this kind.
vehicle::VehicleDesc load_sedan_with_dyno_brake() {
    vehicle::VehicleDesc d = load_sedan();
    LoadDynoDesc dyno_desc;
    dyno_desc.mode = LoadDynoMode::Brake;
    dyno_desc.torque_nm = 50.0;
    dyno_desc.inertia_kgm2 = 0.02;
    ComponentDesc dyno_component;
    dyno_component.id = "dyno";
    dyno_component.params = dyno_desc;
    d.powertrain.components.push_back(dyno_component);
    // car_sedan.json's own shafts[] already unions "gearbox.out" with
    // "diff.carrier" (data/vehicles/car_sedan.json's shafts[2]) - a second
    // shaft entry also naming "gearbox.out" would be rejected as "connected
    // by more than one shaft" (Powertrain's own union-find validation), so
    // the dyno's port is appended onto that EXISTING shaft instead of a new
    // one, exactly like a physical PTO/dyno tapped onto an existing output.
    bool attached = false;
    for (auto& shaft : d.powertrain.shafts) {
        for (const auto& port : shaft.ports) {
            if (!port.is_wheel && port.component == "gearbox" && port.port == "out") {
                shaft.ports.push_back(PortRef{"dyno", "shaft", false});
                attached = true;
                break;
            }
        }
        if (attached) break;
    }
    REQUIRE(attached);
    return d;
}
} // namespace

// --- Pto: raw network row identities -------------------------------------
TEST_CASE("Pto: engaged and slipping obeys |lambda| <= capacity and is dissipative; "
          "stuck (locked) makes omega_in == ratio*omega_out",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const real J_in = 0.15;
    const real J_out = 0.05;
    const NodeId in = nb.add_node(J_in);
    const NodeId out = nb.add_node(J_out);
    const FrictionRowId row = nb.add_friction_row(in, out);
    ShaftNetwork net = nb.build("test-pto-raw");

    PtoDesc desc;
    desc.has_540 = true;
    desc.ratio_540 = 1.6;
    desc.capacity_nm = 400.0;
    desc.static_factor = 1.2;
    desc.engage_time_s = 0.5;
    Pto pto(in, out, row, desc, /*channel_slot=*/0, /*speed_channel_slot=*/1, J_out);

    real channels[2] = {1.0, 540.0}; // requested engaged throughout, 540 mode
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 2;

    Assembly assembly;
    PowertrainEnergy ledger{};

    // Drive engagement to 1.0 first (engage_time_s = 0.5s => 480 substeps).
    net.set_omega(in, 200.0);
    net.set_omega(out, 0.0);
    for (int i = 0; i < 500; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(pto.engagement() == Catch::Approx(1.0));

    // Slipping: the row's own lambda must not exceed the (now fully engaged)
    // kinetic capacity, and it must be dissipative (opposes the slip).
    if (!net.row_stuck(row)) {
        const real v = net.row_velocity(row);
        const real lambda = net.row_lambda(row);
        CHECK(std::abs(lambda) <= desc.capacity_nm * 1.0 + 1e-6);
        CHECK(-v * lambda >= -1e-9); // -v*lambda is dissipated power, must be >= 0
    }

    // Run long enough for the row to lock (kinetic torque decelerates omega_in
    // and accelerates omega_out until v == 0, then it should stay stuck).
    for (int i = 0; i < 20000; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(net.row_stuck(row));
    // Confirmed sign convention (network.cpp): v = omega(a) - ratio_b*omega(b),
    // b defaults to kGround unless overridden - here b IS `out`, so stuck means
    // omega(in) == ratio_b * omega(out) with ratio_b == current_ratio().
    CHECK(net.omega(in) == Catch::Approx(pto.current_ratio() * net.omega(out)).margin(1e-6));
}

// --- Pto: engagement ramp both directions --------------------------------
TEST_CASE("Pto: engagement ramps at 1/engage_time_s toward the requested state and clamps to [0,1]",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId in = nb.add_node(0.15);
    const NodeId out = nb.add_node(0.05);
    const FrictionRowId row = nb.add_friction_row(in, out);
    ShaftNetwork net = nb.build("test-pto-ramp");

    PtoDesc desc;
    desc.has_540 = true;
    desc.capacity_nm = 400.0;
    desc.engage_time_s = 0.4; // rate = 2.5 /s
    Pto pto(in, out, row, desc, 0, 1, 0.05);

    real channels[2] = {1.0, 540.0};
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 2;
    Assembly assembly;
    PowertrainEnergy ledger{};

    // engage_time_s * tick_rate_hz = 0.4 * 960 = 384 substeps to fully ramp;
    // run comfortably past that before requiring the clamp to have engaged.
    const real rate = 1.0 / desc.engage_time_s;
    const int kFullRampSubsteps = 500;
    real expected = 0.0;
    for (int i = 0; i < kFullRampSubsteps; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        expected = std::min(1.0, expected + rate * ctx.h);
        CHECK(pto.engagement() == Catch::Approx(expected).margin(1e-9));
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(pto.engagement() == Catch::Approx(1.0));

    // Now request disengage and ramp back down.
    channels[0] = 0.0;
    for (int i = 0; i < kFullRampSubsteps; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        expected = std::max(0.0, expected - rate * ctx.h);
        CHECK(pto.engagement() == Catch::Approx(expected).margin(1e-9));
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(pto.engagement() == Catch::Approx(0.0));
}

// --- Pto: mode-change eligibility -----------------------------------------
TEST_CASE("Pto: a mode change is refused while engaged or the out node is spinning, "
          "and applied at the first eligible substep",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId in = nb.add_node(0.15);
    const NodeId out = nb.add_node(0.05);
    const FrictionRowId row = nb.add_friction_row(in, out);
    ShaftNetwork net = nb.build("test-pto-mode-switch");

    PtoDesc desc;
    desc.has_540 = true;
    desc.ratio_540 = 1.6;
    desc.has_1000 = true;
    desc.ratio_1000 = 0.864;
    desc.capacity_nm = 400.0;
    desc.engage_time_s = 0.1;
    desc.disengaged_brake_time_s = 1.0;
    Pto pto(in, out, row, desc, 0, 1, 0.05);

    real channels[2] = {1.0, 540.0}; // engaged, 540 mode
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 2;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(in, 150.0);
    net.set_omega(out, 90.0);

    // Fully engage first.
    for (int i = 0; i < 200; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(pto.engagement() == Catch::Approx(1.0));

    // Request 1000 mode while still engaged and spinning fast: must be refused.
    channels[1] = 1000.0;
    for (int i = 0; i < 50; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        CHECK_FALSE(pto.mode_1000());
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }

    // Disengage and let the out node's speed decay (the disengaged stub
    // brake below handles this) until BOTH e==0 and |omega_out| < 1 rad/s.
    channels[0] = 0.0;
    bool switched = false;
    for (int i = 0; i < 20000 && !switched; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        if (pto.mode_1000()) switched = true;
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(switched);
    CHECK(pto.current_ratio() == Catch::Approx(desc.ratio_1000));
}

// --- Pto: disengaged stub brake exact recurrence --------------------------
TEST_CASE("Pto: disengaged (e==0 exactly) stub brake decays omega_out per the exact "
          "implicit-Euler recurrence w' = w*J/(J+h*c), c = J_out/disengaged_brake_time_s",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId in = nb.add_node(0.15);
    const real J_out = 0.05;
    const NodeId out = nb.add_node(J_out);
    const FrictionRowId row = nb.add_friction_row(in, out);
    ShaftNetwork net = nb.build("test-pto-stub-brake");

    PtoDesc desc;
    desc.has_540 = true;
    desc.capacity_nm = 700.0;
    desc.disengaged_brake_time_s = 0.75; // c = J_out / time
    Pto pto(in, out, row, desc, 0, 1, J_out);

    real channels[2] = {0.0, 540.0}; // never requested engaged
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 2;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(in, 0.0);
    net.set_omega(out, 300.0);

    const real c = J_out / desc.disengaged_brake_time_s;
    real expected = 300.0;
    for (int i = 0; i < 100; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        REQUIRE(pto.engagement() == 0.0);
        CHECK(assembly.cap_kinetic(row) == 0.0); // never engaged => the friction row itself carries no capacity
        net.solve(ctx.h, assembly);
        pto.post_solve(ctx, net, assembly, ledger);

        expected = expected * J_out / (J_out + ctx.h * c);
        CHECK(net.omega(out) == Catch::Approx(expected).epsilon(1e-9));
    }
    // After 100 substeps at 960 Hz (~0.104 s) against a 0.75 s time constant,
    // the speed should have decayed noticeably but the wheel must not have
    // been driven backward (a strictly decaying exponential, never overshoot).
    CHECK(net.omega(out) < 300.0);
    CHECK(net.omega(out) > 0.0);
}

// --- Pto: efficiency loss identity into gear_loss -------------------------
TEST_CASE("Pto: an efficiency < 1 books a lagged loss torque at the in-node into gear_loss",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId in = nb.add_node(0.15);
    const NodeId out = nb.add_node(0.05);
    const FrictionRowId row = nb.add_friction_row(in, out);
    ShaftNetwork net = nb.build("test-pto-efficiency");

    PtoDesc desc;
    desc.has_540 = true;
    desc.capacity_nm = 5000.0; // large enough that the row stays stuck (locked)
    desc.engage_time_s = 0.05;
    desc.efficiency = 0.9;
    Pto pto(in, out, row, desc, 0, 1, 0.05);

    real channels[2] = {1.0, 540.0};
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 2;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(in, 100.0);
    net.set_omega(out, 100.0 / desc.ratio_540);

    // A sustained driving torque at `in` (standing in for the engine/clutch
    // shaft) keeps the row genuinely loaded (nonzero lambda_row_) once
    // locked, rather than the degenerate lambda==0 fixed point a torque-free
    // closed system would sit at from these already-synchronized initial
    // speeds - needed so this test actually exercises the efficiency-loss
    // formula's nonzero case, not just 0 == 0.
    const real driving_tau = 20.0;

    real prev_lambda = 0.0; // mirrors Pto's own private lambda_row_prev_
    real expected_gear_loss = 0.0;
    for (int i = 0; i < 2000; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        const real own_loss_tau = -(1.0 - desc.efficiency) * std::abs(prev_lambda) *
                                   (net.omega(in) > 0.0 ? 1.0 : (net.omega(in) < 0.0 ? -1.0 : 0.0));
        assembly.torque(in, driving_tau, 0.0);
        net.solve(ctx.h, assembly);
        const real ledger_before = ledger.gear_loss;
        pto.post_solve(ctx, net, assembly, ledger);
        // gear_loss += -h * omega_new(in) * own_loss_tau (post-solve omega).
        expected_gear_loss += -ctx.h * net.omega(in) * own_loss_tau;
        CHECK(ledger.gear_loss == Catch::Approx(ledger_before + (-ctx.h * net.omega(in) * own_loss_tau)).margin(1e-9));
        prev_lambda = net.row_lambda(row);
    }
    CHECK(ledger.gear_loss == Catch::Approx(expected_gear_loss).epsilon(1e-6));
    // Confirms this test actually exercised the nonzero case (not 0 == 0):
    // the row locked under the sustained driving torque and is carrying it.
    REQUIRE(net.row_stuck(row));
    CHECK(std::abs(prev_lambda) > 1e-3);
    // Efficiency < 1 with the row carrying real torque means SOME loss:
    // once the row has locked and lambda is nonzero, gear_loss should have
    // accumulated in the dissipative direction (>= 0, engine driving pto).
    CHECK(ledger.gear_loss >= -1e-6);
}

// --- Pto: pto_clutch_loss ledger closure ----------------------------------
TEST_CASE("Pto: pto_clutch_loss equals row slip loss while engaged plus the disengaged "
          "stub-brake's own dissipation once released",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId in = nb.add_node(0.15);
    const real J_out = 0.05;
    const NodeId out = nb.add_node(J_out);
    const FrictionRowId row = nb.add_friction_row(in, out);
    ShaftNetwork net = nb.build("test-pto-ledger-closure");

    PtoDesc desc;
    desc.has_540 = true;
    desc.ratio_540 = 1.6;
    desc.capacity_nm = 40.0; // small: guarantees genuine slipping while engaging
    desc.static_factor = 1.2;
    desc.engage_time_s = 0.3;
    desc.disengaged_brake_time_s = 0.5;
    Pto pto(in, out, row, desc, 0, 1, J_out);

    real channels[2] = {1.0, 540.0};
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 2;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(in, 120.0);
    net.set_omega(out, 0.0);

    real expected_loss = 0.0;
    // Phase 1: engage and slip for a while.
    for (int i = 0; i < 1000; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        net.solve(ctx.h, assembly);
        const real v = net.row_velocity(row);
        const real lambda = net.row_lambda(row);
        pto.post_solve(ctx, net, assembly, ledger);
        expected_loss += -ctx.h * v * lambda; // row-slip contribution only (stub inactive while e>0)
    }
    CHECK(ledger.pto_clutch_loss == Catch::Approx(expected_loss).epsilon(1e-6));

    // Phase 2: disengage; the stub brake now dissipates omega_out's KE.
    channels[0] = 0.0;
    for (int i = 0; i < 2000; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        pto.pre_solve(ctx, net, assembly);
        const bool disengaged_exactly = (pto.engagement() == 0.0);
        const real omega_out_old = net.omega(out);
        net.solve(ctx.h, assembly);
        const real v = net.row_velocity(row);
        const real lambda = net.row_lambda(row);
        pto.post_solve(ctx, net, assembly, ledger);
        expected_loss += -ctx.h * v * lambda;
        if (disengaged_exactly) {
            const real c = J_out / desc.disengaged_brake_time_s;
            const real tau0 = -c * omega_out_old;
            const real tau_applied = tau0 - c * (net.omega(out) - omega_out_old);
            expected_loss += -ctx.h * tau_applied * net.omega(out);
        }
    }
    CHECK(ledger.pto_clutch_loss == Catch::Approx(expected_loss).epsilon(1e-6));
    // Both phases dissipate (never inject) energy.
    CHECK(ledger.pto_clutch_loss >= -1e-6);
}

// --- LoadDyno: Brake mode --------------------------------------------------
TEST_CASE("LoadDyno: Brake mode caps the row at torque_nm*scale (static==kinetic) and "
          "books -h*v*lambda into external_absorbed",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId shaft = nb.add_node(0.08);
    const FrictionRowId row = nb.add_friction_row(shaft);
    ShaftNetwork net = nb.build("test-dyno-brake");

    LoadDynoDesc desc;
    desc.mode = LoadDynoMode::Brake;
    desc.torque_nm = 30.0;
    desc.scale_channel = "dyno_scale";
    LoadDyno dyno(shaft, row, desc, /*scale_channel_slot=*/0);

    real channels[1] = {0.5}; // scale
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 1;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(shaft, 80.0);
    real expected_absorbed = 0.0;
    for (int i = 0; i < 500; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        dyno.pre_solve(ctx, net, assembly);
        const real expected_cap = desc.torque_nm * channels[0];
        CHECK(assembly.cap_kinetic(row) == Catch::Approx(expected_cap));
        CHECK(assembly.cap_static(row) == Catch::Approx(expected_cap));
        net.solve(ctx.h, assembly);
        const real v = net.row_velocity(row);
        const real lambda = net.row_lambda(row);
        dyno.post_solve(ctx, net, assembly, ledger);
        expected_absorbed += -ctx.h * v * lambda;
        CHECK(dyno.torque() == Catch::Approx(lambda));
    }
    CHECK(ledger.external_absorbed == Catch::Approx(expected_absorbed).epsilon(1e-6));
    CHECK(ledger.external_absorbed >= -1e-6); // a brake only absorbs
}

// --- LoadDyno: Viscous mode -------------------------------------------------
TEST_CASE("LoadDyno: Viscous mode follows the exact implicit-Euler recurrence "
          "w' = w*J/(J+h*c), c = viscous_nm_s_rad*scale",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const real J = 0.06;
    const NodeId shaft = nb.add_node(J);
    const FrictionRowId row = nb.add_friction_row(shaft); // unused by Viscous mode, but the ctor needs one
    ShaftNetwork net = nb.build("test-dyno-viscous");

    LoadDynoDesc desc;
    desc.mode = LoadDynoMode::Viscous;
    desc.viscous_nm_s_rad = 4.0;
    LoadDyno dyno(shaft, row, desc, -1); // no scale channel => scale == 1

    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = nullptr;
    ctx.channel_count = 0;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(shaft, 150.0);
    const real c = desc.viscous_nm_s_rad;
    real expected = 150.0;
    real expected_absorbed = 0.0;
    for (int i = 0; i < 500; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        dyno.pre_solve(ctx, net, assembly);
        const real omega_old = net.omega(shaft);
        net.solve(ctx.h, assembly);
        dyno.post_solve(ctx, net, assembly, ledger);

        expected = expected * J / (J + ctx.h * c);
        CHECK(net.omega(shaft) == Catch::Approx(expected).epsilon(1e-9));

        const real tau0 = -c * omega_old;
        const real tau_applied = tau0 - c * (net.omega(shaft) - omega_old);
        expected_absorbed += -ctx.h * tau_applied * net.omega(shaft);
    }
    CHECK(ledger.external_absorbed == Catch::Approx(expected_absorbed).epsilon(1e-6));
}

// --- LoadDyno: Quadratic mode -----------------------------------------------
TEST_CASE("LoadDyno: Quadratic mode's linearised-about-omega_old implicit update matches "
          "tau0=-k*w*|w|, d=2*k*|w| and its own energy bookkeeping identity",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const real J = 0.06;
    const NodeId shaft = nb.add_node(J);
    const FrictionRowId row = nb.add_friction_row(shaft);
    ShaftNetwork net = nb.build("test-dyno-quadratic");

    LoadDynoDesc desc;
    desc.mode = LoadDynoMode::Quadratic;
    desc.quadratic_nm_s2_rad2 = 0.02;
    desc.scale_channel = "dyno_scale";
    LoadDyno dyno(shaft, row, desc, 0);

    real channels[1] = {2.0};
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = channels;
    ctx.channel_count = 1;
    Assembly assembly;
    PowertrainEnergy ledger{};

    net.set_omega(shaft, 200.0);
    real expected_absorbed = 0.0;
    for (int i = 0; i < 500; ++i) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        const real omega_old = net.omega(shaft);
        const real k = desc.quadratic_nm_s2_rad2 * channels[0];
        const real expected_tau0 = -k * omega_old * std::abs(omega_old);
        const real expected_d = 2.0 * k * std::abs(omega_old);
        dyno.pre_solve(ctx, net, assembly);
        // Re-run the network's implicit update by hand to confirm the
        // component wrote exactly {tau0, d} into the assembly: with the
        // node otherwise isolated, w_new = w_old + h*tau0/(J + h*d).
        net.solve(ctx.h, assembly);
        const real predicted = omega_old + ctx.h * expected_tau0 / (J + ctx.h * expected_d);
        CHECK(net.omega(shaft) == Catch::Approx(predicted).epsilon(1e-9));
        dyno.post_solve(ctx, net, assembly, ledger);

        const real tau_applied = expected_tau0 - expected_d * (net.omega(shaft) - omega_old);
        expected_absorbed += -ctx.h * tau_applied * net.omega(shaft);
    }
    CHECK(ledger.external_absorbed == Catch::Approx(expected_absorbed).epsilon(1e-6));
    CHECK(net.omega(shaft) < 200.0); // decelerating drag, never negative overshoot into runaway
    CHECK(net.omega(shaft) > 0.0);
}

// --- Row-list placement: car_sedan + a load_dyno-Brake in code -------------
// Mirrors test_powertrain_park_lock.cpp's own row-placement technique
// exactly (build two Powertrains, one with an extra component, and compare
// ShaftNetwork::friction_row_a/b index-by-index).
TEST_CASE("Powertrain: adding a load_dyno-Brake to car_sedan.json in code inserts its own "
          "friction row at index 3 (after clutch/engine/synchro), shifting the 4 shipped "
          "brake rows from [3..6] to [4..7]",
          "[unit][drivetrain][vehicle]") {
    constexpr int kSedanDynoInsertIndex = 3; // clutch(0), engine(1), synchro(2), then dyno

    const Powertrain baseline = build(load_sedan());
    const Powertrain with_dyno = build(load_sedan_with_dyno_brake());

    const int n_baseline = baseline.network().friction_row_count();
    REQUIRE(n_baseline >= kSedanDynoInsertIndex);
    CHECK(with_dyno.network().friction_row_count() == n_baseline + 1);

    for (int i = 0; i < kSedanDynoInsertIndex; ++i) {
        const FrictionRowId id{static_cast<std::uint16_t>(i)};
        CHECK(with_dyno.network().friction_row_a(id) == baseline.network().friction_row_a(id));
        CHECK(with_dyno.network().friction_row_b(id) == baseline.network().friction_row_b(id));
    }
    for (int i = kSedanDynoInsertIndex; i < n_baseline; ++i) {
        const FrictionRowId dyno_id{static_cast<std::uint16_t>(i + 1)};
        const FrictionRowId base_id{static_cast<std::uint16_t>(i)};
        CHECK(with_dyno.network().friction_row_a(dyno_id) == baseline.network().friction_row_a(base_id));
        CHECK(with_dyno.network().friction_row_b(dyno_id) == baseline.network().friction_row_b(base_id));
    }
}

// --- Row-list placement: tractor with/without pto --------------------------
TEST_CASE("Powertrain: tractor.json's pto row sits between the park_lock row and the "
          "brake rows (brakes shift by one); car_sedan has neither pto nor load_dyno",
          "[unit][drivetrain][vehicle]") {
    constexpr int kPtoRowIndex = 5; // clutch(0), engine(1), synchro(2), transfer_case(3), park_lock(4), then pto

    const Powertrain baseline = build(load_tractor_without_pto());
    const Powertrain with_pto = build(load_tractor());
    REQUIRE(with_pto.has_pto());
    REQUIRE_FALSE(baseline.has_pto());

    const int n_baseline = baseline.network().friction_row_count();
    REQUIRE(n_baseline >= kPtoRowIndex);
    CHECK(with_pto.network().friction_row_count() == n_baseline + 1);

    for (int i = 0; i < kPtoRowIndex; ++i) {
        const FrictionRowId id{static_cast<std::uint16_t>(i)};
        CHECK(with_pto.network().friction_row_a(id) == baseline.network().friction_row_a(id));
        CHECK(with_pto.network().friction_row_b(id) == baseline.network().friction_row_b(id));
    }
    for (int i = kPtoRowIndex; i < n_baseline; ++i) {
        const FrictionRowId pto_id{static_cast<std::uint16_t>(i + 1)};
        const FrictionRowId base_id{static_cast<std::uint16_t>(i)};
        CHECK(with_pto.network().friction_row_a(pto_id) == baseline.network().friction_row_a(base_id));
        CHECK(with_pto.network().friction_row_b(pto_id) == baseline.network().friction_row_b(base_id));
    }

    // car_sedan.json declares no pto/load_dyno at all - fully opt-in.
    const Powertrain sedan = build(load_sedan());
    CHECK_FALSE(sedan.has_pto());
    CHECK(sedan.pto_engagement() == 0.0);
    CHECK(sedan.pto_mode() == 0.0);
    CHECK(sedan.pto_rpm() == 0.0);
    CHECK(sedan.pto_clutch_torque() == 0.0);
    CHECK(sedan.pto_clutch_slip() == 0.0);
    CHECK(sedan.dyno_torque() == 0.0);
}

// --- Whole-World: debug_alloc-armed zero-alloc stepping --------------------
TEST_CASE("a tractor with pto toggled mid-run and a load_dyno attached on pto.out "
          "allocates nothing (own) while stepping",
          "[unit][vehicle][world][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan "
             "- nothing to check here on this build.");
        return;
    }

    auto rig = ps_test::build_vehicle_rig_from_desc(load_tractor_with_pto_dyno(), 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.0, 1);
    World& world = *rig.world;
    world.set_control("pto", 1.0);
    for (int i = 0; i < 300; ++i) world.step();

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 60; ++i) world.step();
    world.set_control("pto", 0.0);
    for (int i = 0; i < 60; ++i) world.step();
    world.set_control("pto", 1.0);
    for (int i = 0; i < 60; ++i) world.step();
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();

    WARN("external (Jolt-internal, pre-existing broadphase-rebuild gap) violation count=" << external);
    CHECK(own == 0);
}

// --- Whole-World: 1 vs 10 worker state_hash equality ------------------------
TEST_CASE("tractor state_hash with pto toggled and a mode change mid-run is worker-count "
          "invariant (1 vs 10 workers)",
          "[unit][vehicle][drivetrain][determinism]") {
    auto run = [](int workers) {
        auto rig = ps_test::build_vehicle_rig_from_desc(load_tractor_with_pto_dyno(), 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.0, workers);
        World& w = *rig.world;
        w.set_control("pto", 1.0);
        ps_test::settle(w, 1.0);
        w.set_control("pto", 0.0); // disengage - let the stub brake decay omega_out
        for (int i = 0; i < 300; ++i) w.step(); // 1.25s at 240Hz, well past the 1.0s default stub time constant
        w.set_control("pto_speed", 1000.0); // now eligible for the mode change
        for (int i = 0; i < 60; ++i) w.step();
        w.set_control("pto", 1.0); // re-engage in 1000 mode
        for (int i = 0; i < 120; ++i) w.step();
        return w.state_hash();
    };
    const std::uint64_t h1 = run(1);
    const std::uint64_t h10 = run(10);
    CHECK(h1 == h10);
}
