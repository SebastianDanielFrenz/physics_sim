// PHYS-033 P4 I7c C16b: World::replace_engine_maps's sim-thread-safe hot
// swap of a running vehicle's simulated-engine maps (out/i7c_plan.md D9,
// tests H5-H7; H1-H4, the .psmaps disk cache, live in
// test_engine_map_cache.cpp).
//
// Contract under test (core/src/drivetrain/powertrain.cpp's
// request_replace_engine_maps/apply_pending_engine_maps_swap):
//   - World::replace_engine_maps(id, component_id, maps) is sim-thread/
//     between-World::step()-calls only. It builds the new engine's derived
//     tables and QUEUES a pointer swap (Powertrain::request_replace_
//     engine_maps -> SimulatedEngine::request_replace_maps) - the build
//     allocates on the CALLING thread, never inside a job.
//   - The queued swap is CONSUMED at the very start of the next
//     World::step(), before its tick pipeline runs (World::step calls
//     vehicle::System::apply_pending_engine_maps_swaps - outside any job;
//     C16b review: it used to run inside the vehicle_islands_substep job),
//     so a vehicle's behaviour is bit-identical through the tick the swap
//     was requested on and can differ starting the very next step() call.
//     The queueing call also prefills the engine's ambient-dependent steady
//     tables from the new maps; apply_pending_maps_swap is a pointer and
//     buffer exchange (no allocation, no free, no refill), and the old maps
//     drop right there, outside any job.
//   - An unknown component id, or a known component that is not a
//     simulated engine, or null maps, throws std::runtime_error with a
//     pinned message. H7 (PROVISIONAL, see this file's own H7 case
//     comment) deliberately does not cover an unknown VehicleId:
//     ps::vehicle::System::at(VehicleId) is a plain operator[] with no
//     bounds check, so that path is undefined behaviour, not a pinned
//     error - out of scope for a regression test.

#include "drivetrain/engine_simulated.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <variant>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
namespace da = ps::jobs::debug_alloc;

namespace {

vehicle::VehicleDesc load_sim_vehicle() {
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_sim.json");
}

// H7's "known component id, not a simulated engine" case needs a vehicle
// whose engine component is a TorqueMapEngine: Powertrain::
// request_replace_engine_maps has no general component-id registry, only
// a single tracked engine_component_id_ (whichever component is "the
// engine", simulated or not) - so any id other than that one always fails
// the EARLIER "unknown component id" check first (see this file's header
// comment). car_sedan.json's engine component id is "engine", same as
// car_sedan_sim.json's, so passing "engine" here matches
// engine_component_id_ but the engine behind it is not a SimulatedEngine.
vehicle::VehicleDesc load_torque_map_vehicle() {
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
}

// The loaded desc's own SimulatedEngineDesc - the "engine" component of
// data/vehicles/car_sedan_sim.json (data/engines/toyota_a25a_fks_sim.json,
// kind "simulated"). Throws (via REQUIRE) if the fixture ever stops
// shipping one, rather than silently no-op-ing.
const SimulatedEngineDesc& sim_engine_desc(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) return *e;
    }
    REQUIRE(false); // unreachable given car_sedan_sim.json's own schema
    static const SimulatedEngineDesc dummy{};
    return dummy;
}

// A deliberately different EngineMaps: the loaded engine's own maps with
// every ve node scaled by `ve_scale` (car_sedan_sim.json's ve nodes sit
// well inside (0, 2), so a 0.5 scale stays in-range and starves the engine
// of a large chunk of its charge - a big enough perturbation that it must
// move a running engine's torque, not just round-trip through the loader
// unchanged). Only .maps is read by request_replace_engine_maps
// (powertrain.cpp's own comment: "Only .maps ever differs from the desc
// this engine was originally built from"), so every other EngineMaps field
// is left default.
EngineMapsPtr make_scaled_maps(const vehicle::VehicleDesc& d, real ve_scale) {
    SimulatedEngineMapsDesc maps = sim_engine_desc(d).maps;
    for (real& v : maps.ve.values) v *= ve_scale;
    auto out = std::make_shared<EngineMaps>();
    out->maps = std::move(maps);
    return out;
}

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

Rig make_rig(const vehicle::VehicleDesc& desc, int workers) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    rig.world = std::make_unique<World>(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId ch = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    rig.vehicle = rig.world->create_vehicle(desc, ch);
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.5);
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

} // namespace

// H5: a hot swap changes NOTHING before it is consumed. World::
// replace_engine_maps only queues the swap (the request itself is issued
// between two step() calls, never inside one); the tick that issues it
// keeps running the OLD maps, and the very next step() is the first tick
// that can differ, because World::step applies the queued swap right
// before running the tick pipeline.
TEST_CASE("C16b H5: a hot-swapped engine map is bit-identical through the requesting tick, may differ the next", "[drivetrain][cycle][io]") {
    const vehicle::VehicleDesc desc = load_sim_vehicle();
    Rig a = make_rig(desc, 2);
    Rig b = make_rig(desc, 2);

    constexpr int kSwapAtTick = 50;
    for (int i = 0; i < kSwapAtTick; ++i) {
        a.world->step();
        b.world->step();
    }
    REQUIRE(a.world->state_hash() == b.world->state_hash());

    const EngineMapsPtr swapped = make_scaled_maps(desc, 0.5);
    b.world->replace_engine_maps(b.vehicle, "engine", swapped);
    // Queued, not yet consumed: still bit-identical to a (which never gets
    // a swap at all).
    CHECK(a.world->state_hash() == b.world->state_hash());

    // This step() is the one that consumes the queued swap, at its START
    // (before the pipeline) - so b's behaviour THIS tick already reflects
    // the new maps.
    a.world->step();
    b.world->step();
    CHECK(a.world->state_hash() != b.world->state_hash());

    // Divergence persists (not a one-tick fluke of float rounding).
    for (int i = 0; i < 20; ++i) {
        a.world->step();
        b.world->step();
    }
    CHECK(a.world->state_hash() != b.world->state_hash());
}

// H6: the hot swap is worker-count invariant (the queued build runs on the
// calling thread outside any job; the consuming apply_pending_maps_swap is
// a pointer move done once per vehicle, in the same fixed vehicle order
// every worker count uses) and the tick that consumes it allocates nothing
// in the vehicle job (a pointer move, per apply_pending_maps_swap's own
// header comment - the allocating build already happened on the caller's
// thread before this step() was even called).
TEST_CASE("C16b H6: hot swap is worker-count invariant and its consuming tick allocates nothing in the vehicle job", "[drivetrain][cycle][io][debug_alloc][jobs]") {
    const vehicle::VehicleDesc desc = load_sim_vehicle();
    const bool alloc_hook = da::hook_available();
    if (!alloc_hook) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - the allocation check is skipped on this build.");
    }

    std::uint64_t hash_1w = 0, hash_10w = 0;
    for (const int workers : {1, 10}) {
        Rig rig = make_rig(desc, workers);
        for (int i = 0; i < 50; ++i) rig.world->step();

        const EngineMapsPtr swapped = make_scaled_maps(desc, 0.5);
        rig.world->replace_engine_maps(rig.vehicle, "engine", swapped);

        if (alloc_hook) {
            da::reset();
            da::set_armed(true);
        }
        rig.world->step(); // consumes the queued swap
        if (alloc_hook) {
            da::set_armed(false);
            const std::uint64_t own = da::own_violation_count();
            const std::uint64_t external = da::external_violation_count();
            da::reset();
            WARN("workers=" << workers << " external (Jolt-internal, ungated) violation count=" << external);
            CHECK(own == 0);
        }

        for (int i = 0; i < 20; ++i) rig.world->step();
        (workers == 1 ? hash_1w : hash_10w) = rig.world->state_hash();
    }
    CHECK(hash_1w == hash_10w);
}

// H7 (PROVISIONAL, scoped): every rejection path reachable on a KNOWN
// vehicle. "Unknown VehicleId" is intentionally NOT covered here -
// ps::vehicle::System::at(VehicleId) (core/src/vehicle/system.h) is a bare
// operator[] with no bounds/generation check, so calling replace_engine_maps
// on a bogus/stale VehicleId is undefined behaviour at the System::at()
// call inside Powertrain::request_replace_engine_maps's caller, not a
// pinned, testable error - the same class of precondition every other
// VehicleId-taking World method (wheel_telemetry, powertrain_energy, ...)
// already leaves to the caller.
TEST_CASE("C16b H7: replace_engine_maps rejects a bad request on a known vehicle without applying it", "[drivetrain][cycle][io]") {
    const vehicle::VehicleDesc desc = load_sim_vehicle();
    Rig rig = make_rig(desc, 2);
    const EngineMapsPtr maps = make_scaled_maps(desc, 1.0);

    CHECK_THROWS_WITH(rig.world->replace_engine_maps(rig.vehicle, "not_a_real_component", maps),
                       ContainsSubstring("unknown component id"));
    CHECK_THROWS_WITH(rig.world->replace_engine_maps(rig.vehicle, "engine", nullptr), ContainsSubstring("null maps"));

    // None of the rejected calls queued anything: stepping still reproduces
    // an un-swapped reference run bit-for-bit.
    Rig reference = make_rig(desc, 2);
    for (int i = 0; i < 30; ++i) {
        rig.world->step();
        reference.world->step();
    }
    CHECK(rig.world->state_hash() == reference.world->state_hash());

    // "known component id, not a simulated engine": request_replace_engine_
    // maps has no general component registry (see load_torque_map_vehicle's
    // comment above) - the only way to reach this branch is to pass the
    // correct engine component id ("engine", same id car_sedan_sim.json
    // uses) on a vehicle whose engine is a TorqueMapEngine, not a
    // SimulatedEngine. A separate vehicle/world, since car_sedan.json's
    // engine has no SimulatedEngineMapsDesc to build a Rig from otherwise.
    const vehicle::VehicleDesc tm_desc = load_torque_map_vehicle();
    Rig tm_rig = make_rig(tm_desc, 2);
    CHECK_THROWS_WITH(tm_rig.world->replace_engine_maps(tm_rig.vehicle, "engine", maps),
                       ContainsSubstring("is not a simulated engine"));

    Rig tm_reference = make_rig(tm_desc, 2);
    for (int i = 0; i < 30; ++i) {
        tm_rig.world->step();
        tm_reference.world->step();
    }
    CHECK(tm_rig.world->state_hash() == tm_reference.world->state_hash());
}

// ---------------------------------------------------------------------------
// C16b review additions (R1-R3). Each was shown to fail on its defect; the
// sabotage and the measured numbers are in the review's report.

// R1 (review item 2b): after a swap the engine's ambient-dependent steady
// tables (the ECU's steady demand model, the electronic inverse, the idle
// feed-forward) are those a fresh engine on the NEW maps holds at the same
// ambient - bit for bit - and the refill that produces them happens in
// request_replace_maps (caller thread, between steps), not in the apply
// that runs at the tick boundary. A non-ISA ambient forces the real
// fill_steady_tables path (at ISA the tables are a plain copy).
TEST_CASE("C16b review R1: swapped steady tables follow the new maps, prefilled at request time", "[drivetrain][cycle]") {
    const vehicle::VehicleDesc vd = load_sim_vehicle();
    const SimulatedEngineDesc base = sim_engine_desc(vd);
    SimulatedEngineDesc scaled = base;
    for (real& v : scaled.maps.ve.values) v *= 0.5;
    constexpr real kP = 90000.0;
    constexpr real kT = 300.0;

    SimulatedEngine ref(base); // never swapped
    ref.set_ambient(kP, kT);
    SimulatedEngine e(base);
    e.set_ambient(kP, kT);
    REQUIRE(e.steady_rebuild_count() == 1);

    const std::shared_ptr<const SimulatedEngineMaps> new_maps = SimulatedEngine::build_maps(scaled);
    e.request_replace_maps(new_maps);
    // Queued, not applied: the engine still runs the old tables.
    CHECK(e.steady_tables().steady_wot_nm.y == ref.steady_tables().steady_wot_nm.y);
    CHECK(e.steady_rebuild_count() == 1);

    const std::shared_ptr<const SimulatedEngineMaps> old = e.apply_pending_maps_swap();
    REQUIRE(old != nullptr);
    CHECK(e.maps_ptr() == new_maps);
    // The apply refills nothing (it runs at the tick boundary; the ~68 ms
    // refill must not).
    CHECK(e.steady_rebuild_count() == 1);

    SimulatedEngine fresh(scaled, new_maps);
    fresh.set_ambient(kP, kT);
    REQUIRE(fresh.steady_rebuild_count() == 1);
    const SimulatedSteadyTables& s = e.steady_tables();
    const SimulatedSteadyTables& f = fresh.steady_tables();
    CHECK(s.p_pa == f.p_pa);
    CHECK(s.t_k == f.t_k);
    CHECK(s.steady_wot_nm.x == f.steady_wot_nm.x);
    CHECK(s.steady_wot_nm.y == f.steady_wot_nm.y);
    CHECK(s.steady_motoring_nm.y == f.steady_motoring_nm.y);
    CHECK(s.steady_torque_nm.values == f.steady_torque_nm.values);
    CHECK(s.inverse_plate.values == f.inverse_plate.values);
    CHECK(s.idle_ff == f.idle_ff);
    CHECK(s.idle_p_im == f.idle_p_im);

    // A steady quantity follows the new VE: steady WOT torque at the
    // middle of its axis drops with half the charge.
    const std::size_t mid = s.steady_wot_nm.x.size() / 2;
    const real x_mid = s.steady_wot_nm.x[mid];
    const real wot_old = ref.steady_tables().steady_wot_nm.evaluate(x_mid);
    const real wot_new = s.steady_wot_nm.evaluate(x_mid);
    std::printf("R1 steady WOT at x=%.1f: old maps %.3f N*m, swapped %.3f N*m (fresh on new maps %.3f)\n", x_mid, wot_old, wot_new,
                f.steady_wot_nm.evaluate(x_mid));
    CHECK(wot_new < real(0.8) * wot_old);

    // The engine's next set_ambient at the unchanged ambient (what
    // pre_solve does every substep) refills nothing either.
    e.set_ambient(kP, kT);
    CHECK(e.steady_rebuild_count() == 1);
}

// R2 (review item 2c): the tick that consumes a swap frees nothing inside a
// job - the old maps (and the old steady tables) are released by World::step
// before the pipeline runs. debug_alloc's allocation counters cannot see a
// free, hence own_free_in_job_count(); the first block proves that counter
// is wired (a free inside a job is seen).
TEST_CASE("C16b review R2: the swap tick frees nothing inside a job", "[drivetrain][cycle][debug_alloc][jobs]") {
    if (!da::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - R2 is skipped on this build.");
        return;
    }
    {
        jobs::JobSystem js(2);
        auto p = std::make_unique<int>(7);
        da::reset();
        da::set_armed(true);
        js.parallel_for(1, 1, [&p](std::size_t, std::size_t) { p.reset(); });
        da::set_armed(false);
        CHECK(da::own_free_in_job_count() == 1);
        da::reset();
    }

    const vehicle::VehicleDesc desc = load_sim_vehicle();
    Rig rig = make_rig(desc, 2);
    for (int i = 0; i < 50; ++i) rig.world->step();

    da::reset();
    da::set_armed(true);
    rig.world->step(); // a plain tick
    da::set_armed(false);
    const std::uint64_t plain_frees = da::own_free_in_job_count();

    rig.world->replace_engine_maps(rig.vehicle, "engine", make_scaled_maps(desc, 0.5));
    da::reset();
    da::set_armed(true);
    rig.world->step(); // consumes the swap
    da::set_armed(false);
    const std::uint64_t swap_frees = da::own_free_in_job_count();
    const std::uint64_t swap_allocs = da::own_violation_count();
    da::reset();
    std::printf("R2 frees inside jobs: plain tick %llu, swap tick %llu (own allocations on the swap tick %llu)\n",
                static_cast<unsigned long long>(plain_frees), static_cast<unsigned long long>(swap_frees),
                static_cast<unsigned long long>(swap_allocs));
    CHECK(plain_frees == 0);
    CHECK(swap_frees == 0);
    CHECK(swap_allocs == 0);
}

// R3 (H5, the spec clauses the original H5 case left out): swapping in maps
// identical to the running ones gives a run identical to no swap; swapping
// in maps that differ only in a far-corner VE node (max rpm, max p_im - a
// cell a part-throttle launch in 1st never reads) moves state_hash on the
// very next tick, i.e. the hash folds the CURRENT maps, not the ones the
// engine was built with.
TEST_CASE("C16b review R3 (H5): an identical-maps swap is invisible, a far-corner-node swap shows in the hash at once", "[drivetrain][cycle][io]") {
    const vehicle::VehicleDesc desc = load_sim_vehicle();
    {
        Rig a = make_rig(desc, 2);
        Rig b = make_rig(desc, 2);
        for (int i = 0; i < 50; ++i) {
            a.world->step();
            b.world->step();
        }
        b.world->replace_engine_maps(b.vehicle, "engine", make_scaled_maps(desc, 1.0));
        int mismatched_ticks = 0;
        for (int i = 0; i < 60; ++i) {
            a.world->step();
            b.world->step();
            if (a.world->state_hash() != b.world->state_hash()) ++mismatched_ticks;
        }
        CHECK(mismatched_ticks == 0);
    }
    {
        Rig a = make_rig(desc, 2);
        Rig b = make_rig(desc, 2);
        for (int i = 0; i < 50; ++i) {
            a.world->step();
            b.world->step();
        }
        SimulatedEngineMapsDesc m = sim_engine_desc(desc).maps;
        m.ve.values.back() *= 1.01;
        auto far = std::make_shared<EngineMaps>();
        far->maps = std::move(m);
        b.world->replace_engine_maps(b.vehicle, "engine", far);
        a.world->step();
        b.world->step();
        CHECK(a.world->state_hash() != b.world->state_hash());
    }
}
