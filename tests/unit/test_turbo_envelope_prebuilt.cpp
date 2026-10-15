// Q5 (docs/next_steps_brief.md 2.1): the turbo full-load envelope built OFF the thread that swaps the turbo.
//
// Design (ps/drivetrain/turbo_envelope.h): World::prepare_turbo_envelope (caller thread, cheap) -> build_turbo_envelope
// (any thread) -> World::replace_turbo_configuration(id, component, prepared) (caller thread, between steps). The swap
// itself is immediate, like the synchronous overload; the 0.6 s dyno run is what moved.
//
// Acceptance bands (written before the first run):
//   E1  hyper engine: the prebuilt envelope equals the synchronous one, all five curves bit for bit (memo cleared between
//       the two builds so they are independent runs), and the engine installs the prepared object itself.
//   E2  stale prepared (maps replaced between prepare and replace) or unbuilt prepared: the engine ends with the envelope
//       of ITS maps/turbo (== a fresh full_load_envelope), never the prepared one.
//   W1  car_hyper World, 1 and 10 workers: the state_hash after every tick following the swap is identical for the
//       synchronous and the prebuilt path, and across worker counts; the ticks after the swap allocate nothing in jobs
//       (debug_alloc own violations == 0).
//   T1  timing (printed): replace on a memo miss, synchronous vs prepared; the caller-thread cost of the prepared path
//       (prepare + replace) must be < 20 ms (measured 0.61 s synchronous at the brief).
// Sabotages (run once by hand, each must make its test fail; recorded in the commit message):
//   S1  build_turbo_envelope builds for a different turbo (rotor inertia x 0.9 in its desc copy) -> E1, W1 fail.
//   S2  SimulatedEngine::replace_turbo_configuration drops the `prepared->maps == maps_` check -> E2 fails.
//   S3  the prepared path skips the envelope install (envelope_ stays the old one) -> E1, W1 fail.

#include "drivetrain/engine_simulated.h"
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/turbo_envelope.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;

namespace {

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

vehicle::VehicleDesc load_hyper() {
    const std::string cache = std::string(PS_SOURCE_DIR) + "/out/pedal_p1_map_cache"; // shared with test_pedal_law
    std::filesystem::create_directories(cache);
    io::VehicleLoadOptions o;
    o.maps.pool = &pool();
    o.maps.cache_dir = cache;
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", o);
}

const SimulatedEngineDesc& sim_engine(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) return *e;
    FAIL("no simulated engine");
    static const SimulatedEngineDesc dummy{};
    return dummy;
}

SimulatedEngineDesc electronic_desc(const vehicle::VehicleDesc& v) {
    SimulatedEngineDesc d = sim_engine(v);
    d.fuels.resize(1); // the burnt fuel, as Powertrain builds it
    return d;
}

std::string engine_id(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (std::holds_alternative<SimulatedEngineDesc>(c.params)) return c.id;
    return {};
}

TurboPairDesc replacement_turbo(const SimulatedEngineDesc& d, real inertia_scale) {
    TurboPairDesc t = *d.turbo_pair;
    t.rotor_inertia_kgm2 *= inertia_scale; // a new memo key: a real cache miss
    return t;
}

bool same_curve(const vehicle::Curve1D& a, const vehicle::Curve1D& b) { return a.x == b.x && a.y == b.y; }
bool same_envelope(const FullLoadEnvelope& a, const FullLoadEnvelope& b) {
    return a.key == b.key && same_curve(a.torque_nm, b.torque_nm) && same_curve(a.pressure_ratio, b.pressure_ratio) &&
           same_curve(a.charge_k, b.charge_k) && same_curve(a.exhaust_ratio, b.exhaust_ratio) && same_curve(a.charge_k, b.charge_k) &&
           same_curve(a.model_offset_nm, b.model_offset_nm);
}

} // namespace

TEST_CASE("Q5 E1/T1: the prebuilt turbo envelope equals the synchronous one, bit for bit; the caller thread does not pay for it",
          "[turbo][envelope][drivetrain][q5][long]") {
    const vehicle::VehicleDesc v = load_hyper();
    const SimulatedEngineDesc d = electronic_desc(v);
    const TurboPairDesc t = replacement_turbo(d, 1.37);

    clear_full_load_envelope_memo();
    SimulatedEngine sync(d);
    auto t0 = Clock::now();
    sync.replace_turbo_configuration(t);
    const double sync_ms = ms_since(t0);
    REQUIRE(sync.full_load_envelope_ptr() != nullptr);
    const FullLoadEnvelope ref = *sync.full_load_envelope_ptr();

    clear_full_load_envelope_memo(); // the prebuilt run is an independent dyno run, not a memo hit
    SimulatedEngine pre(d);
    t0 = Clock::now();
    PreparedTurboEnvelope p = pre.prepare_turbo_envelope(t);
    const double prepare_ms = ms_since(t0);
    CHECK_FALSE(p.built);
    double build_ms = 0.0;
    std::thread th([&] {
        const auto b0 = Clock::now();
        build_turbo_envelope(p);
        build_ms = ms_since(b0);
    });
    th.join();
    REQUIRE(p.built);
    REQUIRE(p.envelope != nullptr);
    t0 = Clock::now();
    pre.replace_turbo_configuration(t, &p);
    const double replace_ms = ms_since(t0);

    CHECK(pre.full_load_envelope_ptr() == p.envelope.get()); // installed, not rebuilt
    REQUIRE(pre.full_load_envelope_ptr() != nullptr);
    CHECK(same_envelope(*pre.full_load_envelope_ptr(), ref));
    std::printf("[Q5 T1] hypercar cache miss: synchronous replace %.1f ms on the caller; prebuilt: prepare %.3f ms + replace %.3f ms on the "
                "caller, build %.1f ms on the background thread\n",
                sync_ms, prepare_ms, replace_ms, build_ms);
    CHECK(prepare_ms + replace_ms < 20.0);
}

TEST_CASE("Q5 E2: a stale or unbuilt prepared envelope is never installed", "[turbo][envelope][drivetrain][q5][long]") {
    const vehicle::VehicleDesc v = load_hyper();
    const SimulatedEngineDesc d = electronic_desc(v);
    const TurboPairDesc t = replacement_turbo(d, 1.21);

    SECTION("unbuilt") {
        SimulatedEngine e(d);
        PreparedTurboEnvelope p = e.prepare_turbo_envelope(t);
        e.replace_turbo_configuration(t, &p);
        REQUIRE(e.full_load_envelope_ptr() != nullptr);
        auto dd = d;
        dd.turbo_pair = t;
        CHECK(e.full_load_envelope_ptr() == full_load_envelope(dd, e.maps_ptr()).get());
    }
    SECTION("maps replaced between prepare and replace") {
        SimulatedEngine e(d);
        PreparedTurboEnvelope p = e.prepare_turbo_envelope(t);
        build_turbo_envelope(p);
        SimulatedEngineDesc other = d;
        for (real& x : other.maps.ve.values) x *= 0.9;
        e.request_replace_maps(SimulatedEngine::build_maps(other));
        (void)e.apply_pending_maps_swap();
        REQUIRE(e.maps_ptr() != p.maps);
        e.replace_turbo_configuration(t, &p);
        REQUIRE(e.full_load_envelope_ptr() != nullptr);
        auto dd = other;
        dd.turbo_pair = t;
        const auto fresh = full_load_envelope(dd, e.maps_ptr());
        CHECK(e.full_load_envelope_ptr() == fresh.get());
        CHECK(e.full_load_envelope_ptr() != p.envelope.get());
        CHECK_FALSE(same_envelope(*p.envelope, *fresh)); // the maps matter: the stale one would have been wrong
    }
}

namespace {

struct SwapRun {
    std::vector<std::uint64_t> hashes;
    std::uint64_t own_alloc = 0;
};

SwapRun world_swap(const vehicle::VehicleDesc& desc, int workers, bool prebuilt, double* caller_ms) {
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, {2.0, 0.4, 0.15}, 0.6, 0.0, workers);
    World& w = *rig.world;
    w.set_control("throttle", 0.4);
    ps_test::settle(w, 1.0);
    for (int i = 0; i < 120; ++i) w.step();
    const std::string id = engine_id(desc);
    const TurboPairDesc t = replacement_turbo(sim_engine(desc), 1.55);
    clear_full_load_envelope_memo();
    const auto t0 = Clock::now();
    if (prebuilt) {
        PreparedTurboEnvelope p = w.prepare_turbo_envelope(rig.vehicle, id, t);
        std::thread th([&] { build_turbo_envelope(p); }); // the caller would keep stepping here; the swap is between steps
        th.join();
        w.replace_turbo_configuration(rig.vehicle, id, p);
    } else {
        w.replace_turbo_configuration(rig.vehicle, id, t);
    }
    if (caller_ms) *caller_ms = ms_since(t0);
    namespace da = jobs::debug_alloc;
    const bool use_alloc = da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    SwapRun out;
    // Part pedal after the swap: the envelope is what maps the pedal to a torque request below full load (pedal plan P1),
    // so a wrong or missing envelope shows in the hash; at full pedal it would not.
    w.set_control("throttle", 0.5);
    for (int k = 0; k < 480; ++k) {
        w.step();
        out.hashes.push_back(w.state_hash());
    }
    if (use_alloc) {
        da::set_armed(false);
        out.own_alloc = da::own_violation_count();
        da::reset();
    }
    return out;
}

} // namespace

TEST_CASE("Q5 W1: World swap with a prebuilt envelope is tick-for-tick identical to the synchronous swap, 1 vs 10 workers, no job allocation",
          "[turbo][envelope][drivetrain][q5][determinism][long]") {
    const vehicle::VehicleDesc desc = load_hyper();
    SwapRun runs[2][2]; // [prebuilt][workers 1/10]
    double caller_ms[2] = {0, 0};
    for (int pre = 0; pre < 2; ++pre)
        for (int wi = 0; wi < 2; ++wi) runs[pre][wi] = world_swap(desc, wi == 0 ? 1 : 10, pre != 0, wi == 0 ? &caller_ms[pre] : nullptr);
    std::printf("[Q5 W1] World swap on the caller thread (memo miss, prebuilt includes the join): synchronous %.1f ms, prebuilt "
                "prepare+join+replace %.1f ms; final hash %016llx; own allocations after the swap %llu/%llu/%llu/%llu\n",
                caller_ms[0], caller_ms[1], static_cast<unsigned long long>(runs[0][0].hashes.back()),
                static_cast<unsigned long long>(runs[0][0].own_alloc), static_cast<unsigned long long>(runs[0][1].own_alloc),
                static_cast<unsigned long long>(runs[1][0].own_alloc), static_cast<unsigned long long>(runs[1][1].own_alloc));
    for (int pre = 0; pre < 2; ++pre)
        for (int wi = 0; wi < 2; ++wi) {
            CHECK(runs[pre][wi].hashes == runs[0][0].hashes);
            if (jobs::debug_alloc::hook_available()) CHECK(runs[pre][wi].own_alloc == 0);
        }
}
