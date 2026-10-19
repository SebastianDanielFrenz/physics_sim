// ps_audio A9b: the live engine voice's step worker team must not change the
// result. Every physics step (the PipeNetwork pipe loop and each cylinder's
// CycleStepper::step_live) is spread over a small fixed thread team, but the
// per-pipe / per-cylinder arithmetic and the order of the few shared
// floating-point accumulators are unchanged, so 1, 2, 3 and 4 threads must give
// bit-identical state hashes and PCM. T1 (the whole LiveEngine through a
// transient), T2 (a bare PipeNetwork with a waveguide, volumes, ports and a
// pipe count the thread count does not divide) and T3 (zero heap allocation in
// the threaded step). out/a9b_report.md.
#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"

#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/pipe_network.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps_audio/engine_input.h"
#include "ps_audio/live_engine.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

constexpr real kTwoPi = 6.283185307179586476925286766559;
constexpr real kFs = 48000.0;

real omega_of(real rpm) { return rpm * kTwoPi / 60.0; }

const drivetrain::CycleModelDesc& fixture() {
    static const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    return d;
}

// Coarse cells (5 cm) - these tests' subject is the thread split, not the grid.
ps_audio::LiveEngineSettings live_settings(int threads) {
    ps_audio::LiveEngineSettings s;
    s.substeps = 1;
    s.dx_intake_m = 0.05;
    s.dx_exhaust_m = 0.05;
    s.initial_rpm = 1500.0;
    s.initial_p_intake_pa = 50000.0;
    s.seed = 12345;
    s.variation_scale = 1.0;
    s.threads = threads;
    return s;
}

struct LiveRun {
    std::uint64_t state = 0;
    std::uint64_t subcycled = 0;
    std::vector<float> pcm;
};

// One transient: a 1500 -> 4500 rpm / load ramp, an ignition-cut window, a
// fuel-cut window. Per-step-folded state hash and the rendered PCM.
LiveRun run_live(int threads, int samples) {
    ps_audio::LiveEngine eng(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, live_settings(threads));
    const int ns = eng.source_count();
    std::array<real, ps_audio::kLiveMaxSources> out{};
    LiveRun r;
    r.pcm.reserve(static_cast<std::size_t>(samples) * static_cast<std::size_t>(ns));
    std::uint64_t state = 1469598103934665603ull;
    for (int i = 0; i < samples; ++i) {
        const real t = static_cast<real>(i) / kFs;
        const real f = t < 0.25 ? t / 0.25 : 1.0;
        ps_audio::LiveControl c;
        c.omega_rad_s = omega_of(1500.0 + 3000.0 * f);
        c.p_intake_pa = 50000.0 + 40000.0 * f;
        if (t >= 0.10 && t < 0.12) c.flags |= ps_audio::kInputIgnitionCut;
        if (t >= 0.16 && t < 0.20) c.flags |= ps_audio::kInputFuelCut;
        eng.step_sample(c, out.data());
        for (int s = 0; s < ns; ++s) r.pcm.push_back(static_cast<float>(out[static_cast<std::size_t>(s)]));
        state = eng.fold_state_hash(state);
    }
    r.state = state;
    r.subcycled = eng.core().network() != nullptr ? eng.core().network()->subcycled_pipe_steps() : 0;
    return r;
}

bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

// Pulsing port states at a rising-saw phase (crank-angle-like), the same idea
// as test_pipe_network.cpp's drive_ports.
void drive_ports(PipeNetwork& net, const EngineNetworkLayout& lay, std::uint64_t step) {
    const real t = static_cast<real>(step) * net.h();
    for (int c = 0; c < 4; ++c) {
        const real ph = kTwoPi * (50.0 * t + 0.25 * static_cast<real>(c));
        net.set_port_state(lay.intake_port0 + c, PortState{60.0e3 + 20.0e3 * std::sin(ph), 310.0, 0.02});
        net.set_port_state(lay.exhaust_port0 + c,
                           PortState{130.0e3 + 120.0e3 * std::max(0.0, std::sin(ph)), 1000.0, 1.0});
    }
}

}  // namespace

TEST_CASE("A9b T1: the LiveEngine is bit-identical for 1..4 step threads", "[audio][cycle][pipe]") {
    // Catches: any thread-count-dependent arithmetic, a race between the pipe
    // loop's worker teams and the cylinder loop, a lost/duplicated exhaust or
    // waveguide increment (exch_ folded in a different order).
    const int samples = static_cast<int>(0.25 * kFs);
    const LiveRun a = run_live(1, samples);
    REQUIRE(a.pcm.size() > 0);
    for (int th = 2; th <= 4; ++th) {
        const LiveRun b = run_live(th, samples);
        CHECK(b.state == a.state);
        CHECK(same_bits(b.pcm, a.pcm));
    }
    std::printf("A9b T1: state %016llx, %d samples, subcycled %llu\n",
                static_cast<unsigned long long>(a.state), samples,
                static_cast<unsigned long long>(a.subcycled));
}

TEST_CASE("A9b T2: a pipe network with a waveguide is bit-identical for 1..4 threads", "[audio][cycle][pipe]") {
    // Catches: a lost or double-counted pipe end face (the deferred exchange
    // folded out of pipe order), a race on port.flux / the waveguide faces, or
    // a bad static partition when the thread count does not divide the pipes.
    const GasStateDesc amb{101325.0, 293.15, 0.0};
    EngineNetworkLayout lay;
    PipeNetworkDesc nd = make_engine_pipe_network_desc(fixture(), amb, 50000.0, 0.03, 0.03, &lay, true);
    // Add a high-pressure volume discharging through a short open pipe whose
    // design speed is deliberately low (150 m/s): its actual speed of sound
    // (~400 m/s) forces A8b sub-cycling on every step, so the parallel
    // sub-cycle loop is exercised too. It also makes the count prime (13), so
    // 2, 3 and 4 all leave a remainder to place.
    const int vhi = static_cast<int>(nd.volumes.size());
    nd.volumes.push_back(VolumeDesc{"blast", 1.0e-3, GasStateDesc{400000.0, 400.0, 0.0}});
    PipeDesc dis;
    dis.name = "discharge";
    dis.length_m = 0.12;
    dis.d_start_m = 0.03;
    dis.d_end_m = 0.03;
    dis.dx_target_m = 0.006;
    dis.max_wave_speed_m_s = 150.0;
    dis.start.kind = PipeEndDesc::Kind::Volume;
    dis.start.index = vhi;
    dis.end.kind = PipeEndDesc::Kind::Open;
    dis.initial = GasStateDesc{400000.0, 400.0, 0.0};
    nd.pipes.push_back(dis);
    const real h = 1.0 / kFs;
    const int steps = 3000;
    auto run = [&](int threads, std::uint64_t* subcycled) {
        PipeNetwork net(nd, h);
        net.set_step_threads(threads);
        REQUIRE(net.step_threads() == threads);
        for (int k = 0; k < steps; ++k) {
            drive_ports(net, lay, static_cast<std::uint64_t>(k));
            net.step();
        }
        if (subcycled != nullptr) *subcycled = net.subcycled_pipe_steps();
        return net.fold_state_hash(1469598103934665603ull);
    };
    std::uint64_t subcycled = 0;
    const std::uint64_t a = run(1, &subcycled);
    const int pipes = [&] {
        PipeNetwork net(nd, h);
        return net.pipe_count();
    }();
    REQUIRE(pipes % 2 != 0);
    REQUIRE(pipes % 3 != 0);
    REQUIRE(pipes % 4 != 0);
    for (int th = 2; th <= 4; ++th) {
        CHECK(run(th, nullptr) == a);
    }
    std::printf("A9b T2: hash %016llx, %d pipes, %d steps, subcycled %llu\n",
                static_cast<unsigned long long>(a), pipes, steps,
                static_cast<unsigned long long>(subcycled));
}

TEST_CASE("A9b T3: the threaded pipe step allocates nothing", "[audio][cycle][pipe]") {
    // Catches: a per-step scratch allocation (the parallel scratch must be
    // sized once in set_step_threads, not per step).
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    const GasStateDesc amb{101325.0, 293.15, 0.0};
    EngineNetworkLayout lay;
    const PipeNetworkDesc nd = make_engine_pipe_network_desc(fixture(), amb, 50000.0, 0.05, 0.05, &lay, true);
    PipeNetwork net(nd, 1.0 / kFs);
    net.set_step_threads(3);
    ps::jobs::JobSystem js(2);
    auto run = [&](int n) {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit(
            [&] {
                for (int k = 0; k < n; ++k) {
                    drive_ports(net, lay, static_cast<std::uint64_t>(k));
                    net.step();
                }
            },
            &wg);
        js.wait(wg);
    };
    run(10);  // warm-up
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    run(1000);
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(v == 0);
    CHECK(std::isfinite(net.total_energy()));
}
