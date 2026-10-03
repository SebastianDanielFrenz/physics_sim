// tests/unit/test_pipe_network_simd.cpp - ps_audio A9: the AVX2 vectorised pipe
// kernel (PipeNetwork::step_pipe_simd, selected with PipeKernel::Simd) must be
// BIT-IDENTICAL to the scalar reference (PipeKernel::Reference). Two identical
// networks, one per kernel, are driven identically; after every step the
// full-state FNV fold must agree, and so must the totals, the exchange
// accounting and every cell's derived state. A second scenario covers the
// runtime CFL sub-cycling path (A8b), which the reference re-runs at h / n_sub.
//
// On a non-AVX2 target the "Simd" path is the bit-identical scalar f64x4
// fallback, so the same test still runs and must pass everywhere.

#include "cycle_pipe_fixtures.h"

#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/pipe_network.h"
#include "ps/math/simd.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHFull = 1.0 / 46080.0;

PipeDesc make_pipe(const char* name, double length, double d0, double d1, double dx, PipeEndDesc start,
                   PipeEndDesc end, double c_max = 900.0) {
    PipeDesc p;
    p.name = name;
    p.length_m = length;
    p.d_start_m = d0;
    p.d_end_m = d1;
    p.dx_target_m = dx;
    p.max_wave_speed_m_s = c_max;
    p.start = start;
    p.end = end;
    return p;
}

PipeEndDesc closed_end() { return PipeEndDesc{}; }
PipeEndDesc open_end() { return PipeEndDesc{PipeEndDesc::Kind::Open, -1, EndCorrection::Unflanged}; }
PipeEndDesc volume_end(int v, EndCorrection c = EndCorrection::None) {
    return PipeEndDesc{PipeEndDesc::Kind::Volume, v, c};
}
PipeEndDesc port_end(int p) { return PipeEndDesc{PipeEndDesc::Kind::Port, p, EndCorrection::None}; }

// I4-like network (mirrors test_pipe_network.cpp's fixture): plenum -> 4 tapered
// runners -> 4 intake ports; 4 exhaust ports -> 4 primaries (friction + heat) ->
// collector volume -> collector pipe -> open. Every end kind and both wall
// models appear, so all of step_pipe is exercised.
PipeNetworkDesc i4_like_network() {
    PipeNetworkDesc d;
    d.volumes.push_back(VolumeDesc{"plenum", 2.5e-3, GasStateDesc{60.0e3, 300.0, 0.0}});
    d.volumes.push_back(VolumeDesc{"airbox", 10.0e-3, GasStateDesc{101325.0, 293.15, 0.0}});
    d.volumes.push_back(VolumeDesc{"collector", 5.0e-4, GasStateDesc{110.0e3, 800.0, 1.0}});
    d.orifices.push_back(OrificeDesc{"throttle", 0, 1, 2.0e-4, 500.0});
    d.orifices.push_back(OrificeDesc{"airbox_inlet", 1, -1, 3.0e-3, 500.0});
    for (int k = 0; k < 8; ++k) d.ports.push_back(PortDesc{"port" + std::to_string(k), k < 4 ? 1.5e-4 : 2.5e-4});
    for (int k = 0; k < 4; ++k) {
        PipeDesc r = make_pipe("runner", 0.35, 0.045, 0.040, 0.017, volume_end(0, EndCorrection::Flanged), port_end(k), 500.0);
        r.initial = GasStateDesc{60.0e3, 300.0, 0.0};
        d.pipes.push_back(r);
    }
    for (int k = 0; k < 4; ++k) {
        PipeDesc e = make_pipe("primary", 0.6, 0.036, 0.036, 0.032, port_end(4 + k), volume_end(2), 900.0);
        e.initial = GasStateDesc{110.0e3, 800.0, 1.0};
        e.wall.heat_transfer = true;
        e.wall.wall_temperature_k = 500.0;
        d.pipes.push_back(e);
    }
    PipeDesc c = make_pipe("collector", 0.3, 0.05, 0.05, 0.032, volume_end(2), open_end(), 900.0);
    c.initial = GasStateDesc{105.0e3, 750.0, 1.0};
    d.pipes.push_back(c);
    return d;
}

void drive_ports(PipeNetwork& net, std::uint64_t step) {
    const double t = static_cast<double>(step) * net.h();
    for (int k = 0; k < 4; ++k) {
        const double ph = 2.0 * kPi * (50.0 * t + 0.25 * k);
        net.set_port_state(k, PortState{60.0e3 + 20.0e3 * std::sin(ph), 310.0, 0.02});
        net.set_port_state(4 + k, PortState{130.0e3 + 120.0e3 * std::max(0.0, std::sin(ph)), 1000.0, 1.0});
    }
}

constexpr std::uint64_t kSeed = 1469598103934665603ull;

// Runs ref and sim step for step, driving ports identically, and returns the
// first step index whose full-state fold differs (-1 if none).
template <typename Drive>
int first_divergence(PipeNetwork& ref, PipeNetwork& sim, int steps, Drive&& drive, int* bad_pipe, int* bad_cell,
                     std::uint64_t* fb_ref = nullptr, std::uint64_t* fb_sim = nullptr) {
    if (fb_ref) *fb_ref = 0;
    if (fb_sim) *fb_sim = 0;
    for (int k = 0; k < steps; ++k) {
        drive(ref, static_cast<std::uint64_t>(k));
        drive(sim, static_cast<std::uint64_t>(k));
        ref.step();
        sim.step();
        if (fb_ref) *fb_ref += static_cast<std::uint64_t>(ref.last_step_fallback_count());
        if (fb_sim) *fb_sim += static_cast<std::uint64_t>(sim.last_step_fallback_count());
        if (ref.fold_state_hash(kSeed) != sim.fold_state_hash(kSeed)) return k;
    }
    *bad_pipe = -1;
    *bad_cell = -1;
    for (int p = 0; p < ref.pipe_count(); ++p) {
        for (int i = 0; i < ref.cell_count(p); ++i) {
            const PipeCellState a = ref.cell(p, i), b = sim.cell(p, i);
            if (a.rho != b.rho || a.u != b.u || a.p != b.p || a.t != b.t || a.x_b != b.x_b || a.a != b.a) {
                *bad_pipe = p;
                *bad_cell = i;
                return steps;
            }
        }
    }
    return -1;
}

void check_exchange_equal(const PipeNetwork& ref, const PipeNetwork& sim) {
    const PipeNetworkExchange a = ref.exchange(), b = sim.exchange();
    CHECK(a.ambient_mass_kg == b.ambient_mass_kg);
    CHECK(a.ambient_energy_j == b.ambient_energy_j);
    CHECK(a.ambient_burned_kg == b.ambient_burned_kg);
    CHECK(a.source_mass_kg == b.source_mass_kg);
    CHECK(a.source_energy_j == b.source_energy_j);
    CHECK(a.source_burned_kg == b.source_burned_kg);
    CHECK(a.port_mass_kg == b.port_mass_kg);
    CHECK(a.port_energy_j == b.port_energy_j);
    CHECK(a.port_burned_kg == b.port_burned_kg);
    CHECK(a.waveguide_mass_kg == b.waveguide_mass_kg);
    CHECK(a.waveguide_energy_j == b.waveguide_energy_j);
    CHECK(a.waveguide_burned_kg == b.waveguide_burned_kg);
}

// A single closed -> open pipe whose cell count is EXACTLY n_cells: the closed
// start carries no end correction and the Open end an Unflanged one
// (0.6133 * d/2), so l_eff = (n + 0.5) * dx_target yields
// floor(l_eff / dx_target) == n. The caller asserts the actual count, so a case
// can never silently test a different size.
PipeNetworkDesc single_pipe_network(int n_cells) {
    const double d = 0.05;
    const double dx_target = 0.03;
    const double ext = kUnflangedEndCorrection * 0.5 * d;
    const double l_eff = (static_cast<double>(n_cells) + 0.5) * dx_target;
    PipeDesc p = make_pipe("single", l_eff - ext, d, d, dx_target, closed_end(), open_end(), 900.0);
    p.wall.friction = WallFriction::LaminarBlasius;
    p.wall.heat_transfer = true;
    p.wall.wall_temperature_k = 450.0;
    p.initial = GasStateDesc{101325.0, 300.0, 0.0};
    PipeNetworkDesc nd;
    nd.ambient = GasStateDesc{101325.0, 293.15, 0.0};
    nd.pipes.push_back(p);
    return nd;
}

OperatingPoint make_op(const drivetrain::CycleModelDesc& d, real rpm, real p_im) {
    OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p_im;
    op.t_im_k = 288.15;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = test::mbt_spark_deg_atdc(d);
    return op;
}

std::uint64_t result_hash(const CycleEngineResult& r) {
    std::uint64_t h = kSeed;
    auto fold = [&](real v) {
        std::uint64_t b = 0;
        std::memcpy(&b, &v, sizeof(b));
        for (int k = 0; k < 8; ++k) {
            h ^= (b >> (8 * k)) & 0xffu;
            h *= 1099511628211ull;
        }
    };
    const PointResult& m = r.mean;
    for (real v : {m.w_gross_j, m.w_pump_j, m.m_air_j_cyc, m.m_fuel_kg, m.residual_frac, m.t_exh_k, m.h_exh_j,
                   m.p_max_pa, r.p_plenum_mean_pa, r.throttle_cd_area_m2, r.mass_closure_kg})
        fold(v);
    return h;
}

} // namespace

TEST_CASE("A9 SIMD pipe kernel matches Reference bit for bit over an I4-like run",
          "[drivetrain][cycle][pipe][simd]") {
    // Catches: any re-association, reciprocal, min/max/abs primitive, FMA
    // contraction or branch-to-select slip in the vectorised kernel - all of
    // which would change a cell's bits and break the per-step fold.
    PipeNetwork ref(i4_like_network(), kHFull);
    ref.set_kernel(PipeKernel::Reference);
    PipeNetwork sim(i4_like_network(), kHFull);
    sim.set_kernel(PipeKernel::Simd);
    CHECK(ref.kernel() == PipeKernel::Reference);
    CHECK(sim.kernel() == PipeKernel::Simd);

    // The network's default kernel must be Simd exactly when an AVX2 backend is
    // compiled in.
    PipeNetwork def(i4_like_network(), kHFull);
    if (ps::simd::has_avx2_backend()) {
        CHECK(def.kernel() == PipeKernel::Simd);
    } else {
        CHECK(def.kernel() == PipeKernel::Reference);
    }

    int bad_pipe = -1, bad_cell = -1;
    const int diverge = first_divergence(ref, sim, 4000, [](PipeNetwork& n, std::uint64_t k) { drive_ports(n, k); },
                                         &bad_pipe, &bad_cell);
    std::printf("A9 I4-like: 4000 steps, first hash divergence %d, first bad cell pipe %d cell %d, avx2 %d\n", diverge,
                bad_pipe, bad_cell, ps::simd::has_avx2_backend() ? 1 : 0);
    CHECK(diverge < 0);
    CHECK(bad_pipe < 0);

    CHECK(ref.total_mass() == sim.total_mass());
    CHECK(ref.total_energy() == sim.total_energy());
    CHECK(ref.total_burned_mass() == sim.total_burned_mass());
    CHECK(ref.subcycled_pipe_steps() == sim.subcycled_pipe_steps());
    check_exchange_equal(ref, sim);
    CHECK(std::isfinite(sim.total_energy()));
}

TEST_CASE("A9 SIMD pipe kernel matches Reference through the runtime CFL sub-cycling path",
          "[drivetrain][cycle][pipe][simd]") {
    // ps_audio A8b: a hot supersonic backflow jet into a near-vacuum runner
    // makes step_pipe return n_sub > 1, so the caller re-runs the pipe n_sub
    // times at h / n_sub and averages its end faces. The vectorised kernel must
    // take the identical path and return the identical sub-cycle count.
    auto build = [] {
        PipeDesc p = make_pipe("runner", 0.35, 0.040, 0.040, 0.017, closed_end(), port_end(0), 500.0);
        p.initial = GasStateDesc{6000.0, 300.0, 0.0};
        PipeNetworkDesc d;
        d.pipes.push_back(p);
        d.ports.push_back(PortDesc{"port", 150e-6});
        return d;
    };
    PipeNetwork ref(build(), 1.0 / 48000.0);
    ref.set_kernel(PipeKernel::Reference);
    PipeNetwork sim(build(), 1.0 / 48000.0);
    sim.set_kernel(PipeKernel::Simd);

    int bad_pipe = -1, bad_cell = -1;
    std::uint64_t fb_ref = 0, fb_sim = 0;
    const int diverge = first_divergence(
        ref, sim, 2400,
        [](PipeNetwork& n, std::uint64_t) { n.set_port_state(0, PortState{100000.0, 1000.0, 1.0}); }, &bad_pipe,
        &bad_cell, &fb_ref, &fb_sim);
    std::printf("A9 CFL jet: 2400 steps, first divergence %d, sub-cycled pipe-steps ref %llu sim %llu, "
                "hancock fallbacks ref %llu sim %llu, avx2 %d\n",
                diverge, static_cast<unsigned long long>(ref.subcycled_pipe_steps()),
                static_cast<unsigned long long>(sim.subcycled_pipe_steps()), static_cast<unsigned long long>(fb_ref),
                static_cast<unsigned long long>(fb_sim), ps::simd::has_avx2_backend() ? 1 : 0);
    CHECK(diverge < 0);
    CHECK(bad_pipe < 0);
    CHECK(ref.subcycled_pipe_steps() > 0);  // scenario sanity: sub-cycling really happened
    CHECK(ref.subcycled_pipe_steps() == sim.subcycled_pipe_steps());
    CHECK(ref.total_energy() == sim.total_energy());
    check_exchange_equal(ref, sim);
}

TEST_CASE("A9 SIMD pipe kernel matches Reference on the Hancock fallback path", "[drivetrain][cycle][pipe][simd]") {
    // Catches: a wrong operand order or a dropped mask in the ok / select
    // fallback of the vectorised Hancock half step (l_r_ = select(ok, lr, wr),
    // etc.), which only differs from the reference in the cells where the
    // predictor state is non-positive and is invisible to the other cases.
    //
    // The trigger is the area source term (0.5 * h * u * dA / v): unlike the
    // convective slopes it is not bounded by the CFL check, which sees only
    // |u| + a. A needle-to-wide diverging duct (d 0.5 mm -> 60 mm) combined with
    // a strong density jump and velocity rise therefore pushes the Hancock
    // density and pressure of an interior cell non-positive. n = 16 puts cells
    // 1..12 in the 4-wide vector loop (i + 4 <= n - 1), so the ok-mask path is
    // exercised, not the scalar tail.
    const int n = 16;
    const double dx_target = 0.005;
    const double h = 7.5e-6;
    auto build = [&] {
        PipeDesc p = make_pipe("fallback", (n + 0.5) * dx_target, 5.0e-4, 0.060, dx_target, closed_end(),
                               closed_end(), 500.0);
        p.initial = GasStateDesc{21.9, 76.2, 0.0};
        PipeNetworkDesc d;
        d.pipes.push_back(p);
        return d;
    };
    PipeNetwork ref(build(), h);
    ref.set_kernel(PipeKernel::Reference);
    PipeNetwork sim(build(), h);
    sim.set_kernel(PipeKernel::Simd);
    REQUIRE(ref.cell_count(0) == n);
    REQUIRE(sim.cell_count(0) == n);
    REQUIRE(ref.total_cell_count() == n);

    // T = 76.2 K -> a ~ 175 m/s. At cell 1 a light left cell gives dr = rho, the
    // velocity rises by 100 m/s (du = 0.2 s_max) and u = 225 (0.45 s_max) with
    // s_max = 500 = |u| + a of the fast block, so the area source (ratio ~ 1.2)
    // drives lp (and the density to a smaller degree) non-positive. u - a < 0 at
    // the cell-0 / cell-1 face, so HLLC is subsonic there and does read l_r_[1]:
    // the select's result actually reaches the flux.
    const double t_k = 76.2;
    const double p_of_rho = 287.0 * t_k;
    auto set_state = [&](PipeNetwork& net) {
        for (int i = 0; i < n; ++i) {
            const double rho = (i == 0) ? 0.001 : (i == 1) ? 1.0 : 2.5;
            const double u = (i == 0) ? 125.0 : (i == 1) ? 225.0 : 325.0;
            net.set_cell(0, i, p_of_rho * rho, t_k, u, 0.0);
        }
    };
    set_state(ref);
    set_state(sim);

    const int steps = 2000;
    std::uint64_t fb_ref = 0, fb_sim = 0;
    int first_fb = -1, diverge = -1;
    for (int k = 0; k < steps; ++k) {
        ref.step();
        sim.step();
        const std::uint64_t fr = static_cast<std::uint64_t>(ref.last_step_fallback_count());
        fb_ref += fr;
        fb_sim += static_cast<std::uint64_t>(sim.last_step_fallback_count());
        if (first_fb < 0 && fr > 0) first_fb = k;
        if (ref.fold_state_hash(kSeed) != sim.fold_state_hash(kSeed)) {
            diverge = k;
            break;
        }
    }
    std::printf("A9 fallback: n=%d, %d steps, first fallback step %d, hancock fallbacks ref %llu sim %llu, "
                "first divergence %d, avx2 %d\n",
                n, steps, first_fb, static_cast<unsigned long long>(fb_ref),
                static_cast<unsigned long long>(fb_sim), diverge, ps::simd::has_avx2_backend() ? 1 : 0);

    REQUIRE(first_fb >= 0);   // scenario sanity: the fallback path really ran
    REQUIRE(fb_ref > 0);
    REQUIRE(fb_ref == fb_sim);  // both kernels take it on exactly the same cells
    CHECK(diverge < 0);
    CHECK(ref.total_mass() == sim.total_mass());
    CHECK(ref.total_energy() == sim.total_energy());
    for (int i = 0; i < n; ++i) {
        const PipeCellState a = ref.cell(0, i), b = sim.cell(0, i);
        CHECK(a.rho == b.rho);
        CHECK(a.u == b.u);
        CHECK(a.p == b.p);
        CHECK(a.t == b.t);
        CHECK(a.x_b == b.x_b);
        CHECK(a.a == b.a);
    }
}

TEST_CASE("A9 SIMD pipe kernel matches Reference on every cell-count remainder",
          "[drivetrain][cycle][pipe][simd]") {
    // Catches: an off-by-one in a vector loop guard (i + 4 <= n, i + 4 <= n - 1)
    // or its scalar tail, for a pipe whose cell count is not a multiple of four.
    // The counts cover: all-scalar (n < 4), one vector and no tail (n == 4), one
    // vector plus a 1..3 cell tail, and two vectors plus a tail (n == 9).
    const int counts[] = {1, 2, 3, 4, 5, 7, 8, 9};
    for (const int n : counts) {
        CAPTURE(n);
        PipeNetwork ref(single_pipe_network(n), kHFull);
        ref.set_kernel(PipeKernel::Reference);
        PipeNetwork sim(single_pipe_network(n), kHFull);
        sim.set_kernel(PipeKernel::Simd);
        REQUIRE(ref.cell_count(0) == n);  // the case really is this size
        REQUIRE(sim.cell_count(0) == n);
        REQUIRE(ref.total_cell_count() == n);

        // Non-uniform start: a pressure / temperature / velocity ramp plus a
        // Sod-like jump and a burned-fraction step at the middle, so minmod
        // slopes and the Hancock predictor run - not just the constant path.
        for (int i = 0; i < n; ++i) {
            const double s = (n <= 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
            const double p = 2.5e5 - 2.0e5 * s - (i >= n / 2 ? 3.0e4 : 0.0);
            const double t = 400.0 - 80.0 * s;
            const double u = -5.0 + 10.0 * s;
            const double xb = (i < n / 2) ? 0.0 : 0.8;
            ref.set_cell(0, i, p, t, u, xb);
            sim.set_cell(0, i, p, t, u, xb);
        }

        int diverge = -1;
        std::uint64_t fb_ref = 0, fb_sim = 0;
        for (int k = 0; k < 500; ++k) {
            ref.step();
            sim.step();
            fb_ref += static_cast<std::uint64_t>(ref.last_step_fallback_count());
            fb_sim += static_cast<std::uint64_t>(sim.last_step_fallback_count());
            if (ref.fold_state_hash(kSeed) != sim.fold_state_hash(kSeed)) {
                diverge = k;
                break;
            }
        }
        std::printf("A9 remainder n=%d: 500 steps, first divergence %d, hancock fallbacks ref %llu sim %llu\n", n,
                    diverge, static_cast<unsigned long long>(fb_ref), static_cast<unsigned long long>(fb_sim));
        CHECK(diverge < 0);
        CHECK(fb_ref == fb_sim);
        CHECK(ref.total_mass() == sim.total_mass());
        CHECK(ref.total_energy() == sim.total_energy());
        CHECK(ref.total_burned_mass() == sim.total_burned_mass());
        CHECK(ref.subcycled_pipe_steps() == sim.subcycled_pipe_steps());
        for (int i = 0; i < n; ++i) {
            const PipeCellState a = ref.cell(0, i), b = sim.cell(0, i);
            CAPTURE(i);
            CHECK(a.rho == b.rho);
            CHECK(a.u == b.u);
            CHECK(a.p == b.p);
            CHECK(a.t == b.t);
            CHECK(a.x_b == b.x_b);
            CHECK(a.a == b.a);
        }
    }
}

TEST_CASE("A9 SIMD pipe kernel matches Reference through the CycleEngine integration path",
          "[drivetrain][cycle][pipe][simd]") {
    // Catches: a kernel difference the standalone PipeNetwork scenarios do not
    // reach - the engine hand-off (ports <-> network), the per-step phased
    // cylinder coupling, the settling loop, the throttle trim and the result
    // assembly. Fixture: tests/support/cycle_pipe_fixtures.h (a GENERIC [AS] I4).
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    const OperatingPoint op = make_op(d, 3000.0, 60.0e3);

    // 1. Per step. CycleEngine::run() is monolithic (no public per-step hook),
    // so drive the shared EngineCore exactly as CycleEngine::step_all does -
    // exchange, then every cylinder's phased step - and fold after every step.
    const std::uint64_t n = steps_per_cycle(op.rpm, kStepPreview);
    const real h = 120.0 / (op.rpm * static_cast<real>(n));
    EngineCoreSettings cs;
    cs.dx_intake_m = d.resolution.power_dx_m;
    cs.dx_exhaust_m = d.resolution.power_dx_m;
    EngineCore core_ref(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, h, cs);
    EngineCore core_sim(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, h, cs);
    core_ref.set_pipe_kernel(PipeKernel::Reference);
    core_sim.set_pipe_kernel(PipeKernel::Simd);
    CHECK(core_ref.fold_state_hash(kSeed) == core_sim.fold_state_hash(kSeed));
    int diverge = -1;
    for (std::uint64_t k = 0; k < n; ++k) {
        core_ref.exchange();
        core_sim.exchange();
        for (int c = 0; c < core_ref.cylinders(); ++c) {
            core_ref.cylinder(c).step_phased(k, n, core_ref.phase_deg(c));
            core_sim.cylinder(c).step_phased(k, n, core_sim.phase_deg(c));
        }
        if (core_ref.fold_state_hash(kSeed) != core_sim.fold_state_hash(kSeed)) {
            diverge = static_cast<int>(k);
            break;
        }
    }
    std::printf("A9 engine per-step: %llu steps, first divergence %d\n", static_cast<unsigned long long>(n), diverge);
    CHECK(diverge < 0);

    // 2. End to end (Preview, short): two CycleEngines, one per kernel; the
    // start hash, the result and the final fold must all agree bit for bit.
    CycleEngineSettings st;
    st.h_s = kStepPreview;
    st.settling_cycles = kEngineSettlingCyclesPreview;
    CycleEngine eng_ref(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, st);
    CycleEngine eng_sim(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, st);
    eng_ref.set_pipe_kernel(PipeKernel::Reference);
    eng_sim.set_pipe_kernel(PipeKernel::Simd);
    CHECK(eng_ref.fold_state_hash(kSeed) == eng_sim.fold_state_hash(kSeed));
    const CycleEngineResult r_ref = eng_ref.run();
    const CycleEngineResult r_sim = eng_sim.run();
    CHECK(result_hash(r_ref) == result_hash(r_sim));
    CHECK(eng_ref.fold_state_hash(kSeed) == eng_sim.fold_state_hash(kSeed));
    CHECK(r_ref.mean.m_air_j_cyc == r_sim.mean.m_air_j_cyc);
    CHECK(r_ref.p_plenum_mean_pa == r_sim.p_plenum_mean_pa);

    const PipeNetwork* na = eng_ref.network();
    const PipeNetwork* nb = eng_sim.network();
    REQUIRE(na != nullptr);
    REQUIRE(nb != nullptr);
    CHECK(na->pipe_count() == nb->pipe_count());
    int bad_pipe = -1, bad_cell = -1;
    for (int p = 0; p < na->pipe_count() && bad_pipe < 0; ++p) {
        CHECK(na->cell_count(p) == nb->cell_count(p));
        for (int i = 0; i < na->cell_count(p); ++i) {
            const PipeCellState a = na->cell(p, i), b = nb->cell(p, i);
            if (a.rho != b.rho || a.u != b.u || a.p != b.p || a.t != b.t || a.x_b != b.x_b || a.a != b.a) {
                bad_pipe = p;
                bad_cell = i;
                break;
            }
        }
    }
    std::printf("A9 engine run: cells first mismatch pipe %d cell %d\n", bad_pipe, bad_cell);
    CHECK(bad_pipe < 0);
}
