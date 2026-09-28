// tests/unit/test_cycle_engine.cpp - ps_audio A5: the pipe-aware cycle stepper
// and the multi-cylinder pipe-coupled CycleEngine (core/src/engine_cycle).
// Fixture: tests/support/cycle_pipe_fixtures.h (a GENERIC [AS] pipe I4 - not
// a shipped engine). Every guard names the defect it catches; the sabotage
// each was shown to fail on is recorded in out/audio_a5_progress.md.

#include "cycle_pipe_fixtures.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/geometry.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

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

bool same_bits(const CycleState& a, const CycleState& b) {
    return std::memcmp(a.y, b.y, sizeof(a.y)) == 0;
}

} // namespace

TEST_CASE("A5 S1 pipe-aware stepper: construction rules", "[drivetrain][cycle][pipe]") {
    // Catches: the A4 gap (a pipe engine's zeroed runner rejected by the
    // stepper), and a lumped engine silently accepted without its runner.
    const drivetrain::CycleModelDesc pipe = test::make_pipe_i4_cycle_desc();
    CHECK_NOTHROW(CycleStepper(pipe, test::kCycleFixtureLhv, test::kCycleFixtureAfr));
    CHECK(CycleStepper(pipe, test::kCycleFixtureLhv, test::kCycleFixtureAfr).model().pipe_ports);

    drivetrain::CycleModelDesc lumped_no_runner = test::make_generic_i4_cycle_desc();
    lumped_no_runner.runner_length_m = 0.0;
    CHECK_THROWS_WITH(CycleStepper(lumped_no_runner, test::kCycleFixtureLhv, test::kCycleFixtureAfr),
                      "CycleModelDesc: valve, runner, port, burn and fuel values must be > 0");
    CHECK_FALSE(CycleStepper(test::make_generic_i4_cycle_desc(), test::kCycleFixtureLhv, test::kCycleFixtureAfr)
                    .model()
                    .pipe_ports);

    drivetrain::CycleModelDesc half = pipe;
    half.exhaust.reset();
    CHECK_THROWS_WITH(CycleStepper(half, test::kCycleFixtureLhv, test::kCycleFixtureAfr),
                      "CycleModelDesc: intake and exhaust pipe blocks must be given together");
}

TEST_CASE("A5 S2 step_phased at phase 0 is step_aligned bit for bit", "[drivetrain][cycle][pipe]") {
    // Catches: a phased step whose angle bookkeeping differs from the aligned
    // step (cylinder 1 of a CycleEngine would then not be the single-cylinder
    // model).
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    CycleStepper a(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    const OperatingPoint op = make_op(d, 3000.0, 60.0e3);
    a.reset(op);
    CycleStepper b = a;
    const std::uint64_t n = steps_per_cycle(op.rpm, kStepPreview);
    for (std::uint64_t k = 0; k < 2 * n; ++k) {
        a.step_aligned(k, n);
        b.step_phased(k, n, 0.0);
    }
    CHECK(same_bits(a.state(), b.state()));
    CHECK(a.crank_deg() == b.crank_deg());
}

TEST_CASE("A5 S3 pipe-mode ports: source rates conserve mass and energy", "[drivetrain][cycle][pipe]") {
    // BOOKKEEPING: catches a port source that is integrated with a different
    // magnitude than the accumulator that books it (the CycleEngine's pipe
    // hand-off would then create or destroy gas), or a dropped source term.
    // Small constant sources (the port volumes are tiny; a larger withdrawal
    // would empty the exhaust port while the valve is shut).
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    CycleStepper s(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);
    const OperatingPoint op = make_op(d, 3000.0, 60.0e3);
    s.reset_phased(op, 180.0);
    const GasModel& g = s.model().gas;
    const PortSource ip{2.0e-4, 2.0e-4 * g.fresh.h(290.0), 0.0};
    const PortSource ep{-1.0e-4, -1.0e-4 * g.burned.h(900.0), -1.0e-4};
    s.set_port_sources(ip, ep);
    const std::uint64_t n = steps_per_cycle(op.rpm, kStepFull);
    for (std::uint64_t k = 0; k < 2 * n; ++k) s.step_phased(k, n, 180.0);
    s.reset_accumulators();
    const CycleState st0 = s.state();
    for (std::uint64_t k = 2 * n; k < 3 * n; ++k) s.step_phased(k, n, 180.0);
    const real* y = s.state().y;
    const real* y0 = st0.y;
    const real t_cycle = static_cast<real>(n) * (120.0 / (op.rpm * static_cast<real>(n)));
    const real dm = (y[kCylM] + y[kIpM] + y[kEpM]) - (y0[kCylM] + y0[kIpM] + y0[kEpM]);
    const real du = (y[kCylU] + y[kIpU] + y[kEpU]) - (y0[kCylU] + y0[kIpU] + y0[kEpU]);
    const real rhs_m = y[kAccRunnerMass] - y[kAccResMass] + y[kAccFuel];
    const real rhs_u = -y[kAccWork] + y[kAccRunnerEnthalpy] - y[kAccResEnthalpy] + y[kAccQComb] - y[kAccQWall];
    const real scale_u = std::fabs(y[kAccQComb]) + std::fabs(y[kAccWork]) + std::fabs(y[kAccEvEnthalpy]);
    const real scale_m = std::fabs(y[kAccIvMass]) + std::fabs(y[kAccEvOutMass]);
    std::printf("A5 S3: dm %.6e rhs %.6e (err %.3e), dU %.6e rhs %.6e (err %.3e), air %.4e kg, fuel %.4e kg\n",
                dm, rhs_m, dm - rhs_m, du, rhs_u, du - rhs_u, y[kAccIvFresh], y[kAccFuel]);
    // The accumulators book exactly the prescribed rates.
    CHECK(std::fabs(y[kAccRunnerMass] - ip.mdot * t_cycle) <= 1e-12 * ip.mdot * t_cycle);
    CHECK(std::fabs(y[kAccResMass] + ep.mdot * t_cycle) <= 1e-12 * -ep.mdot * t_cycle);
    CHECK(std::fabs(y[kAccRunnerEnthalpy] - ip.edot * t_cycle) <= 1e-12 * std::fabs(ip.edot) * t_cycle);
    // ... and the states moved by exactly what was booked.
    CHECK(std::fabs(dm - rhs_m) < 1e-9 * scale_m);
    CHECK(std::fabs(du - rhs_u) < 1e-9 * scale_u);
    // Sanity: a firing cycle through the ports.
    CHECK(y[kAccQComb] > 0.0);
    CHECK(y[kAccIvFresh] > 0.0);
    CHECK(y[kIpM] > 0.0);
    CHECK(y[kEpM] > 0.0);
}

// ---------------------------------------------------------------------------
// CycleEngine
// ---------------------------------------------------------------------------

namespace {

std::uint64_t result_hash(const CycleEngineResult& r) {
    std::uint64_t h = 1469598103934665603ull;
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

CycleEngineSettings preview_settings() {
    CycleEngineSettings s;
    s.h_s = kStepPreview;
    s.settling_cycles = kEngineSettlingCyclesPreview;
    return s;
}

} // namespace

TEST_CASE("A5 E1 engine network layout from the pipe blocks and installation", "[drivetrain][cycle][pipe]") {
    // Catches: a mapping that drops or duplicates a pipe, wires a primary to
    // the wrong junction, or builds the installation chain wrongly.
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    const GasStateDesc amb{101325.0, 288.15, 0.0};
    EngineNetworkLayout lay;
    const PipeNetworkDesc nd = make_engine_pipe_network_desc(d, amb, 60.0e3, 0.05, &lay);
    CHECK(nd.pipes.size() == 12u);   // snorkel + 4 runners + 4 primaries + 3 merge pipes
    CHECK(nd.volumes.size() == 5u);  // airbox, plenum, 3 junctions
    CHECK(nd.ports.size() == 8u);
    CHECK(nd.orifices.size() == 1u);
    CHECK(lay.airbox_volume == 0);
    CHECK(lay.plenum_volume == 1);
    CHECK(nd.orifices[0].volume_a == 1);
    CHECK(nd.orifices[0].volume_b == 0);
    // Primaries 1 and 4 end in junction y14 (volume 2), 2 and 3 in y23 (volume 3).
    for (int c = 1; c <= 4; ++c) {
        const PipeDesc& p = nd.pipes[static_cast<std::size_t>(4 + c)];
        CHECK(p.start.kind == PipeEndDesc::Kind::Port);
        CHECK(p.start.index == 4 + c - 1);
        CHECK(p.end.kind == PipeEndDesc::Kind::Volume);
        CHECK(p.end.index == ((c == 1 || c == 4) ? 2 : 3));
    }
    // y14/y23 merge pipes feed the flange junction (volume 4); the last ends at the waveguide.
    CHECK(nd.pipes[9].end.index == 4);
    CHECK(nd.pipes[10].end.index == 4);
    CHECK(nd.pipes[11].end.kind == PipeEndDesc::Kind::Waveguide);
    const real a38 = 0.25 * kPi * 0.038 * 0.038, a45 = 0.25 * kPi * 0.045 * 0.045, a50 = 0.25 * kPi * 0.05 * 0.05;
    CHECK(nd.volumes[2].volume_m3 == kJunctionLengthM * ((a38 + a38) + a45));
    CHECK(nd.volumes[4].volume_m3 == kJunctionLengthM * ((a45 + a45) + a50));
    // pipe, catalyst (2 halves around a Resistance), pipe, chamber, tailpipe -> Open.
    CHECK(nd.waveguide.nodes.size() == 7u);
    CHECK(nd.waveguide.segments.size() == 6u);
    CHECK(nd.waveguide.nodes[2].kind == WgNodeDesc::Kind::Resistance);
    CHECK(nd.waveguide.nodes[2].k_loss == 1.5);
    CHECK(nd.waveguide.nodes.back().kind == WgNodeDesc::Kind::Open);
    CHECK(nd.waveguide.segments.back().end_b == EndCorrection::Unflanged);

    // No installation: throttle to ambient, flange pipe open, no waveguide.
    drivetrain::CycleModelDesc bare = d;
    bare.installation.reset();
    const PipeNetworkDesc nb = make_engine_pipe_network_desc(bare, amb, 60.0e3, 0.05, &lay);
    CHECK(lay.airbox_volume == -1);
    CHECK(nb.orifices[0].volume_b == -1);
    CHECK(nb.pipes.size() == 11u);
    CHECK(nb.waveguide.nodes.empty());
    CHECK(nb.pipes.back().end.kind == PipeEndDesc::Kind::Open);

    // Firing phases of 1-3-4-2: cylinder 1 at 0, 3 at 180, 4 at 360, 2 at 540.
    const std::vector<real> ph = engine_firing_phases_deg(d);
    CHECK(ph == std::vector<real>{0.0, 540.0, 180.0, 360.0});
}

TEST_CASE("A5 E2 CycleEngine closes mass per cycle and holds the plenum", "[drivetrain][cycle][pipe]") {
    // Catches: a hand-off that creates or destroys gas between the pipes and
    // the ports (e.g. sources scaled with a different h than the flux), and a
    // throttle trim that does not converge.
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    struct Pt {
        real rpm, p;
    };
    for (const Pt pt : {Pt{1500.0, 30.0e3}, Pt{3000.0, 60.0e3}, Pt{6000.0, 90.0e3}}) {
        CAPTURE(pt.rpm, pt.p);
        CycleEngine eng(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, make_op(d, pt.rpm, pt.p));
        const CycleEngineResult r = eng.run();
        const real ve =
            r.mean.m_air_j_cyc / (pt.p / (287.05 * 288.15) * 0.25 * kPi * d.bore_m * d.bore_m * d.stroke_m);
        std::printf("A5 E2 %5.0f rpm %3.0f kPa: n %llu h %.4g us, air %.5e kg (VE~%.4f), plenum %.2f kPa "
                    "(err %+.3e), cda %.4e/%.4e, closure %.3e kg of %.3e (rel %.2e), air change %.2e\n",
                    pt.rpm, pt.p / 1000.0, static_cast<unsigned long long>(r.steps_per_cycle), r.h_s * 1e6,
                    r.mean.m_air_j_cyc, ve, r.p_plenum_mean_pa / 1000.0, r.plenum_error_rel, r.throttle_cd_area_m2,
                    r.throttle_cd_area_max_m2, r.mass_closure_kg, r.mass_scale_kg,
                    r.mass_closure_kg / r.mass_scale_kg, r.air_change_rel);
        CHECK(r.h_s <= kStepFull);
        CHECK(std::fabs(r.mass_closure_kg) <= 1e-12 * r.mass_scale_kg);
        CHECK_FALSE(r.throttle_saturated);
        CHECK(std::fabs(r.plenum_error_rel) < 0.02);
        CHECK(r.mean.m_air_j_cyc > 0.0);
        CHECK(r.mean.w_gross_j > 0.0);
    }
}

TEST_CASE("A5 E3 CycleEngine is run-to-run identical", "[drivetrain][cycle][pipe]") {
    // Catches: hidden state (a mutable static, an uninitialised read).
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    const OperatingPoint op = make_op(d, 4500.0, 60.0e3);
    CycleEngine a(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, preview_settings());
    CycleEngine b(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, preview_settings());
    const std::uint64_t h0 = a.fold_state_hash(1469598103934665603ull);
    CHECK(h0 == b.fold_state_hash(1469598103934665603ull));
    const CycleEngineResult ra = a.run();
    const CycleEngineResult rb = b.run();
    CHECK(result_hash(ra) == result_hash(rb));
    CHECK(a.fold_state_hash(1469598103934665603ull) == b.fold_state_hash(1469598103934665603ull));
    CHECK(a.fold_state_hash(1469598103934665603ull) != h0);
    CHECK_THROWS_AS(a.run(), std::logic_error);
}

TEST_CASE("A5 E4 CycleEngine::run allocates nothing inside a job", "[drivetrain][cycle][pipe]") {
    // Catches: a heap allocation in the per-step loop or the result assembly.
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    CycleEngineSettings st = preview_settings();
    st.settling_cycles = 1;
    CycleEngine eng(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, make_op(d, 3000.0, 60.0e3), st);
    ps::jobs::JobSystem js(2);
    CycleEngineResult r;
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit([&] { r = eng.run(); }, &wg);
        js.wait(wg);
    }
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(v == 0);
    CHECK(r.mean.m_air_j_cyc > 0.0);
}

TEST_CASE("A5 E5 reservoir reference mode closes mass", "[drivetrain][cycle][pipe]") {
    // Catches: reservoir rates booked differently from what the ports gain.
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    CycleEngineSettings st = preview_settings();
    st.reservoir_ports = true;
    CycleEngine eng(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, make_op(d, 3000.0, 60.0e3), st);
    CHECK(eng.network() == nullptr);
    const CycleEngineResult r = eng.run();
    std::printf("A5 E5 reservoir: air %.5e kg, closure %.3e kg\n", r.mean.m_air_j_cyc, r.mass_closure_kg);
    CHECK(std::fabs(r.mass_closure_kg) <= 1e-12 * r.mass_scale_kg);
    CHECK(r.mean.m_air_j_cyc > 0.0);
    CHECK(r.p_plenum_mean_pa == r.p_target_pa);
}

TEST_CASE("A5 diag: throttle trim trace per cycle", "[.][diag][cycle][pipe]") {
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    const real pts[][2] = {{1500.0, 30000.0}, {6000.0, 30000.0}, {6000.0, 60000.0}, {3000.0, 90000.0}, {6000.0, 90000.0}};
    for (int q = 0; q < 2; ++q) {
        for (const auto& pt : pts) {
            CycleEngineSettings s = q == 0 ? preview_settings() : CycleEngineSettings{};
            s.settling_cycles = 12;
            CycleEngine eng(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, make_op(d, pt[0], pt[1]), s);
            const CycleEngineResult r = eng.run();
            std::printf("%s %5.0f rpm %3.0f kPa: err %+.4f cda_max %.3e\n", q == 0 ? "Preview" : "Full   ", pt[0],
                        pt[1] / 1000.0, r.plenum_error_rel, r.throttle_cd_area_max_m2);
            for (int c = 0; c < 13; ++c) {
                std::printf("   cyc %2d  p %8.1f  (%+.4f)  cda %.4e\n", c, r.trace_plenum_mean_pa[static_cast<std::size_t>(c)],
                            r.trace_plenum_mean_pa[static_cast<std::size_t>(c)] / pt[1] - 1.0,
                            r.trace_cd_area_m2[static_cast<std::size_t>(c)]);
            }
        }
    }
}
