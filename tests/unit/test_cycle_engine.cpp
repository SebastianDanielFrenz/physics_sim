// tests/unit/test_cycle_engine.cpp - ps_audio A5: the pipe-aware cycle stepper
// and the multi-cylinder pipe-coupled CycleEngine (core/src/engine_cycle).
// Fixture: tests/support/cycle_pipe_fixtures.h (a GENERIC [AS] pipe I4 - not
// a shipped engine). Every guard names the defect it catches; the sabotage
// each was shown to fail on is recorded in out/audio_a5_progress.md.

#include "cycle_pipe_fixtures.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

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
