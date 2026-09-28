// tests/unit/engine_cycle_standalone_main.cpp — link-time guard (ps_audio A1
// coordinator review, second commit): proves ps_engine_cycle links and runs
// WITHOUT ps_core, so a future ps_audio target can depend on it alone.
// Target ps_engine_cycle_standalone links ONLY ps_engine_cycle + ps_warnings
// + ps_options (tests/unit/CMakeLists.txt) - explicitly not ps_core, not
// even ps_headless_env; ps_engine_cycle's own PRIVATE link to ps_detmath
// (core/CMakeLists.txt - the deterministic transcendental-math leaf target,
// also split out of ps_core during this fix) is forwarded onto this
// target's final link line the normal static-lib-archive way, so it links
// ps_detmath too, but never ps_core itself. Plain main() (not Catch2 -
// Catch2's own executables all assume ps_core via ps_test_catch_main),
// registered as a CTest smoke test via add_test(), CTest LABELS "cycle" so
// it shows up under the [cycle] topic alongside the rest of the
// cycle-model tests.
//
// Note on how this actually catches the cycle: a static archive only pulls
// in the .o members that resolve a symbol something else already
// references (per-object extraction) - this file itself calls only into
// cycle_stepper.cpp/cycle_point.cpp, so a reintroduced map_generator.cpp
// would sit in ps_engine_cycle's archive UNREFERENCED by this file and
// never get pulled in, silently passing despite needing
// ps::jobs::JobSystem. tests/unit/CMakeLists.txt therefore links this
// target against ps_engine_cycle via $<LINK_LIBRARY:WHOLE_ARCHIVE,...>,
// forcing every object file in the archive to be linked regardless of
// whether main() happens to call it.
//
// Sabotage performed by hand during the fix that added this file (not
// re-run automatically - deliberately breaking the target on every build
// would defeat the point of a guard): temporarily moved
// map_generator.cpp/map_cache.cpp back into ps_engine_cycle's source list
// in core/CMakeLists.txt (and out of ps_core's), reconfigured and rebuilt
// just this target. Actual captured link error (clang-cl/lld-link,
// release preset):
//
//   lld-link: error: undefined symbol: public: void __cdecl
//   ps::jobs::JobSystem::submit(class ps::jobs::JobFn, struct
//   ps::jobs::JobSystem::WaitGroup *)
//   >>> referenced by ps_engine_cycle.lib(map_generator.cpp.obj):
//       (... ps::drivetrain::generate_engine_maps(...))
//   lld-link: error: undefined symbol: public: void __cdecl
//   ps::jobs::JobSystem::wait(struct ps::jobs::JobSystem::WaitGroup &)
//   >>> referenced by ps_engine_cycle.lib(map_generator.cpp.obj): (same)
//
// confirming the dependency really is one-directional now (ps::jobs::
// JobSystem's implementation, core/src/jobs/job_system.cpp, lives only in
// ps_core, which this target does not link), not just accidentally
// order-independent under lld/link.exe. Reverted back to the fixed
// configuration immediately after capturing this. See
// out/audio_a1_progress.md for the round-2 fix's full narrative, including
// an EARLIER sabotage-adjacent finding this same verification step turned
// up before ps_detmath existed: ps_engine_cycle's own "live" files (not
// just map_generator.cpp) call ps::math::sincos/asin/exp/log/pow/cos
// (PLAN.md D7 - core/ must never call the platform libm transcendentals
// directly), whose implementation used to be compiled only into ps_core
// (src/math/transcendental.cpp + detmath/*.cpp) - so even with
// map_generator.cpp/map_cache.cpp correctly excluded, this guard STILL
// failed to link on the very first attempt, with unresolved ps::math::*
// symbols instead. Fixed by extracting that implementation into its own
// leaf ps_detmath target (core/CMakeLists.txt) that both ps_core and
// ps_engine_cycle link - not part of the JobSystem cycle the coordinator
// named, but the same root cause (ps_engine_cycle silently depending on
// ps_core-only object code) and caught only because this guard actually
// runs a real cycle point rather than just linking.

#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/pipe_network.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

// ps_audio A2: a short pipe-network run, so the guard also exercises (and
// links) pipe_network.cpp without ps_core. A closed tube with a pressure step
// into a junction volume: finite, moving, and mass closure to rounding.
bool pipe_run_ok() {
    PipeNetworkDesc d;
    d.volumes.push_back(VolumeDesc{"junction", 5.0e-4, GasStateDesc{101325.0, 293.15, 0.0}});
    PipeDesc p;
    p.name = "tube";
    p.length_m = 0.5;
    p.d_start_m = 0.04;
    p.d_end_m = 0.04;
    p.dx_target_m = 0.025;
    p.start = PipeEndDesc{};
    p.end = PipeEndDesc{PipeEndDesc::Kind::Volume, 0, EndCorrection::None};
    p.initial = GasStateDesc{200.0e3, 600.0, 1.0};
    d.pipes.push_back(p);
    PipeNetwork net(d, 1.0 / 46080.0);
    const real m0 = net.total_mass();
    for (int k = 0; k < 2000; ++k) net.step();
    const real dm = std::fabs(net.total_mass() - m0) / m0;
    const real pv = net.volume_state(0).p;
    const bool ok = std::isfinite(net.total_energy()) && dm < 1e-12 && pv > 110.0e3;
    std::printf("engine_cycle_standalone: pipe run %d cells, rel mass drift %.3g, junction p %.6g Pa\n",
                net.total_cell_count(), dm, pv);
    return ok;
}

// ps_audio A3: one small FV + waveguide network (the waveguide is part of
// ps_engine_cycle too). A pressurised FV pipe discharges through a waveguide
// face into a Kirchhoff-loss segment with a radiating open end.
bool fv_waveguide_run_ok() {
    PipeNetworkDesc d;
    PipeDesc p;
    p.name = "header";
    p.length_m = 0.4;
    p.d_start_m = 0.04;
    p.d_end_m = 0.04;
    p.dx_target_m = 0.025;
    p.start = PipeEndDesc{};
    p.end = PipeEndDesc{PipeEndDesc::Kind::Waveguide, 0, EndCorrection::None};
    p.initial = GasStateDesc{102.0e3, 293.15, 0.0};
    d.pipes.push_back(p);
    WgNodeDesc j;
    j.kind = WgNodeDesc::Kind::Junction;
    j.name = "flange";
    WgNodeDesc e;
    e.kind = WgNodeDesc::Kind::Open;
    e.name = "tail";
    d.waveguide.nodes = {j, e};
    WgSegmentDesc s;
    s.name = "tailpipe";
    s.length_m = 0.6;
    s.diameter_m = 0.04;
    s.node_a = 0;
    s.node_b = 1;
    s.end_b = EndCorrection::Unflanged;
    d.waveguide.segments.push_back(s);
    PipeNetwork net(d, 1.0 / 46080.0);
    const real m0 = net.total_mass();
    for (int k = 0; k < 2000; ++k) net.step();
    const real closure = std::fabs(net.total_mass() - m0 - net.exchange().waveguide_mass_kg) / m0;
    const real work = net.waveguide_face(0).acoustic_work_j;
    const bool ok = std::isfinite(net.total_energy()) && std::isfinite(net.waveguide().wave_energy()) &&
                    closure < 1e-12 && net.exchange().waveguide_mass_kg < 0.0 && work > 0.0;
    std::printf("engine_cycle_standalone: FV+waveguide run, mass out %.4g kg, closure %.3g, acoustic work %.4g J\n",
                -net.exchange().waveguide_mass_kg, closure, work);
    return ok;
}

// ps_audio A5: a small multi-cylinder pipe engine (the [AS] pipe I4 fixture:
// runners, a 4-2-1 exhaust, an installation waveguide) runs without ps_core.
bool pipe_engine_run_ok() {
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    OperatingPoint op;
    op.rpm = 3000.0;
    op.p_im_pa = 60.0e3;
    op.t_im_k = 288.15;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = test::mbt_spark_deg_atdc(d);
    CycleEngineSettings st;
    st.h_s = kStepPreview;
    st.settling_cycles = 2;
    CycleEngine eng(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, st);
    const CycleEngineResult r = eng.run();
    const bool ok = std::isfinite(r.mean.w_gross_j) && r.mean.m_air_j_cyc > 0.0 && r.mean.p_max_pa > op.p_amb_pa &&
                    std::fabs(r.mass_closure_kg) <= 1e-9 * r.mass_scale_kg && eng.network() != nullptr &&
                    eng.network()->waveguide_face_count() == 1;
    std::printf("engine_cycle_standalone: pipe I4 %d cylinders, air %.4g kg/cyl, p_plenum %.6g Pa, closure %.3g kg\n",
                r.cylinders, r.mean.m_air_j_cyc, r.p_plenum_mean_pa, r.mass_closure_kg);
    return ok;
}

} // namespace

int main() {
    if (!pipe_engine_run_ok()) {
        std::fprintf(stderr, "engine_cycle_standalone: FAIL - pipe engine run\n");
        return 1;
    }
    if (!pipe_run_ok()) {
        std::fprintf(stderr, "engine_cycle_standalone: FAIL - pipe network run\n");
        return 1;
    }
    if (!fv_waveguide_run_ok()) {
        std::fprintf(stderr, "engine_cycle_standalone: FAIL - FV + waveguide run\n");
        return 1;
    }
    const drivetrain::CycleModelDesc d = test::make_generic_i4_cycle_desc();
    const CycleStepper proto(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr);

    OperatingPoint op;
    op.rpm = 3000.0;
    op.p_im_pa = 60.0e3;
    op.t_im_k = 300.0;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = test::mbt_spark_deg_atdc(d);

    const PointResult r = run_operating_point(proto, op, StepperSettings{kStepPreview, kCyclesPreview});

    const real values[] = {r.w_gross_j, r.w_pump_j, r.m_air_j_cyc, r.m_fuel_kg, r.residual_frac,
                            r.t_exh_k,   r.h_exh_j,  r.p_max_pa,    r.ca50_deg};
    bool all_finite = true;
    for (real v : values) {
        if (!std::isfinite(v)) all_finite = false;
    }
    // Not just "finite": a stepper that silently never ran the RHS (e.g. a
    // skipped RK4 loop) would return all-zero results and still pass a bare
    // isfinite check. A real part-load point produces meaningful net work
    // and a peak pressure above ambient, and takes at least one step.
    const bool sane = all_finite && r.steps > 0 && r.p_max_pa > op.p_amb_pa && std::fabs(r.w_gross_j) > 0.0;
    if (!sane) {
        std::fprintf(stderr,
                      "engine_cycle_standalone: FAIL - finite=%d steps=%llu w_gross=%.6g p_max=%.6g (p_amb=%.6g)\n",
                      all_finite ? 1 : 0, static_cast<unsigned long long>(r.steps), r.w_gross_j, r.p_max_pa, op.p_amb_pa);
        return 1;
    }
    std::printf("engine_cycle_standalone: OK - steps=%llu w_gross=%.6g J p_max=%.6g Pa\n",
                static_cast<unsigned long long>(r.steps), r.w_gross_j, r.p_max_pa);
    return 0;
}
