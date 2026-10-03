#include "drivetrain/engine_simulated.h"
#include "mvem_fixtures.h"
#include "ps/jobs/debug_alloc.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <chrono>
#include <cstdio>
#include <limits>
using namespace ps;
using namespace ps::drivetrain;
namespace {
Sensors running(real rpm,real p,real t) {
    Sensors s; s.omega=rpm*6.283185307179586/60; s.ignition=1;
    s.ambient={p,t}; return s;
}
}
TEST_CASE("Live atmosphere follows every tiny sample without whole control-map rebuilds","[ambient_live][mvem]") {
    auto d=ps::test::mvem::electronic_engine(); auto maps=SimulatedEngine::build_maps(d); SimulatedEngine engine(d,maps);
    const auto start=std::chrono::steady_clock::now();
    for(int i=0;i<80;++i) {
        const real p=98000+real(i)*.001, t=286.5+real(i)*.00001;
        auto s=running(3000,p,t); auto pre=engine.pre_solve(s,{.45,0,1},{},1.0/960);
        (void)engine.post_solve(s.omega,1.0/960);
        CHECK(engine.ambient_pressure()==p); CHECK(engine.ambient_temperature()==t);
        CHECK(engine.steady_rebuild_count()==0); CHECK(std::isfinite(pre.tau0));
    }
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::printf("[ambient_live] 80 continuously changing samples: %.3f ms, zero grid rebuilds\n",ms);
    // Invalid samples must preserve the physical boundary.
    const auto p=engine.ambient_pressure(), t=engine.ambient_temperature();
    auto bad=running(3000,std::numeric_limits<real>::quiet_NaN(),-1);
    (void)engine.pre_solve(bad,{.45,0,1},{},1.0/960);
    CHECK(engine.ambient_pressure()==p); CHECK(engine.ambient_temperature()==t);
}
TEST_CASE("Live operating-point control preserves altitude pedal law against setup table reference","[ambient_live][mvem]") {
    auto d=ps::test::mvem::electronic_engine(); auto maps=SimulatedEngine::build_maps(d);
    for(real pedal:{0.2,.45,.7,1.0}) {
        SimulatedEngine live(d,maps), reference(d,maps); reference.set_ambient(80000,275);
        auto s=running(3000,80000,275); PreSolveResult a,b;
        for(int i=0;i<600;++i) {
            a=live.pre_solve(s,{pedal,0,1},{},1.0/960); b=reference.pre_solve(s,{pedal,0,1},{},1.0/960);
            (void)live.post_solve(s.omega,1.0/960); (void)reference.post_solve(s.omega,1.0/960);
        }
        // Old reference bilinearly interpolates a 41-column grid; live solve is continuous.
        CHECK(a.tau0==Catch::Approx(b.tau0).margin(1.0));
        CHECK(live.p_im()==Catch::Approx(reference.p_im()).epsilon(.01));
        CHECK(live.steady_rebuild_count()==0); CHECK(reference.steady_rebuild_count()==1);
    }
}
TEST_CASE("Live atmosphere control does not allocate inside jobs and hashes deterministically","[ambient_live][mvem][determinism]") {
    auto d=ps::test::mvem::electronic_engine(); auto maps=SimulatedEngine::build_maps(d);
    SimulatedEngine a(d,maps), b(d,maps);
    auto s=running(3000,97000,280); (void)a.pre_solve(s,{.3,0,1},{},1.0/960); (void)b.pre_solve(s,{.3,0,1},{},1.0/960);
    const bool hook=jobs::debug_alloc::hook_available(); jobs::debug_alloc::reset(); jobs::debug_alloc::set_armed(hook);
    jobs::debug_alloc::enter_job();
    for(int i=0;i<100;++i) {
        s.ambient.pressure_pa=97000+real(i)*.01; s.ambient.temperature_k=280+real(i)*.0001;
        (void)a.pre_solve(s,{.3,0,1},{},1.0/960); (void)a.post_solve(s.omega,1.0/960);
        (void)b.pre_solve(s,{.3,0,1},{},1.0/960); (void)b.post_solve(s.omega,1.0/960);
    }
    jobs::debug_alloc::exit_job(); jobs::debug_alloc::set_armed(false);
    if(hook) CHECK(jobs::debug_alloc::own_violation_count()==0);
    CHECK(a.fold_state_hash(1)==b.fold_state_hash(1)); CHECK(a.steady_rebuild_count()==0);
}

TEST_CASE("Non-reference materialized control cache is included in engine state hash","[ambient_live][mvem]") {
    auto d=ps::test::mvem::electronic_engine(); auto maps=SimulatedEngine::build_maps(d);
    SimulatedEngine a(d,maps), b(d,maps); REQUIRE(a.fold_state_hash(1)==b.fold_state_hash(1));
    a.set_ambient(80000,275); CHECK(a.fold_state_hash(1)!=b.fold_state_hash(1));
    b.set_ambient(80000,275); CHECK(a.fold_state_hash(1)==b.fold_state_hash(1));
}

TEST_CASE("Identical map swap remains invisible during live non-reference atmosphere","[ambient_live][mvem]") {
    auto d=ps::test::mvem::electronic_engine(); auto maps=SimulatedEngine::build_maps(d);
    SimulatedEngine a(d,maps), b(d,maps); auto s=running(3000,80000,275);
    (void)a.pre_solve(s,{.3,0,1},{},1.0/960); (void)b.pre_solve(s,{.3,0,1},{},1.0/960);
    const auto hash=a.fold_state_hash(1); a.request_replace_maps(maps); const auto old=a.apply_pending_maps_swap();
    REQUIRE(old!=nullptr); CHECK(a.fold_state_hash(1)==hash);
    for(int i=0;i<10;++i) {
        (void)a.pre_solve(s,{.3,0,1},{},1.0/960); (void)b.pre_solve(s,{.3,0,1},{},1.0/960);
        (void)a.post_solve(s.omega,1.0/960); (void)b.post_solve(s.omega,1.0/960);
        CHECK(a.fold_state_hash(1)==b.fold_state_hash(1));
    }
}
