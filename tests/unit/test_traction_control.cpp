#include "ps/vehicle/traction_control.h"
#include "drivetrain/throttle_stage.h"
#include "vehicle_test_helpers.h"
#include "ps/jobs/job_system.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <array>
#include <cstdio>
#include <limits>
using namespace ps;
using namespace ps::vehicle;
namespace {
std::array<TractionWheelState,4> wheels(real ground, real peripheral) {
    return {{{peripheral,ground,3500,true,true},{peripheral,ground,3500,true,true},
             {peripheral,ground,3500,true,true},{peripheral,ground,3500,true,true}}};
}
}
TEST_CASE("Physics TC cuts actual forward reverse and standstill spin", "[traction_control][unit]") {
    for (real sign : {real(-1),real(1)}) {
        TractionControlDesc d; TractionControlState s;
        auto w = wheels(sign*10,sign*20);
        for(int i=0;i<50;++i)step_traction_control(d,s,w,sign*10,1,0,true,.004);
        CHECK(s.positive_slip == Catch::Approx(1)); CHECK(s.torque_fraction < .2);
        CHECK(s.intervening);
        w=wheels(0,sign*5);
        CHECK(traction_driven_slip(w,2)==Catch::Approx(2.5));
        w=wheels(sign*10,sign*8);
        CHECK(traction_driven_slip(w,2)==0);
    }
}
TEST_CASE("Physics TC ignores estimator history and releases the logged false cut", "[traction_control][unit][low_speed]") {
    TractionControlDesc d; d.min_fraction=.15; TractionControlState s;
    s.initialized=true; s.estimated_speed_m_s=3.9623; s.torque_fraction=.15;
    auto w=wheels(27.39/3.6,27.39/3.6*1.02);
    for(int i=0;i<40;++i)step_traction_control(d,s,w,27.39/3.6,1,0,true,.004);
    CHECK(s.estimated_speed_m_s==Catch::Approx(27.39/3.6));
    CHECK(s.positive_slip==Catch::Approx(.02)); CHECK(s.torque_fraction==1);
    CHECK_FALSE(s.intervening);
}
TEST_CASE("Physics TC allows airborne wheel acceleration and reacts on landing", "[traction_control][unit]") {
    TractionControlDesc d; TractionControlState s;
    auto w=wheels(10,50);
    for(auto& c:w)c.contact=false;
    s.torque_fraction=.1;
    CHECK(step_traction_control(d,s,w,10,1,0,true,.004)==1);
    CHECK_FALSE(s.intervening); CHECK(s.positive_slip==0);
    // One spinning airborne corner must not cut three rolling loaded wheels.
    w=wheels(10,10); w[0].peripheral_speed_m_s=50; w[0].contact=false;
    CHECK(step_traction_control(d,s,w,10,1,0,true,.004)==1);
    w[0].contact=true; w[0].normal_load_n=0;
    CHECK(step_traction_control(d,s,w,10,1,0,true,.004)==1);
    w[0].normal_load_n=3500;
    for(int i=0;i<50;++i)step_traction_control(d,s,w,10,1,0,true,.004);
    CHECK(s.intervening); CHECK(s.torque_fraction<.2);
    w=wheels(10,10);
    for(int i=0;i<40;++i)step_traction_control(d,s,w,10,1,0,true,.004);
    CHECK(s.torque_fraction==1);
}
TEST_CASE("Physics TC respects per-patch corner speed, driven wheels and braking", "[traction_control][unit]") {
    TractionControlDesc d; TractionControlState s;
    auto w=wheels(10,10);
    for(int i=0;i<4;++i)w[i].peripheral_speed_m_s=w[i].ground_speed_m_s=8+i;
    CHECK(step_traction_control(d,s,w,10,1,0,true,.004)==1);
    w[0].peripheral_speed_m_s=40;w[0].driven=false;
    CHECK(step_traction_control(d,s,w,10,1,0,true,.004)==1);
    w[1].peripheral_speed_m_s=40;
    for(int i=0;i<50;++i)step_traction_control(d,s,w,10,1,0,true,.004);
    CHECK(s.intervening);
    CHECK(step_traction_control(d,s,w,10,1,1,true,.004)==1);
    CHECK(step_traction_control(d,s,w,10,0,0,true,.004)==1);
    CHECK(step_traction_control(d,s,w,10,1,0,false,.004)==1);
}
TEST_CASE("Physics TC validates data and preserves throttle safety priority", "[traction_control][unit]") {
    TractionControlDesc d;TractionControlState s;CHECK(fold_traction_control_hash(d,s,42)==42);
    auto w=wheels(10,20);step_traction_control(d,s,w,10,1,0,true,.004);
    CHECK(fold_traction_control_hash(d,s,42)!=42);
    CHECK(step_traction_control(d,s,w,10,1,0,true,0)==1);CHECK_FALSE(s.initialized);
    d.target_slip=0;CHECK_THROWS(validate_traction_control(d));d.target_slip=.08;
    d.kp=std::numeric_limits<real>::infinity();CHECK_THROWS(validate_traction_control(d));
    drivetrain::ThrottleStage stage;drivetrain::ComponentSubstepCtx ctx;ctx.throttle=.8;
    ctx.tcu_throttle_floor=.7;ctx.tcu_throttle_ceiling=.9;ctx.traction_throttle_ceiling=.2;
    auto cmd=stage.evaluate(ctx);CHECK(cmd.command==.8);CHECK(cmd.ceiling==.2);CHECK(cmd.floor==.2);
}
TEST_CASE("Full hypercar TC reduces low-grip wheelspin with physical contact feedback","[traction_control][vehicle][long]") {
    jobs::JobSystem pool(10);io::EngineMapOptions opt;opt.pool=&pool;
    auto base=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hyper.json",opt);
    for(auto& w:base.wheels)w.tyre.lambda_mux*=.25;
    struct Result {real mean_slip=0,speed=0,min_fraction=1;int cuts=0;std::vector<std::uint64_t> hashes;};
    const auto run=[&](bool enabled,int workers=1) {
        auto desc=base;desc.traction_control.enabled=enabled;
        auto rig=ps_test::build_vehicle_rig_from_desc(desc,1500,{2,.4,.15},.5,0,workers);
        // Assists I2 (owner Q2): car_hyper carries the Racing assist unit, whose channel default (On) wins over
        // traction_control.enabled - the channel is set explicitly so 'off' is really off.
        rig.world->set_control("assist.traction_control",enabled?1.0:0.0);
        rig.world->set_control("assist.aero_control",0.0); // the same unit's wing automation (assists I5) stays out of the TC comparison
        ps_test::settle(*rig.world,.5);rig.world->set_control("gear_request",1);rig.world->set_control("throttle",1);
        Result out;
        for(int tick=0;tick<1920;++tick) {
            rig.world->step();out.hashes.push_back(rig.world->state_hash());const auto pt=rig.world->powertrain_state(rig.vehicle);
            if(enabled&&tick%480==479)std::printf("TC detail t=%.1f rpm=%.0f state=%d e=%.3f fraction=%.3f estimated=%.3f actual=%.3f slip=%.3f\n",(tick+1)/240.,pt.engine_rpm,int(pt.engine_state),pt.clutch_engagement,pt.traction_torque_fraction,pt.traction_estimated_speed_m_s,rig.world->get_motion(rig.chassis).linear.x,pt.traction_positive_slip);
            if(tick>240)CHECK(pt.engine_state!=drivetrain::EngineState::Stalled);
            out.min_fraction=std::min(out.min_fraction,pt.traction_torque_fraction);if(pt.traction_control_intervening)++out.cuts;
            if(tick>=960)for(std::size_t w:{std::size_t(2),std::size_t(3)})out.mean_slip+=std::abs(rig.world->wheel_telemetry(rig.vehicle,w).slip_ratio)/1920.;
        }
        out.speed=rig.world->get_motion(rig.chassis).linear.x;
        rig.world->set_control("assist.traction_control",0);rig.world->step();CHECK(rig.world->powertrain_state(rig.vehicle).traction_torque_fraction==1);
        rig.world->reset_vehicle(rig.vehicle,{});rig.world->step();CHECK_FALSE(rig.world->powertrain_state(rig.vehicle).traction_control_intervening);
        return out;
    };
    const auto off=run(false),on=run(true);
    std::printf("TC probe off slip=%.4f v=%.3f on slip=%.4f v=%.3f min_fraction=%.3f cuts=%d\n",off.mean_slip,off.speed,on.mean_slip,on.speed,on.min_fraction,on.cuts);
    CHECK(on.cuts>0);CHECK(on.min_fraction<.8);CHECK(on.mean_slip<off.mean_slip*.7);CHECK(on.speed>off.speed);
    const auto parallel=run(true,4);CHECK(parallel.mean_slip==on.mean_slip);CHECK(parallel.speed==on.speed);CHECK(parallel.hashes==on.hashes);
}

TEST_CASE("Airborne vehicle accelerates its driven wheels without a TC cut", "[traction_control][vehicle][unit]") {
    auto desc=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_sedan.json");
    desc.traction_control.enabled=true;
    auto rig=ps_test::build_vehicle_rig_from_desc(desc,1500,{2,.4,.15},.6,0,4);
    auto& world=*rig.world;
    auto pose=world.get_pose(rig.chassis);pose.position.z+=10;
    world.backend().set_pose(rig.chassis,pose);
    world.backend().set_motion(rig.chassis,{});
    world.reset_vehicle(rig.vehicle,{.gear=1});
    world.set_control("assist.traction_control",1);
    world.set_control("throttle",1);
    world.set_control("gear_request",1);
    real max_wheel_speed=0;
    for(int tick=0;tick<120;++tick) {
        world.step();
        const auto pt=world.powertrain_state(rig.vehicle);
        CHECK_FALSE(pt.traction_control_intervening);
        CHECK(pt.traction_torque_fraction==1);
        for(std::size_t i=0;i<desc.wheels.size();++i) {
            CHECK(world.wheel_telemetry(rig.vehicle,i).fz==0);
            max_wheel_speed=std::max(max_wheel_speed,std::abs(world.wheel_omega(rig.vehicle,i)));
        }
    }
    CHECK(max_wheel_speed>10);
}