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
std::array<TractionWheelSensor,4> sensors(real speed,bool awd=true) {
    return {{{speed,0,1,.8,awd},{speed,0,1,-.8,awd},{speed,0,-1,.8,true},{speed,0,-1,-.8,true}}};
}
}
TEST_CASE("Sensor TC detects common AWD acceleration wheelspin without ground speed","[traction_control][unit]") {
    TractionControlDesc d;TractionControlState s;auto w=sensors(0);
    step_traction_control(d,s,w,0,0,1,0,true,.01);
    // All four wheels agree but accelerate far faster than the IMU reports.
    for(int i=1;i<=100;++i){w=sensors(i*.08);step_traction_control(d,s,w,1,0,1,0,true,.01);}
    CHECK(s.estimated_speed_m_s==Catch::Approx(1));CHECK(s.positive_slip>3);CHECK(s.torque_fraction<.1);
    CHECK(s.intervening);
    // Coast removes the intervention, reanchors only from wheel sensors.
    CHECK(step_traction_control(d,s,w,0,0,0,0,true,.01)==1);
    CHECK_FALSE(s.intervening);
}
TEST_CASE("Sensor TC preserves normal acceleration turns reverse and braking","[traction_control][unit]") {
    TractionControlDesc d;
    SECTION("normal AWD launch") {
        TractionControlState s;auto w=sensors(0);step_traction_control(d,s,w,0,0,1,0,true,.01);
        for(int i=1;i<=500;++i){w=sensors(i*.03);CHECK(step_traction_control(d,s,w,3,0,1,0,true,.01)==1);}
    }
    SECTION("one driven wheel spinning with nondriven speed references") {
        TractionControlState s;auto w=sensors(10,false);step_traction_control(d,s,w,0,0,.7,0,true,.01);
        w[2].peripheral_speed_m_s=25;
        for(int i=0;i<20;++i)step_traction_control(d,s,w,0,0,.7,0,true,.01);
        CHECK(s.intervening);CHECK(s.estimated_speed_m_s==10);
    }
    SECTION("yaw compensation") {
        TractionControlState s;auto w=sensors(10);
        for(auto& wheel:w)wheel.peripheral_speed_m_s=10-wheel.y_m*.5;
        CHECK(step_traction_control(d,s,w,0,.5,1,0,true,.01)==1);
    }
    SECTION("reverse AWD acceleration wheelspin") {
        TractionControlState s;auto w=sensors(-10);step_traction_control(d,s,w,0,0,1,0,true,.01);w=sensors(-25);
        for(int i=0;i<20;++i)step_traction_control(d,s,w,0,0,1,0,true,.01);
        CHECK(s.intervening);
        CHECK(step_traction_control(d,s,w,0,0,1,1,true,.01)==1);
        CHECK(step_traction_control(d,s,w,0,0,1,0,false,.01)==1);
    }
    SECTION("slower rolling wheels are not positive spin") {
        TractionControlState s;auto w=sensors(10);step_traction_control(d,s,w,0,0,1,0,true,.01);
        w=sensors(7);CHECK(step_traction_control(d,s,w,0,0,1,0,true,.01)==1);
    }
}
TEST_CASE("Sensor TC validation reset hash and throttle safety priority","[traction_control][unit]") {
    TractionControlDesc d;TractionControlState s;CHECK(fold_traction_control_hash(d,s,42)==42);
    auto w=sensors(0);step_traction_control(d,s,w,0,0,1,0,true,.01);
    CHECK(fold_traction_control_hash(d,s,42)!=42);s={};CHECK(fold_traction_control_hash(d,s,42)==42);
    d.target_slip=0;CHECK_THROWS(validate_traction_control(d));d.target_slip=.08;
    d.kp=std::numeric_limits<real>::infinity();CHECK_THROWS(validate_traction_control(d));
    drivetrain::ThrottleStage stage;drivetrain::ComponentSubstepCtx ctx;ctx.throttle=.8;
    ctx.tcu_throttle_floor=.7;ctx.tcu_throttle_ceiling=.9;ctx.traction_throttle_ceiling=.2;
    auto cmd=stage.evaluate(ctx);CHECK(cmd.command==.8);CHECK(cmd.ceiling==.2);CHECK(cmd.floor==.2);
}
TEST_CASE("Full hypercar TC reduces low-grip wheelspin with sensor-only feedback","[traction_control][vehicle]") {
    jobs::JobSystem pool(10);io::EngineMapOptions opt;opt.pool=&pool;
    auto base=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hyper.json",opt);
    for(auto& w:base.wheels)w.tyre.lambda_mux*=.25;
    struct Result {real mean_slip=0,speed=0,min_fraction=1;int cuts=0;std::vector<std::uint64_t> hashes;};
    const auto run=[&](bool enabled,int workers=1) {
        auto desc=base;desc.traction_control.enabled=enabled;
        auto rig=ps_test::build_vehicle_rig_from_desc(desc,1500,{2,.4,.15},.5,0,workers);
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
