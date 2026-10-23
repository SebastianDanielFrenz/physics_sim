#include "drivetrain/hybrid_charge.h"
#include "drivetrain/powertrain.h"
#include "drivetrain/engine_simulated.h"
#include "vehicle_test_helpers.h"
#include "ps/io/vehicle_io.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>
#include <array>
#include <cstdio>
using namespace ps;
using namespace ps::drivetrain;

TEST_CASE("Charging chooses feasible power before minimizing fuel", "[hybrid_charge]") {
    auto capacity=[](real rpm){return rpm*20;};
    auto fuel=[](real rpm,real){return 3+(rpm-3000)*(rpm-3000)*1e-6;};
    const auto low=select_hybrid_charge_point(40000,1000,5000,false,capacity,fuel);
    const auto efficient=select_hybrid_charge_point(40000,1000,5000,true,capacity,fuel);
    CHECK(low.rpm==2000);CHECK(low.target_met);
    CHECK(efficient.rpm==3000);CHECK(efficient.power_w==40000);CHECK(efficient.fuel_optimized);
    CHECK(fuel(efficient.rpm,40000)<fuel(low.rpm,40000));
    const auto limited=select_hybrid_charge_point(200000,1000,5000,true,capacity,fuel);
    CHECK_FALSE(limited.target_met);CHECK(limited.power_w==100000);CHECK(limited.rpm==5000);
    const auto unknown=select_hybrid_charge_point(40000,1000,5000,true,capacity,
        [](real,real){return std::numeric_limits<real>::quiet_NaN();});
    CHECK(unknown.rpm==low.rpm);CHECK_FALSE(unknown.fuel_optimized);
}

TEST_CASE("Disconnected hybrid reaches its crank power target and charges at bounded generator power", "[hybrid_charge][vehicle]") {
    for(const auto [requested,scale,charge_limit]: {std::array<real,3>{25000,1,200000}, {50000,1,200000}, {100000,1,200000}, {100000,4,200000}, {50000,1,20000}}) {
        auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hybrid.json");
        for(auto& c:d.powertrain.components) {
            if(auto* b=std::get_if<BatteryDesc>(&c.params)){b->capacity_kwh=20;b->initial_soc=.19;b->max_charge_w=charge_limit;}
            if(auto* h=std::get_if<HybridControlDesc>(&c.params))h->charge_power_w=requested;
            if(auto* m=std::get_if<ElectricMotorDesc>(&c.params);m&&m->crank_assist){m->regen_torque_nm*=scale;m->max_power_w*=scale;}
        }
        auto rig=ps_test::build_vehicle_rig_from_desc(d,1800,{2,.4,.15},.6,0,2);
        auto& w=*rig.world;
        w.reset_vehicle(rig.vehicle,{.gear=0,.engine_state=EngineState::Off});
        w.set_control("gear_request",0);w.set_control("throttle",0);w.set_control("assist.drive_mode",1);
        real watts=0,min_rpm=1e9,max_rpm=0;
        for(int i=0;i<240*25;++i){
            w.step();const auto p=w.powertrain_state(rig.vehicle);
            if(i>=240*20){watts-=p.battery_power_w;min_rpm=std::min(min_rpm,p.engine_rpm);max_rpm=std::max(max_rpm,p.engine_rpm);}
        }
        watts/=240*5;
        std::printf("charge request %.1f kW battery %.2f kW rpm %.0f..%.0f\n",requested/1000,watts/1000,min_rpm,max_rpm);
        CHECK(watts==Catch::Approx(std::min(std::min(requested,real(50000)*scale)*.95,charge_limit)).epsilon(.03));
        CHECK(max_rpm-min_rpm<100);
        CHECK(w.powertrain_state(rig.vehicle).gear==0);
    }
}

TEST_CASE("Wheel traction cuts cannot disturb a disconnected charging engine", "[hybrid_charge]") {
    auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hybrid.json");
    for(auto& c:d.powertrain.components)if(auto* b=std::get_if<BatteryDesc>(&c.params))b->initial_soc=.19;
    Powertrain baseline(d.powertrain,d.brake_system,d.wheels,d.controllers,d.name);
    Powertrain cut(d.powertrain,d.brake_system,d.wheels,d.controllers,d.name);
    vehicle::VehicleResetOptions options;options.gear=0;options.engine_state=EngineState::Running;
    baseline.reset(options,std::vector<real>(4,0));cut.reset(options,std::vector<real>(4,0));
    ComponentSubstepCtx ctx;ctx.h=1.0/960;ctx.gear_request=0;ctx.throttle=0;
    for(int i=0;i<960*15;++i){
        auto intervention=ctx;
        if(i>960*5){intervention.traction_throttle_ceiling=0;intervention.traction_ignition_cut=true;intervention.traction_torque_fraction=0;}
        baseline.begin_substep();baseline.pre_solve_components(ctx);baseline.solve(ctx.h);baseline.post_solve_components(ctx);
        cut.begin_substep();cut.pre_solve_components(intervention);cut.solve(ctx.h);cut.post_solve_components(intervention);
    }
    REQUIRE(baseline.hybrid_charging());REQUIRE_FALSE(baseline.hybrid_engine_coupled());
    CHECK(baseline.engine_omega()==cut.engine_omega());
    CHECK(baseline.throttle_eff()==cut.throttle_eff());
    CHECK(baseline.ledger().battery_draw==cut.ledger().battery_draw);
}

TEST_CASE("Simulated engine charging fuel estimate agrees with a naturally aspirated steady point", "[hybrid_charge]") {
    auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_sedan_sim.json");
    const SimulatedEngineDesc* desc=nullptr;
    for(const auto& c:d.powertrain.components)if(auto* e=std::get_if<SimulatedEngineDesc>(&c.params))desc=e;
    REQUIRE(desc!=nullptr);
    auto maps=SimulatedEngine::build_maps(*desc);
    SimulatedEngine engine(*desc,maps);
    const auto sp=SimulatedEngine::steady_point(*maps,2500,.4,AmbientSample{});
    REQUIRE(sp.net_torque_nm>0);
    const auto estimate=engine.charging_fuel_g_s(2500,sp.net_torque_nm);
    CHECK(estimate==Catch::Approx(sp.fuel_g_s).epsilon(.005));
}
