#include "vehicle_test_helpers.h"
#include "drivetrain/electric_drive.h"
#include "ps/io/vehicle_io.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstdio>
using namespace ps;
using namespace ps::drivetrain;
namespace {
auto ev_desc(){return io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_electric.json");}
}
TEST_CASE("EV drives from zero RPM, draws battery energy and closes the shaft ledger", "[ev][vehicle]") {
    auto d=ev_desc();
    auto rig=ps_test::build_vehicle_rig_from_desc(d,1850,{2,.4,.15},.6,0,4);
    auto& w=*rig.world;
    w.set_control("assist.traction_control",1);
    ps_test::settle(w,.5);
    const real before=w.powertrain_state(rig.vehicle).battery_soc;
    w.set_control("gear_request",1);w.set_control("throttle",1);
    real peak=0,max_error=0;
    for(int i=0;i<1200;++i){w.step();peak=std::max(peak,w.powertrain_state(rig.vehicle).electric_power_w);max_error=std::max(max_error,std::abs(w.powertrain_energy(rig.vehicle).balance_error));}
    const auto p=w.powertrain_state(rig.vehicle);
    const auto e=w.powertrain_energy(rig.vehicle);
    std::printf("EV v=%.2f km/h peak=%.1f kW SOC %.6f -> %.6f balance %.9f J\n",w.get_motion(rig.chassis).linear.length()*3.6,peak/1000,before,p.battery_soc,max_error);
    CHECK(p.electric_fitted);CHECK(p.electric_ready);CHECK(p.electric_motor_count==2);
    CHECK(p.engine_state==EngineState::Off);CHECK(p.motor_rpm>1000);
    CHECK(p.transmission_kind==4);CHECK(p.transmission_selector>0);CHECK(p.speedo_mps>10);
    CHECK(w.get_motion(rig.chassis).linear.x>10);CHECK(peak>100000);
    CHECK(p.battery_soc<before);CHECK(e.battery_draw>0);CHECK(e.electric_loss>0);
    CHECK(max_error<1e-5);
    CHECK(e.battery_draw==Catch::Approx((before-p.battery_soc)*p.battery_capacity_kwh*3600000).margin(1e-4));
    w.set_control("throttle",0);w.set_control("brake",.1);
    const real regen_start=p.battery_soc;
    for(int i=0;i<240;++i)w.step();
    CHECK(w.powertrain_state(rig.vehicle).battery_soc>regen_start);
    CHECK(w.powertrain_state(rig.vehicle).battery_power_w<0);
}
TEST_CASE("EV neutral reverse ready and empty battery have no idle or starter dependency", "[ev][vehicle]") {
    for(int mode:{0,-1,1}) {
        auto d=ev_desc();
        if(mode==1)std::get<BatteryDesc>(d.powertrain.components[0].params).initial_soc=0;
        auto rig=ps_test::build_vehicle_rig_from_desc(d,1850,{2,.4,.15},.6,0,1);
        auto& w=*rig.world;w.set_control("gear_request",mode);w.set_control("throttle",.25);
        for(int i=0;i<480;++i)w.step();
        const auto p=w.powertrain_state(rig.vehicle);
        if(mode==-1)CHECK(w.get_motion(rig.chassis).linear.x < -1);
        else CHECK(std::abs(w.get_motion(rig.chassis).linear.x)<.1);
        if(mode==1){CHECK_FALSE(p.electric_ready);CHECK(p.battery_soc==0);}
        w.set_control("electric.ready",0);w.step();CHECK_FALSE(w.powertrain_state(rig.vehicle).electric_ready);
    }
}
TEST_CASE("EV battery validation rejects invalid stores", "[ev][unit]") {
    BatteryDesc d;d.capacity_kwh=0;CHECK_THROWS(Battery(d));
    d.capacity_kwh=90;d.initial_soc=1.1;CHECK_THROWS(Battery(d));
}
TEST_CASE("Electric assist shares an ICE shaft and operates with combustion ignition off", "[ev][hybrid][vehicle]") {
    auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_sedan.json");
    d.powertrain.components.push_back({"hv_battery",BatteryDesc{}});
    ElectricMotorDesc motor;motor.battery="hv_battery";
    d.powertrain.components.push_back({"assist_motor",motor});
    d.powertrain.shafts.front().ports.push_back({"assist_motor","shaft",false});
    auto rig=ps_test::build_vehicle_rig_from_desc(d,1700,{2,.4,.15},.6,0,4);
    auto& w=*rig.world;
    w.reset_vehicle(rig.vehicle,{.gear=1,.engine_state=EngineState::Off});
    w.set_control("assist.auto_clutch",0);w.set_control("ignition",0);
    w.set_control("gear_request",1);w.set_control("throttle",.3);
    for(int i=0;i<720;++i)w.step();
    const auto p=w.powertrain_state(rig.vehicle);
    CHECK(p.engine_state==EngineState::Off);CHECK(p.electric_ready);
    CHECK(p.electric_power_w>0);CHECK(w.powertrain_energy(rig.vehicle).electric_out>0);
    CHECK(w.get_motion(rig.chassis).linear.x>1);
    CHECK(p.battery_soc<.9);
}
TEST_CASE("Dual motor shared battery is deterministic across worker counts", "[ev][determinism]") {
    const auto run=[](int workers){
        auto rig=ps_test::build_vehicle_rig_from_desc(ev_desc(),1850,{2,.4,.15},.6,0,workers);
        auto& w=*rig.world;w.set_control("gear_request",1);w.set_control("throttle",.5);
        std::vector<std::uint64_t> hashes;
        for(int i=0;i<480;++i){w.step();hashes.push_back(w.state_hash());}
        return hashes;
    };
    CHECK(run(1)==run(10));
}

TEST_CASE("Shared EV battery limits total discharge power and full packs refuse regeneration", "[ev][vehicle]") {
    auto d=ev_desc();
    auto& b=std::get<BatteryDesc>(d.powertrain.components[0].params);
    b.max_discharge_w=10000;
    auto rig=ps_test::build_vehicle_rig_from_desc(d,1850,{2,.4,.15},.6,0,4);
    auto& w=*rig.world;w.set_control("gear_request",1);w.set_control("throttle",1);
    real peak=0;
    for(int i=0;i<480;++i){w.step();peak=std::max(peak,w.powertrain_state(rig.vehicle).battery_power_w);}
    CHECK(peak<=10100);CHECK(peak>5000);
    d=ev_desc();std::get<BatteryDesc>(d.powertrain.components[0].params).initial_soc=1;
    auto full=ps_test::build_vehicle_rig_from_desc(d,1850,{2,.4,.15},.6,0,1);
    auto motion=full.world->get_motion(full.chassis);motion.linear.x=10;
    full.world->backend().set_motion(full.chassis,motion);
    full.world->reset_vehicle(full.vehicle,{});
    full.world->set_control("gear_request",1);full.world->set_control("brake",.1);
    for(int i=0;i<120;++i)full.world->step();
    CHECK(full.world->powertrain_state(full.vehicle).battery_soc==1);
    CHECK(full.world->powertrain_state(full.vehicle).battery_power_w==0);
}
TEST_CASE("Fuel cell converts finite hydrogen to battery energy with explicit losses", "[ev][fuel_cell]") {
    auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_fuel_cell.json");
    auto rig=ps_test::build_vehicle_rig_from_desc(d,1850,{2,.4,.15},.6,0,4);auto& w=*rig.world;
    const real initial_soc=.4;
    for(int i=0;i<480;++i)w.step();
    auto p=w.powertrain_state(rig.vehicle);auto e=w.powertrain_energy(rig.vehicle);
    CHECK(p.fuel_cell_fitted);CHECK(p.fuel_cell_power_w>0);CHECK(p.battery_power_w<0);
    CHECK(p.fuel_cell_hydrogen_kg<5);CHECK(p.battery_soc>initial_soc);
    CHECK(e.fuel_cell_electric==Catch::Approx((p.battery_soc-initial_soc)*p.battery_capacity_kwh*3600000).margin(1e-4));
    CHECK(e.fuel_cell_chemical==Catch::Approx((5-p.fuel_cell_hydrogen_kg)*120000000).margin(1e-3));
    CHECK(e.fuel_cell_electric+e.fuel_cell_loss==Catch::Approx(e.fuel_cell_chemical).margin(1e-4));
    w.set_control("electric.ready",0);for(int i=0;i<24;++i)w.step();
    p=w.powertrain_state(rig.vehicle);CHECK(p.fuel_cell_power_w==0);
    const auto before=p.fuel_cell_hydrogen_kg;
    for(int i=0;i<240;++i)w.step();CHECK(w.powertrain_state(rig.vehicle).fuel_cell_hydrogen_kg==before);
}
TEST_CASE("Fuel cell obeys empty fuel and full battery limits", "[ev][fuel_cell]") {
    for(bool full:{false,true}) {
        auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_fuel_cell.json");
        for(auto& c:d.powertrain.components) {
            if(auto* b=std::get_if<BatteryDesc>(&c.params))b->initial_soc=full?1:0;
            if(auto* f=std::get_if<FuelCellDesc>(&c.params))f->initial_hydrogen_kg=full?5:0;
        }
        auto rig=ps_test::build_vehicle_rig_from_desc(d,1850,{2,.4,.15},.6,0,1);
        for(int i=0;i<120;++i)rig.world->step();
        auto p=rig.world->powertrain_state(rig.vehicle);
        CHECK(p.battery_soc==(full?1:0));CHECK(p.fuel_cell_power_w==0);
    }
}
TEST_CASE("Hybrid modes keep electric primary and start combustion only as required", "[ev][hybrid_control]") {
 for(int mode=0;mode<4;++mode) {
  auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hybrid.json");
  auto rig=ps_test::build_vehicle_rig_from_desc(d,1800,{2,.4,.15},.6,0,2);auto& w=*rig.world;
  w.reset_vehicle(rig.vehicle,{.gear=0,.engine_state=EngineState::Off});
  w.set_control("assist.drive_mode",mode);w.set_control("gear_request",99);w.set_control("throttle",.2);
  for(int i=0;i<960;++i)w.step();
  auto p=w.powertrain_state(rig.vehicle);
  INFO("mode="<<mode<<" rpm="<<p.engine_rpm<<" v="<<w.get_motion(rig.chassis).linear.x);
  CHECK(p.hybrid_fitted);CHECK(p.electric_power_w>0);CHECK(w.get_motion(rig.chassis).linear.x>1);
  if(mode<2){CHECK(p.engine_state==EngineState::Off);CHECK_FALSE(p.hybrid_ice_requested);}
  else {CHECK(p.engine_state==EngineState::Running);CHECK(p.hybrid_ice_requested);}
  w.set_control("throttle",1);for(int i=0;i<1200;++i)w.step();p=w.powertrain_state(rig.vehicle);
  if(mode==0){CHECK(p.engine_state==EngineState::Off);CHECK_FALSE(p.hybrid_ice_requested);}
  else {CHECK(p.hybrid_ice_requested);CHECK(p.engine_state==EngineState::Running);}
 }
}
TEST_CASE("Hybrid low SOC starts neutral generator and charges to configured upper band", "[ev][hybrid_control]") {
 auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hybrid.json");
 for(auto& c:d.powertrain.components)if(auto* b=std::get_if<BatteryDesc>(&c.params)){b->capacity_kwh=.01;b->initial_soc=.19;}
 auto rig=ps_test::build_vehicle_rig_from_desc(d,1800,{2,.4,.15},.6,0,2);auto& w=*rig.world;
 w.reset_vehicle(rig.vehicle,{.gear=0,.engine_state=EngineState::Off});w.set_control("gear_request",0);w.set_control("throttle",0);w.set_control("assist.drive_mode",1);
 bool started=false,charged=false;real max_speed=0;
 for(int i=0;i<7200;++i){w.step();const auto p=w.powertrain_state(rig.vehicle);started|=p.engine_state==EngineState::Running;charged|=p.battery_soc>=.8-1e-6;const auto motion=w.get_motion(rig.chassis).linear;max_speed=std::max(max_speed,std::hypot(motion.x,motion.y));}
 auto p=w.powertrain_state(rig.vehicle);
 INFO("end SOC="<<p.battery_soc<<" rpm="<<p.engine_rpm<<" charging="<<p.hybrid_charging);
 CHECK(started);CHECK(charged);CHECK_FALSE(p.hybrid_charging);CHECK(p.engine_state==EngineState::Off);CHECK(max_speed<.1);
}
TEST_CASE("Hybrid rejects charging without generator hardware and stops ICE after leaving Sport", "[ev][hybrid_control]") {
 auto unsupported=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_sedan.json");
 unsupported.powertrain.components.push_back({"hybrid",HybridControlDesc{}});
 CHECK_THROWS(ps_test::build_vehicle_rig_from_desc(unsupported,1800,{2,.4,.15},.6,0,1));
 auto d=io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hybrid.json");
 auto rig=ps_test::build_vehicle_rig_from_desc(d,1800,{2,.4,.15},.6,0,1);auto& w=*rig.world;
 w.set_control("assist.drive_mode",2);w.set_control("throttle",0);w.set_control("gear_request",0);
 for(int i=0;i<960;++i)w.step();CHECK(w.powertrain_state(rig.vehicle).engine_state==EngineState::Running);
 w.set_control("assist.drive_mode",0);for(int i=0;i<1200;++i)w.step();
 CHECK(w.powertrain_state(rig.vehicle).engine_state==EngineState::Off);
}
