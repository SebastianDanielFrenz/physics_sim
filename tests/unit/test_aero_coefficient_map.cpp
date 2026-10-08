#include "ps/aero/aerodynamics.h"
#include "ps/math/transcendental.h"
#include "ps/world/world.h"
#include "ps/io/aero_coefficient_map_io.h"
#include "ps/jobs/debug_alloc.h"
#include "../../core/src/io/aero_io.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
using namespace ps;
namespace {
aero::CoefficientData fixture(bool optional=false) {
    aero::CoefficientData d;d.axes[0]={-180,-90,0,90,180};d.axes[1]={-90,0,90};d.axes[2]={0,40};
    if(optional){d.axes[3]={.1,.5};d.axes[4]={0,20};}
    d.area_m2=2;d.lengths_m={2,3,4};d.reference_point_local={1,0,0};
    d.geometry_sha256=std::string(64,'a');d.solver="test fixture, not CFD";d.source="synthetic test only";
    for(real yaw:d.axes[0])for(real pitch:d.axes[1])for(real speed:d.axes[2])
        for(real height:optional?d.axes[3]:std::vector<real>{0})for(real offset:optional?d.axes[4]:std::vector<real>{0}) {
            real x=0,y=0,z=0;
            if(pitch==-90)z=1;else if(pitch==90)z=-1;else {
                if(yaw==0)x=-1;if(yaw==-180||yaw==180)x=1;if(yaw==-90)y=1;if(yaw==90)y=-1;
            }
            d.rows.push_back({x,y,z-speed*.001-height-offset*.01,.1,.2,.3});
        }
    return d;
}
aero::Desc desc(bool optional=false) {aero::Desc d;d.coefficient_map.table=aero::CoefficientMap::create(fixture(optional));return d;}
aero::Telemetry run(const aero::Desc& d,Vec3 v) {aero::State s;return aero::step(d,s,{},Motion{v,{}},{},.01);}
void same(Vec3 a,Vec3 b) {CHECK((a-b).length()<1e-8);}
nlohmann::json json_fixture() {
    auto d=fixture();return {{"schema","physics_sim.aero_coefficients/1"},{"frame","ISO_BODY_X_FORWARD_Y_LEFT_Z_UP"},{"units","SI_DEGREES_DIMENSIONLESS_COEFFICIENTS"},
      {"reference",{{"area_m2",d.area_m2},{"lengths_m",{2,3,4}},{"point_local_m",{1,0,0}}}},
      {"provenance",{{"geometry_sha256",d.geometry_sha256},{"solver",d.solver},{"source",d.source}}},
      {"axes",{{"yaw_deg",d.axes[0]},{"pitch_deg",d.axes[1]},{"speed_m_s",d.axes[2]}}},{"coefficients",d.rows}};
}
}
TEST_CASE("CFD table multilinear all dimensions seam clamp poles validation","[aero][aero_map]") {
    auto raw=fixture(true);auto map=aero::CoefficientMap::create(raw);
    auto sample=map->evaluate({45,0,20,.3,10});CHECK(sample.coefficients[0]==Catch::Approx(-.5));CHECK(sample.coefficients[1]==Catch::Approx(-.5));CHECK(sample.coefficients[2]==Catch::Approx(-.42));CHECK(sample.clamped_axes==0);
    CHECK(map->evaluate({-180,0,20,.3,10}).coefficients==map->evaluate({180,0,20,.3,10}).coefficients);
    CHECK(map->evaluate({540,0,20,.3,10}).coefficients==map->evaluate({-180,0,20,.3,10}).coefficients);
    CHECK(map->evaluate({0,0,100,0,30}).clamped_axes==28);
    same(run(desc(),{}).wrench_world.force,{});same(run(desc(),{}).wrench_world.torque,{});
    SECTION("bad rows"){raw.rows.pop_back();CHECK_THROWS(aero::CoefficientMap::create(raw));}
    SECTION("duplicate axis"){raw.axes[2]={0,0};CHECK_THROWS(aero::CoefficientMap::create(raw));}
    SECTION("seam"){raw.rows.back()[0]+=1;CHECK_THROWS(aero::CoefficientMap::create(raw));}
    SECTION("pole"){raw.rows[24][0]+=1;CHECK_THROWS(aero::CoefficientMap::create(raw));}
    SECTION("nonfinite"){raw.area_m2=std::numeric_limits<real>::infinity();CHECK_THROWS(aero::CoefficientMap::create(raw));}
    SECTION("nonfinite coefficient"){raw.rows[8][0]=std::numeric_limits<real>::quiet_NaN();CHECK_THROWS(aero::CoefficientMap::create(raw));}
    SECTION("negative speed"){raw.axes[2][0]=-1;CHECK_THROWS(aero::CoefficientMap::create(raw));}
}
TEST_CASE("CFD table signed body forces moments wake wind rotation and full yaw","[aero][aero_map]") {
    auto d=desc();const auto f=run(d,{20,0,0});const real q=.5*1.225*400;
    same(f.wrench_world.force,{-2*q,0,-.04*q});same(f.wrench_world.torque,{.4*q,1.24*q,2.4*q});
    CHECK(run(d,{-20,0,0}).wrench_world.force.x>0);CHECK(run(d,{0,20,0}).wrench_world.force.y<0);CHECK(run(d,{0,0,20}).wrench_world.force.z<0);
    aero::State state;auto inv=Quat::from_axis_angle(Vec3::unit_x(),3.141592653589793);
    auto upside=aero::step(d,state,Pose{{},inv},Motion{inv.rotate({20,0,0}),{}},{},.01);same(upside.wrench_world.force,inv.rotate(f.wrench_world.force));same(upside.wrench_world.torque,inv.rotate(f.wrench_world.torque));
    aero::Environment e;e.wind_world={-20,0,0};same(aero::step(d,state,{},Motion{},e,.01).wrench_world.force,f.wrench_world.force);
    aero::WakeSource wake{{1,1},{0,0,0},{30,0,0},{40,2,.1,.4,.2}};
    auto draft=aero::step(d,state,Pose{{-6,0,0},{}},Motion{{20,0,0},{}},{},.01,{},std::span(&wake,1));CHECK(draft.body_airspeed_m_s<20);CHECK(std::abs(draft.wrench_world.force.x)<std::abs(f.wrench_world.force.x));
    for(int angle=-180;angle<=180;++angle){real a=angle*3.141592653589793/180;Vec3 v{20*math::cos(a),20*math::sin(a),0};auto t=run(d,v);CHECK(std::isfinite(t.wrench_world.force.length_squared()));CHECK(dot(t.wrench_world.force,v)<=1e-7);}
}
TEST_CASE("CFD ownership actual wing offset real ground fallback and fan accounting","[aero][aero_map]") {
    auto d=desc(true);d.body.drag_area_xyz_m2={5,5,5};d.coefficient_map.ground_range_m=2;
    aero::SurfaceDesc wing;wing.name="wing";wing.cl0=-1;wing.max_offset_rad=20*3.141592653589793/180;wing.rate_rad_s=.1;d.surfaces.push_back(wing);
    aero::FanDesc fan;fan.name="fan";fan.rate_per_s=100;d.fans.push_back(fan);aero::validate(d);
    aero::State state;state.surface_command[0]=1;state.fan_command[0]=1;state.fan_energy_j=100;
    std::array<aero::GroundSample,3> ground{{{}, {true,0}, {true,.3}}};
    d.coefficient_map.scope=aero::MapScope::ReplacePassiveAero;
    auto full=aero::step(d,state,{},Motion{{20,0,0},{}},{},.01,ground);CHECK(full.coefficient_map_active);CHECK(full.coefficient_map_inputs[4]==Catch::Approx(.001*180/3.141592653589793));same(full.surfaces[0].wrench_world.force,{});CHECK(full.fan_energy_used_j==100);CHECK(full.fans[0].force_world.z<0);
    state={};auto fallback=aero::step(d,state,{},Motion{{20,0,0},{}},{},.01);CHECK(fallback.coefficient_map_ground_missing);CHECK_FALSE(fallback.coefficient_map_active);CHECK(fallback.surfaces[0].wrench_world.force.z<0);CHECK(fallback.body_force_local.x==Catch::Approx(-1225));
    state={};d.coefficient_map.scope=aero::MapScope::ReplaceBody;auto body=aero::step(d,state,{},Motion{{20,0,0},{}},{},.01,ground);CHECK(body.surfaces[0].wrench_world.force.z<0);
}
TEST_CASE("CFD file schema duplicate keys missing dimensions and ownership validation","[aero][aero_map]") {
    auto dir=std::filesystem::temp_directory_path()/"ps_aero_map_unit";std::filesystem::create_directories(dir);auto path=dir/"map.json";
    auto ref=nlohmann::json{{"coefficient_map",{{"file","map.json"},{"scope","replace_body"}}}};
    auto load=[&](nlohmann::json j){std::ofstream(path)<<j.dump();return io::read_aero(ref,dir.string());};
    auto j=json_fixture();CHECK(load(j).coefficient_map.table!=nullptr);
    SECTION("units"){j["units"]="kph";CHECK_THROWS(load(j));}
    SECTION("dimension"){j["axes"].erase("speed_m_s");CHECK_THROWS(load(j));}
    SECTION("unknown dimension"){j["axes"]["flap_deg"]={0};CHECK_THROWS(load(j));}
    SECTION("provenance"){j.erase("provenance");CHECK_THROWS(load(j));}
    SECTION("row width"){j["coefficients"][0].push_back(0);CHECK_THROWS(load(j));}
    SECTION("duplicate key"){auto text=j.dump();text.insert(1,"\"schema\":\"bad\",");std::ofstream(path)<<text;CHECK_THROWS(io::read_aero(ref,dir.string()));}
    SECTION("scope"){ref["coefficient_map"]["scope"]="add";CHECK_THROWS(load(j));}
    SECTION("missing file"){ref["coefficient_map"]["file"]="absent";CHECK_THROWS(load(j));}
    SECTION("public selection binding") {
        std::ofstream(path)<<json_fixture().dump();auto selection=dir/"selection.json";
        aero::Desc authored;authored.drag_area_m2=.5;
        nlohmann::json select={{"enabled",true},{"coefficient_map",ref.at("coefficient_map")}};
        std::ofstream(selection)<<select.dump();io::load_aero_coefficient_map_reference(selection.string(),authored);
        CHECK(authored.coefficient_map.table!=nullptr);CHECK(authored.drag_area_m2==.5);
        auto before=authored.coefficient_map.table;select["coefficient_map"]["scope"]="bad";std::ofstream(selection)<<select.dump();
        CHECK_THROWS(io::load_aero_coefficient_map_reference(selection.string(),authored));CHECK(authored.coefficient_map.table==before);
        std::ofstream(selection)<<"{\"enabled\":false}";io::load_aero_coefficient_map_reference(selection.string(),authored);
        CHECK_FALSE(authored.coefficient_map.table);CHECK(authored.drag_area_m2==.5);
    }
    std::filesystem::remove_all(dir);
}
TEST_CASE("CFD map hash reset and worker determinism allocation free","[aero][aero_map][determinism][debug_alloc]") {
    auto d=desc();aero::State state;auto h=aero::fold_state_hash(d,state,42);auto changed=fixture();changed.rows[8][2]-=.01;auto d2=d;d2.coefficient_map.table=aero::CoefficientMap::create(changed);CHECK(aero::fold_state_hash(d2,state,42)!=h);
    aero::SurfaceDesc actuator;actuator.name="actuator";actuator.max_offset_rad=.2;actuator.rate_rad_s=1;actuator.cl0=0;actuator.cd0=0;actuator.lift_slope_per_rad=0;d.surfaces.push_back(actuator);
    auto run_world=[&](int workers) {
        WorldConfig cfg;cfg.job_workers=workers;cfg.gravity={};World world(cfg);vehicle::VehicleDesc vehicle;vehicle.name="map";vehicle.aero=d;
        BodyDesc body;body.mass=1000;body.motion=BodyMotionType::Dynamic;body.shape=BoxShape{{1,.5,.2}};body.allow_sleep=false;auto b=world.create_body(body);auto id=world.create_vehicle(vehicle,b);world.backend().set_motion(b,Motion{{20,0,0},{}});world.set_aero_surface_command(id,0,1);
        for(int i=0;i<10;++i)world.step();std::array<std::uint64_t,30> hashes{};
        const bool armed=jobs::debug_alloc::hook_available();jobs::debug_alloc::reset();jobs::debug_alloc::set_armed(armed);
        for(auto& hash:hashes){world.step();hash=world.state_hash();}jobs::debug_alloc::set_armed(false);if(armed)CHECK(jobs::debug_alloc::own_violation_count()==0);
        CHECK(world.vehicle_aero_telemetry(id).coefficient_map_active);
        CHECK(world.vehicle_aero_telemetry(id).surfaces[0].offset_rad>0);
        vehicle::VehicleResetOptions opts;world.reset_vehicle(id,opts);world.step();CHECK(world.vehicle_aero_telemetry(id).coefficient_map_active);CHECK(world.vehicle_aero_telemetry(id).surfaces[0].offset_rad==0);return hashes;
    };CHECK(run_world(1)==run_world(4));
}

namespace {
aero::CoefficientData lift_fixture(bool pitch_axis=true) {
    auto d=fixture(pitch_axis);
    d.axes[5]={0,.28};
    const auto old=d.rows;d.rows.clear();
    for(const auto& row:old)for(real lift:d.axes[5]) {
        auto expanded=row;expanded[2]-=lift*2;d.rows.push_back(expanded);
    }
    return d;
}
}
TEST_CASE("CFD wing lift interpolates independently and validates optional sixth axis","[aero][aero_map][wing_lift]") {
    auto d=lift_fixture();auto map=aero::CoefficientMap::create(d);
    const auto low=map->evaluate({45,0,20,.3,10,0});
    const auto middle=map->evaluate({45,0,20,.3,10,.14});
    CHECK(middle.coefficients[2]==Catch::Approx(low.coefficients[2]-.28));
    CHECK(middle.clamped_axes==0);
    const auto pitched=map->evaluate({45,0,20,.3,20,.14});
    CHECK(pitched.coefficients[2]==Catch::Approx(middle.coefficients[2]-.1));
    CHECK(map->evaluate({45,0,20,.3,10,.4}).clamped_axes==32);
    CHECK(map->evaluate({45,0,20,.3,10,-.1}).clamped_axes==32);
    CHECK(map->evaluate(std::array<real,5>{45,0,20,.3,10}).coefficients==low.coefficients);
    CHECK(aero::CoefficientMap::create(fixture(false))->content_hash()==0xa2e69c3d4422760cULL); // legacy five-axis byte-layout digest, independently calculated
    auto absent=fixture(true);CHECK(aero::CoefficientMap::create(absent)->content_hash()!=map->content_hash());
    auto changed=d;changed.axes[5][1]=.3;CHECK(aero::CoefficientMap::create(changed)->content_hash()!=map->content_hash());
    changed=d;changed.axes[5][0]=-.1;CHECK_THROWS(aero::CoefficientMap::create(changed));
    changed=d;changed.axes[5][1]=0;CHECK_THROWS(aero::CoefficientMap::create(changed));
    changed=d;changed.rows.pop_back();CHECK_THROWS(aero::CoefficientMap::create(changed));
    CHECK(aero::CoefficientMap::create(lift_fixture(false))->evaluate({0,0,20,0,0,.14}).clamped_axes==0);
}
TEST_CASE("CFD wing lift samples actual transient before passive suppression","[aero][aero_map][wing_lift]") {
    aero::Desc d;d.coefficient_map.table=aero::CoefficientMap::create(lift_fixture(false));
    d.coefficient_map.scope=aero::MapScope::ReplacePassiveAero;
    aero::SurfaceDesc wing;wing.name="rear_wing";wing.max_offset_rad=.2;wing.rate_rad_s=.1;wing.max_lift_m=.28;
    d.surfaces.push_back(wing);aero::validate(d);
    aero::State state;state.surface_command[0]=1;
    const auto t=aero::step(d,state,{},Motion{{20,0,0},{}},{},.1);
    CHECK(state.offset_rad[0]==Catch::Approx(.01));
    CHECK(state.lift_m[0]==Catch::Approx(.014));
    CHECK(t.coefficient_map_inputs[5]==state.lift_m[0]);
    CHECK(t.surfaces[0].lift_m==state.lift_m[0]);
    CHECK(t.coefficient_map_coefficients[2]==Catch::Approx(-.048));
    CHECK(t.surfaces[0].wrench_world.force.length()==0);
    auto altered=state;altered.lift_m[0]+=.001;
    CHECK(aero::fold_state_hash(d,state,42)!=aero::fold_state_hash(d,altered,42));
    auto legacy=d;legacy.coefficient_map.table=aero::CoefficientMap::create(fixture(false));
    CHECK(aero::fold_state_hash(legacy,state,42)==aero::fold_state_hash(legacy,altered,42));
    d.coefficient_map.wing_surface=1;CHECK_THROWS(aero::validate(d));
    state={};CHECK(state.lift_m[0]==0);
}
TEST_CASE("CFD JSON lift-only axis requires binding and dense rows","[aero][aero_map][wing_lift]") {
    auto dir=std::filesystem::temp_directory_path()/"ps_lift_map_unit";std::filesystem::create_directories(dir);
    auto file=dir/"lift.json";auto j=json_fixture();auto data=lift_fixture(false);
    j["axes"]["wing_lift_m"]=data.axes[5];j["coefficients"]=data.rows;
    std::ofstream(file)<<j.dump();
    nlohmann::json ref={{"surfaces",nlohmann::json::array({{{"name","rear_wing"},{"max_offset_rad",.2},{"max_lift_m",.28}}})},
        {"coefficient_map",{{"file","lift.json"},{"scope","replace_passive_aero"},{"wing_surface","rear_wing"}}}};
    auto loaded=io::read_aero(ref,dir.string());CHECK(loaded.coefficient_map.table->data().axes[5]==data.axes[5]);
    ref["coefficient_map"].erase("wing_surface");CHECK_THROWS(io::read_aero(ref,dir.string()));
    ref["coefficient_map"]["wing_surface"]="rear_wing";
    j["axes"]["wing_lift_m"]=nlohmann::json::array();std::ofstream(file)<<j.dump();CHECK_THROWS(io::read_aero(ref,dir.string()));
    std::filesystem::remove_all(dir);
}

TEST_CASE("CFD lift map world snapshots reset and worker parity","[aero][aero_map][wing_lift][determinism]") {
    const auto run=[](int workers) {
        WorldConfig cfg;cfg.job_workers=workers;cfg.gravity={};World world(cfg);
        vehicle::VehicleDesc car;car.name="lift_map";car.aero.coefficient_map.table=aero::CoefficientMap::create(lift_fixture(false));
        car.aero.coefficient_map.scope=aero::MapScope::ReplacePassiveAero;
        aero::SurfaceDesc wing;wing.name="wing";wing.max_offset_rad=.2;wing.rate_rad_s=.1;wing.max_lift_m=.28;
        car.aero.surfaces.push_back(wing);
        BodyDesc body;body.mass=1000;body.motion=BodyMotionType::Dynamic;body.shape=BoxShape{{1,.5,.2}};body.allow_sleep=false;
        const auto b=world.create_body(body);const auto id=world.create_vehicle(car,b);
        world.backend().set_motion(b,Motion{{20,0,0},{}});world.set_aero_surface_command(id,0,1);
        for(int i=0;i<10;++i)world.step(); std::array<std::uint64_t,20> hashes{};
        const bool armed=jobs::debug_alloc::hook_available();jobs::debug_alloc::reset();jobs::debug_alloc::set_armed(armed);
        for(auto& hash:hashes){world.step();hash=world.state_hash();}
        jobs::debug_alloc::set_armed(false);if(armed)CHECK(jobs::debug_alloc::own_violation_count()==0);
        const auto t=world.vehicle_aero_telemetry(id);
        CHECK(t.coefficient_map_inputs[5]>0);CHECK(t.coefficient_map_inputs[5]==t.surfaces[0].lift_m);
        world.reset_vehicle(id,{});world.step();CHECK(world.vehicle_aero_telemetry(id).coefficient_map_inputs[5]==0);
        return hashes;
    };CHECK(run(1)==run(4));
}
