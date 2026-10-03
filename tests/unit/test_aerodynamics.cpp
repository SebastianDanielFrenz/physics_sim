#include "ps/aero/aerodynamics.h"
#include "ps/world/world.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/io/vehicle_io.h"
#include "../../core/src/io/aero_io.h"
#include "vehicle_test_helpers.h"
#include <nlohmann/json.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <fstream>
#include <algorithm>
using namespace ps;
namespace {
aero::Desc wing(real cl0=1) {
    aero::Desc d; aero::SurfaceDesc s; s.name="wing"; s.cl0=cl0; d.surfaces.push_back(s); return d;
}
aero::Telemetry evaluate(aero::Desc d,Vec3 speed,std::span<const aero::GroundSample> ground={}) {
    aero::State state; return aero::step(d,state,{},Motion{speed,{}},{},.01,ground);
}
void same_vector(Vec3 a,Vec3 b,real tolerance=1e-8) { CHECK((a-b).length()<tolerance); }
BodyId chassis(World& world,Vec3 pos={}) {
    BodyDesc b; b.motion=BodyMotionType::Dynamic; b.mass=1000; b.shape=BoxShape{{1,.5,.2}};
    b.pose.position=pos; b.allow_sleep=false; b.linear_damping=0; b.angular_damping=0;
    return world.create_body(b);
}
}
TEST_CASE("Aero zero speed wind v squared signed lift and offset moments","[aero]") {
    auto d=wing(); auto zero=evaluate(d,{}); same_vector(zero.wrench_world.force,{});
    auto a=evaluate(d,{20,0,0}), b=evaluate(d,{40,0,0});
    CHECK(a.wrench_world.force.z>0); same_vector(b.wrench_world.force,a.wrench_world.force*4);
    d.surfaces[0].position_local={2,0,0}; auto offset=evaluate(d,{20,0,0});
    CHECK(offset.wrench_world.torque.y==Catch::Approx(-2*offset.wrench_world.force.z));
    d.surfaces[0].cl0=-1; CHECK(evaluate(d,{20,0,0}).wrench_world.force.z<0);
    aero::State s; aero::Environment e; e.wind_world={-20,0,0};
    const auto wind=aero::step(wing(),s,{},Motion{},e,.01); same_vector(wind.wrench_world.force,a.wrench_world.force);
    e.air_density=0; same_vector(aero::step(d,s,{},Motion{{20,0,0},{}},e,.01).wrench_world.force,{});
}
TEST_CASE("Aero stall reverse spanwise angular airflow and rotation covariance","[aero]") {
    auto d=wing(0); const real pi=3.141592653589793;
    auto small=evaluate(d,{20,0,-3}), stalled=evaluate(d,{20,0,-20});
    CHECK(stalled.surfaces[0].cl<small.surfaces[0].cl*2);
    CHECK(stalled.surfaces[0].cd>small.surfaces[0].cd);
    auto reverse=evaluate(d,{-20,0,0}); CHECK(std::abs(reverse.surfaces[0].cl)<1e-8);
    auto side=evaluate(d,{0,20,0}); CHECK(std::abs(side.surfaces[0].cl)<1e-8);
    d.surfaces[0].position_local={2,1,0}; aero::State s;
    auto spinning=aero::step(d,s,{},Motion{{20,0,0},{0,1,0}},{},.01);
    CHECK(std::abs(spinning.surfaces[0].alpha_rad)>0);
    auto upright=evaluate(d,{20,0,-3}); const auto q=Quat::from_axis_angle(Vec3::unit_x(),pi);
    auto inverted=aero::step(d,s,Pose{{},q},Motion{q.rotate({20,0,-3}),{}},{},.01);
    same_vector(inverted.wrench_world.force,q.rotate(upright.wrench_world.force));
    same_vector(inverted.wrench_world.torque,q.rotate(upright.wrench_world.torque));
}
TEST_CASE("Aero ground effect bounded fading and missing ground is safe","[aero]") {
    auto d=wing(-1); d.surfaces[0].ground.range_m=1; d.surfaces[0].ground.gain=2;
    std::array<aero::GroundSample,1> g{{{true,0}}};
    auto close=evaluate(d,{20,0,0},g); CHECK(close.surfaces[0].ground_multiplier==3);
    g[0].clearance_m=.5; auto medium=evaluate(d,{20,0,0},g); CHECK(medium.surfaces[0].ground_multiplier==1.5);
    g[0].clearance_m=2; auto far=evaluate(d,{20,0,0},g); CHECK(far.surfaces[0].ground_multiplier==1);
    g[0].hit=false; same_vector(evaluate(d,{20,0,0},g).wrench_world.force,far.wrench_world.force);
    CHECK(close.wrench_world.force.z<medium.wrench_world.force.z);
}
TEST_CASE("Aero active wing rate limited continuous geometry and coupled drag","[aero]") {
    auto d=wing(-.2); auto& w=d.surfaces[0]; w.max_offset_rad=1; w.min_offset_rad=-.2; w.rate_rad_s=.5; w.max_lift_m=.28;
    aero::State state; state.surface_command[0]=1;
    auto a=aero::step(d,state,{},Motion{{30,0,0},{}},{},.01);
    CHECK(a.surfaces[0].offset_rad==Catch::Approx(.005)); CHECK(a.surfaces[0].lift_m==Catch::Approx(.0014));
    auto initial_cd=a.surfaces[0].cd;
    for(int i=0;i<300;++i) a=aero::step(d,state,{},Motion{{30,0,0},{}},{},.01);
    CHECK(a.surfaces[0].offset_rad==1); CHECK(a.surfaces[0].cd>initial_cd);
    state.surface_command[0]=-1; a=aero::step(d,state,{},Motion{{30,0,0},{}},{},.01);
    CHECK(a.surfaces[0].offset_rad==Catch::Approx(.995));
}
TEST_CASE("Aero moving car truck wakes directional finite bounded and deterministic","[aero]") {
    aero::Environment env;
    aero::WakeSource car{{1,1},{0,0,0},{30,0,0},{30,1,.12,.3,.1}};
    aero::WakeSource truck{{2,1},{0,0,0},{30,0,0},{60,2,.15,.45,.2}};
    std::array<aero::WakeSource,2> wakes{car,truck};
    auto behind=aero::sample_air(env,{-5,0,0},{},wakes); CHECK(behind.wind_world.x>0); CHECK(behind.turbulence>0);
    same_vector(aero::sample_air(env,{5,0,0},{},wakes).wind_world,{});
    same_vector(aero::sample_air(env,{-5,20,0},{},wakes).wind_world,{});
    same_vector(aero::sample_air(env,{-70,0,0},{},wakes).wind_world,{});
    const auto repeated=aero::sample_air(env,{-5,0,0},{},wakes); same_vector(repeated.wind_world,behind.wind_world,1e-12);
    std::array<aero::WakeSource,20> many{}; many.fill(truck);
    CHECK(aero::sample_air(env,{-3,0,0},{},many).wind_world.x<=22.5+1e-10);
    truck.velocity_world={-30,0,0}; CHECK(aero::sample_air(env,{5,0,0},{},std::span(&truck,1)).wind_world.x<0);
    same_vector(aero::sample_air(env,{5,0,0},truck.owner,std::span(&truck,1)).wind_world,{});
    auto d=wing(); aero::State state;
    auto solo=aero::step(d,state,{},Motion{{30,0,0},{}},env,.01);
    auto draft=aero::step(d,state,Pose{{-5,0,0},{}},Motion{{30,0,0},{}},env,.01,{},wakes);
    CHECK(draft.body_airspeed_m_s<solo.body_airspeed_m_s);
    CHECK(std::abs(draft.wrench_world.force.x)<std::abs(solo.wrench_world.force.x));
    CHECK(draft.wrench_world.force.z<solo.wrench_world.force.z);
}
TEST_CASE("Aero suction fan zero speed seal leakage prepaid power and inversion","[aero]") {
    aero::Desc d; aero::FanDesc fan; fan.name="suction"; fan.max_power_w=1000; fan.rate_per_s=10; d.fans.push_back(fan);
    aero::State state; state.fan_command[0]=1;
    std::array<aero::GroundSample,1> g{{{true,0}}};
    auto empty=aero::step(d,state,{},Motion{},{},.1,g); CHECK(empty.fan_power_w==0); same_vector(empty.wrench_world.force,{});
    state.fan_energy_j=150;
    auto paid=aero::step(d,state,{},Motion{},{},.1,g);
    CHECK(paid.fan_power_w==1000); CHECK(paid.wrench_world.force.z==-1000); CHECK(state.fan_energy_j==50);
    auto partial=aero::step(d,state,{},Motion{},{},.1,g); CHECK(partial.fan_power_w==500); CHECK(state.fan_energy_j==0);
    CHECK(partial.wrench_world.force.z==-500);
    state.fan_energy_j=1000; g[0].clearance_m=.2; CHECK(aero::step(d,state,{},Motion{},{},.1,g).wrench_world.force.z==0);
    g[0].clearance_m=0; const auto q=Quat::from_axis_angle(Vec3::unit_x(),3.141592653589793);
    auto inverted=aero::step(d,state,Pose{{},q},Motion{},{},.1,g); CHECK(inverted.wrench_world.force.z>999);
    g[0].hit=false; auto miss=aero::step(d,state,{},Motion{},{},.1,g); same_vector(miss.wrench_world.force,{}); CHECK(miss.fan_energy_used_j>0);
}
TEST_CASE("Aero native and JSON validation legacy compatibility","[aero][vehicle_io]") {
    auto legacy=ps::io::read_aero(nlohmann::json{{"drag_area_m2",.7}}); auto t=evaluate(legacy,{20,0,0});
    CHECK(t.wrench_world.force.x==Catch::Approx(-.5*1.225*.7*400));
    CHECK_THROWS(ps::io::read_aero(nlohmann::json{{"typo",1}}));
    CHECK_THROWS(ps::io::read_aero(nlohmann::json{{"surfaces",nlohmann::json::array({{{"name","bad"},{"area_m2",-1}}})}}));
    auto d=wing(); d.drag_area_m2=.7; CHECK_THROWS(aero::validate(d));
    d.drag_area_m2=0; d.surfaces[0].normal_local={1,0,0}; CHECK_THROWS(aero::validate(d));
    CHECK_THROWS(aero::validate(aero::Environment{-1,{}}));
    CHECK_THROWS(aero::validate(aero::Environment{1,{std::numeric_limits<real>::infinity(),0,0}}));
    // Load a small vehicle fixture, avoiding unrelated engine-map generation in these focused tests.
    auto sedan=ps::io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_sedan.json"); CHECK(sedan.aero.drag_area_m2>0);
}
TEST_CASE("Aero World wind shadow fan ground exclusion and inverted overhead probe","[aero]") {
    WorldConfig cfg; cfg.gravity={}; cfg.job_workers=1; World world(cfg);
    auto body=chassis(world,{0,0,2}); vehicle::VehicleDesc desc; desc.name="aircraft"; desc.aero=wing();
    desc.aero.surfaces[0].position_local={.5,0,0}; auto id=world.create_vehicle(desc,body);
    world.set_aero_environment({1.225,{-20,0,0}});
    for(int i=0;i<10;++i) world.step();
    CHECK(world.get_motion(body).linear.z>0);
    CHECK(world.vehicle_shadow_dv_error(id)<1e-5); CHECK(world.vehicle_shadow_dw_error(id)<1e-5);
    CHECK(world.vehicle_aero_telemetry(id).body_airspeed_m_s>19);
    CHECK_THROWS(world.set_aero_surface_command(id,9,1));
    // A fan inside the chassis must probe THROUGH self to an overhead road when inverted.
    World fan_world(cfg); auto fan_body=chassis(fan_world,{0,0,2});
    BodyDesc ceiling; ceiling.shape=BoxShape{{5,5,.1}}; ceiling.pose.position={0,0,2.15}; ceiling.motion=BodyMotionType::Static;
    // Use a thin chassis to avoid chassis/ceiling collision, so only suction contributes.
    fan_world.destroy_body(fan_body);
    BodyDesc thin; thin.motion=BodyMotionType::Dynamic; thin.mass=1000; thin.shape=BoxShape{{1,.5,.01}};
    thin.pose.position={0,0,2}; thin.pose.orientation=Quat::from_axis_angle(Vec3::unit_x(),3.141592653589793); thin.allow_sleep=false;
    fan_body=fan_world.create_body(thin); fan_world.create_body(ceiling);
    vehicle::VehicleDesc fd; fd.name="fan"; aero::FanDesc f; f.name="suction"; f.seal_height_m=.2; f.rate_per_s=1000; fd.aero.fans.push_back(f);
    auto fid=fan_world.create_vehicle(fd,fan_body); fan_world.set_aero_fan_command(fid,0,1); fan_world.credit_aero_fan_energy(fid,100);
    fan_world.step(); CHECK(fan_world.vehicle_aero_telemetry(fid).wrench_world.force.z>0);
    CHECK(fan_world.vehicle_aero_telemetry(fid).fan_energy_used_j>0);
}
TEST_CASE("Aero multiple vehicle wake and actuator exact worker parity","[aero][determinism]") {
    auto run=[](int workers) {
        WorldConfig cfg; cfg.gravity={}; cfg.job_workers=workers; World world(cfg);
        for(int i=0;i<3;++i) {
            auto b=chassis(world,{-real(i)*6,0,10}); vehicle::VehicleDesc d; d.name="wake-car"; d.aero=wing(-.5);
            d.aero.wake={30,1,.12,.3,.1}; d.aero.surfaces[0].max_offset_rad=.5;
            auto v=world.create_vehicle(d,b); world.backend().set_motion(b,Motion{{25,0,0},{}}); world.set_aero_surface_command(v,0,.5);
        }
        std::vector<std::uint64_t> hashes;
        for(int i=0;i<120;++i) { world.step(); hashes.push_back(world.state_hash()); }
        return hashes;
    };
    CHECK(run(1)==run(4));
}
TEST_CASE("Aero downforce feeds actual suspension wheel loads","[aero][vehicle]") {
    auto d=ps::io::load_vehicle_json(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_sedan.json");
    d.aero=wing(-1); d.aero.surfaces[0].area_m2=3;
    auto rig=ps_test::build_vehicle_rig_from_desc(d,1500,{2,.4,.15},.6,0,1);
    ps_test::settle(*rig.world,1); rig.world->set_aero_environment({1.225,{-30,0,0}});
    ps_test::settle(*rig.world,1);
    real load=0; for(std::size_t i=0;i<4;++i) load+=rig.world->wheel_telemetry(rig.vehicle,i).fz;
    CHECK(load>1500*9.81+500);
    const auto& telemetry=rig.world->vehicle_aero_telemetry(rig.vehicle);
    CHECK(telemetry.axle_balance_valid);
    CHECK(telemetry.front_balance>0); CHECK(telemetry.front_balance<1);
}

TEST_CASE("Aero legacy drag keeps world-space arithmetic at arbitrary orientation","[aero]") {
    aero::Desc d; d.drag_area_m2=.62; aero::State state;
    const Vec3 v{20,3,-2}; const auto rotation=Quat::from_axis_angle(Vec3{1,2,3},.73);
    auto t=aero::step(d,state,Pose{{},rotation},Motion{v,{}},{},.01);
    CHECK(t.wrench_world.force==v*(-real(.5)*real(1.225)*d.drag_area_m2*v.length()));
    same_vector(t.wrench_world.torque,{});
}
TEST_CASE("Aero authored hypercar configuration is valid and art-aligned","[aero]") {
    std::ifstream file(std::string(PS_SOURCE_DIR)+"/data/vehicles/car_hyper.json");
    nlohmann::json j; file>>j; auto d=ps::io::read_aero(j.at("aero"));
    REQUIRE(d.surfaces.size()==3); CHECK(d.surfaces[2].name=="rear_wing"); CHECK(d.fans.empty()); CHECK(d.drag_area_m2==0);
    CHECK(d.surfaces[2].position_local.x==Catch::Approx(-1.714));
    CHECK(d.surfaces[2].position_local.z==Catch::Approx(.475));
    auto calm=evaluate(d,{30,0,0}); CHECK(calm.downforce_n>700); CHECK(calm.wrench_world.force.x<0);
    aero::State state; state.surface_command[2]=1; aero::Telemetry brake;
    for(int i=0;i<200;++i) brake=aero::step(d,state,{},Motion{{30,0,0},{}},{},.01);
    CHECK(brake.surfaces[2].lift_m==Catch::Approx(.28)); CHECK(brake.surfaces[2].offset_rad==Catch::Approx(55*3.141592653589793/180));
    CHECK(brake.surfaces[2].cd>calm.surfaces[2].cd);
}
TEST_CASE("Aero wheelless aircraft publishes aero telemetry in World snapshot","[aero][snapshot]") {
    WorldConfig cfg; cfg.job_workers=1; cfg.gravity={}; World world(cfg);
    auto b=chassis(world,{0,0,10}); vehicle::VehicleDesc d; d.name="plane"; d.aero=wing(1);
    auto v=world.create_vehicle(d,b); world.backend().set_motion(b,Motion{{30,0,0},{}}); world.step();
    const auto* snapshot=&world.snapshots().read();
    REQUIRE(snapshot!=nullptr); REQUIRE(snapshot->vehicles.size()==1); CHECK(snapshot->vehicles[0].wheels.empty());
    CHECK(snapshot->vehicles[0].aero.wrench_world.force.z>0);
    CHECK(snapshot->vehicles[0].aero.wrench_world.force==world.vehicle_aero_telemetry(v).wrench_world.force);
}

TEST_CASE("Aero drag remains dissipative across full stall and reverse angle sweep","[aero]") {
    auto d=wing(.3); const real pi=3.141592653589793;
    for(int degrees=-180;degrees<=180;++degrees) {
        const real alpha=real(degrees)*pi/180;
        Vec3 velocity{30*ps::math::cos(alpha),4,30*ps::math::sin(alpha)};
        auto t=evaluate(d,velocity);
        CHECK(std::isfinite(t.wrench_world.force.length_squared()));
        CHECK(dot(t.wrench_world.force,velocity)<=1e-8);
    }
}
TEST_CASE("Aero ground fan energy and combined wakes stay deterministic without job allocation","[aero][determinism][debug_alloc]") {
    auto run=[](int workers) {
        WorldConfig cfg; cfg.gravity={}; cfg.job_workers=workers; World world(cfg);
        BodyDesc floor; floor.motion=BodyMotionType::Static; floor.shape=BoxShape{{100,100,.1}}; floor.pose.position={0,0,-.1};
        world.create_body(floor);
        for(int i=0;i<3;++i) {
            BodyDesc b; b.motion=BodyMotionType::Dynamic; b.mass=1000; b.allow_sleep=false;
            b.shape=BoxShape{{1,.5,.01}}; b.pose.position={-real(i)*6,0,.2}; auto body=world.create_body(b);
            vehicle::VehicleDesc d; d.name="fan-wake"; d.aero=wing(-.2); d.aero.wake={30,1,.12,.3,.1};
            d.aero.surfaces[0].ground={Vec3{0,0,-1},1,.5}; d.aero.surfaces[0].max_offset_rad=.1;
            aero::FanDesc fan; fan.name="fan"; fan.pressure_pa=100; fan.seal_height_m=1; fan.max_power_w=1000; fan.rate_per_s=5;
            d.aero.fans.push_back(fan); const auto id=world.create_vehicle(d,body);
            world.backend().set_motion(body,Motion{{25,0,0},{}}); world.set_aero_surface_command(id,0,.5);
            world.set_aero_fan_command(id,0,1); world.credit_aero_fan_energy(id,1000);
        }
        for(int i=0;i<10;++i) world.step(); // warm snapshot buffers and telemetry
        std::array<std::uint64_t,80> hashes{};
        const bool armed=jobs::debug_alloc::hook_available(); jobs::debug_alloc::reset(); jobs::debug_alloc::set_armed(armed);
        for(std::size_t i=0;i<hashes.size();++i) { world.step(); hashes[i]=world.state_hash(); }
        jobs::debug_alloc::set_armed(false);
        if(armed) CHECK(jobs::debug_alloc::own_violation_count()==0);
        const auto& t=world.vehicle_aero_telemetry({0,1});
        CHECK(t.fan_energy_used_j>0); CHECK(t.surfaces[0].ground_multiplier>1);
        return hashes;
    };
    CHECK(run(1)==run(4));
}
