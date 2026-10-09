#include "ps/drivetrain/turbo_maps.h"
#include "ps/io/vehicle_io.h"
#include "../../core/src/drivetrain/turbo_pair.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <cmath>
#include "drivetrain/engine_simulated.h"
#include "ps/jobs/job_system.h"
#include <cstdio>

namespace {
using json=nlohmann::json;
std::filesystem::path fixture() { return std::filesystem::path(PS_SOURCE_DIR)/"tests/fixtures/turbo_stage_small.json"; }
ps::drivetrain::TurboPairDesc load_stage(int count=1, const json* replacement=nullptr) {
    const auto dir=std::filesystem::temp_directory_path()/"ps_stage_map_test";
    std::filesystem::create_directories(dir);
    json hw;
    std::ifstream(std::filesystem::path(PS_SOURCE_DIR)/"data/turbos/hyper_turbo_approx.json")>>hw;
    hw["performance_map"]=fixture().string();
    if(replacement) { std::ofstream(dir/"map.json")<<replacement->dump(); hw["performance_map"]=(dir/"map.json").string(); }
    std::ofstream(dir/"hardware.json")<<hw.dump();
    json cfg={{"format","physics_sim.turbo_configuration/1"},{"turbo","hardware.json"},
              {"count",count},{"target_boost_bar",1.8},{"intercooler_effectiveness",0.95}};
    std::ofstream(dir/"config.json")<<cfg.dump();
    return ps::io::load_turbo_configuration_json((dir/"config.json").string());
}
}

TEST_CASE("Offline stage map loads provenance and rejects invalid data", "[turbo_stage]") {
    const auto d=load_stage();
    REQUIRE(d.maps->stages);
    const auto& m=*d.maps->stages;
    CHECK(m.solver=="meanline-3");
    CHECK(m.units==1);
    CHECK(m.compressor_covered(120000,.2));
    CHECK_FALSE(m.compressor_covered(0,.2));
    CHECK_FALSE(m.compressor_covered(240000,.2));
    json j; std::ifstream(fixture())>>j;
    SECTION("bad dimension") {
        j["compressor"].erase(j["compressor"].begin());
        CHECK_THROWS_WITH(load_stage(1,&j),Catch::Matchers::ContainsSubstring("dimensions"));
    }
    SECTION("bad axis") {
        j["corrected_rpm"][1]=-1;
        CHECK_THROWS(load_stage(1,&j));
    }
    SECTION("unsupported solver") {
        j["solver"]="other";
        CHECK_THROWS_WITH(load_stage(1,&j),Catch::Matchers::ContainsSubstring("unsupported solver"));
    }
    SECTION("negative flow") {
        j["turbine"][0]["mass_flow_kg_s"]=-1;
        CHECK_THROWS_WITH(load_stage(1,&j),Catch::Matchers::ContainsSubstring("invalid turbine"));
    }
}

TEST_CASE("Generated compressor work drives shaft and thermal outputs", "[turbo_stage]") {
    const auto d=load_stage();
    ps::drivetrain::TurboPair t(d);
    t.energy_j=.5*d.rotor_inertia_kgm2*std::pow(120000*6.283185307179586/60,2);
    t.step(1.0/960,101325,288.15,.2,.21,1000,1);
    REQUIRE(t.compressor_stage_used);
    CHECK(t.compressor_w==Catch::Approx(.2*d.maps->stages->compressor_work_j_kg.value(120000,.2)).epsilon(1e-10));
    CHECK(t.charge_k>288.15);
    REQUIRE(t.turbine_stage_used);
    CHECK_FALSE(t.stage_map_fallback);
}

TEST_CASE("Generated turbine starts from rest and scales twin units", "[turbo_stage]") {
    const auto one=load_stage(1), two=load_stage(2);
    ps::drivetrain::TurboPair a(one), b(two);
    // Wall conductance is an INSTALLATION total, intentionally not count
    // scaled. Disable that separate effect when testing rotor similarity.
    a.desc.exhaust_wall_gas_conductance_w_k=b.desc.exhaust_wall_gas_conductance_w_k=0;
    a.step(1.0/960,101325,288.15,.1,.11,1000,1);
    b.step(1.0/960,101325,288.15,.2,.22,1000,1);
    REQUIRE(a.turbine_stage_used);
    REQUIRE(b.turbine_stage_used);
    CHECK(a.rpm()>0);
    CHECK(a.rpm()==Catch::Approx(b.rpm()).epsilon(1e-9));
    CHECK(a.exhaust_pa==Catch::Approx(b.exhaust_pa).epsilon(1e-9));
    CHECK(a.stage_map_fallback); // zero-speed compressor coverage explicitly absent
    a.reset(101325,288.15);
    CHECK_FALSE(a.stage_map_fallback);
}

TEST_CASE("Stage map domain is not a rotor limiter", "[turbo_stage]") {
    const auto d=load_stage();
    ps::drivetrain::TurboPair t(d);
    t.energy_j=.5*d.rotor_inertia_kgm2*std::pow(500000*6.283185307179586/60,2);
    t.step(1.0/960,101325,288.15,.2,.21,1000,1);
    CHECK(t.rpm()>400000);
    CHECK(t.stage_map_fallback);
    CHECK_FALSE(t.compressor_stage_used);
    CHECK_FALSE(t.turbine_stage_used);
    CHECK(std::isfinite(t.energy_j));
}

TEST_CASE("Turbine inverse lookup reproduces compressible mass balance", "[turbo_stage]") {
    const auto d=load_stage(2);
    const auto& m=*d.maps->stages;
    double pressure=0,power=0,torque=0;
    const double flow=.8,k=1000,pa=101325,area=.0004;
    REQUIRE(m.turbine_at(120000,flow,pa,k,area,pressure,power,torque));
    const double er=pressure/pa;
    const double chi=m.turbine_coordinate(er);
    const double through=2*m.turbine_flow_kg_s.value(120000,chi)*pressure/101325;
    const double bypass=area*m.nozzle_flux(pressure,pa,k,m.exhaust_gamma,m.exhaust_r);
    CHECK(through+bypass==Catch::Approx(flow).epsilon(2e-7));
    CHECK(power==Catch::Approx(through*m.turbine_torque_nm_per_kg_s.value(120000,chi)*120000*6.283185307179586/60).epsilon(1e-10));
    CHECK(pressure>pa);
}

TEST_CASE("Generated-stage hold remains finite and converges with timestep", "[turbo_stage]") {
    const auto d=load_stage(2);
    auto run=[&](double h) {
        ps::drivetrain::TurboPair t(d);
        t.desc.exhaust_wall_gas_conductance_w_k=0;
        for(int i=0;i<int(10/h);++i) t.step(h,101325,288.15,.4,.42,1000,1);
        CHECK(std::isfinite(t.charge_pa));
        CHECK(std::isfinite(t.energy_j));
        CHECK(t.energy_j>=0);
        CHECK(t.turbine_stage_used);
        return std::pair(t.rpm(),t.charge_pa);
    };
    const auto a=run(1.0/960), b=run(1.0/1920);
    CHECK(a.first==Catch::Approx(b.first).epsilon(.02));
    CHECK(a.second==Catch::Approx(b.second).epsilon(.02));
}

TEST_CASE("Authored exhaust nozzle conserves mass at low and extreme flow", "[turbo_stage][turbo_exhaust]") {
    using namespace ps::drivetrain;
    for(double area:{.0005,.00336}) for(double k:{500.,1300.}) for(double flow:{0.,.05,.5,2.,10.}) {
        const auto pressure=TurboPair::compressible_exhaust_pressure(flow,area,101325,k);
        CHECK(pressure>=101325);
        CHECK(area*TurboStageMaps::nozzle_flux(pressure,101325,k,1.33,287.05)==Catch::Approx(flow).margin(2e-6));
    }
    const double p1=TurboPair::compressible_exhaust_pressure(2,.001,101325,1300);
    const double p2=TurboPair::compressible_exhaust_pressure(4,.001,101325,1300);
    CHECK(p2==Catch::Approx(2*p1).epsilon(1e-10)); // choked flow scales linearly, not quadratically
}

TEST_CASE("Authored hyper high boost exhaust diagnostic", "[.turbo_exhaust_hyper]") {
    using namespace ps::drivetrain;
    ps::jobs::JobSystem pool(6);
    ps::io::EngineMapOptions options;
    options.pool=&pool;
    options.cache_dir=(std::filesystem::temp_directory_path()/"ps_i7d_engine_cache").string();
    auto base=std::get<SimulatedEngineDesc>(ps::io::load_engine_json(
        std::string(PS_SOURCE_DIR)+"/data/engines/hyper_v8_one1_audio.json",options));
    for(double boost:{1.8,4.3,30.}) for(double crank_rpm:{4000.,7500.,8000.}) {
        for(bool compressible:{false,true}) {
            base.turbo_pair->target_boost_pa=boost*1e5;
            base.turbo_pair->compressible_exhaust=compressible;
            SimulatedEngine engine(base);
            Sensors sensors;
            const double omega=crank_rpm*6.283185307179586/60;
            sensors.omega=omega;sensors.ignition=1;
            PreSolveResult result;
            double kw=0;
            for(int i=0;i<16*960;++i) {
                result=engine.pre_solve(sensors,ThrottleCommand{1,0,1},TcuRequest{},1./960);
                (void)engine.post_solve(omega,1./960);
                if(i>=15*960) kw+=(result.tau0-result.friction_cap_kinetic)*omega/1000/960;
            }
            std::printf("[authored hyper] compressible %d target %.1f crank %.0f actual %.3f exhaust %.3f barabs mean %.1f kW\n",
                compressible,boost,crank_rpm,result.boost_bar,result.p_exhaust_pa/1e5,kw);
            CHECK(std::isfinite(kw));
            if(compressible) CHECK(kw>0);
        }
    }
}

TEST_CASE("Experimental generated turbo on the held hyper engine", "[.turbo_stage_hyper]") {
    using namespace ps::drivetrain;
    const auto installation=std::filesystem::path(PS_SOURCE_DIR)/"out/turbo_sim/examples/standard/configuration.json";
    REQUIRE(std::filesystem::exists(installation)); // run tools/turbo_sim/build_examples.py first
    ps::jobs::JobSystem pool(6);
    ps::io::EngineMapOptions options;
    options.pool=&pool;
    options.cache_dir=(std::filesystem::temp_directory_path()/"ps_i7d_engine_cache").string();
    auto base=std::get<SimulatedEngineDesc>(ps::io::load_engine_json(
        std::string(PS_SOURCE_DIR)+"/data/engines/hyper_v8_one1_audio.json",options));
    base.turbo_pair=ps::io::load_turbo_configuration_json(installation.string());
    for(double boost:{1.8,4.3}) for(double crank_rpm:{4000,7500}) {
        base.turbo_pair->target_boost_pa=boost*1e5;
        auto run=[&](double h) {
            SimulatedEngine engine(base);
            Sensors sensors;
            const double omega=crank_rpm*6.283185307179586/60;
            sensors.omega=omega;sensors.ignition=1;
            PreSolveResult result;
            int compressor_samples=0,turbine_samples=0,fallback_samples=0;
            for(int i=0;i<int(16/h);++i) {
                result=engine.pre_solve(sensors,ThrottleCommand{1,0,1},TcuRequest{},h);
                (void)engine.post_solve(omega,h);
                if(i>=int(15/h)) {
                    compressor_samples+=result.turbo_compressor_stage_used;
                    turbine_samples+=result.turbo_turbine_stage_used;
                    fallback_samples+=result.turbo_stage_map_fallback;
                }
            }
            const double kw=(result.tau0-result.friction_cap_kinetic)*omega/1000;
            std::printf("[stage hyper] requested %.2f bar crank %.0f h %.7f actual %.3f bar rotor %.0f exhaust %.3f barabs power %.1f kW used comp/turb/fallback %d/%d/%d\n",
                boost,crank_rpm,h,result.boost_bar,result.turbo_rpm,result.p_exhaust_pa/1e5,kw,
                compressor_samples,turbine_samples,fallback_samples);
            CHECK(std::isfinite(kw));CHECK(std::isfinite(result.turbo_rpm));CHECK(std::isfinite(result.boost_bar));
            CHECK(turbine_samples>0);
            return std::pair(kw,result.boost_bar);
        };
        const auto a=run(1.0/960),b=run(1.0/1920);
        CHECK(a.first==Catch::Approx(b.first).epsilon(.05).margin(1));
        CHECK(a.second==Catch::Approx(b.second).epsilon(.05).margin(.05));
    }
}
