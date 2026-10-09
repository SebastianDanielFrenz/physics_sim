#include "ps/drivetrain/adaptive_pressure.h"
#include "ps/drivetrain/engine_maps.h"
#include "engine_cycle/map_cache.h"
#include "ps/jobs/job_system.h"
#include "cycle_fixtures.h"
#include "mvem_fixtures.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <atomic>
#include <filesystem>
using namespace ps::drivetrain;
using Catch::Approx;
TEST_CASE("adaptive axes preserve nonlinear and signed maps independently", "[adaptive_pressure]") {
    Table2D straight,curved;
    straight.x=curved.x={1000,5000};
    for(int j=0;j<=40;++j)straight.y.push_back(25000+2500*j);
    curved.y=straight.y;
    for(int row=0;row<2;++row)for(auto p:straight.y){straight.values.push_back(p-75000);curved.values.push_back((row+1)*p*p);}
    const auto reference=curved;
    reduce_pressure_axis(straight,0.005);reduce_pressure_axis(curved,0.005);
    CHECK(straight.y.size()==2);
    CHECK(curved.y.size()>2);
    CHECK(curved.y.size()<reference.y.size());
    for(auto p:reference.y)CHECK(pressure_error(reference,curved,p)<=0.005);
}
TEST_CASE("engine generation reuses sampled points across refinement passes", "[adaptive_pressure]") {
    auto d=ps::test::make_generic_i4_cycle_desc();d.grid_rpm={2000,4000};d.grid_p_im={40000,90000};
    auto fuel=ps::test::mvem::gasoline();
    EngineMapGenTask first(d,fuel,MapQuality::Preview);
    for(std::size_t i=0;i<first.point_count();++i)first.run_point(i);
    d.grid_p_im={40000,65000,90000};
    EngineMapGenTask next(d,fuel,MapQuality::Preview);
    CHECK(next.reuse_points_from(first)==first.point_count());
    for(std::size_t i=0;i<next.point_count();++i)next.run_point(i);
    auto reused=next.finish();auto independent=generate_engine_maps(d,fuel,MapQuality::Preview);
    CHECK(engine_maps_content_hash(*reused)==engine_maps_content_hash(*independent));
    d.compression_ratio+=1;
    EngineMapGenTask other(d,fuel,MapQuality::Preview);
    CHECK_THROWS_AS(other.reuse_points_from(first),std::invalid_argument);
}
TEST_CASE("adaptive engine maps converge and retain independent cached axes", "[adaptive_pressure]") {
    auto d=ps::test::make_generic_i4_cycle_desc();d.grid_rpm={2000,4000};d.grid_p_im={40000,90000};
    auto fuel=ps::test::mvem::gasoline();const auto legacy=engine_map_cache_key(d,fuel,MapQuality::Preview);
    d.pressure_sampling.enabled=true;d.pressure_sampling.relative_error=0.05;
    bool intermediate=false;std::size_t previous=0;
    MapGenerationControl progress;
    progress.on_progress=[&](std::size_t done,std::size_t total) {
        CHECK(done>=previous);CHECK(done<=total);previous=done;
        if(done*100/total>=1 && done<total)intermediate=true;
    };
    auto result=generate_engine_maps(d,fuel,MapQuality::Preview,nullptr,&progress);
    CHECK(intermediate);
    CHECK(result->generator_version==engine_map_generator_version(d));
    CHECK(result->maps.ve.y.front()==Approx(40000));CHECK(result->maps.ve.y.back()==Approx(90000));
    CHECK(result->maps.ve.validate().empty());CHECK(result->maps.eta_i.validate().empty());
    CHECK(engine_map_cache_key(d,fuel,MapQuality::Preview)!=legacy);
    const auto adaptive_key=engine_map_cache_key(d,fuel,MapQuality::Preview);
    CHECK(result->content_hash==engine_maps_content_hash(*result));
    const auto cache=(std::filesystem::temp_directory_path()/"ps_adaptive_pressure_cache_tests").generic_string();
    store_cached_engine_maps(cache,adaptive_key,*result);
    auto cached=load_cached_engine_maps(cache,adaptive_key,result->generator_version);
    REQUIRE(cached!=nullptr);
    CHECK(cached->maps.ve.y==result->maps.ve.y);
    CHECK(cached->maps.pmep_excess_pa.y==result->maps.pmep_excess_pa.y);
    CHECK(cached->content_hash==result->content_hash);
    ps::jobs::JobSystem workers(2);
    int snapshots=0;
    EngineMapsPtr first_snapshot;
    MapGenerationControl streaming;
    streaming.on_partial_maps=[&](EngineMapsPtr snapshot) {
        ++snapshots;if(!first_snapshot)first_snapshot=snapshot;
        CHECK(snapshot->maps.ve.validate().empty());
        CHECK(snapshot->maps.eta_i.validate().empty());
    };
    auto parallel=generate_engine_maps(d,fuel,MapQuality::Preview,&workers,&streaming);
    CHECK(snapshots>=2);
    CHECK(first_snapshot->maps.ve.y.front()==Approx(40000));
    CHECK(first_snapshot->maps.ve.y.back()==Approx(90000));
    CHECK(engine_maps_content_hash(*parallel)==engine_maps_content_hash(*result));
    auto extended=d;
    extended.pressure_sampling.max_pa=600000;
    auto high=generate_engine_maps(extended,fuel,MapQuality::Preview,&workers);
    CHECK(high->maps.ve.y.back()==Approx(600000));
    CHECK(high->maps.pmep_excess_pa.y.back()==Approx(600000));
    CHECK(high->content_hash==engine_maps_content_hash(*high));
    d.pressure_sampling.relative_error=0.04;
    CHECK(engine_map_cache_key(d,fuel,MapQuality::Preview)!=adaptive_key);
    std::atomic<bool> cancel{true};MapGenerationControl control;control.cancel=&cancel;
    CHECK_THROWS_AS(generate_engine_maps(d,fuel,MapQuality::Preview,nullptr,&control),GenerationCancelled);
    d.pressure_sampling.max_points=5;
    d.pressure_sampling.relative_error=0.000001;
    CHECK_THROWS(generate_engine_maps(d,fuel,MapQuality::Preview));
    d.pressure_sampling.allow_budget_limit=true;
    auto bounded=generate_engine_maps(d,fuel,MapQuality::Preview);
    CHECK(bounded->maps.ve.y.size()<=5);
    CHECK(bounded->maps.ve.y.front()==Approx(40000));
    CHECK(bounded->maps.ve.y.back()==Approx(90000));
    CHECK(bounded->maps.source.find("budget limited")!=std::string::npos);
    CHECK(bounded->content_hash==engine_maps_content_hash(*bounded));
}
