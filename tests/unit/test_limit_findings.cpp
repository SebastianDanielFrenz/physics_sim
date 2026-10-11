// tests/unit/test_limit_findings.cpp - engine garage P3b-1: limit findings of the dyno sweep (placeholder: diagnostics).
#include <catch2/catch_test_macros.hpp>

#include "drivetrain/engine_simulated.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

using namespace ps;
using namespace ps::drivetrain;

namespace {
const std::string kEngines = std::string(PS_SOURCE_DIR) + "/data/engines/";
std::string cache_dir() {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "ps_garage_p3b1_maps";
    std::filesystem::create_directories(p);
    return p.string();
}
jobs::JobSystem& map_pool() {
    static jobs::JobSystem pool(10);
    return pool;
}
ComponentParams load(const std::string& file, MapQuality q) {
    io::EngineMapOptions o;
    o.quality = q;
    o.pool = &map_pool();
    o.cache_dir = cache_dir();
    return io::load_engine_json(kEngines + file, o);
}
void dump(const char* title, const DynoResult& r) {
    std::printf("== %s: %zu points, %zu findings\n", title, r.points.size(), r.limits.size());
    for (const DynoPoint& p : r.points)
        std::printf("  rpm %6.0f  T %7.1f  boost %.3f  turbo %8.0f conv %d\n", p.rpm, p.torque_nm, p.boost_bar, p.turbo_rpm, p.converged ? 1 : 0);
    for (const LimitFinding& f : r.limits)
        std::printf("  [%7.0f] %-20s %-8s value %.5g limit %.5g %s  part %s%s\n", f.rpm, to_string(f.kind), to_string(f.severity), f.value,
                    f.limit, to_string(f.unit), f.part.slot.c_str(), f.part.field.c_str());
}
} // namespace

TEST_CASE("limit findings diag: crafted", "[.][diag]") {
    static jobs::JobSystem pool(10);
    const ComponentParams hyper = load("hyper_v8_one1_audio.json", MapQuality::Sketch);
    DynoSweepSpec spec;
    spec.rpm = {3000.0, 5000.0, 7000.0, 8000.0};
    for (double s : {0.30, 0.45, 0.60, 0.75}) {
        ComponentParams c = hyper;
        std::get<SimulatedEngineDesc>(c).turbo_pair->wastegate_area_m2 = std::get<SimulatedEngineDesc>(hyper).turbo_pair->wastegate_area() * s;
        char t[64];
        std::snprintf(t, sizeof t, "wastegate x%.2f", s);
        dump(t, run_dyno_sweep(c, nullptr, spec, &pool));
    }
    {
        const auto& td = *std::get<SimulatedEngineDesc>(hyper).turbo_pair;
        std::printf("stock wg area %.5g turbine %.5g maxflow %.4g ref_rpm %.0f target %.0f\n", td.wastegate_area(), td.turbine_area_m2,
                    td.max_mass_flow_kg_s, td.reference_rpm, td.target_boost_pa);
    }
    for (double k : {1.10, 1.20, 1.30}) {
        ComponentParams c = hyper;
        auto& d = std::get<SimulatedEngineDesc>(c);
        d.piston.bore_m *= std::sqrt(k);
        char t[64];
        std::snprintf(t, sizeof t, "bore^2 x%.2f (air demand)", k);
        dump(t, run_dyno_sweep(c, nullptr, spec, &pool));
    }
    {
        ComponentParams c = hyper;
        std::get<SimulatedEngineDesc>(c).turbo_pair->target_boost_pa *= 1.6;
        dump("target x1.6", run_dyno_sweep(c, nullptr, spec, &pool));
    }
    {
        ComponentParams c = hyper;
        std::get<SimulatedEngineDesc>(c).turbo_pair->max_mass_flow_kg_s *= 0.5;
        dump("max flow x0.5", run_dyno_sweep(c, nullptr, spec, &pool));
    }
    {
        ComponentParams c = hyper;
        std::get<SimulatedEngineDesc>(c).turbo_pair->reference_rpm *= 0.6;
        dump("reference_rpm x0.6", run_dyno_sweep(c, nullptr, spec, &pool));
    }
}

TEST_CASE("limit findings diag: stock engines", "[.][diag]") {
    static jobs::JobSystem pool(10);
    const ComponentParams hyper = load("hyper_v8_one1_audio.json", MapQuality::Sketch);
    DynoSweepSpec spec;
    dump("hyper stock", run_dyno_sweep(hyper, nullptr, spec, &pool));
    const ComponentParams na = io::load_engine_json(kEngines + "toyota_a25a_fks_sim.json");
    dump("a25a sim", run_dyno_sweep(na, nullptr, DynoSweepSpec{}, nullptr));
    const ComponentParams tm = io::load_engine_json(kEngines + "i4_2l_na.json");
    dump("i4 torque map", run_dyno_sweep(tm, nullptr, DynoSweepSpec{}, nullptr));
}
