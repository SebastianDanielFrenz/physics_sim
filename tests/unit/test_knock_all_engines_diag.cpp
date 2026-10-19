// Hidden diag (owner 2026-10-18 "opt all engines in to knock"): WOT dyno table + load time of every shipped simulated
// engine, for before/after comparison of the knock opt-in. Run: ps_unit_tests "[knock_all]".
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>

using namespace ps;
using namespace ps::drivetrain;

TEST_CASE("knock_all diag: WOT dyno table of the shipped simulated engines", "[.][diag][knock_all]") {
    const std::filesystem::path dir = std::filesystem::path(PS_SOURCE_DIR) / "data" / "engines";
    jobs::JobSystem pool(10);
    for (const char* name : {"hyper_v8_one1_audio", "demo_i4_pipes", "honda_k20a_dc5", "bmw_n52b30", "toyota_a25a_fks_gen",
                             "toyota_a25a_fks_sim"}) {
        io::EngineMapOptions o;
        o.quality = MapQuality::Full;
        o.pool = &pool;
        const auto t0 = std::chrono::steady_clock::now();
        const ComponentParams p = io::load_engine_json((dir / (std::string(name) + ".json")).generic_string(), o);
        const double load_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("KNOCKALL %s load %.2f s\n", name, load_s);
        DynoSweepSpec spec;
        spec.rpm = {1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000};
        DynoResult r;
        try {
            r = run_dyno_sweep(p, nullptr, spec, &pool);
        } catch (const std::exception& e) {
            std::printf("KNOCKALL %s dyno threw: %s\n", name, e.what());
            continue;
        }
        double peak_t = 0, peak_t_rpm = 0, peak_p = 0, peak_p_rpm = 0;
        for (const auto& pt : r.points) {
            const double kr = std::isfinite(pt.knock_retard_deg) ? pt.knock_retard_deg : 0.0;
            std::printf("KNOCKALL %s rpm %5.0f T %7.1f P %7.1f spark %5.2f retard %5.2f boost %4.2f\n", name, pt.rpm, pt.torque_nm,
                        pt.power_kw, pt.spark_deg, kr, pt.boost_bar);
            if (pt.torque_nm > peak_t) peak_t = pt.torque_nm, peak_t_rpm = pt.rpm;
            if (pt.power_kw > peak_p) peak_p = pt.power_kw, peak_p_rpm = pt.rpm;
        }
        std::printf("KNOCKALL %s PEAK T %.1f @ %.0f  P %.1f kW @ %.0f\n", name, peak_t, peak_t_rpm, peak_p, peak_p_rpm);
    }
}
