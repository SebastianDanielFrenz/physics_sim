#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/traction_control.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
using namespace ps;
using namespace ps::vehicle;
namespace {
const VehicleDesc& hyper() {
    static const VehicleDesc d = [] {
        jobs::JobSystem pool(10);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_assists_diag_cache").string();
        std::filesystem::create_directories(opt.cache_dir);
        return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    }();
    return d;
}
// Full-throttle launch from standstill, 1st gear, auto-clutch, TC mode `mode` (1 On, 2 Relaxed, 3 Sport).
// Returns the time to 30 km/h (negative: not reached) and the lowest torque fraction after 0.5 s.
struct Launch { real t30 = -1, v_end = 0, min_fraction_late = 1; };
Launch launch(int mode) {
    VehicleDesc desc = hyper();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0, 4);
    World& w = *rig.world;
    w.set_control("assist.traction_control", mode);
    w.set_control("assist.auto_clutch", 1.0);
    ps_test::settle(w, .5);
    w.set_control("gear_request", 1);
    w.set_control("throttle", 1.0);
    Launch r;
    for (int k = 0; k < 240 * 4; ++k) {
        w.step();
        const real v = w.get_motion(rig.chassis).linear.x;
        if (r.t30 < 0 && v >= 30.0 / 3.6) r.t30 = (k + 1) / 240.0;
        if (k > 120) r.min_fraction_late = std::min(r.min_fraction_late, w.powertrain_state(rig.vehicle).traction_torque_fraction);
        r.v_end = v;
    }
    return r;
}
}

TEST_CASE("Hypercar with TC on launches from standstill through 30 km/h", "[traction_control][vehicle][low_speed]") {
    for (int mode : {1, 2, 3}) {
        const Launch r = launch(mode);
        std::printf("TC launch mode=%d t30=%.2f s v_end=%.1f m/s min_fraction(after 0.5 s)=%.3f\n", mode, r.t30, r.v_end, r.min_fraction_late);
        CHECK(r.t30 > 0);
        CHECK(r.t30 < 2.5);
        CHECK(r.v_end > 25.0);
    }
}
