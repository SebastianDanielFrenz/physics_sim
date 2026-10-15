// Validation: 100-0 km/h braking distance within 3% of a mu-based estimate
// including load transfer (PLAN.md P3 acceptance, PHYS-019).
//
// The estimate is the standard longitudinal load-transfer-corrected peak-mu
// bound (Milliken, "Race Car Vehicle Dynamics", ch. 18) [D]: braking shifts
// load onto the front axle by dFz = mass*a*h_cg/wheelbase, and the maximum
// sustainable deceleration is sum_corners Dx(Fz_corner)/mass with
// Dx(Fz) = Fz (pdx1 + pdx2 dfz) lambda_mux, solved by fixed-point iteration
// on a. Distance = v0^2 / (2 a). Every input is read from the loaded desc
// (wheel attachments -> static split and wheelbase, the tyres' own
// longitudinal coefficients) and the settled rig (chassis mass, CoM height
// above the ground) - nothing is hand-copied from a data file.
//
// Until assists I5 part 2 this test emulated a per-axle ABS in test code and
// stayed [!mayfail] at about +9 % (41.1 vs 37.69 m). It now brakes with the
// real ABS of the assist unit (data/electronics/assist_unit_abs_esc.json,
// fitted to car_sedan through VehicleLoadOptions::electronics["assists"]):
// a sensor-only per-wheel slip controller that scales each corner's service
// brake capacity (docs/data_formats/electronics.md "abs"). The same check
// with the same bands is case A1 of tests/unit/test_abs_esc.cpp.

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

using namespace ps;
using namespace ps::vehicle;

namespace {

real estimate_stop_distance(const VehicleDesc& d, real mass, real h, real v0) {
    real xf = 0, xr = 0;
    int nf = 0, nr = 0;
    for (const WheelDesc& w : d.wheels) {
        if (w.is_front) { xf += w.attachment_local.x; ++nf; } else { xr += w.attachment_local.x; ++nr; }
    }
    xf /= nf;
    xr /= nr;
    const real L = xf - xr, g = 9.81;
    const auto dx_axle = [&](bool front, real fz_axle) {
        real s = 0;
        for (const WheelDesc& w : d.wheels) {
            if (w.is_front != front) continue;
            const real fz = fz_axle / (front ? nf : nr);
            const TyreDesc& t = w.tyre;
            s += fz * (t.pdx1 + t.pdx2 * (fz - t.fz0) / t.fz0) * t.lambda_mux;
        }
        return s;
    };
    real a = g;
    for (int i = 0; i < 200; ++i) {
        const real ff = mass * g * (-xr) / L + mass * a * h / L;
        const real fr = mass * g * xf / L - mass * a * h / L;
        const real an = (dx_axle(true, ff) + dx_axle(false, fr)) / mass;
        if (std::abs(an - a) < 1e-13) { a = an; break; }
        a = an;
    }
    return v0 * v0 / (2 * a);
}

} // namespace

TEST_CASE("100-0 km/h braking distance with ABS is within 3% of the load-transfer-corrected mu estimate", "[validation][vehicle][abs][long]") {
    io::VehicleLoadOptions o;
    o.electronics["assists"] = std::string(PS_SOURCE_DIR) + "/data/electronics/assist_unit_abs_esc.json";
    const VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", o);
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6);
    World& w = *rig.world;
    for (const char* ch : {"throttle", "brake", "handbrake", "steer", "assist.traction_control", "assist.auto_shift", "assist.esc"}) {
        w.set_control(ch, 0.0);
    }
    w.set_control("assist.abs", 1.0);
    ps_test::settle(w, 0.5);

    const real v0 = 100.0 / 3.6;
    Motion m;
    m.linear = Vec3{v0, 0.0, 0.0};
    w.backend().set_motion(rig.chassis, m);
    VehicleResetOptions ro;
    ro.gear = 0;
    ro.hubs_roll_with_chassis = true; // the wheels turn at the chassis speed (no first-tick slip spike)
    w.reset_vehicle(rig.vehicle, ro);
    ps_test::settle(w, 0.5); // coast: the suspension settles at speed before the brake

    const Pose p_start = w.backend().get_pose(rig.chassis);
    const real v_brake = w.backend().get_motion(rig.chassis).linear.length();
    const real h = p_start.position.z; // ground top at z = 0
    const real est = estimate_stop_distance(desc, w.backend().get_mass(rig.chassis), h, v_brake);

    w.set_control("brake", 1.0);
    real stop_distance = -1.0;
    for (int i = 0; i < 15 * 240; ++i) {
        w.step();
        if (w.backend().get_motion(rig.chassis).linear.length() < 0.3) {
            stop_distance = distance(p_start.position, w.backend().get_pose(rig.chassis).position);
            break;
        }
    }
    INFO("stop_distance=" << stop_distance << " m from " << v_brake << " m/s, estimate=" << est << " m (h " << h << " m)");
    REQUIRE(stop_distance > 0.0);
    CHECK(std::abs(stop_distance / est - 1.0) <= 0.03);
}
