// tests/validation/test_vehicle_shadow_chassis.cpp — PHYS-033 I1 W1: the
// vehicle shadow chassis (VehicleInstance::step_tick, PLAN.md D3) must
// reproduce IRigidBackend's own end-of-tick chassis velocity essentially
// exactly whenever nothing outside the shadow's own model touches the
// chassis (no contact, no constraint, a constant or absent external
// force-element wrench) - the I1 plan section 2's numbered conditions.
// World::vehicle_shadow_dv_error/dw_error report the gap between the
// shadow's prediction made at the end of the PREVIOUS tick and the
// backend's actual chassis velocity at the start of THIS tick (a one-tick
// offset, see VehicleInstance::shadow_dv_error's own comment) - 0 on a
// vehicle's very first tick, and a genuine (expected) spike on the one tick
// right after anything reaches into the chassis from outside the shadow's
// model (a test's own set_motion, a newly-introduced force element).

#include "vehicle_test_helpers.h"

#include "ps/world/force_element.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ps;

namespace {

class ConstantWrenchElement final : public IForceElement {
public:
    ConstantWrenchElement(Frame frame, Wrench w) : frame_(frame), w_(w) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext&) const override { return w_; }

private:
    Frame frame_;
    Wrench w_;
};

// Flips sign every tick - PHYS-033 I1 plan section 5.2a(3): proves the
// shadow's one-tick-lag external-force estimate is exactly that (not, say,
// an average or a low-pass filter).
class AlternatingWrenchElement final : public IForceElement {
public:
    AlternatingWrenchElement(Frame frame, Vec3 force_amplitude) : frame_(frame), amp_(force_amplitude) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        const real sign = (ctx.tick % 2 == 0) ? real(1.0) : real(-1.0);
        return Wrench{amp_ * sign, Vec3::zero()};
    }

private:
    Frame frame_;
    Vec3 amp_;
};

} // namespace

TEST_CASE("Shadow chassis: settled car + step steer at speed matches Jolt within 1e-4", "[validation][vehicle]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{20.0, 0.0, 0.0}, Vec3::zero()});
    // PHYS-033 P4 I3 plan section 11: drop straight into an already-driving
    // 3rd-gear state (gear_request alone only starts a shift from Neutral,
    // which would spend several ticks slipping the clutch and desync the
    // shadow from Jolt for reasons that have nothing to do with what this
    // test checks) - reset_vehicle sets the network directly, no shift, no
    // projection loss, matching the now-20 m/s chassis speed set above via
    // its own hub-speed resolution (VehicleInstance::reset).
    rig.world->set_control("gear_request", 3.0);
    vehicle::VehicleResetOptions reset_opts;
    reset_opts.gear = 3;
    reset_opts.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, reset_opts);
    rig.world->set_control("steer", 0.3);
    rig.world->set_control("throttle", 0.3);

    real max_dv = 0.0, max_dw = 0.0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(1.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        if (i == 0) continue; // the tick right after set_motion: an expected spike, not a bug
        max_dv = std::max(max_dv, rig.world->vehicle_shadow_dv_error(rig.vehicle));
        max_dw = std::max(max_dw, rig.world->vehicle_shadow_dw_error(rig.vehicle));
    }
    INFO("max_dv=" << max_dv << " m/s, max_dw=" << max_dw << " rad/s");
    CHECK(max_dv < 1e-4);
    CHECK(max_dw < 1e-4);
}

TEST_CASE("Shadow chassis: a constant force element on the chassis matches Jolt within 1e-4", "[validation][vehicle]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ConstantWrenchElement fe(Frame(rig.chassis), Wrench{Vec3{0.0, 1000.0, 0.0}, Vec3::zero()});
    rig.world->add_force_element(&fe);

    ps_test::settle(*rig.world, 1.0); // the constant wrench is present throughout - no discontinuity to exclude

    real max_dv = 0.0, max_dw = 0.0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(1.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        max_dv = std::max(max_dv, rig.world->vehicle_shadow_dv_error(rig.vehicle));
        max_dw = std::max(max_dw, rig.world->vehicle_shadow_dw_error(rig.vehicle));
    }
    INFO("max_dv=" << max_dv << " m/s, max_dw=" << max_dw << " rad/s");
    CHECK(max_dv < 1e-4);
    CHECK(max_dw < 1e-4);
}

TEST_CASE("Shadow chassis: an alternating force element's dv_error matches the one-tick-lag estimate",
          "[validation][vehicle]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    AlternatingWrenchElement fe(Frame(rig.chassis), Vec3{0.0, 1000.0, 0.0});
    rig.world->add_force_element(&fe);

    ps_test::settle(*rig.world, 1.0);

    const real dt = rig.world->dt();
    const real mass = rig.world->get_mass(rig.chassis);
    const real expected = dt * 2000.0 / mass; // I1 plan 5.2a(3): dt*2000/m

    std::vector<real> dv_samples;
    const std::uint64_t n = static_cast<std::uint64_t>(0.2 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        dv_samples.push_back(rig.world->vehicle_shadow_dv_error(rig.vehicle));
    }
    // Every tick alternates, so every dv_error sample (after the settle has
    // reached the alternating steady state) should sit near `expected`.
    for (real dv : dv_samples) {
        INFO("dv=" << dv << " expected=" << expected);
        CHECK(dv == Catch::Approx(expected).epsilon(0.01));
    }
}

TEST_CASE("Shadow chassis negative control: a chassis in contact with an obstacle shows a large dv_error",
          "[validation][vehicle]") {
    // The wheels' own suspension holds the chassis at its normal ride
    // height regardless of spawn height (found the hard way while writing
    // this test - a low spawn height alone just gets absorbed by the
    // suspension, never producing a real chassis-body contact). Instead,
    // settle normally, then drop a narrow static obstacle (clear of the
    // +-0.78 m wheel track, car_sedan.json's own attachment_local.y) into
    // the middle of the chassis box - a contact the shadow model has no way
    // to know about (I1 plan section 3.4: "not estimated, by design").
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    const Pose chassis_pose = rig.world->get_pose(rig.chassis);

    BodyDesc obstacle;
    obstacle.shape = BoxShape{Vec3{0.3, 0.3, 0.5}};
    obstacle.motion = BodyMotionType::Static;
    obstacle.pose.position = chassis_pose.position; // centred inside the chassis box - guaranteed overlap
    rig.world->create_body(obstacle);

    real max_dv = 0.0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(0.5 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        max_dv = std::max(max_dv, rig.world->vehicle_shadow_dv_error(rig.vehicle));
    }
    INFO("max_dv=" << max_dv << " m/s");
    CHECK(max_dv > 1e-2);
}

namespace {

// I1 plan section 4.3: the low-speed lateral damper is explicit, and all
// four wheels' damping torques sum onto shared chassis DOFs - a per-vehicle-
// file check that the (loose, cheap-upper-bound) explicit-integration
// stability quantity gamma stays under 1 at the configured
// low_speed_damping_ratio.
//
// PHYS-033 I1b commit C7 (plan section 3.7): r_i now uses the wheel's
// CONTACT-PATCH lever (z = -origin_height, i.e. -h_cg: the patch sits
// directly below the chassis origin at rest), not the wheel-centre
// attachment point the damper's OWN force is still evaluated relative to a
// contact-plane frame at - the damper acts at the contact point since I1b's
// commit C6 (WheelInstance::step's ContactFrame), so this bound's own lever
// must match or it silently under-estimates gamma (measured: using the
// wheel-centre lever here under-estimated gamma by ~1.94x for the car
// (0.490 -> 0.950) and ~1.96x for the tractor (0.333 -> 0.652)). y_i stays
// the wheel's UNSTEERED
// lateral axis (unit_y) - adequate for this order-of-magnitude sanity
// bound, not a claim of per-wheel steer-angle exactness.
real low_speed_damping_gamma(const std::string& path, real mass, Vec3 half_extents, real origin_height) {
    const vehicle::VehicleDesc desc = io::load_vehicle_json(path);
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, mass, half_extents, origin_height);
    ps_test::settle(*rig.world, 1.0);

    const BodyIntegrationParams ip = rig.world->backend().get_integration_params(rig.chassis);
    // The damper is evaluated and applied once per SUBSTEP
    // (WheelInstance::step, called from the substep loop), not once per
    // tick - the explicit-integration stability step is therefore the
    // substep dt (960 Hz default), not the 240 Hz tick dt.
    const real h = real(1.0) / real(960.0);
    real gamma = 0.0;
    for (const vehicle::WheelDesc& w : desc.wheels) {
        const vehicle::TyreDesc& t = w.tyre;
        if (t.low_speed_damping_ratio <= 0.0) continue;
        const real omega_n = std::sqrt(t.pky1 * 9.81 / std::max(t.relaxation_length_y, real(1e-6)));
        const real m_corner = t.fz0 / 9.81;
        const real c = real(2.0) * t.low_speed_damping_ratio * m_corner * omega_n;
        const Vec3 r{w.attachment_local.x, w.attachment_local.y, -origin_height}; // I1b C7: the patch lever, not the wheel centre
        const Vec3 y_local = Vec3::unit_y();
        const Vec3 r_cross_y = cross(r, y_local);
        const real quad_term = ip.inv_inertia_world.rows[0].x * r_cross_y.x * r_cross_y.x +
                                ip.inv_inertia_world.rows[1].y * r_cross_y.y * r_cross_y.y +
                                ip.inv_inertia_world.rows[2].z * r_cross_y.z * r_cross_y.z;
        gamma += c * (ip.inv_mass + quad_term);
    }
    return gamma * h;
}

} // namespace

// PHYS-033 I1b commit C7: with the patch lever, car_sedan's gamma rises from
// 0.490 (wheel-centre lever) to 0.950 - close to, but still under, 1
// (the I1b plan's own back-of-envelope arithmetic, gamma ~= 0.17 +
// 0.633*zeta = 1.12 at zeta=1.5, over-estimated the rise: that formula
// scaled the OLD gamma by a single z-only lever ratio, while the real r_i
// used here is the full 3D patch offset (x, y AND z), whose cross product
// with the wheel's lateral axis does not scale as simply). S7 ("gamma >= 1")
// does NOT trigger - re-run the zeta sweep diagnostic anyway
// (test_vehicle_standstill_wiggle.cpp's "zeta sweep diagnostic") and see the
// I1b report: the ACTUAL yaw/lateral-kick decay behaviour still does not
// regrow at zeta=1.5 with the patch-point fix in place, consistent with
// this bound (now much closer to 1, but not over it).
TEST_CASE("Low-speed damping explicit-integration stability bound (car_sedan)", "[validation][vehicle]") {
    const real gamma = low_speed_damping_gamma(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                                Vec3{2.0, 0.4, 0.15}, 0.6);
    INFO("car_sedan: gamma=" << gamma);
    CHECK(gamma < 1.0);
}

TEST_CASE("Low-speed damping explicit-integration stability bound (tractor)", "[validation][vehicle]") {
    const real gamma = low_speed_damping_gamma(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json", 6000.0,
                                                Vec3{1.2, 0.35, 0.9}, 1.28);
    INFO("tractor: gamma=" << gamma);
    CHECK(gamma < 1.0);
}
