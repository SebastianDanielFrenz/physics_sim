// tests/unit/test_wheel_contact_frame.cpp — PHYS-033 I1b: white-box pins for
// ps::vehicle::WheelInstance's ContactFrame (core/src on this target's own
// include path, tests/unit/CMakeLists.txt - same pattern as
// test_articulation_core.cpp's internal-header access).
//
// Defect 2 (force decomposition, decision D-B) is pinned by cases that land
// at commit C6 (ungated below): the ground-plane normal/long/lat directions
// and the bilateral force assembly. Defect 1 (the contact PATCH, decision
// D-A) needs commit C7's patch-point computation - those assertions live in
// a separate, [!mayfail]-tagged TEST_CASE at the bottom, until C7 removes
// the tag.
//
// Every case below calls measure_contact()/step() directly (no World::step()
// - a purely kinematic/one-shot check), with zero chassis motion and a
// freshly constructed (unrelaxed) tyre state, so MF(0) with SV=0, gamma=0,
// qsx1=0 gives Fx=Fy=Mz=Mx=Froll=0 EXACTLY (pacejka_tyre_model.cpp) - only
// the suspension force (fz) and the contact frame itself are exercised.

#include "vehicle/wheel_instance.h"

#include "ps/io/vehicle_io.h"
#include "ps/math/transcendental.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

using namespace ps;
using namespace ps::vehicle;

namespace {

// Ground: a static BoxShape{50,50,0.5} whose top face passes through the
// world origin BEFORE tilting, then tilted about `tilt_axis` by `theta` -
// exactly tests/support/vehicle_test_helpers.h's build_vehicle_rig
// convention (kept independent here rather than included, since this file
// is unit/white-box, not validation/vehicle-rig level).
BodyId make_tilted_ground(World& world, real theta, Vec3 tilt_axis = Vec3::unit_y()) {
    const Quat tilt = Quat::from_axis_angle(tilt_axis.normalized(), theta);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{50.0, 50.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.orientation = tilt;
    ground.pose.position = -tilt.rotate(Vec3{0.0, 0.0, 0.5});
    return world.create_body(ground);
}

// The car's FL WheelDesc (data/vehicles/car_sedan.json), with attachment
// moved to the chassis origin - the plan's "a WheelInstance built from the
// car FL WheelDesc with attachment (0,0,0)" setup, so every world-space
// number below is easy to reason about directly from the chassis pose.
WheelDesc fl_wheel_desc_at_origin() {
    const VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    WheelDesc w = desc.wheels[0]; // FL
    w.attachment_local = Vec3::zero();
    return w;
}

std::unique_ptr<WheelInstance> make_wheel(const WheelDesc& desc) {
    return std::make_unique<WheelInstance>(desc, create_pacejka_tyre_model(desc.tyre));
}

// Zero motion, zero relaxation state, dt = 1/960 - exactly the plan's "then
// Fx=Fy=Mz=Mx=Froll=0 EXACTLY" setup.
WheelSubstepInput zero_motion_input(const Pose& pose, real steer = 0.0) {
    WheelSubstepInput in;
    in.chassis_pose = pose;
    in.chassis_motion = Motion{};
    in.chassis_com_world = pose.position;
    in.steer_angle = steer;
    in.dt = real(1.0) / real(960.0);
    return in;
}

} // namespace

// --- Case a (C6 portion): level chassis, flat ground - clean normal, no line-contact jitter ---
TEST_CASE("Wheel contact frame: level chassis on flat ground - normal is clean, no line-contact jitter",
          "[vehicle][contact]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    make_tilted_ground(world, 0.0);

    const WheelDesc wd = fl_wheel_desc_at_origin();
    auto wheel = make_wheel(wd);

    // Chassis origin height chosen (as in the plan) so the wheel settles at
    // x=0.06 m of compression purely from the geometry (no dynamics: the
    // shape cast alone determines x here) - R=0.317, travel_max=0.18: cast
    // starts at world z = 0.18+0.257 = 0.437, sweeps down 0.18 to 0.257;
    // it touches (centre height = R = 0.317) at fraction (0.437-0.317)/0.18
    // = 2/3, i.e. x = 0.18*(1/3) = 0.06.
    for (int k = 0; k < 50; ++k) {
        Pose pose;
        pose.position = Vec3{0.0, static_cast<real>(k) * 1e-4, 0.257};
        wheel->measure_contact(pose, 0.0, world.backend());
        REQUIRE(wheel->compression() == Catch::Approx(0.06).margin(1e-4));

        const ContactFrame& cf = wheel->contact_frame();
        CHECK_FALSE(cf.normal_fallback);
        CHECK(std::abs(cf.normal.x) < 1e-4);
        CHECK(std::abs(cf.normal.y) < 1e-4);
        CHECK(cf.normal.z == Catch::Approx(1.0).margin(1e-4));
    }
}

// --- Case e (C6): pitched chassis at rest on flat ground - no spurious horizontal force ---
TEST_CASE("Wheel contact frame: pitched chassis at rest produces no spurious horizontal force",
          "[vehicle][contact]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    make_tilted_ground(world, 0.0);

    const WheelDesc wd = fl_wheel_desc_at_origin();

    for (real x_sign : {real(1.0), real(-1.0)}) {
        auto wheel = make_wheel(wd);
        const real pitch = real(3.0) * real(3.14159265358979323846) / real(180.0);
        const Quat pitch_q = Quat::from_axis_angle(Vec3::unit_y(), pitch);

        // A wheel at chassis-local x = +-1.35 (car_sedan.json's wheelbase
        // half), chassis pitched 3 deg about Y, settled at the same 0.06 m
        // compression as case a (chosen the same way - see that case's
        // comment; the extra x*sin(pitch) term is folded into the height so
        // the wheel still bottoms out at the same compression regardless of
        // which end of the car it is).
        Pose pose;
        pose.orientation = pitch_q;
        const Vec3 base = Vec3{0.0, 0.0, 0.257};
        pose.position = pitch_q.rotate(base) + Vec3{0.0, 0.0, 0.0};
        pose.position.x += x_sign * 1.35; // place the wheel's chassis origin at the car's own front/rear x

        wheel->measure_contact(pose, 0.0, world.backend());
        REQUIRE(wheel->hit_ground());

        WheelSubstepInput in = zero_motion_input(pose);
        in.chassis_com_world = pose.position;
        const WheelSubstepOutput out = wheel->step(in);

        const real fz = out.contact_force_world.length() > 0.0 ? dot(out.contact_force_world, wheel->contact_frame().normal) : 0.0;
        INFO("x_sign=" << x_sign << " R=" << out.contact_force_world.x << "," << out.contact_force_world.y << ","
                        << out.contact_force_world.z << " fz=" << fz);
        CHECK(std::abs(out.contact_force_world.x) < 1e-6 * std::max(fz, real(1.0)));
        CHECK(std::abs(out.contact_force_world.y) < 1e-6 * std::max(fz, real(1.0)));
    }
}

// --- Case f (C6 portion): bottomed start is robust ---
TEST_CASE("Wheel contact frame: a bottomed start is robust (fallback normal, finite everything)",
          "[vehicle][contact]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    // Ground top face 0.05 m ABOVE the wheel's bump position - the cast
    // starts already overlapping the ground (fraction 0).
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{50.0, 50.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.pose.position = Vec3{0.0, 0.0, 0.18 + 0.257 + 0.05 - 0.5};
    world.create_body(ground);

    const WheelDesc wd = fl_wheel_desc_at_origin();
    auto wheel = make_wheel(wd);

    Pose pose;
    pose.position = Vec3{0.0, 0.0, 0.257};
    wheel->measure_contact(pose, 0.0, world.backend());

    REQUIRE(wheel->hit_ground());
    CHECK(wheel->compression() == Catch::Approx(0.18).margin(1e-9)); // fully bottomed
    const ContactFrame& cf = wheel->contact_frame();
    CHECK(cf.normal_fallback);
    CHECK(std::isfinite(cf.point.x));
    CHECK(std::isfinite(cf.point.y));
    CHECK(std::isfinite(cf.point.z));
    CHECK(std::isfinite(cf.normal.x));
    CHECK(std::isfinite(cf.long_dir.x));
    CHECK(std::isfinite(cf.lat_dir.x));
    // Fallback normal is the suspension axis (world-up here, level chassis).
    CHECK(cf.normal.z == Catch::Approx(1.0).margin(1e-6));
}

// --- Cases b, c, d (commit C7, decision D-A): the true contact-patch point ---
// [!mayfail] until C7 lands: at C6, ContactFrame::point is still the wheel
// CENTRE (cf. this file's own header comment and wheel_instance.cpp's
// measure_contact), so every assertion below that the patch differs from
// the centre necessarily fails today. Removed at C7 with no other change to
// this file needed - the setups already probe exactly what D-A specifies.
TEST_CASE("Wheel contact frame: the contact point is the ground patch, not the wheel centre",
          "[vehicle][contact]") {
    const WheelDesc wd = fl_wheel_desc_at_origin();
    constexpr real kR = 0.317; // wd.wheel_radius, spelled out for the arithmetic comments below

    SECTION("case b: 10% cross slope, chassis parallel to the ground") {
        WorldConfig cfg;
        cfg.job_workers = 1;
        World world(cfg);
        const real theta = ps::math::atan(real(0.10));
        const Quat tilt = Quat::from_axis_angle(Vec3::unit_x(), theta);
        make_tilted_ground(world, theta, Vec3::unit_x());

        auto wheel = make_wheel(wd);
        Pose pose;
        pose.orientation = tilt;
        pose.position = tilt.rotate(Vec3{0.0, 0.0, 0.257});
        wheel->measure_contact(pose, 0.0, world.backend());
        REQUIRE(wheel->hit_ground());

        const ContactFrame& cf = wheel->contact_frame();
        const Vec3 expected_n = tilt.rotate(Vec3::unit_z());
        CHECK((cf.normal - expected_n).length() < 1e-4);
        // patch lies on the ground plane (through the world origin here).
        CHECK(std::abs(dot(cf.point, expected_n)) < 1e-5);

        WheelSubstepInput in = zero_motion_input(pose);
        const WheelSubstepOutput out = wheel->step(in);
        CHECK(out.contact_point_world == cf.point);
        const real fz = out.contact_force_world.length();
        if (fz > 1e-6) {
            const Vec3 dir = out.contact_force_world.normalized();
            CHECK(cross(dir, cf.normal).length() < 1e-9);
        }
    }

    SECTION("case c: level chassis on a 10 deg longitudinal slope") {
        WorldConfig cfg;
        cfg.job_workers = 1;
        World world(cfg);
        const real theta = real(10.0) * real(3.14159265358979323846) / real(180.0);
        make_tilted_ground(world, theta, Vec3::unit_y());

        auto wheel = make_wheel(wd);
        Pose pose; // chassis itself stays LEVEL (identity orientation)
        pose.position = Vec3{0.0, 0.0, 0.257};
        wheel->measure_contact(pose, 0.0, world.backend());
        REQUIRE(wheel->hit_ground());

        const ContactFrame& cf = wheel->contact_frame();
        const Vec3 centre = pose.position; // attachment (0,0,0) + axis*x, x measured below
        INFO("patch.x=" << cf.point.x << " centre-ish x=" << centre.x);
        CHECK(std::abs(cf.point.x - centre.x) > 0.05); // R*sin(10deg) ~= 0.055
    }

    SECTION("case d: chassis rolled 3 deg on flat ground (rim contact snap)") {
        WorldConfig cfg;
        cfg.job_workers = 1;
        World world(cfg);
        make_tilted_ground(world, 0.0);

        auto wheel = make_wheel(wd);
        const real roll = real(3.0) * real(3.14159265358979323846) / real(180.0);
        const Quat roll_q = Quat::from_axis_angle(Vec3::unit_x(), roll);
        Pose pose;
        pose.orientation = roll_q;
        pose.position = roll_q.rotate(Vec3{0.0, 0.0, 0.257});
        wheel->measure_contact(pose, 0.0, world.backend());
        REQUIRE(wheel->hit_ground());

        const ContactFrame& cf = wheel->contact_frame();
        CHECK(std::abs(cf.point.z) < 1e-5); // on the (flat, z=0) plane after the snap
        const Vec3 spin = pose.orientation.rotate(Vec3::unit_y());
        (void)kR;
        CHECK(std::abs(dot(cf.point - pose.position, spin)) < 1e-3);
    }
}
