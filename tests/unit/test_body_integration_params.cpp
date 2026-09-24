// tests/unit/test_body_integration_params.cpp — IRigidBackend::
// get_integration_params (PHYS-033 I1 plan commit C): the exact
// inv_mass/inv_inertia_world/gravity_factor/damping/gyroscopic values the
// backend's own velocity integrator will apply next step(), read BEFORE any
// caller (I1's W1 vehicle shadow chassis) exists - no behaviour change, just
// the query itself.

#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ps;

TEST_CASE("get_integration_params matches a box body's known mass/inertia and round-trips flags", "[world]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    // Same box as tests/support/vehicle_test_helpers.h's car_sedan rig
    // (CLAUDE.md's "Vehicle system" section documents I = 91.25 / 2011.25 /
    // 2080 kg*m^2 for this exact box/mass pair, box-inertia formula
    // I_xx = m/3*(hy^2+hz^2) etc. for half-extents (hx,hy,hz)).
    BodyDesc desc;
    desc.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = 1500.0;
    desc.gravity_enabled = true;
    desc.linear_damping = 0.02;
    desc.angular_damping = 0.03;
    desc.allow_sleep = false;
    const BodyId id = world.create_body(desc);

    const BodyIntegrationParams ip = world.backend().get_integration_params(id);

    REQUIRE(ip.inv_mass == Catch::Approx(1.0 / 1500.0).epsilon(1e-5));
    // Identity orientation at creation: inv_inertia_world == the body-local
    // inverse inertia, a diagonal matrix for a box.
    REQUIRE(ip.inv_inertia_world(0, 0) == Catch::Approx(1.0 / 91.25).epsilon(1e-3));
    REQUIRE(ip.inv_inertia_world(1, 1) == Catch::Approx(1.0 / 2011.25).epsilon(1e-3));
    REQUIRE(ip.inv_inertia_world(2, 2) == Catch::Approx(1.0 / 2080.0).epsilon(1e-3));
    REQUIRE(ip.inv_inertia_world(0, 1) == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(ip.inv_inertia_world(1, 2) == Catch::Approx(0.0).margin(1e-9));

    REQUIRE(ip.gravity_factor == Catch::Approx(1.0));
    REQUIRE(ip.linear_damping == Catch::Approx(0.02));
    REQUIRE(ip.angular_damping == Catch::Approx(0.03));
    REQUIRE_FALSE(ip.gyroscopic);
}

TEST_CASE("get_integration_params: gravity_enabled=false round-trips to gravity_factor=0", "[world]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    BodyDesc desc;
    desc.shape = SphereShape{0.5};
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = 10.0;
    desc.gravity_enabled = false;
    desc.allow_sleep = false;
    const BodyId id = world.create_body(desc);

    const BodyIntegrationParams ip = world.backend().get_integration_params(id);
    REQUIRE(ip.gravity_factor == Catch::Approx(0.0));
}

TEST_CASE("get_integration_params: zero inverse mass for a static body", "[world]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    BodyDesc desc;
    desc.shape = BoxShape{Vec3{1.0, 1.0, 1.0}};
    desc.motion = BodyMotionType::Static;
    const BodyId id = world.create_body(desc);

    const BodyIntegrationParams ip = world.backend().get_integration_params(id);
    REQUIRE(ip.inv_mass == 0.0);
}
