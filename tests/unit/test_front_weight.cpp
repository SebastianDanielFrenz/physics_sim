// tests/unit/test_front_weight.cpp — a removable front weight: a wheel-less, machine-less body hung on a locked
// three-point FRONT socket of the tractor (loader: a wheel-less vehicle with a "towed" hitch point). The centre of
// mass of the pair moves forward; the coupling holds it as one body (A1 solver, nothing new).
//
// Recorded sabotage (2026-11-16): the loader gate back to "wheels empty needs a machine" -> FW1 throws on load.

#include "ps/machine/coupling.h"
#include "ps/vehicle/wheel_telemetry.h"
#include "machine_test_rig.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using namespace ps;

namespace {

constexpr real kFrontX = 1.7; // the socket plane, chassis frame of the tractor

void add_front_socket(vehicle::VehicleDesc& v) {
    machine::HitchPointDesc p = v.hitch_points.at(1); // the rear three-point, as a template for class and flags
    p.name = "socket_front";
    p.linkage.reset();
    p.max_tongue_load_n = 0.0;
    p.position_local = Vec3{kFrontX, 0.0, -0.5};
    p.lower_pin_local = Vec3{kFrontX, 0.0, -0.5};
    p.top_pin_local = Vec3{kFrontX, 0.0, 0.1};
    v.hitch_points.push_back(p);
}

struct Pair {
    ps_test::MountRig rig;
    BodyId weight_body{};
    VehicleId weight{};
    HitchId hitch{};
};

Pair build(real weight_kg, bool with_weight) {
    ps_test::MountCfg cfg;
    cfg.bare_tractor = true;
    cfg.edit_tractor = add_front_socket;
    Pair p;
    p.rig = ps_test::build_mount(cfg);
    if (!with_weight) return p;
    vehicle::VehicleDesc w = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/machines/front_weight.json");
    BodyDesc b;
    b.shape = BoxShape{Vec3{0.2, 0.5, 0.25}};
    b.mass = weight_kg;
    b.friction = 0.5;
    b.pose.position = Vec3{kFrontX + 0.4, 0.0, 0.78}; // weight origin: 0.4 m ahead of the plane, level with the tractor's lower pins (found with check_attach)
    p.weight_body = p.rig.world->create_body(b);
    p.weight = p.rig.world->create_vehicle(w, p.weight_body);
    p.hitch = p.rig.world->attach(p.rig.tractor, "socket_front", p.weight, "socket_front");
    return p;
}

} // namespace

TEST_CASE("Front weight: a wheel-less block on the front socket moves the axle loads by statics and stays rigid", "[machine][vehicle]") {
    constexpr real kMass = 900.0;
    Pair without = build(0.0, false);
    without.rig.run(3.0);
    const real f0 = without.rig.axle_load(true), r0 = without.rig.axle_load(false);

    Pair with = build(kMass, true);
    with.rig.run(4.0);
    const real f1 = with.rig.axle_load(true), r1 = with.rig.axle_load(false);

    // Moments about the rear axle: the weight's centre 0.4 m ahead of the plane, the rear axle at x_r, the front at x_f.
    const auto& w = with.rig.world;
    const Vec3 tp = w->get_pose(with.rig.tractor_body).position;
    const Vec3 wp = w->get_pose(with.weight_body).position;
    CAPTURE(f0, r0, f1, r1, wp.x - tp.x);
    REQUIRE(std::abs((f1 + r1) - (f0 + r0) - kMass * 9.81) < 0.02 * kMass * 9.81); // the whole weight reaches the ground
    CHECK(f1 - f0 > 0.9 * kMass * 9.81 * 0.5);                                      // and more than half of it on the front axle
    CHECK(f1 - f0 > r1 - r0);
    // Rigid: the weight keeps its place in the tractor's frame to millimetres and degrees.
    CHECK(std::abs((wp.x - tp.x) - (kFrontX + 0.4)) < 0.01);
    CHECK(std::abs(wp.y - tp.y) < 0.005);
    CHECK(std::abs((wp.z - tp.z) - (0.78 - 1.28)) < 0.01);
    CHECK(w->coupling_state(with.hitch).max_error_m < 0.005);
}

// Negative control: the relaxed gate only opened for a towed hitch point. A wheel-less file with neither a machine block nor a
// towed point (none at all, or only a towing one) is still rejected with the message it always had.
TEST_CASE("Front weight: a wheel-less vehicle with no machine block and no towed hitch point is still rejected", "[machine][vehicle][io]") {
    namespace fs = std::filesystem;
    const std::string tag = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const char* const bodies[] = {
        R"({"format":"physics_sim.vehicle/2","name":"nw","source":"x","wheels":[]})",
        R"({"format":"physics_sim.vehicle/2","name":"nw","source":"x","wheels":[],"hitch_points":[{"name":"h","role":"towing"}]})",
    };
    int n = 0;
    for (const char* body : bodies) {
        const fs::path f = fs::temp_directory_path() / ("ps_front_weight_neg_" + tag + "_" + std::to_string(n++) + ".json");
        { std::ofstream(f) << body; }
        CHECK_THROWS_WITH(io::load_vehicle_json(f.string()), Catch::Matchers::ContainsSubstring("has no 'wheels' array"));
        std::error_code ec;
        fs::remove(f, ec);
    }
}
