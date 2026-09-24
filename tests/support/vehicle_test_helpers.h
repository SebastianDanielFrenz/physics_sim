// tests/support/vehicle_test_helpers.h — shared setup for the PLAN.md P3
// vehicle validation tests: a World with a static ground box (flat or
// tilted) and one vehicle built directly from a data/vehicles/*.json file
// (not through a Scenario - the tests drive World::set_control /
// World::step themselves so each acceptance criterion can assert on exact
// telemetry rather than parsing a CSV).

#pragma once

#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/math/transcendental.h"
#include "ps/world/world.h"

#include <memory>
#include <string>

namespace ps_test {

// A large flat MeshShape ground tagged with a single SurfaceId on every
// triangle (mirrors tests/unit/test_mesh_heightfield_shapes.cpp's
// flat_quad_mesh) - used where a test needs the wheel/tyre surface lookup
// (SurfaceTable::lambda_mu_for/crr_for) to resolve to something other than
// kInvalidSurfaceId (a plain BoxShape ground, as build_vehicle_rig below
// uses by default, carries no Jolt material and so always reports
// kInvalidSurfaceId - fine for tests that don't care about the surface, not
// for one comparing behaviour across surfaces).
inline ps::MeshShape flat_surface_mesh(ps::real half_extent, ps::SurfaceId surface) {
    ps::MeshShape m;
    m.vertices = {
        ps::Vec3{-half_extent, -half_extent, 0.0},
        ps::Vec3{half_extent, -half_extent, 0.0},
        ps::Vec3{half_extent, half_extent, 0.0},
        ps::Vec3{-half_extent, half_extent, 0.0},
    };
    m.indices = {0, 1, 2, 0, 2, 3}; // CCW viewed from +z, Jolt's MeshShapeSettings convention
    m.triangle_surface_ids = {surface, surface};
    return m;
}

struct VehicleTestRig {
    std::unique_ptr<ps::World> world;
    ps::BodyId chassis;
    ps::VehicleId vehicle;
    std::shared_ptr<const ps::io::SurfaceTable> surfaces;
};

// slope_grade: rise/run (0 == flat). The ground box is tilted about world
// Y so the surface DESCENDS in +X (i.e. +X is downhill - found the hard
// way analysing the parking-slope creep tests, where the sign of this
// tilt matters: see test_vehicle_parking_slope.cpp's own comment);
// chassis_origin_height is the vehicle's own
// chassis-local origin height above the LOCAL ground normal at x=0
// (matches the vehicle JSON's own attachment_local.z convention - see
// data/vehicles/car_sedan.json's header comment for how it was derived).
//
// IMPORTANT (found the hard way - PLAN.md P3): chassis_half_extents must
// NOT reach down/out far enough to overlap any wheel's suspension-travel
// envelope (the cylinder shape cast in wheel_instance.cpp's
// measure_contact starts at the "full bump" wheel position and sweeps
// toward "full extension"). IRigidBackend::shape_cast has no way to
// exclude a specific body from the query, and Jolt's CastShape reports an
// immediate fraction-0 "hit" against a shape the query already starts
// inside of (inconsistently, depending on broadphase iteration order in
// this codebase's observed case - with the chassis box just barely
// overlapping, 3 of 4 wheels reported the spurious self-hit at zero
// steer, and a DIFFERENT single wheel did once a steer angle rotated its
// query cylinder enough to grow its footprint back into the margin) - the
// wheel then reads back as fully bottomed against its own chassis, and
// the resulting huge/wrong Fz feeds a runaway tyre force and yaw spin.
// Keep the chassis collision box narrower/higher than the wheel
// attachment envelope by a margin comfortably bigger than one wheel
// radius (a steered wheel's query cylinder sweeps a cone-like footprint
// as it rotates, not just its unrotated bounding box) - exactly like a
// real chassis "belly" sits well above and inboard of the wheels. This is
// not merely a test convenience: any scenario author assembling a real
// chassis body around this vehicle system has the same obligation.
// PLAN.md P4 I1 (PHYS-033): the VehicleDesc-taking core of build_vehicle_rig
// below, split out so a test can mutate a loaded desc (e.g. zeta = 0,
// qsy = 0, an attachment z) before building the rig, and so a test can run
// at a finer tick/substep rate (tick_hz/substep_hz) for the "does the W1
// result move toward the converged solution" check (I1 plan section 5.1) -
// never committed as a test's normal rate, just used locally to produce a
// reference number. `tilt_axis` generalises the old hard-coded world-Y tilt
// (a vehicle facing +X driving up/down a longitudinal grade) to an
// arbitrary axis - unit_x() gives a CROSS slope for a vehicle facing +X
// (rises in +Y - this direction was already correct), unit_y() (the
// default, matching the old behaviour exactly) a longitudinal slope that
// DESCENDS in +X (the direction this comment used to get backwards - i.e.
// +X is downhill, not uphill).
// `ground_half_extent` (PHYS-033 P4 I3 plan section 13's own request,
// default 1000 - matches every existing caller exactly): the accel/top-speed
// validation tests (13.1/13.2) drive far enough (their own 2000/4000 m
// callers) that the old hardcoded 1000 m half-extent would run the car off
// the edge exactly like the original 60x60 m box did (see the comment
// below).
inline VehicleTestRig build_vehicle_rig_from_desc(const ps::vehicle::VehicleDesc& desc, ps::real chassis_mass,
                                                   const ps::Vec3& chassis_half_extents, ps::real chassis_origin_height,
                                                   ps::real slope_grade = 0.0, int job_workers = 4,
                                                   ps::Vec3 tilt_axis = ps::Vec3::unit_y(),
                                                   ps::real tick_hz = 240.0, ps::real substep_hz = 960.0,
                                                   ps::real ground_half_extent = 1000.0) {
    VehicleTestRig rig;

    ps::WorldConfig cfg;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
    cfg.job_workers = job_workers;
    rig.world = std::make_unique<ps::World>(cfg);

    const ps::real theta = ps::math::atan(slope_grade);
    const ps::Quat tilt = ps::Quat::from_axis_angle(tilt_axis.normalized(), theta);

    ps::BodyDesc ground;
    // ground_half_extent x ground_half_extent m (default 1000): comfortably
    // larger than any distance a validation test drives (found the hard way
    // - PLAN.md P3's braking test drove a car off the edge of an earlier
    // 60x60 m ground box at ~57 m into an 8 s run, sending it into free fall
    // - which looked like a runaway instability in "speed" until traced to
    // vz, not a physics bug at all).
    ground.shape = ps::BoxShape{ps::Vec3{ground_half_extent, ground_half_extent, 0.5}};
    ground.motion = ps::BodyMotionType::Static;
    ground.friction = 1.0;
    // Placed so the point directly "above" world (0,0,0) along the box's
    // own local +Z (i.e. local point (0,0,0.5)) lands exactly at world
    // (0,0,0) - see out/handover_p3.md's parking-slope scenario note.
    ground.pose.orientation = tilt;
    ground.pose.position = -tilt.rotate(ps::Vec3{0.0, 0.0, 0.5});
    rig.world->create_body(ground);

    ps::BodyDesc chassis;
    chassis.shape = ps::BoxShape{chassis_half_extents};
    chassis.motion = ps::BodyMotionType::Dynamic;
    chassis.mass = chassis_mass;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.orientation = tilt;
    chassis.pose.position = tilt.rotate(ps::Vec3{0.0, 0.0, chassis_origin_height});
    rig.chassis = rig.world->create_body(chassis);

    rig.surfaces = std::make_shared<ps::io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    rig.world->set_surface_table(rig.surfaces);

    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);

    return rig;
}

inline VehicleTestRig build_vehicle_rig(const std::string& vehicle_json_path, ps::real chassis_mass,
                                         const ps::Vec3& chassis_half_extents, ps::real chassis_origin_height,
                                         ps::real slope_grade = 0.0, int job_workers = 4,
                                         ps::Vec3 tilt_axis = ps::Vec3::unit_y(), ps::real tick_hz = 240.0,
                                         ps::real substep_hz = 960.0, ps::real ground_half_extent = 1000.0) {
    const ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(vehicle_json_path);
    return build_vehicle_rig_from_desc(desc, chassis_mass, chassis_half_extents, chassis_origin_height, slope_grade,
                                        job_workers, tilt_axis, tick_hz, substep_hz, ground_half_extent);
}

// Same as build_vehicle_rig but for a FLAT ground tagged with a specific
// SurfaceId (via flat_surface_mesh above) instead of a plain untagged
// BoxShape - for tests that compare behaviour across surfaces (e.g. full-
// throttle acceleration on asphalt vs. grass/dirt, which needs
// SurfaceTable::lambda_mu_for/crr_for to resolve to something other than
// kInvalidSurfaceId). No slope_grade parameter: a tilted MeshShape ground
// is not needed by any current test.
// PHYS-033 I1b re-checks (a)/(b): tick_hz/substep_hz added as trailing
// optional params (default 240/960, matching every existing caller exactly -
// additive, hash-neutral) so a fine-tick reference run (e.g. 960/3840, the
// same "does the result converge" technique build_vehicle_rig_from_desc
// already offers) can also pick a non-asphalt surface.
inline VehicleTestRig build_vehicle_rig_on_surface(const std::string& vehicle_json_path, ps::real chassis_mass,
                                                     const ps::Vec3& chassis_half_extents, ps::real chassis_origin_height,
                                                     const std::string& surface_name, int job_workers = 4,
                                                     ps::real tick_hz = 240.0, ps::real substep_hz = 960.0) {
    VehicleTestRig rig;

    ps::WorldConfig cfg;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
    cfg.job_workers = job_workers;
    rig.world = std::make_unique<ps::World>(cfg);

    rig.surfaces = std::make_shared<ps::io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const ps::SurfaceId ground_surface = rig.surfaces->id_for(surface_name);

    ps::BodyDesc ground;
    ground.shape = flat_surface_mesh(1000.0, ground_surface); // see build_vehicle_rig's own comment on why 1000 m
    ground.motion = ps::BodyMotionType::Static;
    ground.friction = 1.0;
    rig.world->create_body(ground);

    ps::BodyDesc chassis;
    chassis.shape = ps::BoxShape{chassis_half_extents};
    chassis.motion = ps::BodyMotionType::Dynamic;
    chassis.mass = chassis_mass;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = ps::Vec3{0.0, 0.0, chassis_origin_height};
    rig.chassis = rig.world->create_body(chassis);

    rig.world->set_surface_table(rig.surfaces);

    const ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(vehicle_json_path);
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);

    return rig;
}

inline void settle(ps::World& world, ps::real seconds) {
    const std::uint64_t n = static_cast<std::uint64_t>(seconds / world.dt() + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) world.step();
}

} // namespace ps_test
