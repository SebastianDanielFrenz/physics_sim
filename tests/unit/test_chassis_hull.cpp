// Chassis collision hull (racing_game owner report 2026-11-03: on foot the player cannot climb onto the hypercar, a
// flipped car lands on the whole chassis up to the wheels instead of its roof, and an obstacle meets the wheels
// before the body). The chassis body used to be a thin box kept inside the wheels' travel envelope, because a wheel
// cast that starts inside its own chassis reads it as ground (vault PHYS-017). Now every wheel cast excludes the own
// chassis, so the chassis can carry a hull of the real outline: data/collision/car_hyper_hull.json
// (tools/collisiongen/chassis_hull.py), a ConvexCompoundShape whose mass properties come from the authored box
// (BodyDesc::mass_shape).
//
// H1  hull + box mass properties: same mass, inertia, mass centre and free flight as the plain box body; set_mass too.
// H2  a settled car_hyper on the hull carries the same corner loads and ride height as on the old box (the hull never
//     touches flat ground): measured 3090.16 / 3090.15 / 4268.47 / 4267.36 N and 0.500 m on both.
//     Recorded sabotage (2026-11-03): the own-chassis exclusion removed in VehicleInstance's wheel pass (an empty span
//     instead of the chassis) -> every wheel cast starts inside the hull and reads it as ground: corner loads 18352 /
//     18352 / 22943 / 776221 N, the car is thrown to z 15.9 m; H2, H3, H4 (wheels loaded while inverted) and H5 fail.
// H3  roof: a ray from above lands on the roof, a walker-sized capsule just below the roof overlaps the chassis with
//     a walkable normal, and a dropped 80 kg box comes to rest on the roof.
// H4  flipped: an inverted car rests on its roof (chassis on the ground, every wheel clear): origin 0.609 m up.
// H5  a 0.35 m barrier: driven into it at 3 m/s, the nose meets it before any wheel loads up (contact at tick 119,
//     no wheel compressed); on the old box the barrier passes under the chassis and a front wheel climbs it at tick
//     174 (the reported symptom).

#include "engine_map_test_cache.h"
#include "vehicle_test_helpers.h"

#include "ps/io/chassis_hull_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using namespace ps;

namespace {

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
constexpr real kHyperMass = 1500.0;               // car_hyper.json "source": chassis mass
const Vec3 kHyperBox{2.0, 0.4, 0.12};              // racing_game's car_hyper chassis box (catalog half_extents)
constexpr real kHyperSpawnZ = 0.5;                 // car_hyper.json: chassis spawn height

const ConvexCompoundShape& hyper_hull() {
    static const ConvexCompoundShape h = io::load_chassis_hull_json(kData + "/collision/car_hyper_hull.json");
    return h;
}

const vehicle::VehicleDesc& hyper_desc() {
    static const vehicle::VehicleDesc d = io::load_vehicle_json(kData + "/vehicles/car_hyper.json", ps_test::shared_map_options());
    return d;
}

BodyDesc chassis_desc(bool hull, const Pose& pose) {
    BodyDesc c;
    c.motion = BodyMotionType::Dynamic;
    c.mass = kHyperMass;
    c.friction = 0.5;
    c.allow_sleep = false;
    c.pose = pose;
    c.shape = BoxShape{kHyperBox};
    if (hull) {
        c.shape = hyper_hull();
        c.mass_shape = BoxShape{kHyperBox};
    }
    return c;
}

struct Rig {
    std::unique_ptr<World> world;
    BodyId chassis;
    VehicleId vehicle;
    std::shared_ptr<const io::SurfaceTable> surfaces;
};

Rig hyper_rig(bool hull, const Pose& pose) {
    Rig r;
    WorldConfig cfg;
    cfg.job_workers = ps_test::kRigDefaultWorkers;
    r.world = std::make_unique<World>(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    r.world->create_body(ground);
    r.chassis = r.world->create_body(chassis_desc(hull, pose));
    r.surfaces = std::make_shared<io::SurfaceTable>(kData + "/surfaces/surfaces.json");
    r.world->set_surface_table(r.surfaces);
    r.vehicle = r.world->create_vehicle(hyper_desc(), r.chassis);
    return r;
}

Pose at(real x, real y, real z) {
    Pose p;
    p.position = Vec3{x, y, z};
    return p;
}

bool chassis_touches(const World& w, BodyId chassis, BodyId other) {
    for (const ContactEvent& e : w.last_contact_events()) {
        if ((e.a == chassis && e.b == other) || (e.b == chassis && e.a == other)) return true;
    }
    return false;
}

} // namespace

TEST_CASE("H1 hull with box mass properties flies like the box", "[vehicle][backend][chassis_hull]") {
    const ConvexCompoundShape& hull = hyper_hull();
    REQUIRE(hull.pieces.size() == 10);

    WorldConfig cfg;
    cfg.gravity = Vec3::zero();
    World w(cfg);
    Pose p = at(0.0, 0.0, 10.0);
    BodyDesc box = chassis_desc(false, p);
    BodyDesc hb = chassis_desc(true, at(0.0, 50.0, 10.0));
    box.velocity.linear = hb.velocity.linear = Vec3{3.0, -1.0, 0.5};
    box.velocity.angular = hb.velocity.angular = Vec3{1.3, -0.7, 2.1};
    const BodyId a = w.create_body(box);
    const BodyId b = w.create_body(hb);

    const BodyIntegrationParams pa = w.backend().get_integration_params(a);
    const BodyIntegrationParams pb = w.backend().get_integration_params(b);
    CHECK(pa.inv_mass == pb.inv_mass);
    for (int i = 0; i < 3; ++i) {
        CHECK(pa.inv_inertia_world.rows[i].x == pb.inv_inertia_world.rows[i].x);
        CHECK(pa.inv_inertia_world.rows[i].y == pb.inv_inertia_world.rows[i].y);
        CHECK(pa.inv_inertia_world.rows[i].z == pb.inv_inertia_world.rows[i].z);
    }

    for (int i = 0; i < 240; ++i) w.step();
    const Pose qa = w.backend().get_pose(a);
    const Pose qb = w.backend().get_pose(b);
    CHECK(qa.position.x == qb.position.x);
    CHECK(qb.position.y - qa.position.y == 50.0);
    CHECK(qa.position.z == qb.position.z);
    CHECK(qa.orientation.w == qb.orientation.w);
    CHECK(qa.orientation.x == qb.orientation.x);
    CHECK(qa.orientation.y == qb.orientation.y);
    CHECK(qa.orientation.z == qb.orientation.z);

    // contents_mass: set_mass rescales the mass shape, not the hull.
    w.backend().set_mass(a, 1700.0);
    w.backend().set_mass(b, 1700.0);
    const BodyIntegrationParams sa = w.backend().get_integration_params(a);
    const BodyIntegrationParams sb = w.backend().get_integration_params(b);
    CHECK(sa.inv_mass == sb.inv_mass);
    const Mat3 ia = w.backend().get_inertia_world(a);
    const Mat3 ib = w.backend().get_inertia_world(b);
    for (int i = 0; i < 3; ++i) CHECK((ia.rows[i] - ib.rows[i]).length() <= 1e-9 * ia.rows[i].length());
}

TEST_CASE("H2 settled hyper on the hull: corner loads and ride height as on the box", "[vehicle][chassis_hull]") {
    Rig box = hyper_rig(false, at(0.0, 0.0, kHyperSpawnZ));
    Rig hull = hyper_rig(true, at(0.0, 0.0, kHyperSpawnZ));
    box.world->set_control("handbrake", 1.0);
    hull.world->set_control("handbrake", 1.0);
    ps_test::settle(*box.world, 3.0);
    ps_test::settle(*hull.world, 3.0);

    real sum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        const real fb = box.world->wheel_telemetry(box.vehicle, i).fz;
        const real fh = hull.world->wheel_telemetry(hull.vehicle, i).fz;
        INFO("wheel " << i << " box " << fb << " hull " << fh);
        CHECK(std::abs(fh - fb) <= 1e-6 * fb);
        sum += fh;
    }
    CHECK(std::abs(sum - kHyperMass * 9.81) <= 0.02 * kHyperMass * 9.81);
    const real zb = box.world->backend().get_pose(box.chassis).position.z;
    const real zh = hull.world->backend().get_pose(hull.chassis).position.z;
    INFO("ride height box " << zb << " hull " << zh);
    CHECK(std::abs(zh - zb) <= 1e-6);
    CHECK(std::abs(zh - kHyperSpawnZ) <= 0.01); // car_hyper.json: attachment z chosen to settle at the spawn height
    // Nothing but the wheels carries it: no chassis-ground contact on the hull.
    CHECK(hull.world->last_contact_events().empty());
}

TEST_CASE("H3 roof: ray, walker capsule and a dropped box land on the roof", "[vehicle][chassis_hull]") {
    Rig r = hyper_rig(true, at(0.0, 0.0, kHyperSpawnZ));
    r.world->set_control("handbrake", 1.0);
    ps_test::settle(*r.world, 2.0);
    const real cz = r.world->backend().get_pose(r.chassis).position.z;
    const real roof_z = cz + 0.62; // hull roof 0.628 m above the body origin, less the convex radius rounding

    const RayCastHit ray = r.world->backend().ray_cast(Vec3{0.2, 0.0, cz + 5.0}, Vec3{0.0, 0.0, -1.0}, 10.0);
    REQUIRE(ray.hit);
    CHECK(ray.body == r.chassis);
    CHECK(std::abs(ray.point.z - roof_z) <= 0.03);

    // racing_game's walker: capsule r 0.30, 1.75 m tall, resolved with collide_shape_into.
    const CapsuleShape walker{0.575, 0.30};
    Pose capsule_pose;
    capsule_pose.orientation = Quat::from_axis_angle(Vec3::unit_x(), 1.5707963267948966); // capsule axis Y -> Z
    std::vector<ShapeContactHit> hits;
    capsule_pose.position = Vec3{0.2, 0.0, roof_z - 0.04 + 0.875};
    r.world->backend().collide_shape_into(ShapeDesc{walker}, capsule_pose, hits);
    const auto on_chassis = std::find_if(hits.begin(), hits.end(), [&](const ShapeContactHit& h) { return h.body == r.chassis; });
    REQUIRE(on_chassis != hits.end());
    CHECK(std::abs(on_chassis->normal.z) > 0.7); // walkable (sign follows the backend's pair order)
    hits.clear();
    capsule_pose.position.z = roof_z + 0.05 + 0.875;
    r.world->backend().collide_shape_into(ShapeDesc{walker}, capsule_pose, hits);
    CHECK(std::none_of(hits.begin(), hits.end(), [&](const ShapeContactHit& h) { return h.body == r.chassis; }));

    BodyDesc crate;
    crate.shape = BoxShape{Vec3{0.2, 0.2, 0.1}};
    crate.mass = 80.0;
    crate.friction = 0.8;
    crate.pose.position = Vec3{0.2, 0.0, roof_z + 1.0};
    const BodyId c = r.world->create_body(crate);
    ps_test::settle(*r.world, 2.0);
    const Pose cp = r.world->backend().get_pose(c);
    INFO("crate z " << cp.position.z << " roof " << roof_z);
    CHECK(cp.position.z - 0.1 > roof_z - 0.06);
    CHECK(cp.position.z - 0.1 < roof_z + 0.03);
    CHECK(r.world->backend().get_motion(c).linear.length() < 0.05);
}

TEST_CASE("H4 an inverted hyper rests on its roof", "[vehicle][chassis_hull]") {
    Pose p = at(0.0, 0.0, 1.2);
    p.orientation = Quat::from_axis_angle(Vec3::unit_x(), 3.141592653589793);
    Rig r = hyper_rig(true, p);
    ps_test::settle(*r.world, 4.0);
    const Pose q = r.world->backend().get_pose(r.chassis);
    const Vec3 up = q.orientation.rotate(Vec3::unit_z());
    INFO("origin z " << q.position.z << " up z " << up.z);
    CHECK(up.z < -0.95);
    CHECK(q.position.z > 0.55); // roof 0.628 m from the origin; the old box rested at 0.12
    CHECK(q.position.z < 0.66);
    for (std::size_t i = 0; i < 4; ++i) CHECK(r.world->wheel_telemetry(r.vehicle, i).fz == 0.0);
    CHECK(r.world->backend().get_motion(r.chassis).linear.length() < 0.05);
}

TEST_CASE("H5 a low barrier meets the nose before a wheel", "[vehicle][chassis_hull]") {
    for (const bool hull : {true, false}) {
        Rig r = hyper_rig(hull, at(0.0, 0.0, kHyperSpawnZ));
        BodyDesc barrier;
        barrier.shape = BoxShape{Vec3{0.25, 2.0, 0.175}};
        barrier.motion = BodyMotionType::Static;
        barrier.pose.position = Vec3{4.25, 0.0, 0.175}; // face at x 4.0, 0.35 m high
        const BodyId wall = r.world->create_body(barrier);
        ps_test::settle(*r.world, 2.0);
        real static_compression[4];
        for (std::size_t i = 0; i < 4; ++i) static_compression[i] = r.world->wheel_telemetry(r.vehicle, i).compression;

        Motion m = r.world->backend().get_motion(r.chassis);
        m.linear = Vec3{3.0, 0.0, 0.0};
        r.world->backend().set_motion(r.chassis, m);
        int chassis_tick = -1;
        int wheel_tick = -1;
        for (int t = 0; t < 480 && chassis_tick < 0 && wheel_tick < 0; ++t) {
            r.world->step();
            if (chassis_touches(*r.world, r.chassis, wall)) chassis_tick = t;
            for (std::size_t i = 0; i < 4; ++i) {
                if (r.world->wheel_telemetry(r.vehicle, i).compression > static_compression[i] + 0.03) wheel_tick = t;
            }
        }
        INFO((hull ? "hull" : "box") << ": chassis contact tick " << chassis_tick << ", wheel climbs tick " << wheel_tick);
        if (hull) {
            CHECK(chassis_tick >= 0);
            CHECK(wheel_tick < 0);
        } else {
            CHECK(chassis_tick < 0); // the reported symptom: the barrier passes under the box and a wheel climbs it
            CHECK(wheel_tick >= 0);
        }
    }
}
