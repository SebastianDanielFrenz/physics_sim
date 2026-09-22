// adapters/godot/src/collision_import.cpp — see collision_import.h.
//
// The test-ground flat-plane loader reads test_ground.json with
// nlohmann::json - already a pinned FetchContent dependency of this
// repository (core/src/io/scenario.cpp uses it the same way), so linking it
// into the adapter is not a new dependency (PLAN.md P2 rule: "no new
// dependencies beyond godot-cpp").

#include "collision_import.h"
#include "frame_convert.h"

#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <nlohmann/json.hpp>

#include <fstream>

using godot::BoxShape3D;
using godot::CapsuleShape3D;
using godot::CollisionShape3D;
using godot::Node;
using godot::Object;
using godot::SphereShape3D;
using godot::StaticBody3D;

namespace ps_godot {

namespace {

void import_recursive(Node* node, ps::World& world, const ps::Vec3& origin_m, int& created, int& skipped) {
    if (node == nullptr) return;

    if (StaticBody3D* body = Object::cast_to<StaticBody3D>(node)) {
        const int child_count = body->get_child_count();
        for (int i = 0; i < child_count; ++i) {
            CollisionShape3D* col = Object::cast_to<CollisionShape3D>(body->get_child(i));
            if (col == nullptr) continue;
            godot::Ref<godot::Shape3D> shape = col->get_shape();
            if (shape.is_null()) continue;

            ps::BodyDesc desc;
            desc.motion = ps::BodyMotionType::Static;

            bool matched = true;
            if (BoxShape3D* box = Object::cast_to<BoxShape3D>(shape.ptr())) {
                const godot::Vector3 size = box->get_size();
                // ps::BoxShape stores HALF extents; Godot's BoxShape3D::size
                // is the FULL size.
                desc.shape = ps::BoxShape{ps::Vec3{
                    static_cast<ps::real>(size.z) * 0.5, // sim x (forward) <- godot -z (backward) magnitude
                    static_cast<ps::real>(size.x) * 0.5, // sim y (left)    <- godot -x
                    static_cast<ps::real>(size.y) * 0.5, // sim z (up)      <- godot y
                }};
            } else if (SphereShape3D* sph = Object::cast_to<SphereShape3D>(shape.ptr())) {
                desc.shape = ps::SphereShape{static_cast<ps::real>(sph->get_radius())};
            } else if (CapsuleShape3D* cap = Object::cast_to<CapsuleShape3D>(shape.ptr())) {
                desc.shape = ps::CapsuleShape{static_cast<ps::real>(cap->get_height()) * 0.5, static_cast<ps::real>(cap->get_radius())};
            } else {
                matched = false;
            }

            if (!matched) {
                ++skipped;
                continue;
            }

            const godot::Transform3D global = body->get_global_transform() * col->get_transform();
            desc.pose = godot_to_iso_pose(global, origin_m);
            world.create_body(desc);
            ++created;
        }
    }

    const int child_count = node->get_child_count();
    for (int i = 0; i < child_count; ++i) {
        import_recursive(node->get_child(i), world, origin_m, created, skipped);
    }
}

} // namespace

int import_static_collision_from_scene(Node* root, ps::World& world, const ps::Vec3& origin_m, int* out_skipped) {
    int created = 0;
    int skipped = 0;
    import_recursive(root, world, origin_m, created, skipped);
    if (out_skipped != nullptr) *out_skipped = skipped;
    if (skipped > 0) {
        godot::UtilityFunctions::print("ps_godot: import_static_collision_from_scene skipped ", skipped,
                                        " collision shape(s) with no ps::ShapeDesc equivalent (mesh/concave/heightmap - see collision_import.h)");
    }
    return created;
}

ps::BodyId import_test_ground_flat_plane(const std::string& track_dir, ps::World& world) {
    const std::string path = track_dir + "/test_ground.json";
    std::ifstream f(path);
    if (!f.is_open()) {
        godot::UtilityFunctions::print("ps_godot: import_test_ground_flat_plane could not open ", path.c_str());
        return ps::BodyId{};
    }

    nlohmann::json j;
    try {
        f >> j;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::print("ps_godot: import_test_ground_flat_plane failed to parse ", path.c_str(), ": ", e.what());
        return ps::BodyId{};
    }

    const double half_extent_m = j.value("world", nlohmann::json::object()).value("half_extent_m", 50000.0);

    ps::BodyDesc desc;
    desc.motion = ps::BodyMotionType::Static;
    // A thin slab at z=0 (world ENU/ISO 8855 z=up, same axes - see
    // data/tracks/README.md's "Frame" section: no basis change is needed
    // between the track's world frame and a vehicle's own ISO 8855 frame).
    // half_height 0.05 m keeps the slab's top at z=0 (exact on every flat
    // zone) without a visible step.
    constexpr ps::real kSlabHalfHeight = 0.05;
    desc.shape = ps::BoxShape{ps::Vec3{half_extent_m, half_extent_m, kSlabHalfHeight}};
    desc.pose = ps::Pose{ps::Vec3{0.0, 0.0, -kSlabHalfHeight}, ps::Quat::identity()};
    desc.friction = 1.0; // dry asphalt (surfaces.json lambda_mu reference value); see header comment for the coarser-than-README limitation
    return world.create_body(desc);
}

} // namespace ps_godot
