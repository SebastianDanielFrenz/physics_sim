// adapters/godot/src/collision_import.cpp — see collision_import.h.
//
// import_test_ground_real (PLAN.md M1) delegates the actual file parsing to
// core's own loaders (ps::io::load_test_ground_collision, ps::terrain::
// load_procedural_terrain_source, ps::io::SurfaceTable) - this file only
// wires them into a World and reports the result to the Godot console.

#include "collision_import.h"
#include "frame_convert.h"

#include "ps/io/test_ground_loader.h"
#include "ps/terrain/procedural_terrain.h"

#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

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

TestGroundLoadResult import_test_ground_real(const std::string& track_dir,
                                              ps::World& world,
                                              std::shared_ptr<const ps::io::SurfaceTable>& out_surfaces,
                                              const ps::terrain::TerrainConfig& terrain_config,
                                              std::uint32_t samples_per_tile) {
    TestGroundLoadResult result;

    // data/tracks/<track>/ -> data/surfaces/surfaces.json (two levels up,
    // then into surfaces/ - same relation data/tracks/README.md documents:
    // "data/surfaces/surfaces.json is the *one* shared table ... referenced
    // rather than embedding a copy").
    const std::string surfaces_path = track_dir + "/../../surfaces/surfaces.json";
    std::shared_ptr<ps::io::SurfaceTable> surfaces;
    try {
        surfaces = std::make_shared<ps::io::SurfaceTable>(surfaces_path);
    } catch (const std::exception& e) {
        result.error = std::string("SurfaceTable load failed (") + surfaces_path + "): " + e.what();
        godot::UtilityFunctions::print("ps_godot: import_test_ground_real: ", result.error.c_str());
        return result;
    }

    try {
        const std::vector<ps::BodyId> bodies = ps::io::load_test_ground_collision(world, track_dir, *surfaces);
        result.collision_bodies = static_cast<int>(bodies.size());
    } catch (const std::exception& e) {
        result.error = std::string("load_test_ground_collision failed: ") + e.what();
        godot::UtilityFunctions::print("ps_godot: import_test_ground_real: ", result.error.c_str());
        return result;
    }

    std::shared_ptr<ps::terrain::ITerrainSource> terrain_source;
    try {
        terrain_source = ps::terrain::load_procedural_terrain_source(track_dir + "/terrain.json", samples_per_tile);
    } catch (const std::exception& e) {
        result.error = std::string("load_procedural_terrain_source failed: ") + e.what();
        godot::UtilityFunctions::print("ps_godot: import_test_ground_real: ", result.error.c_str());
        return result;
    }

    world.set_surface_table(surfaces);
    world.set_terrain_source(terrain_source, terrain_config);

    out_surfaces = std::move(surfaces);
    result.ok = true;
    godot::UtilityFunctions::print("ps_godot: import_test_ground_real: ", result.collision_bodies,
                                    " collision body/bodies, terrain source installed from ", track_dir.c_str());
    return result;
}

} // namespace ps_godot
