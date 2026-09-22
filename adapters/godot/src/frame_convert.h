// adapters/godot/src/frame_convert.h — godot_cpp-facing wrapper around
// frame_convert_core.h. This file (plus its .cpp-free inline bodies below)
// is the ONLY place that touches both a ps:: math type and a godot::
// Variant type for a pose/direction conversion - every other adapter source
// file goes through here rather than converting components by hand
// (PLAN.md P2: "All conversion between the sim's ISO 8855 frame ... and
// Godot's ... happens in exactly one place in the adapter").

#pragma once

#include "frame_convert_core.h"

#include "ps/math/pose.h"

#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace ps_godot {

inline godot::Vector3 to_godot(const Vec3f& v) { return godot::Vector3(v.x, v.y, v.z); }

inline godot::Vector3 iso_to_godot(const ps::Vec3& sim_pos_absolute, const ps::Vec3& origin_m) {
    return to_godot(iso_to_godot_position(sim_pos_absolute, origin_m));
}

inline ps::Vec3 godot_to_iso(const godot::Vector3& godot_pos, const ps::Vec3& origin_m) {
    return godot_to_iso_position(Vec3f{godot_pos.x, godot_pos.y, godot_pos.z}, origin_m);
}

inline godot::Basis to_godot_basis(const ps::Quat& sim_rot) {
    const Basis3f b = iso_to_godot_basis(sim_rot);
    // godot::Basis(x_axis, y_axis, z_axis) takes the three COLUMN vectors.
    return godot::Basis(to_godot(b.col_x), to_godot(b.col_y), to_godot(b.col_z));
}

inline godot::Transform3D iso_to_godot_transform(const ps::Pose& sim_pose, const ps::Vec3& origin_m) {
    return godot::Transform3D(to_godot_basis(sim_pose.orientation), iso_to_godot(sim_pose.position, origin_m));
}

// Inverse direction, used by collision_import.h to turn Godot-authored
// static geometry (StaticBody3D/CollisionShape3D) into ps::BodyDesc poses.
inline Basis3f from_godot_basis(const godot::Basis& b) {
    const godot::Vector3 cx = b.get_column(0);
    const godot::Vector3 cy = b.get_column(1);
    const godot::Vector3 cz = b.get_column(2);
    return Basis3f{Vec3f{cx.x, cx.y, cx.z}, Vec3f{cy.x, cy.y, cy.z}, Vec3f{cz.x, cz.y, cz.z}};
}

inline ps::Quat godot_to_iso_rotation(const godot::Basis& b) { return ps_godot::godot_to_iso_rotation(from_godot_basis(b)); }

inline ps::Pose godot_to_iso_pose(const godot::Transform3D& t, const ps::Vec3& origin_m) {
    return ps::Pose{godot_to_iso(t.origin, origin_m), godot_to_iso_rotation(t.basis)};
}

} // namespace ps_godot
