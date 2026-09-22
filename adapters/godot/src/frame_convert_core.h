// adapters/godot/src/frame_convert_core.h — Godot-free numeric core of the
// ONE basis change between the sim's ISO 8855 frame (x forward, y left, z
// up, ps::real/double) and Godot's frame (x right, y up, -z forward,
// float), plus the floating-origin rebase offset (PLAN.md P2: "the
// adapter's single ISO->Godot conversion applies the offset" / "all
// conversion... happens in exactly one place in the adapter").
//
// Kept Godot-type-free (only plain floats/ps:: math types) so it can be
// unit-tested by adapters/godot/tests without linking godot-cpp or loading
// the engine - frame_convert.h (same directory) is the thin godot::Vector3/
// Quaternion/Transform3D-facing wrapper around this file, and does no math
// of its own.
//
// Basis mapping (verified proper/orthogonal, determinant +1, so it never
// mirrors anything - see the derivation in this file's .cpp):
//   godot.x (right)    = -sim.y   (sim left is godot's -x, i.e. sim right = godot +x)
//   godot.y (up)       =  sim.z
//   godot.z (backward) = -sim.x   (sim forward is godot's -z)

#pragma once

#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"

namespace ps_godot {

// A plain float 3-vector - deliberately not ps::Vec3 (that is ps::real =
// double; this is exactly the float32 Godot itself will store) and
// deliberately not any Godot type (see file comment).
struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Three world-space direction columns (local x/y/z axes of a rotated frame,
// expressed in Godot world axes) - exactly what a godot::Basis needs,
// without depending on godot::Basis itself.
struct Basis3f {
    Vec3f col_x;
    Vec3f col_y;
    Vec3f col_z;
};

// Pure direction/basis-change map, no origin, no precision loss beyond the
// final float cast (the input is already whatever precision the caller
// gives it).
[[nodiscard]] Vec3f iso_to_godot_dir(const ps::Vec3& sim_dir);
[[nodiscard]] ps::Vec3 godot_to_iso_dir(const Vec3f& godot_dir);

// Position with floating-origin rebase (PLAN.md P2 large-world paragraph):
// the subtraction against `origin_m` happens in ps::real (double), in sim
// axes, BEFORE the basis change and BEFORE the float32 cast - the whole
// point of origin rebasing is that this subtraction is where precision is
// recovered, so nothing upstream of it may already have lost it.
[[nodiscard]] Vec3f iso_to_godot_position(const ps::Vec3& sim_pos_absolute, const ps::Vec3& origin_m);

// Inverse of the above, used when Godot-authored geometry (e.g. a
// hand-placed StaticBody3D in the demo scene) needs to become an absolute
// sim-space position for ps::BodyDesc::pose (collision_import.h).
[[nodiscard]] ps::Vec3 godot_to_iso_position(const Vec3f& godot_pos, const ps::Vec3& origin_m);

// Orientation: expands q_sim's rotation of the sim's own local x/y/z axes,
// then maps each resulting world direction through iso_to_godot_dir. This
// composes two proper rotations (the sim orientation and the fixed basis
// change), so the result is always a valid orthonormal right-handed basis -
// no separate quaternion-conjugation formula to get subtly wrong.
[[nodiscard]] Basis3f iso_to_godot_basis(const ps::Quat& sim_rot);
[[nodiscard]] ps::Quat godot_to_iso_rotation(const Basis3f& godot_basis);

} // namespace ps_godot
