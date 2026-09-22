// ps/articulation/articulation_desc.h — public, additive descriptor types
// for PLAN.md D10/P1.5a: our own reduced-coordinate (Featherstone) solver
// for articulated chains, replacing Jolt joints for anything that needs
// energy-conserving, mass-ratio-robust articulation. See PLAN.md section
// 5 ("Frame and IForceElement seams") and P1.5a's scope list.
//
// Pure data description; no Jolt type, no algorithm code (that lives in
// core/src/articulation, internal). A World consumes an ArticulationDesc
// via World::create_articulation (ps/world/world.h).

#pragma once

#include "ps/math/pose.h"
#include "ps/math/mat3.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

#include <vector>

namespace ps::articulation {

enum class JointType { Revolute, Prismatic, Fixed };

struct JointLimits {
    bool enabled = false;
    real min = 0.0;   // rad (Revolute) or m (Prismatic)
    real max = 0.0;
};

// A force-limited velocity actuator: a hydraulic-cylinder preview (PLAN.md
// P1.5a scope item 2). Drives the joint toward target_speed, but the
// generalized force/torque it may apply is clamped to +-max_force; against
// a load that needs more than max_force to track target_speed, the joint
// stalls at exactly max_force (see tests/validation for the acceptance
// test). enabled == false leaves the joint unactuated by this mechanism
// (World::set_joint_force still applies direct actuation, scope item 2's
// other half).
struct VelocityActuator {
    bool enabled = false;
    real target_speed = 0.0; // rad/s (Revolute) or m/s (Prismatic)
    real max_force = 0.0;    // N*m (Revolute) or N (Prismatic)
};

struct LinkDesc {
    int parent = -1; // index into ArticulationDesc::links; -1 == attaches to the root
    JointType joint = JointType::Revolute;
    Pose joint_frame_in_parent = Pose::identity(); // joint origin, parent-local (parent == root frame if parent == -1)
    Vec3 axis = Vec3::unit_z();                    // joint axis, in the joint frame above (Revolute/Prismatic only)

    real mass = 1.0;
    Vec3 com = Vec3::zero();       // COM offset from this link's own (post-joint) frame origin, link-local
    Mat3 inertia = Mat3::zero();   // rotational inertia about the COM, link-local axes

    JointLimits limits;
    real joint_damping = 0.0; // viscous, N*m*s/rad (Revolute) or N*s/m (Prismatic); default frictionless

    VelocityActuator velocity_actuator;

    real q0 = 0.0;  // initial joint position
    real qd0 = 0.0; // initial joint velocity
};

struct RootDesc {
    bool floating = false;
    Pose pose = Pose::identity(); // Fixed root: permanent world pose. Floating root: initial world pose.
    Motion velocity{};             // Floating root only: initial world-frame spatial velocity.

    // Floating root only: the root body's own rigid-body properties (a
    // fixed root has zero DoF, so its own inertia never enters the
    // dynamics - PLAN.md doesn't require carrying it in that case, but the
    // fields stay present for a uniform ArticulationDesc regardless of
    // root.floating).
    real mass = 1.0;
    Vec3 com = Vec3::zero();     // root-frame-local
    Mat3 inertia = Mat3::zero(); // about com, root-frame-local axes
};

enum class LoopClosureType { Point, Hinge };

// A holonomic constraint between two links (or a link and the world),
// solved at the acceleration level in joint space (PLAN.md P1.5a scope
// item 4; Featherstone ch. 8). Point: anchor_a and anchor_b are pinned
// together (3 constraint equations). Hinge: anchor_a/anchor_b pinned AND
// axis_a/axis_b kept parallel (5 constraint equations - a relative hinge
// DoF remains free about the shared axis).
struct LoopClosureDesc {
    LoopClosureType type = LoopClosureType::Point;
    int link_a = -1; // index into ArticulationDesc::links; -1 == world
    int link_b = -1;
    Vec3 anchor_a = Vec3::zero(); // link_a-local (or world-space if link_a == -1)
    Vec3 anchor_b = Vec3::zero(); // link_b-local (or world-space if link_b == -1)
    Vec3 axis_a = Vec3::unit_z(); // Hinge only, link_a-local
    Vec3 axis_b = Vec3::unit_z(); // Hinge only, link_b-local
};

struct ArticulationDesc {
    RootDesc root;
    std::vector<LinkDesc> links;
    std::vector<LoopClosureDesc> loop_closures;
};

} // namespace ps::articulation
