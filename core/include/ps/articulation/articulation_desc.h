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

#include "ps/backend/shape_desc.h" // ShapeDesc only (PLAN.md D2: no Jolt type here either)
#include "ps/math/pose.h"
#include "ps/math/mat3.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

#include <vector>

namespace ps::articulation {

enum class JointType { Revolute, Prismatic, Fixed };

// PLAN.md P1.5b: how an articulation's links interact with Jolt contacts.
// Selected per articulation (ArticulationDesc::contact_strategy below), not
// globally - a scenario can run one articulation with each strategy side by
// side for the comparison measurements. None (default) preserves every
// pre-P1.5b articulation's behaviour exactly: links do not collide with
// anything.
//
//  - Compliant: Jolt is used only as a narrowphase collision query
//    (IRigidBackend::collide_shape_into) against static and dynamic
//    geometry; contact forces are computed by this codebase (Hunt-Crossley
//    normal force + regularised Coulomb friction, contact.h/.cpp) and
//    applied to the link's Frame, with the reaction applied directly to the
//    other body when it is dynamic.
//  - ProxyBody: each contact-enabled link gets one Jolt body that is pose-
//    and velocity-driven from the link every tick; Jolt solves the contact
//    (so the OTHER body's reaction is exactly Jolt's own solve, not
//    reconstructed), and the resulting velocity change is read back after
//    the step and converted into an external wrench applied to the link
//    next tick.
enum class ContactStrategy { None, Compliant, ProxyBody };

// Hunt-Crossley normal contact + regularised Coulomb friction parameters
// (Compliant strategy) and the proxy body's mass scale (ProxyBody strategy) -
// see contact.cpp's header comment for the exact force law and its sources.
struct ContactMaterial {
    real stiffness = 1.0e6;                    // N/m^stiffness_exponent (Hunt-Crossley k)
    real stiffness_exponent = 1.5;              // Hertzian-contact exponent (Hunt-Crossley n)
    real damping = 5.0e3;                       // N*s/m^stiffness_exponent (Hunt-Crossley b, scales the closing-speed term)
    real friction = 0.8;                        // Coulomb mu
    real friction_regularization_speed = 0.01;  // m/s; below this the friction force blends smoothly to zero (no stick-slip discontinuity)
    real proxy_mass_scale = 1.0;                // ProxyBody only: proxy body mass = link mass * this
};

// Additive per-link contact shape (PLAN.md P1.5b); default-disabled, so an
// ArticulationDesc built before this field existed behaves identically.
struct LinkContactDesc {
    bool enabled = false;
    ShapeDesc shape = SphereShape{};
    Pose shape_offset = Pose::identity(); // link-local (link's own post-joint frame)
    ContactMaterial material{};
};

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

    LinkContactDesc contact; // PLAN.md P1.5b; disabled by default
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

    // PLAN.md P1.5b: None (default) preserves pre-P1.5b behaviour exactly
    // (links never collide, every LinkDesc::contact is ignored). Compliant/
    // ProxyBody only take effect on links whose own LinkContactDesc::enabled
    // is true - a strategy with no contact-enabled link is a documented
    // no-op, same cost as None.
    ContactStrategy contact_strategy = ContactStrategy::None;
};

} // namespace ps::articulation
