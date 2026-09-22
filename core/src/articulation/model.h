// core/src/articulation/model.h — runtime kinematic tree built from a
// public ps::articulation::ArticulationDesc (internal; see spatial.h for
// the underlying algebra and its Featherstone ch. 2 citations).
//
// Body/frame conventions (used by rnea.h/crba.h/aba.h too):
//  - Bodies are numbered 0..N-1 in a flat array, parent index < child
//    index always (ArticulationDesc::links is required to already be in
//    that order - a parent must be declared before any of its children).
//  - If the root is floating, body 0 IS the root (a 6-DoF joint straight
//    to the fixed world) and ArticulationDesc::links[i] becomes body
//    index (i + 1). If the root is fixed, ArticulationDesc::links[i] is
//    body index i directly, and a link whose parent == -1 attaches
//    straight to the (immovable) world via Xtree - there is no body for
//    the root itself since it has zero DoF.
//  - BodyModel::Xtree is the FIXED part of the parent -> this-body
//    transform (i.e. the joint_frame_in_parent pose, or - only for a
//    parent == -1 link under a FIXED root - that pose composed with the
//    root's own fixed world pose). The q-dependent part is computed by
//    compute_joint_transform() and composed on top of Xtree by
//    compute_xup(); body 0 of a floating root is the one exception,
//    where compute_xup() builds world -> body0 directly from the
//    instance's root_pose (Xtree is unused/identity for it).
//  - A body's own frame IS its link frame (mass/COM/inertia and any
//    Frame a force element attaches to are all expressed there) - i.e.
//    there is no separate fixed offset between "joint output" and "link
//    frame", matching ArticulationDesc::LinkDesc's field list (no such
//    offset is exposed there).

#pragma once

#include "ps/articulation/articulation_desc.h"
#include "spatial.h"

#include <vector>

namespace ps::articulation {

enum class BodyJointKind { Revolute, Prismatic, Fixed, Floating6 };

struct BodyModel {
    int parent = -1; // body index, or -1 for "world" (fixed-root link with LinkDesc::parent == -1)
    BodyJointKind joint = BodyJointKind::Revolute;
    Vec3 axis = Vec3::unit_z();             // 1-DoF joints only, joint-frame-local
    spatial::SpatialTransform Xtree;        // fixed parent -> pre-joint transform (see file comment)
    spatial::SpatialInertia inertia;        // about this body's own frame origin, this body's own axes

    real joint_damping = 0.0;
    JointLimits limits;
    VelocityActuator vel_actuator;

    int dof_index = -1; // index into nv-sized arrays (qd/qdd/tau); -1 if ndof == 0
    int ndof = 0;        // 0 (Fixed), 1 (Revolute/Prismatic), 6 (Floating6, body 0 only)
    int q_index = -1;    // index into the scalar q array; -1 for Fixed and for Floating6 (uses ArticulationState::root_pose)

    int source_link = -1; // index into ArticulationDesc::links, or -1 for the synthetic floating-root body
};

struct Model {
    bool floating_root = false;
    std::vector<BodyModel> bodies; // body[0] == root if floating_root
    int num_q = 0;                 // scalar joint-position array size (Revolute/Prismatic only)
    int num_v = 0;                 // generalized-velocity array size (nv); Floating6 contributes 6
    std::vector<LoopClosureDesc> loop_closures;
};

// Per-instance state the algorithms below read/write. q/qd/qdd/tau are
// sized Model::num_q / Model::num_v respectively; root_pose/root_vel are
// meaningful only when Model::floating_root (root_vel is the root body's
// OWN spatial velocity, expressed in the root's OWN axes - i.e. exactly
// qd[0..5] restated as a SpatialVector, kept alongside q/qd for
// convenience/clarity at call sites).
struct ArticulationState {
    Pose root_pose = Pose::identity();
    std::vector<real> q;
    std::vector<real> qd;
};

[[nodiscard]] Model build_model(const articulation::ArticulationDesc& desc);

// Motion subspace columns for body i (1 column for Revolute/Prismatic, 6
// for Floating6 - the canonical basis of R^6, i.e. the free joint's own
// axes -, 0 for Fixed). Featherstone ch. 2/7's "S" matrix, one spatial
// vector per column.
[[nodiscard]] std::vector<spatial::SpatialVector> motion_subspace(const BodyModel& b);

// q-dependent joint transform only (pre-joint frame -> this body's frame);
// compute_xup() below composes it with Xtree, except for a floating body 0
// which is built directly from ArticulationState::root_pose.
[[nodiscard]] spatial::SpatialTransform compute_joint_transform(const BodyModel& b, const ArticulationState& st);

[[nodiscard]] spatial::SpatialTransform compute_xup(const Model& m, int body_index, const ArticulationState& st);

// this-frame joint velocity contribution S_i * qd_i (a single SpatialVector
// for 1-DoF joints; for Floating6, simply the root's own spatial velocity
// qd[0..5], since S is the identity there).
[[nodiscard]] spatial::SpatialVector joint_velocity(const BodyModel& b, const ArticulationState& st);

} // namespace ps::articulation
