#include "model.h"

#include <cassert>

namespace ps::articulation {

using spatial::SpatialTransform;
using spatial::SpatialVector;

Model build_model(const articulation::ArticulationDesc& desc) {
    Model m;
    m.floating_root = desc.root.floating;
    m.loop_closures = desc.loop_closures;

    const int link_offset = m.floating_root ? 1 : 0;
    m.bodies.reserve(desc.links.size() + static_cast<std::size_t>(link_offset));

    if (m.floating_root) {
        BodyModel root;
        root.parent = -1;
        root.joint = BodyJointKind::Floating6;
        root.inertia = spatial::SpatialInertia::from_com(desc.root.mass, desc.root.com, desc.root.inertia);
        root.dof_index = 0;
        root.ndof = 6;
        root.q_index = -1;
        root.source_link = -1;
        m.num_v = 6;
        m.bodies.push_back(root);
    }

    for (std::size_t i = 0; i < desc.links.size(); ++i) {
        const LinkDesc& ld = desc.links[i];
        BodyModel b;
        b.parent = (ld.parent < 0) ? (m.floating_root ? 0 : -1) : (ld.parent + link_offset);
        b.axis = ld.axis.normalized();
        b.inertia = spatial::SpatialInertia::from_com(ld.mass, ld.com, ld.inertia);
        b.joint_damping = ld.joint_damping;
        b.limits = ld.limits;
        b.vel_actuator = ld.velocity_actuator;
        b.source_link = static_cast<int>(i);

        // Xtree: fixed parent -> pre-joint transform. If this link attaches
        // directly to a FIXED root (parent == -1 and !floating_root), fold
        // the root's own permanent world pose in too, since there is no
        // body-0 slot to carry it in that case (see model.h file comment).
        if (ld.parent < 0 && !m.floating_root) {
            const Pose pre_joint_world = desc.root.pose * ld.joint_frame_in_parent;
            b.Xtree = spatial::spatial_transform_from_pose(pre_joint_world);
        } else {
            b.Xtree = spatial::spatial_transform_from_pose(ld.joint_frame_in_parent);
        }

        switch (ld.joint) {
            case JointType::Revolute:
                b.joint = BodyJointKind::Revolute;
                b.ndof = 1;
                break;
            case JointType::Prismatic:
                b.joint = BodyJointKind::Prismatic;
                b.ndof = 1;
                break;
            case JointType::Fixed:
                b.joint = BodyJointKind::Fixed;
                b.ndof = 0;
                break;
        }

        if (b.ndof == 1) {
            b.q_index = m.num_q++;
            b.dof_index = m.num_v++;
        }

        m.bodies.push_back(b);
    }

    return m;
}

std::vector<SpatialVector> motion_subspace(const BodyModel& b) {
    switch (b.joint) {
        case BodyJointKind::Revolute:
            return {SpatialVector{b.axis, Vec3::zero()}};
        case BodyJointKind::Prismatic:
            return {SpatialVector{Vec3::zero(), b.axis}};
        case BodyJointKind::Fixed:
            return {};
        case BodyJointKind::Floating6:
            return {
                SpatialVector{Vec3::unit_x(), Vec3::zero()},
                SpatialVector{Vec3::unit_y(), Vec3::zero()},
                SpatialVector{Vec3::unit_z(), Vec3::zero()},
                SpatialVector{Vec3::zero(), Vec3::unit_x()},
                SpatialVector{Vec3::zero(), Vec3::unit_y()},
                SpatialVector{Vec3::zero(), Vec3::unit_z()},
            };
    }
    return {};
}

SpatialTransform compute_joint_transform(const BodyModel& b, const ArticulationState& st) {
    switch (b.joint) {
        case BodyJointKind::Revolute: {
            const Quat rot = Quat::from_axis_angle(b.axis, st.q[static_cast<std::size_t>(b.q_index)]);
            return SpatialTransform{rot.inverse().to_mat3(), Vec3::zero()};
        }
        case BodyJointKind::Prismatic:
            return SpatialTransform{Mat3::identity(), b.axis * st.q[static_cast<std::size_t>(b.q_index)]};
        case BodyJointKind::Fixed:
        case BodyJointKind::Floating6:
            return SpatialTransform{}; // identity; Floating6's body-0 transform is built from root_pose directly
    }
    return SpatialTransform{};
}

SpatialTransform compute_xup(const Model& m, int body_index, const ArticulationState& st) {
    const BodyModel& b = m.bodies[static_cast<std::size_t>(body_index)];
    if (m.floating_root && body_index == 0) {
        return spatial::spatial_transform_from_pose(st.root_pose);
    }
    const SpatialTransform xj = compute_joint_transform(b, st);
    return spatial::compose(b.Xtree, xj);
}

SpatialVector joint_velocity(const BodyModel& b, const ArticulationState& st) {
    if (b.joint == BodyJointKind::Floating6) {
        return SpatialVector{Vec3{st.qd[0], st.qd[1], st.qd[2]}, Vec3{st.qd[3], st.qd[4], st.qd[5]}};
    }
    if (b.ndof == 0) return SpatialVector{};
    const auto s = motion_subspace(b);
    return s[0] * st.qd[static_cast<std::size_t>(b.dof_index)];
}

} // namespace ps::articulation
