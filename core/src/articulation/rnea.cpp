#include "rnea.h"

namespace ps::articulation {

using spatial::SpatialInertia;
using spatial::SpatialTransform;
using spatial::SpatialVector;
using spatial::cross_force;
using spatial::cross_motion;
using spatial::spatial_dot;

RneaOutput rnea(const RneaInput& in) {
    const Model& m = *in.model;
    const ArticulationState& st = *in.state;
    const std::size_t n = m.bodies.size();

    RneaOutput out;
    out.tau.assign(static_cast<std::size_t>(m.num_v), real(0.0));
    out.v.assign(n, SpatialVector{});
    out.a.assign(n, SpatialVector{});

    std::vector<SpatialTransform> xup(n);
    // Gravity folded into the recursion as the "world" body's spatial
    // acceleration (Featherstone ch. 3's standard trick): a body rigidly
    // attached to a base accelerating upward at -gravity experiences, via
    // I*a, exactly the same joint loading as one at rest under gravity -
    // this avoids adding a separate m*g force term to every single body.
    const SpatialVector a_world{Vec3::zero(), -in.gravity};

    // Pass 1 (base -> tip): velocities and accelerations, per body's OWN frame.
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = m.bodies[i];
        xup[i] = compute_xup(m, static_cast<int>(i), st);
        const SpatialVector v_parent = (b.parent < 0) ? SpatialVector{} : out.v[static_cast<std::size_t>(b.parent)];
        const SpatialVector a_parent = (b.parent < 0) ? a_world : out.a[static_cast<std::size_t>(b.parent)];
        const SpatialVector vj = joint_velocity(b, st);

        out.v[i] = xup[i].apply_motion(v_parent) + vj;

        SpatialVector sj_qdd{};
        if (b.joint == BodyJointKind::Floating6) {
            sj_qdd = SpatialVector{Vec3{in.qdd[0], in.qdd[1], in.qdd[2]}, Vec3{in.qdd[3], in.qdd[4], in.qdd[5]}};
        } else if (b.ndof == 1) {
            const auto s = motion_subspace(b);
            sj_qdd = s[0] * in.qdd[static_cast<std::size_t>(b.dof_index)];
        }
        out.a[i] = xup[i].apply_motion(a_parent) + sj_qdd + cross_motion(out.v[i], vj);
    }

    // Pass 2 (tip -> base): forces.
    std::vector<SpatialVector> f(n);
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = m.bodies[i];
        const SpatialVector f_ext = in.ext_force.empty() ? SpatialVector{} : in.ext_force[i];
        f[i] = b.inertia * out.a[i] + cross_force(out.v[i], b.inertia * out.v[i]) - f_ext;
    }
    for (std::size_t idx = n; idx-- > 0;) {
        const BodyModel& b = m.bodies[idx];
        if (b.joint == BodyJointKind::Floating6) {
            out.tau[0] = f[idx].top.x; out.tau[1] = f[idx].top.y; out.tau[2] = f[idx].top.z;
            out.tau[3] = f[idx].bottom.x; out.tau[4] = f[idx].bottom.y; out.tau[5] = f[idx].bottom.z;
        } else if (b.ndof == 1) {
            const auto s = motion_subspace(b);
            out.tau[static_cast<std::size_t>(b.dof_index)] = spatial_dot(s[0], f[idx]);
        }
        if (b.parent >= 0) {
            f[static_cast<std::size_t>(b.parent)] += xup[idx].apply_force_inverse(f[idx]);
        }
    }

    return out;
}

} // namespace ps::articulation
