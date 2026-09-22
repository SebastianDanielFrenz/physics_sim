#include "crba.h"

namespace ps::articulation {

using spatial::SpatialInertia;
using spatial::SpatialTransform;
using spatial::SpatialVector;
using spatial::spatial_dot;

MassMatrix crba(const Model& model, const ArticulationState& state) {
    const std::size_t n = model.bodies.size();
    std::vector<SpatialTransform> xup(n);
    std::vector<SpatialInertia> composite(n);

    for (std::size_t i = 0; i < n; ++i) {
        xup[i] = compute_xup(model, static_cast<int>(i), state);
        composite[i] = model.bodies[i].inertia;
    }

    // Backward accumulation: fold each body's composite inertia (its own
    // plus everything already folded from its own subtree) up into its
    // parent, expressed in the parent's own frame (Xtree/Xup direction is
    // parent->child, so the child->parent direction needed here is
    // Xup.inverse()).
    for (std::size_t idx = n; idx-- > 0;) {
        const int parent = model.bodies[idx].parent;
        if (parent >= 0) {
            composite[static_cast<std::size_t>(parent)] =
                composite[static_cast<std::size_t>(parent)] + xup[idx].inverse().apply_inertia(composite[idx]);
        }
    }

    MassMatrix M;
    M.n = model.num_v;
    M.data.assign(static_cast<std::size_t>(M.n) * static_cast<std::size_t>(M.n), real(0.0));

    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& bi = model.bodies[i];
        if (bi.ndof == 0) continue;
        const auto si = motion_subspace(bi);

        // F holds, for each of body i's own DoF columns, the spatial force
        // Ic_i * S_i column - expressed initially in body i's own frame,
        // then walked up the ancestor chain one body at a time.
        std::vector<SpatialVector> f(si.size());
        for (std::size_t c = 0; c < si.size(); ++c) f[c] = composite[i] * si[c];

        int j = static_cast<int>(i);
        while (true) {
            const BodyModel& bj = model.bodies[static_cast<std::size_t>(j)];
            const auto sj = motion_subspace(bj);
            if (bj.ndof > 0) {
                for (std::size_t a = 0; a < si.size(); ++a) {
                    for (std::size_t c = 0; c < sj.size(); ++c) {
                        const real val = spatial_dot(sj[c], f[a]);
                        const int row = bi.dof_index + static_cast<int>(a);
                        const int col = bj.dof_index + static_cast<int>(c);
                        M.at(row, col) = val;
                        M.at(col, row) = val;
                    }
                }
            }
            const int parent = bj.parent;
            if (parent < 0) break;
            for (auto& fv : f) fv = xup[static_cast<std::size_t>(j)].apply_force_inverse(fv);
            j = parent;
        }
    }

    return M;
}

} // namespace ps::articulation
