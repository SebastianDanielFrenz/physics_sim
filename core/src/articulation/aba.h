// core/src/articulation/aba.h — Articulated-Body Algorithm: O(n) forward
// dynamics.
//
// Source: R. Featherstone, "Rigid Body Dynamics Algorithms", Springer
// 2008, ch. 7 (the articulated-body inertia recursion). Three passes, all
// per-body quantities expressed in that body's OWN frame (model.h's
// Xup/Xtree convention):
//  1. outward (base->tip): velocities v_i and the velocity-product bias
//     term c_i = v_i x (S_i*qd_i); initialize articulated-body inertia
//     IA_i = I_i and bias force pA_i = v_i x* (I_i*v_i) - f_ext_i.
//  2. inward (tip->base): for each DoF body, U_i = IA_i*S_i,
//     D_i = S_i^T*U_i, u_i = tau_i - S_i^T*pA_i; fold a rank-reduced
//     inertia IA_i - U_i*D_i^-1*U_i^T and bias force
//     pA_i + IA_i^a*c_i + U_i*D_i^-1*u_i up into the parent (a Fixed,
//     0-DoF body folds its whole IA_i/pA_i up unreduced, since there is no
//     D to invert).
//  3. outward again (base->tip): a_i = Xup_i*a_parent + c_i, then
//     qdd_i = D_i^-1*(u_i - U_i^T*a_i), a_i += S_i*qdd_i.
// This is the PLAN.md P1.5a hot-path forward-dynamics call (one per
// articulation per 960 Hz substep); crba.h/rnea.h exist only as reference
// implementations for the cross-check test and are not called from here.

#pragma once

#include "model.h"
#include "spatial.h"

#include <vector>

namespace ps::articulation {

struct AbaInput {
    const Model* model = nullptr;
    const ArticulationState* state = nullptr;
    std::vector<real> tau;                          // size model->num_v, generalized applied force per DoF
    // Featherstone's a_0 base-acceleration trick (ch. 3/7): correct only
    // for a UNIFORM field. Instance (instance.cpp) no longer uses this -
    // it applies gravity per body via ext_force instead (gravity.h), so it
    // can support a future non-uniform field; this field stays available
    // (and still correct) for direct callers, e.g. the ABA-vs-CRBA
    // cross-check test.
    Vec3 gravity = Vec3{0, 0, -9.81};
    std::vector<spatial::SpatialVector> ext_force;   // size model->bodies.size() or empty; per-body external spatial force, body-local axes
};

struct AbaOutput {
    std::vector<real> qdd; // size model->num_v
};

[[nodiscard]] AbaOutput aba(const AbaInput& in);

} // namespace ps::articulation
