// core/src/articulation/rnea.h — Recursive Newton-Euler Algorithm.
//
// Source: R. Featherstone, "Rigid Body Dynamics Algorithms", Springer
// 2008, ch. 3 (inverse dynamics via the recursive Newton-Euler method).
// Used in P1.5a (PLAN.md) both as inverse dynamics proper (bias-force
// computation C(q,qd) inside ABA's cross-check) and as the building block
// for the CRBA-equivalent mass matrix (crba.h): with qd == 0, gravity == 0
// and qdd == e_j (the j-th unit vector), RNEA returns exactly column j of
// the joint-space mass matrix M(q), because tau = M(q)*qdd + C(q,qd) and
// C(q,0) == 0.
//
// Two outward passes (velocities then accelerations) followed by one
// inward pass (forces), all expressed in each body's OWN frame - see
// model.h's file comment for the Xup/Xtree conventions this relies on.

#pragma once

#include "model.h"
#include "spatial.h"

#include <vector>

namespace ps::articulation {

struct RneaInput {
    const Model* model = nullptr;
    const ArticulationState* state = nullptr;
    std::vector<real> qdd;                              // size model->num_v
    Vec3 gravity = Vec3{0, 0, -9.81};                    // world-frame gravity ACCELERATION vector
    std::vector<spatial::SpatialVector> ext_force;       // size model->bodies.size(); external spatial force on each body, expressed in that body's OWN frame (moment first, then linear force) - zero-initialized if empty
};

struct RneaOutput {
    std::vector<real> tau;                    // size model->num_v (generalized force per DoF, Floating6 fills 6 slots)
    std::vector<spatial::SpatialVector> v;    // per-body spatial velocity, body-local axes (kept for ABA/CRBA/test reuse)
    std::vector<spatial::SpatialVector> a;    // per-body spatial acceleration, body-local axes
};

[[nodiscard]] RneaOutput rnea(const RneaInput& in);

} // namespace ps::articulation
