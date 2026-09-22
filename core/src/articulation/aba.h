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
//
// PHYS-009 Step 2 (PLAN.md P9 prep, profiling): pass 2's per-body inertia
// congruence transform up into the parent (transform_inertia_to_parent,
// aba.cpp) was measured as ~70% of a call's cost - a dense 6x6 Mat6
// congruence built from two more dense 6x6 SpatialTransform expansions.
// Mat6 stays (needed for the articulated-body inertia IA once it has been
// rank-reduced at least once: ch. 7 notes it generally loses the
// restricted rigid-body-inertia structure a plain SpatialInertia assumes),
// but see aba.cpp for the structured-transform fast path taken where the
// operand still has that structure.

#pragma once

#include "model.h"
#include "small_matrix.h"
#include "spatial.h"

#include <array>
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

// Dense, general 6x6 spatial operator - needed only for the articulated-
// body inertia IA (ch. 7: once rank-reduced it generally no longer has the
// restricted (mass, first-moment, rotational-inertia-about-a-point)
// structure a single rigid body's SpatialInertia assumes, so it cannot
// always be carried in that compact form the way CRBA's composite inertia
// can). Public (moved out of aba.cpp's anonymous namespace) so AbaScratch
// below - and therefore Instance's per-instance preallocated scratch,
// PHYS-009 Step 1 rule 2 - can name it.
struct Mat6 {
    real m[6][6] = {};

    static Mat6 from_rigid(const spatial::SpatialInertia& i);
    [[nodiscard]] spatial::SpatialVector apply(const spatial::SpatialVector& v) const;
    Mat6 operator-(const Mat6& o) const;
    Mat6 operator+(const Mat6& o) const;
    Mat6 operator*(const Mat6& o) const;
    static Mat6 from_linear_map(const spatial::SpatialTransform& x, bool force_map);
};

// Per-body DoF-indexed scratch (U columns, u entries): fixed capacity
// kMaxJointDof (== 6, a floating base is the largest joint this project
// has) instead of a std::vector, so the tip->base / base->tip passes below
// never heap-allocate per body (PLAN.md D5 "fix the biggest costs first" /
// PHYS-009 Step 2 profiling: this - plus the per-call n-sized vectors -
// was the dominant cost of the original implementation, one heap
// allocation of some kind for essentially every body on every aba() call).
struct DofScratch {
    std::array<spatial::SpatialVector, kMaxJointDof> U{};
    std::array<real, kMaxJointDof> u{};
};

// Reusable per-call scratch for the n-sized arrays every aba() call needs.
// PHYS-009 Step 1 rule 2: this used to be `thread_local`, which allocates
// the first time (or first time at a larger n) EVERY worker thread happens
// to run a job calling aba() - a heap allocation inside a job body, which
// PLAN.md D8 forbids. Now owned per Instance (instance.h's aba_scratch_
// member), sized once at construction (Instance::Instance, on the
// caller's thread, never inside a job) and reused, in place, by every
// aba() call that instance ever makes (both directly and via
// loop_closure.cpp's solve_constrained_dynamics/build_linear_algebra,
// which take the same scratch through - see loop_closure.h). Call sites
// that are not per-tick hot paths (the ABA-vs-CRBA cross-check unit test,
// e.g.) may omit the scratch argument entirely; aba() then falls back to a
// stack-local instance which still allocates once, same as before this
// change - acceptable there since it never runs inside a job.
struct AbaScratch {
    // Reusable AbaInput for hot-path callers (loop_closure.cpp's
    // solve_constrained_dynamics_into, closures-empty branch): AbaInput
    // owns its `tau`/`ext_force` vectors by value, so a caller that
    // default-constructs a fresh AbaInput every call pays two allocations
    // per call regardless of how well aba()'s OWN internal scratch is
    // reused (found by the PHYS-009 Step 1 rule-2 debug_alloc-armed test:
    // still ~70 violations/tick with only AbaScratch's other members
    // preallocated). Instance sizes ain.tau/ain.ext_force once at
    // construction (instance.cpp); every later use just assigns new
    // values into the same, already-capacity'd vectors.
    AbaInput ain;

    std::vector<spatial::SpatialTransform> xup;
    std::vector<spatial::SpatialVector> v;
    std::vector<spatial::SpatialVector> c; // velocity-product bias term, v_i x (S_i*qd_i)
    std::vector<Mat6> IA;
    std::vector<spatial::SpatialVector> pA;
    std::vector<DofScratch> dof;
    std::vector<SmallMatrix> Dinv;
    std::vector<spatial::SpatialVector> a;

    // Idempotent at a stable n (the common case: an instance's body count
    // never changes after construction) - std::vector::resize to an
    // already-current size performs no allocation and no deallocation, so
    // calling this from inside aba() every call is a safety net, not a
    // per-call cost.
    void ensure(std::size_t n) {
        xup.resize(n);
        v.resize(n);
        c.resize(n);
        IA.resize(n);
        pA.resize(n);
        dof.resize(n);
        Dinv.resize(n);
        a.resize(n);
    }
};

[[nodiscard]] AbaOutput aba(const AbaInput& in, AbaScratch* scratch = nullptr);

// Same computation, writing into a caller-owned qdd_out instead of
// returning a fresh AbaOutput by value. PHYS-009 Step 1 rule 2: AbaOutput's
// own qdd vector is itself a per-call allocation (a freshly
// default-constructed vector, .assign()'d up from zero capacity) even once
// the internal scratch is reused - this is the hot-path entry point that
// avoids it too. qdd_out is resized up front; callers that keep reusing
// the SAME vector across calls (Instance's rk4_k1_..k4_.qddot, aba_scratch_
// itself has no equivalent buffer, it is a distinct per-instance member -
// see instance.h) see it stay at a stable capacity, so this resize is a
// no-op after the first call, exactly like AbaScratch::ensure.
void aba_into(const AbaInput& in, AbaScratch& scratch, std::vector<real>& qdd_out);

} // namespace ps::articulation
