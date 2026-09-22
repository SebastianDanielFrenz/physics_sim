// core/src/articulation/loop_closure.h — closed-loop kinematic constraints
// (internal; PLAN.md P1.5a scope item 4).
//
// Source: R. Featherstone, "Rigid Body Dynamics Algorithms", Springer
// 2008, ch. 8 ("closed loops" - cut a loop into a spanning tree plus
// constraint equations g(q) = 0, solved at the acceleration level via
// Lagrange multipliers in joint space: with the constraint Jacobian
// J = dg/dq, the constrained forward-dynamics solve is the classic
// saddle-point system
//   [ M   J^T ] [ qdd    ]   [ tau - C ]
//   [ J   0   ] [ lambda ] = [ -Jdot*qd ]
// solved here via the Schur complement (J*M^-1*J^T)*lambda = -(J*qdd_free
// + Jdot*qd), reusing ABA (aba.h) as the M^-1 operator - ABA with qd=0 and
// a candidate generalized force tau returns exactly M^-1*tau, since with
// qd=0 the bias term C(q,0) vanishes.
//
// Stabilisation choice (PLAN.md: "Beware: plain Baumgarte or unweighted
// projection is exactly the energy leak we are escaping. Choose the
// stabilisation by measurement... e.g. mass-matrix-weighted position/
// velocity projection onto the constraint manifold"): implemented here as
// exactly that - project_velocity_onto_constraints() solves the Schur-
// complement system A*mu = -J*qd for a mass-weighted velocity correction
// Δqd = M^-1*J^T*mu (an orthogonal projection in the mass inner product,
// so it can only remove kinetic energy, never add it - provable, not just
// measured), and project_position_onto_constraints() solves A*mu = -g(q)
// for the analogous position correction. No Baumgarte force term is added
// to the acceleration-level solve at all (the Jolt-joint energy leak D10
// exists to avoid).
//
// A plain/unweighted projection alternative was NOT implemented or
// measured separately - D10 already rejected it in principle and building
// a second stabiliser only to confirm the expected energy-leak behaviour
// was not worth the time this phase; the measurements actually taken (see
// tests/validation/test_articulation_four_bar_energy.cpp and the P1.5a
// report) are: no stabilisation at all (closure error grows unbounded to
// NaN within 60 s - confirms it is required); velocity-only correction,
// once per tick (bounded but large closure error, ~455 mm over 60 s -
// confirms position drift needs its own correction, not just velocity);
// both corrections, once per tick (closure error ~1e-6 mm, but 1.9%
// energy drift - fails the < 0.5% bound); both corrections, every substep
// instead of once per tick (closure error ~3e-8 mm, energy drift 0.40% -
// the configuration actually shipped). The per-tick-vs-per-substep
// difference was not root-caused further (plausibly the position
// correction's potential-energy step is what is not conservative, and
// making each individual correction smaller by applying it more often
// reduces the resulting bias) - flagged here rather than left unstated,
// since "smaller corrections happen to measure better" is an empirical
// finding, not a proof this stays under 0.5% for every geometry/mass
// ratio.
//
// Only LoopClosureType::Point is implemented (three constraint rows: the
// two anchors coincide). LoopClosureType::Hinge is accepted by the public
// descriptor and parsed by build_model, but not yet given the extra two
// axis-alignment rows this file would need - a Hinge closure currently
// behaves as a Point closure (position-only), which is NOT what
// PLAN.md P1.5a asks for; flagged clearly in the P1.5a report rather than
// silently shipped as correct.
//
// PHYS-009 Step 3: every workspace vector this file's constraint solve
// needs (ClosureLinearAlgebra's J/minv_jrow/A, plus the smaller per-call
// buffers building them) used to be a fresh local, i.e. a closure-bearing
// articulation (e.g. the four-bar) allocated on every substep even after
// Step 1 preallocated the open-chain aba() scratch - flagged explicitly in
// this file's older comments as out of that pass's scope. LoopClosureScratch
// below is the analogous preallocation for this file: every buffer sized
// once (LoopClosureScratch::ensure(), called from Instance's constructor,
// same as AbaScratch::ensure()) and reused in place from then on. Passing
// lc_scratch == nullptr (still the default on every function below that
// takes it) falls back to a fresh stack-local LoopClosureScratch per call -
// exactly the old allocating behaviour, kept for the non-hot-path callers
// this file already had (tests/validation/test_articulation_four_bar_energy.cpp,
// tests/unit/test_articulation_four_bar.cpp) that call these functions
// directly, never from inside a job.

#pragma once

#include "aba.h"
#include "model.h"
#include "spatial.h"

#include <vector>

namespace ps::articulation {

struct LoopClosureRuntime {
    int body_a = -1; // -1 == world
    int body_b = -1;
    Vec3 anchor_a = Vec3::zero(); // body_a-local, or world-space if body_a == -1
    Vec3 anchor_b = Vec3::zero();
};

// Shared machinery build_linear_algebra (loop_closure.cpp) fills: J (m x nv,
// row-major), minv_jrow (m x nv, row-major - row i is M^-1 * J_row_i, from
// an aba() probe) and A = J * M^-1 * J^T (m x m). m == 3 * closures.size().
struct ClosureLinearAlgebra {
    int m = 0;
    std::vector<real> J;
    std::vector<real> minv_jrow;
    std::vector<real> A;
};

// Preallocated workspace for the whole file's constraint solve (see this
// header's top comment). Every vector is sized by ensure() and never
// resized again for the rest of its owning Instance's lifetime.
struct LoopClosureScratch {
    ClosureLinearAlgebra la;
    std::vector<Pose> poses;
    std::vector<spatial::SpatialTransform> xup;
    std::vector<Vec3> r_a, r_b;
    std::vector<spatial::SpatialVector> v_probe;
    std::vector<spatial::SpatialVector> v_real;
    std::vector<spatial::SpatialVector> a_bias;
    ArticulationState probe_state;   // q/qd sized like the owning Instance's state
    ArticulationState zero_v_state;  // q/qd sized like the owning Instance's state
    std::vector<real> j_row;         // size nv
    std::vector<real> g;             // size m, constraint_error's output
    std::vector<real> jdot_qd;       // size m
    std::vector<real> free_qdd;      // size nv, the unconstrained aba_into() result
    std::vector<real> probe_qdd;     // size nv, build_linear_algebra's per-row aba_into() probe output (distinct from free_qdd - must not alias it, build_linear_algebra runs AFTER free_qdd is computed and free_qdd is still needed afterwards)
    std::vector<real> j_qdd_free;    // size m
    std::vector<real> rhs;           // size m
    std::vector<real> mu;            // size m, solve_spd's output (lambda, or a projection's mu)
    std::vector<real> delta;         // size nv, M^-1*J^T*mu
    std::vector<real> spd_a;         // size m*m, solve_spd's own mutated copy of A
    std::vector<real> spd_b;         // size m,   solve_spd's own mutated copy of rhs

    // n == model.bodies.size(), nv == model.num_v, nq == model.num_q.
    void ensure(const Model& model, std::size_t num_closures);
};

// link_to_body maps ArticulationDesc::links index -> internal body index
// (Instance::link_to_body_ / Instance::body_index_for_link).
[[nodiscard]] std::vector<LoopClosureRuntime> build_loop_closure_runtime(const Model& model, const std::vector<int>& link_to_body);

// Forward kinematics: world pose of every body from q/root_pose alone
// (qd-independent). Shared by Instance::recompute_world_kinematics and
// this file so both use identical pose bookkeeping.
[[nodiscard]] std::vector<Pose> forward_kinematics_poses(const Model& model, const ArticulationState& state);

// Same computation, writing into a caller-owned, already-sized `out`
// instead of returning a fresh vector (PHYS-009 Step 1 rule 2: Instance's
// per-RK4-stage gravity evaluation calls this once per aba() evaluation -
// 4x per substep - and used to allocate a fresh vector<Pose> every time).
void forward_kinematics_poses_into(const Model& model, const ArticulationState& state, std::vector<Pose>& out);

// Returns qdd satisfying every closure's acceleration-level constraint
// (see file comment); falls back to a plain aba() call when closures is
// empty (no cost added to the common, loop-free case). `scratch` is
// forwarded to every internal aba() call; `lc_scratch` backs every other
// workspace buffer this file's constraint solve needs - both are optional
// (default nullptr: a fresh, allocating stack-local is used instead) for
// the non-hot-path callers that never run inside a job.
[[nodiscard]] AbaOutput solve_constrained_dynamics(const Model& model, const ArticulationState& state,
                                                     const std::vector<LoopClosureRuntime>& closures,
                                                     const std::vector<real>& tau, const Vec3& gravity,
                                                     const std::vector<spatial::SpatialVector>& ext_force,
                                                     AbaScratch* scratch = nullptr, LoopClosureScratch* lc_scratch = nullptr);

// Hot-path entry point: writes into a caller-owned qdd_out instead of
// returning an AbaOutput by value. The open-chain case (closures.empty(),
// the common one - PLAN.md D8 "the sub-step chain of a single vehicle is
// sequential" applies equally to a single articulation instance) forwards
// straight to aba_into and allocates nothing; a closure-bearing instance
// (e.g. a four-bar linkage) now goes entirely through `lc_scratch`'s
// preallocated buffers too (PHYS-009 Step 3 - previously routed through the
// allocating solve_constrained_dynamics path every substep, flagged as
// out-of-scope by PHYS-009 Step 1; fixed here).
void solve_constrained_dynamics_into(const Model& model, const ArticulationState& state,
                                      const std::vector<LoopClosureRuntime>& closures,
                                      const std::vector<real>& tau, const Vec3& gravity,
                                      const std::vector<spatial::SpatialVector>& ext_force,
                                      AbaScratch& scratch, std::vector<real>& qdd_out, LoopClosureScratch& lc_scratch);

// Mass-weighted position and velocity projection onto the constraint
// manifold (drift stabilisation, see file comment); no-op when closures
// is empty. Mutates state in place. Split into its two independent halves
// since they have different conservation properties: the velocity
// projection is an orthogonal projection in the mass inner product, so it
// can only remove kinetic energy, never add it; the position projection
// changes q (and hence potential energy under gravity) with no matching
// kinetic-energy adjustment, which is the more likely source of any
// measured energy drift - see the P1.5a report's measurement of running
// with/without each half.
void project_velocity_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                        AbaScratch* scratch = nullptr, LoopClosureScratch* lc_scratch = nullptr);
void project_position_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                        AbaScratch* scratch = nullptr, LoopClosureScratch* lc_scratch = nullptr);
inline void project_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                      AbaScratch* scratch = nullptr, LoopClosureScratch* lc_scratch = nullptr) {
    project_velocity_onto_constraints(model, state, closures, scratch, lc_scratch);
    project_position_onto_constraints(model, state, closures, scratch, lc_scratch);
}

// Constraint position error g(q) (3 per closure: anchor_a - anchor_b,
// world axes), for tests/telemetry.
[[nodiscard]] std::vector<real> constraint_error(const Model& model, const ArticulationState& state,
                                                   const std::vector<LoopClosureRuntime>& closures);
// Same, writing into a caller-owned, already-sized `out` and reusing
// `poses_scratch` for the forward-kinematics pass instead of allocating
// either (PHYS-009 Step 3 hot-path variant, used by
// project_position_onto_constraints when given an lc_scratch).
void constraint_error_into(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                            std::vector<Pose>& poses_scratch, std::vector<real>& out);

} // namespace ps::articulation
