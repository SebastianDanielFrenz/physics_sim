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
// empty (no cost added to the common, loop-free case). `scratch`, when
// given, is forwarded to every internal aba() call (PHYS-009 Step 1 rule
// 2: Instance passes its own preallocated AbaScratch so the open-chain
// fast path never allocates inside a job; the loop-closure probe path
// below still allocates its own J/minv_jrow/A buffers per call - out of
// this pass's scope, see instance.h's Instance::aba_scratch_ comment).
[[nodiscard]] AbaOutput solve_constrained_dynamics(const Model& model, const ArticulationState& state,
                                                     const std::vector<LoopClosureRuntime>& closures,
                                                     const std::vector<real>& tau, const Vec3& gravity,
                                                     const std::vector<spatial::SpatialVector>& ext_force,
                                                     AbaScratch* scratch = nullptr);

// Hot-path entry point: writes into a caller-owned qdd_out instead of
// returning an AbaOutput by value. The open-chain case (closures.empty(),
// the common one - PLAN.md D8 "the sub-step chain of a single vehicle is
// sequential" applies equally to a single articulation instance) forwards
// straight to aba_into and allocates nothing; a closure-bearing instance
// (e.g. a four-bar linkage) still goes through the allocating
// solve_constrained_dynamics path internally (see loop_closure.cpp) and
// copies the result into qdd_out - PHYS-009 Step 1 left this path
// unoptimised (out of scope: only the aba() scratch was required to move
// off the hot path), flagged here rather than silently implied clean.
void solve_constrained_dynamics_into(const Model& model, const ArticulationState& state,
                                      const std::vector<LoopClosureRuntime>& closures,
                                      const std::vector<real>& tau, const Vec3& gravity,
                                      const std::vector<spatial::SpatialVector>& ext_force,
                                      AbaScratch& scratch, std::vector<real>& qdd_out);

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
                                        AbaScratch* scratch = nullptr);
void project_position_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                        AbaScratch* scratch = nullptr);
inline void project_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                      AbaScratch* scratch = nullptr) {
    project_velocity_onto_constraints(model, state, closures, scratch);
    project_position_onto_constraints(model, state, closures, scratch);
}

// Constraint position error g(q) (3 per closure: anchor_a - anchor_b,
// world axes), for tests/telemetry.
[[nodiscard]] std::vector<real> constraint_error(const Model& model, const ArticulationState& state,
                                                   const std::vector<LoopClosureRuntime>& closures);

} // namespace ps::articulation
