// core/src/articulation/instance.h — one runtime articulation: model +
// state + integration (internal; PLAN.md P1.5a scope items 5/6).
//
// Integrator: classical RK4 in joint coordinates is the default, chosen
// by measurement over semi-implicit (symplectic) Euler (see
// tests/bench/energy_study.cpp's articulation section for the
// reproducible numbers). At 960 Hz substeps, both meet the single/double
// pendulum period and single-link energy-drift acceptance criteria, but
// semi-implicit Euler measured badly outside those: a 3-link chain's
// energy drift at 90 deg over 60 s was 14.1% (required < 0.5%), and a
// floating base's momentum drift over 60 s was 3.0e-2 linear / 9.9e-2
// angular relative (required < 1e-6) - both driven by semi-implicit
// Euler's O(h) local truncation error on the velocity-product
// (Coriolis/gyroscopic) terms, which does not average out for a chain or
// a tumbling asymmetric body the way it happens to for a single pendulum.
// RK4 measured at 2.8e-13 linear / 5.0e-12 angular momentum drift and
// 9.2e-9 energy drift on the same two scenarios - PLAN.md implicit
// midpoint was not implemented (a real nonlinear per-substep solve, ch.
// 7's ABA has no natural implicit form without a Newton iteration on top
// of it) given RK4 already clears every acceptance threshold by several
// orders of margin; revisit only if a future cost pass needs the extra
// margin traded back for speed (RK4 is 4 ABA evaluations per substep vs.
// semi-implicit Euler's 1 - see the cost benchmark in
// tests/bench/energy_study.cpp).
//
// The floating-base orientation is integrated on the quaternion manifold:
// semi-implicit Euler uses the exponential map directly
// (q_new = q_old * exp(h * w_body_new)); RK4 evaluates the standard linear
// kinematic rate qdot = 0.5*q(x)(w,0) at each of its four stages (the
// exponential-map update is a single finite step, not a smooth ODE
// right-hand side an RK4 stage can evaluate at an offset state) and
// renormalises the quaternion after forming each stage's offset state, so
// within-step drift off the unit sphere never exceeds O(h) before being
// corrected (Hairer/Lubich/Wanner, "Geometric Numerical Integration", ch.
// IV, on integrating Lie-group/manifold-valued ODEs this way).

#pragma once

#include "aba.h"
#include "contact_source.h"
#include "gravity.h"
#include "loop_closure.h"
#include "model.h"
#include "ps/math/pose.h"

#include <vector>

namespace ps::articulation {

// PLAN.md P1.5a scope item 5: measured against each other on the energy
// criteria (tests/bench/energy_study.cpp); see instance.cpp's
// Instance::step_tick for the measurement-based choice and instance.h's
// top comment for the quaternion-manifold handling each one uses.
enum class IntegratorKind { SemiImplicitEuler, Rk4 };

struct InstanceConfig {
    // A uniform gravitational acceleration - wrapped in a GravityField
    // (gravity.h) and applied per body at its own COM as an ordinary
    // external spatial force, NOT Featherstone's a_0 base-acceleration
    // trick (see gravity.h's file comment for why: a future non-uniform
    // field, e.g. a curved-Earth P11 world, needs gravity resolved at
    // each link's own position, which the trick cannot express).
    Vec3 gravity{0, 0, -9.81};
    real substep_rate_hz = 960.0;
    IntegratorKind integrator = IntegratorKind::Rk4; // chosen by measurement; see the comparison above
};

class Instance {
public:
    explicit Instance(Model model, const articulation::ArticulationDesc& desc);

    [[nodiscard]] const Model& model() const { return model_; }

    // Direct joint force/torque input (PLAN.md P1.5a scope item 2's first
    // half); persists until changed, like a held actuator command - not
    // reset per tick.
    void set_joint_force(int link_index, real tau);

    // Internal body index for a public link_index (ArticulationDesc::links
    // index), or -1 for the root's own body (fixed root has none).
    [[nodiscard]] int body_index_for_link(int link_index) const {
        return link_index < 0 ? (model_.floating_root ? 0 : -1) : link_to_body_[static_cast<std::size_t>(link_index)];
    }

    // Accumulate (not overwrite - multiple force elements may attach to
    // the same link) a world-frame wrench applied at frame_world_pose for
    // this tick; cleared by clear_external_wrenches() at the start of each
    // tick's gather stage.
    void accumulate_external_wrench(int body_index, const Pose& frame_world_pose, const Wrench& wrench);
    void clear_external_wrenches();

    // PLAN.md P1.5b (see contact_source.h's header comment for why this
    // exists at substep, not tick, granularity): set once at setup time
    // (World::create_articulation), never per tick/substep - a null source
    // (default, every pre-P1.5b articulation) costs nothing extra in
    // step_tick below beyond one pointer comparison.
    void set_contact_source(IContactSubstepSource* source, IRigidBackend* backend) {
        contact_source_ = source;
        contact_backend_ = backend;
    }
    [[nodiscard]] bool has_contact_source() const { return contact_source_ != nullptr; }

    // Runs num_substeps of semi-implicit Euler at dt_sub = 1/InstanceConfig::substep_rate_hz,
    // holding the externally-accumulated wrenches constant across all of
    // them (PLAN.md D3). Converts the accumulated world-frame wrenches to
    // per-body spatial forces once, at the start, using each body's
    // current cached world pose. EXCEPT when contact_source_ is set
    // (PLAN.md P1.5b, contact_source.h): that contribution is instead
    // recomputed fresh every substep (still held constant across one
    // substep's internal RK4 stages only) - a stiff contact spring aliases
    // and goes unstable if sampled/held at the coarser 240 Hz tick rate
    // (contact_source.h's header comment has the numbers).
    void step_tick(const InstanceConfig& cfg, real tick_dt);

    [[nodiscard]] real joint_position(int link_index) const;
    [[nodiscard]] real joint_velocity(int link_index) const;
    [[nodiscard]] Pose link_world_pose(int link_index) const; // link_index == -1 -> root
    [[nodiscard]] Motion link_world_motion(int link_index) const;

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    void recompute_world_kinematics();
    // Writes into tau_scratch_ (PHYS-009 Step 1 rule 2: no per-call vector,
    // reused every call at a stable size == model_.num_v).
    void compute_tau_into(const std::vector<real>& qd_for_actuators, std::vector<real>& out) const;
    void step_semi_implicit_euler(const InstanceConfig& cfg, const std::vector<spatial::SpatialVector>& ext_force, real h);
    void step_rk4(const InstanceConfig& cfg, const std::vector<spatial::SpatialVector>& ext_force, real h);
    void clamp_joint_limits();

    // One evaluation of the ODE right-hand side at an arbitrary (possibly
    // mid-step, RK4-internal) state - used by step_rk4's four stages;
    // step_semi_implicit_euler calls aba() on state_ directly instead since
    // it only ever needs the derivative at the current state. Quaternion
    // rate uses the standard linear kinematic relation qdot = 0.5*q(x)(w,0)
    // (NOT the exponential map used for the semi-implicit Euler path -
    // exponential-map "stepping" is a single finite update, not a smooth
    // ODE right-hand side an RK4 stage can evaluate at an offset state) -
    // renormalised at every stage's input, so drift off the unit sphere
    // cannot accumulate within a step (PLAN.md P1.5a: "normalisation that
    // doesn't bias energy" - renormalising a state that already deviates
    // from unit norm by only O(h) each stage is itself an O(h) correction,
    // not the O(1)-per-step bias naive "integrate then normalise less
    // often" schemes can exhibit).
    struct Derivative {
        std::vector<real> qdot;  // size model_.num_q
        std::vector<real> qddot; // size model_.num_v
        Vec3 root_pos_dot;
        real root_quat_dot[4] = {0, 0, 0, 0}; // x,y,z,w rate
    };
    // Fills `out` (already sized by resize_scratch(), see the constructor)
    // in place - PHYS-009 Step 1 rule 2: this is called 4x per substep by
    // step_rk4 and used to heap-allocate a fresh Derivative (two vectors)
    // plus a fresh forward-kinematics pose vector (via add_gravity) every
    // single time; now it and everything it calls write into Instance's
    // own preallocated scratch members instead.
    // Not const: writes into aba_scratch_/ext_force_g_scratch_/fk_pose_scratch_.
    void evaluate_derivative_into(const ArticulationState& st, const std::vector<real>& tau,
                                   const std::vector<spatial::SpatialVector>& ext_force, const Vec3& gravity, Derivative& out);
    // Writes ext_force + this state's per-body gravity contribution into
    // out (PHYS-009 Step 1 rule 2's add_gravity, now allocation-free: uses
    // fk_pose_scratch_ instead of a fresh forward_kinematics_poses() call).
    // Not const: writes into fk_pose_scratch_.
    void add_gravity_into(const ArticulationState& st, const std::vector<spatial::SpatialVector>& base_ext_force,
                           const GravityField& field, std::vector<spatial::SpatialVector>& out);
    // out := base + scale*d (RK4's stage-offset state); out must already be
    // sized like state_ (resize_scratch()) - PHYS-009 Step 1 rule 2: this
    // used to construct a fresh ArticulationState (two vector allocations)
    // per call, 3x per substep (s2/s3/s4).
    void add_scaled_into(const ArticulationState& base, const Derivative& d, real scale, ArticulationState& out) const;

    Model model_;
    ArticulationState state_;
    std::vector<real> direct_tau_;          // size model_.num_v, held until changed
    std::vector<Wrench> ext_wrench_world_;  // size model_.bodies.size(), world axes, accumulated per tick
    std::vector<Pose> body_world_pose_;     // cache, refreshed after every step_tick
    std::vector<Motion> body_world_motion_; // cache, refreshed after every step_tick

    // link_index (ArticulationDesc::links index) -> body index, for the
    // public accessors above (World-facing API addresses links by their
    // original descriptor index, not the internal body numbering).
    std::vector<int> link_to_body_;

    // PLAN.md P1.5a scope item 4 (loop_closure.h); empty for the common
    // open-chain case, at which point every solve below degenerates to a
    // plain aba() call at no extra cost.
    std::vector<LoopClosureRuntime> loop_closures_;

    // --- Step-time scratch, preallocated once in the constructor and
    // reused in place by every step_tick() call for this instance's
    // lifetime (PHYS-009 Step 1 rule 2: PLAN.md D8 "no allocation inside
    // jobs" - System::step_all (system.cpp) now runs one job per
    // articulation, so everything step_tick touches must not allocate). ---
    AbaScratch aba_scratch_;
    // PHYS-009 Step 3: backs every workspace buffer loop_closure.cpp's
    // constraint solve needs (ClosureLinearAlgebra's J/minv_jrow/A and the
    // smaller buffers building them) - sized once here, like aba_scratch_
    // above, so a closure-bearing instance (e.g. the four-bar) no longer
    // allocates every substep either. Harmless (zero-sized buffers) for the
    // common open-chain case, same as loop_closures_ itself.
    LoopClosureScratch loop_closure_scratch_;
    std::vector<spatial::SpatialVector> ext_force_scratch_;   // size bodies.size(), body-local, held constant across a tick's substeps
    std::vector<spatial::SpatialVector> ext_force_g_scratch_; // size bodies.size(), ext_force + gravity, reused per aba() evaluation
    // PLAN.md P1.5b (contact_source.h): only touched when contact_source_ is
    // set. contact_wrench_scratch_ is world-frame, refilled by the source
    // every substep; combined_ext_force_scratch_ is ext_force_scratch_ plus
    // that substep's contact contribution, body-local - what actually gets
    // passed to step_rk4/step_semi_implicit_euler for a contact-bearing
    // substep instead of ext_force_scratch_ directly.
    std::vector<Wrench> contact_wrench_scratch_;
    std::vector<spatial::SpatialVector> combined_ext_force_scratch_;
    IContactSubstepSource* contact_source_ = nullptr;
    IRigidBackend* contact_backend_ = nullptr;
    std::vector<Pose> fk_pose_scratch_;                       // size bodies.size(), forward-kinematics scratch for add_gravity_into
    std::vector<real> tau_scratch_;                           // size num_v
    std::vector<real> qdd_scratch_;                            // size num_v, semi-implicit Euler's own aba() output buffer
    std::vector<spatial::SpatialVector> v_local_scratch_;      // size bodies.size(), recompute_world_kinematics' body-local velocity scratch
    ArticulationState rk4_s2_, rk4_s3_, rk4_s4_; // RK4 stage-offset states, pre-sized q/qd
    Derivative rk4_k1_, rk4_k2_, rk4_k3_, rk4_k4_; // pre-sized qdot/qddot
};

} // namespace ps::articulation
