// core/src/drivetrain/powertrain.h — Powertrain (PHYS-033 P4 I3 plan
// sections 4/5/6/7): builds one ShaftNetwork + component set from a
// vehicle's (PowertrainDesc, BrakeSystemDesc, WheelDesc list,
// ControllerDesc list) at construction (sim thread, ordinary allocation
// allowed - PLAN.md D8 only forbids allocation INSIDE the per-substep job),
// then drives the fixed per-substep sequence VehicleInstance::step_tick
// calls into (plan section 7.2): begin_substep -> pre_solve_components
// (controllers, a possible topology-state switch, then every component's
// own pre_solve, plus each wheel's own pre_solve via add_hub_terms) ->
// solve -> post_solve_components + each wheel's own post_solve. Internal
// (ps::drivetrain); owned one-per-vehicle by VehicleInstance.
//
// engine/clutch/gearbox/tcu are owned here as TYPED members, not as generic
// IComponent entries in `components_` (components.h) - the plan's substep
// order (section 7.2 step 3/3b) needs the gearbox's phase_step to run,
// possibly switch the network's topology state, and feed the TCU's output
// into the clutch/engine BEFORE any component's ordinary pre_solve runs;
// a generic IComponent::pre_solve(ctx, net, assembly) has no room for that
// orchestration. `components_` still holds gear_pair/differential (array
// order) plus brakes (appended last, exactly as I2 did).

#pragma once

#include "clutch.h"
#include "components.h"
#include "engine_torque_map.h"
#include "gearbox.h"
#include "manual_tcu.h"
#include "network.h"

#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/math/vec3.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/vehicle_reset.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ps::drivetrain {

class Powertrain {
public:
    // Throws std::runtime_error (naming vehicle_name) on any graph/topology
    // error - plan section 4's validation (unknown ports, double-connected
    // ports, an unconnected non-wheel port, a non-rigid shaft, a lossy gear
    // sharing its input node with another rigid row, a non-positive wheel
    // hub inertia, a cluster whose reduced mass matrix is not invertible, a
    // controller naming an unknown or wrong-typed component).
    Powertrain(const PowertrainDesc& pt, const BrakeSystemDesc& brakes, const std::vector<vehicle::WheelDesc>& wheels,
               const std::vector<ControllerDesc>& controllers, const std::string& vehicle_name);

    [[nodiscard]] NodeId hub_node(std::size_t wheel_index) const { return hub_nodes_[wheel_index]; }
    [[nodiscard]] real hub_omega(std::size_t wheel_index) const { return network_.omega(hub_nodes_[wheel_index]); }
    [[nodiscard]] real hub_inertia(std::size_t wheel_index) const { return network_.node_inertia(hub_nodes_[wheel_index]); }
    // Initial-condition seeding only (VehicleInstance's constructor, once -
    // matches network.h's ShaftNetwork::set_omega's own contract).
    void seed_hub_omega(std::size_t wheel_index, real omega) { network_.set_omega(hub_nodes_[wheel_index], omega); }
    // true iff this wheel's hub port was joined by a shaft to anything (plan
    // section 4.1) - an unconnected hub is a free wheel. Backs
    // World::wheel_is_driven.
    [[nodiscard]] bool wheel_is_driven(std::size_t wheel_index) const { return driven_[wheel_index] != 0; }

    // Per-substep sequence (plan section 7.2), called in this exact order by
    // VehicleInstance::step_tick:
    void begin_substep(); // Assembly::begin_substep(_rows) - zeroes this substep's tau0/d/capacities
    void add_hub_terms(std::size_t wheel_index, real tau0, real d) { assembly_.torque(hub_nodes_[wheel_index], tau0, d); }
    // Controllers (gearbox.phase_step, a possible topology-state switch,
    // tcu.step) then engine/clutch/gearbox-mesh-loss/synchro-row THEN every
    // components_ entry's own pre_solve, in that fixed order (plan section
    // 7.2 step 3/3b/4 - see this file's own top comment for why engine/
    // clutch/gearbox are not generic IComponents).
    void pre_solve_components(const ComponentSubstepCtx& ctx);
    void solve(real h); // network_.solve(h, assembly_), rotor-port alpha capture, integrator_loss/rotor_ke (plan section 7.3)
    void post_solve_components(const ComponentSubstepCtx& ctx);
    // Called once per wheel's own post_solve (WheelSubstepOutput's
    // wheel_out_j/tyre_loss_j) - these two ledger terms are NOT a component
    // (they are the hub/tyre boundary itself, plan section 7.3), so
    // VehicleInstance adds them directly rather than through IComponent.
    void add_wheel_energy(real wheel_out_delta, real tyre_loss_delta) {
        ledger_.wheel_out += wheel_out_delta;
        ledger_.tyre_loss += tyre_loss_delta;
    }

    // Tick-level ledger bookkeeping (plan section 7.3's "Check per tick"):
    // begin_tick() snapshots the cumulative ledger before this tick's
    // substeps; end_tick() computes balance_error from the deltas since
    // then. Both plain struct copies (PowertrainEnergy is all `real`
    // fields) - no allocation.
    void begin_tick() { ledger_tick_start_ = ledger_; }
    void end_tick();

    // World::reset_vehicle (plan section 4.5). Call only between ticks,
    // never from a job. `hub_speeds` is this substep's target omega for
    // EVERY wheel's hub node, wheel-index order, size == wheel count -
    // VehicleInstance computes it from the chassis's CURRENT velocity (if
    // opts.hubs_roll_with_chassis) or zero, since only it has the chassis
    // pose/velocity and per-wheel geometry; Powertrain itself never touches
    // the chassis. Sets every node's omega directly and consistently (no
    // network solve, no momentum-conserving projection - see this header's
    // top comment on why apply_topology_switch's algorithm is deliberately
    // NOT reused here), clears every friction row's stick/warm-start state,
    // clears every wheel-owning caller's tyre relaxation state (the caller
    // does this on each WheelInstance directly - Powertrain does not own
    // wheels), and books the SIGNED rotational-KE delta as
    // ledger_.external_reset (positive or negative, unlike engagement_loss
    // which is always >= 0 - a reset is an arbitrary teleport, not a
    // physical inelastic engagement).
    void reset(const vehicle::VehicleResetOptions& opts, const std::vector<real>& hub_speeds);

    [[nodiscard]] const PowertrainEnergy& ledger() const { return ledger_; }
    [[nodiscard]] int friction_cap_hits() const { return network_.friction_cap_hits(); }
    [[nodiscard]] int nonspd_hits() const { return network_.nonspd_hits(); }
    [[nodiscard]] int lcp_stage3_hits() const { return network_.lcp_stage3_hits(); }

    // PHYS-033 P4 I3 plan section 6.7 (Q11, rotor reaction on the chassis):
    // every non-hub rotating port with a nonzero axis_local (engine crank,
    // clutch in/out, gearbox in/out, differential carrier, a gear_pair's
    // in/out) - hub ports are handled separately by VehicleInstance (using
    // the wheel's own world spin axis, not a fixed chassis-local one).
    // alpha (rad/s^2) is THIS substep's (omega_new-omega_old)/h, captured by
    // solve() above - only meaningful (and only non-garbage) right after a
    // solve() call, before the next begin_substep().
    [[nodiscard]] bool rotor_reaction_enabled() const { return rotor_reaction_; }
    [[nodiscard]] std::size_t rotor_port_count() const { return rotor_ports_.size(); }
    [[nodiscard]] Vec3 rotor_port_axis_local(std::size_t i) const { return rotor_ports_[i].axis_local; }
    [[nodiscard]] real rotor_port_inertia(std::size_t i) const { return rotor_ports_[i].J; }
    [[nodiscard]] real rotor_port_alpha(std::size_t i) const { return rotor_port_alpha_[i]; }

    // PHYS-033 P4 I3 plan section 7.5/7.6: read-only accessors for
    // VehicleInstance's snapshot/speedo/debug fill (commit 7 wires the
    // debug one into tests; commit 5's World::powertrain_debug already
    // exists as a stub returning a default PowertrainDebug - this class
    // does not build one itself yet in commit 6, only exposes the pieces).
    [[nodiscard]] bool has_engine() const { return engine_ != nullptr; }
    [[nodiscard]] const EngineTorqueMap& engine() const { return *engine_; }
    // This substep's engine pre_solve() result, cached (mirrors
    // last_clutch_result_/last_gearbox_result_) - PowertrainSnapshot's
    // EngineSoundState fill (commit 6: load/limiter/dfco/starter flags,
    // crank_angle) needs it; only meaningful when has_engine().
    [[nodiscard]] const EngineTorqueMap::PreSolveResult& last_engine_result() const { return last_engine_result_; }
    [[nodiscard]] bool has_clutch() const { return clutch_ != nullptr; }
    [[nodiscard]] bool has_gearbox() const { return gearbox_ != nullptr; }
    [[nodiscard]] const Gearbox& gearbox() const { return *gearbox_; }
    [[nodiscard]] real gearbox_output_ratio_magnitude() const; // |diff.ratio| for the speedo (plan section 7.6)
    [[nodiscard]] real gearbox_out_omega() const { return gearbox_ ? network_.omega(gearbox_out_) : real(0.0); }
    [[nodiscard]] real gearbox_in_omega() const { return gearbox_ ? network_.omega(gearbox_in_) : real(0.0); }
    [[nodiscard]] real engine_omega() const { return engine_ ? network_.omega(engine_crank_) : real(0.0); }
    [[nodiscard]] real driven_wheel_mean_radius() const { return driven_wheel_mean_radius_; }
    // Speedo-formula (plan section 7.6) road-speed proxy, fed to the TCU as
    // Sensors::approx_speed_mps for launch-vs-shift detection; 0 without a
    // gearbox (no r_fd) or with no driven wheels.
    [[nodiscard]] real approx_speed_mps() const {
        const real r_fd = gearbox_output_ratio_magnitude();
        return r_fd > real(0.0) ? std::abs(gearbox_out_omega()) / r_fd * driven_wheel_mean_radius_ : real(0.0);
    }
    [[nodiscard]] real diff_ratio() const { return diff_ratio_; }
    [[nodiscard]] real clutch_engagement() const { return last_clutch_result_.engagement; }
    [[nodiscard]] real clutch_slip() const; // omega(clutch.in) - omega(clutch.out)
    [[nodiscard]] real clutch_torque() const { return has_clutch() ? network_.row_lambda(clutch_row_) : real(0.0); }
    [[nodiscard]] bool has_tcu() const { return tcu_ != nullptr; }
    // Only meaningful when has_tcu() - PowertrainSnapshot's shift_phase
    // derivation (plan section 6.5.2's ClutchOpening/ClutchClosing bracket
    // the gearbox's own mechanical phases).
    [[nodiscard]] ManualTcu::ClutchMode tcu_clutch_mode() const { return tcu_->clutch_mode(); }
    // This substep's gearbox phase_step() result, cached (mirrors
    // last_clutch_result_) - PowertrainSnapshot's DrivelineSoundState fill
    // needs grind_active; only meaningful when has_gearbox().
    [[nodiscard]] const Gearbox::StepResult& last_gearbox_result() const { return last_gearbox_result_; }

    // PHYS-033 P4 I3 commit 7: World::powertrain_debug's read-only
    // accessors (ps/drivetrain/powertrain_state.h's PowertrainDebug). Every
    // one of these reads state that was ALREADY being computed and
    // hashed for real (gearbox_lambda_mesh_prev_/OpenDifferential's own
    // lambda_mesh_prev_, both car_sedan.json and tractor.json's gearbox/
    // diff being efficiency < 1) - none of these accessors change what is
    // computed, only expose it; none is itself hashed (plain reads).
    [[nodiscard]] real clutch_cap_kinetic() const { return last_clutch_result_.cap_kinetic; }
    [[nodiscard]] real clutch_cap_static() const { return last_clutch_result_.cap_static; }
    [[nodiscard]] bool clutch_stuck() const { return has_clutch() ? network_.row_stuck(clutch_row_) : false; }
    [[nodiscard]] real gearbox_mesh_torque_debug() const { return gearbox_lambda_mesh_prev_; }
    [[nodiscard]] real diff_mesh_torque_debug() const { return diff_ ? diff_->lambda_mesh() : real(0.0); }
    [[nodiscard]] real synchro_torque_debug() const {
        return has_gearbox() ? network_.row_lambda(gearbox_synchro_row_) : real(0.0);
    }
    [[nodiscard]] real throttle_eff() const { return last_engine_result_.throttle_eff; }
    [[nodiscard]] real engine_torque_demand() const { return last_engine_result_.torque_demand; }
    // The TCU request the engine will read NEXT substep (tests only).
    [[nodiscard]] const EngineTorqueMap::TcuRequest& tcu_request_debug() const { return tcu_prev_request_; }
    [[nodiscard]] real idle_integrator() const { return has_engine() ? engine_->idle_integrator() : real(0.0); }
    // Fills out[wheel_index] with the diff row's torque on that hub
    // (coef*lambda, plan section 4.5), 0 for every undriven/non-diff hub;
    // out's remaining entries (beyond hub_nodes_.size()) are left at 0.
    void fill_hub_shaft_torque_debug(std::array<real, 8>& out) const {
        out.fill(real(0.0));
        if (diff_ == nullptr) return;
        const real coef = diff_->hub_coef();
        const real lambda = diff_->lambda_mesh();
        for (std::size_t i = 0; i < hub_nodes_.size() && i < out.size(); ++i) {
            if (hub_nodes_[i] == diff_->left_node() || hub_nodes_[i] == diff_->right_node()) out[i] = coef * lambda;
        }
    }

    // Network omega/stick-sets/topology-state (ShaftNetwork::fold_state_hash)
    // plus every component's own additional state (GearPair/OpenDifferential's
    // lambda_mesh_prev_, engine's state machine, gearbox's phase machine, the
    // TCU's arbitration/auto-clutch state, each lossy row's lambda_prev) -
    // plan section 7.7's full hash list.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    // Returns the engagement_loss for this switch (plan section 5.3):
    // snapshots every node's omega, switches state, then runs a ZERO
    // (tau0/d/friction-caps all 0) network_.solve() - algebraically exactly
    // the momentum-conserving projection T_newT*J*T_new * nu = T_newT*J*
    // w_old for ANY h (h cancels when tau0=d=0 and no friction row is
    // active, see this method's own definition) - a NEW helper method, not
    // reusing the real assembly_ (which must stay untouched for the row
    // that follows this call in the SAME substep). Known simplification vs.
    // the plan's literal text: this also clears every OTHER friction row's
    // LCP warm-start set (row_stuck_prev_/row_set_) to its "not stuck"
    // default, not just the synchro row's - harmless (warm-start only
    // affects which active-set guess box_lcp.cpp tries first, never the
    // solved answer) but a deviation worth flagging if ever revisited.
    [[nodiscard]] real apply_topology_switch(int new_state, real h);

    ShaftNetwork network_;
    Assembly assembly_;
    Assembly projection_assembly_; // dedicated scratch for apply_topology_switch, never touches assembly_
    std::vector<std::unique_ptr<IComponent>> components_; // gear_pair(s), differential(s), then brakes (appended last)
    // Non-owning alias into components_ (nullptr if this vehicle's
    // powertrain has no differential) - PHYS-033 P4 I3 commit 7's
    // diff_mesh_torque_debug/fill_hub_shaft_torque_debug read it directly
    // rather than scanning components_ by dynamic_cast every call.
    OpenDifferential* diff_ = nullptr;
    std::vector<NodeId> hub_nodes_;
    std::vector<std::uint8_t> driven_; // parallel to hub_nodes_, WheelDesc order
    real driven_wheel_mean_radius_ = 0.0; // mean radius of driven wheels, computed once at construction
    std::vector<real> omega_old_;      // per-node scratch, sized once at construction; filled fresh in solve()
    std::vector<real> topo_switch_scratch_; // per-node scratch for apply_topology_switch, sized once at construction
    PowertrainEnergy ledger_{};
    PowertrainEnergy ledger_tick_start_{};

    // --- engine (nullptr if this vehicle's powertrain has none) -----------
    std::unique_ptr<EngineTorqueMap> engine_;
    NodeId engine_crank_;
    FrictionRowId engine_friction_row_; // engine-to-ground Coulomb friction (plan section 5.1 row 2)
    EngineTorqueMap::PreSolveResult last_engine_result_{}; // this substep's pre_solve() output, cached

    // --- clutch --------------------------------------------------------
    std::unique_ptr<Clutch> clutch_;
    NodeId clutch_in_, clutch_out_;
    FrictionRowId clutch_row_; // plan section 5.1 row 0
    Clutch::Result last_clutch_result_{};

    // --- gearbox ---------------------------------------------------------
    std::unique_ptr<Gearbox> gearbox_;
    NodeId gearbox_in_, gearbox_out_;
    FrictionRowId gearbox_synchro_row_; // plan section 5.1 row 1
    real gearbox_lambda_mesh_prev_ = 0.0; // STATE: persists substep to substep (plan section 5.4/7.7)
    real gearbox_own_loss_tau_ = 0.0;     // this substep's own mesh-loss torque, stashed pre_solve -> post_solve
    real gearbox_omega_old_in_ = 0.0;
    Gearbox::StepResult last_gearbox_result_{}; // this substep's phase_step output, cached for the TCU's sensors next substep

    // --- differential's ratio, cached for the speedo (plan section 7.6) ---
    // and (has_diff_/diff_carrier_/diff_left_/diff_right_) for
    // World::reset_vehicle (plan section 4.5), which needs to set every
    // node's omega directly in a way consistent with the diff's own rigid
    // row (carrier = diff_ratio_/2 * (left + right), powertrain.cpp's
    // constructor) without running a network solve.
    real diff_ratio_ = 1.0;
    bool has_diff_ = false;
    NodeId diff_carrier_{}, diff_left_{}, diff_right_{};

    // --- TCU (nullptr if this vehicle has no manual_tcu controller) -------
    std::unique_ptr<ManualTcu> tcu_;
    EngineTorqueMap::TcuRequest tcu_prev_request_{}; // LAGGED one substep, fed into the engine (plan section 7.2)

    // --- rotor reaction (plan section 6.7) --------------------------------
    struct RotorPort {
        NodeId node;
        real J = 0.0;
        Vec3 axis_local = Vec3::zero();
    };
    bool rotor_reaction_ = true;
    std::vector<RotorPort> rotor_ports_;
    std::vector<real> rotor_port_omega_old_; // scratch, sized once at construction
    std::vector<real> rotor_port_alpha_;     // this substep's result, sized once at construction
};

} // namespace ps::drivetrain
