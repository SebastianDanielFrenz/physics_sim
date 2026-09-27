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
#include "energy_store.h"
#include "engine_torque_map.h"
#include "gearbox.h"
#include "manual_tcu.h"
#include "network.h"
#include "throttle_stage.h"

#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/math/vec3.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/vehicle_reset.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ps::drivetrain {

// PHYS-033 P4 I4 C3 (plan section 2.4): one control channel a powertrain
// component wants World::create_vehicle to resolve for it, through the same
// ensure_control_channel_index/prefix mechanism steer/throttle/.../assist.*
// already use (world.cpp).
struct ChannelDecl {
    std::string name;
    real default_value = 0.0;
};

// PHYS-033 P4 I7a C3: the TCU's own lagged Outputs (throttle_floor/
// ignition_cut/throttle_ceiling), cached one substep so
// ComponentSubstepCtx::tcu_throttle_floor/tcu_ignition_cut/
// tcu_throttle_ceiling (which ThrottleStage::evaluate reads) stay order-
// independent (plan section 7.2). Distinct from the engine's own TcuRequest
// (engine.h), which now carries only ignition_cut - floor/ceiling moved into
// ThrottleCommand instead (see throttle_stage.h's "which TCU intervention
// lives where" table).
struct TcuThrottleLimits {
    real throttle_floor = 0.0;
    bool ignition_cut = false;
    real throttle_ceiling = 1.0;
};

// PHYS-033 P4 I7a C5d review fix (B2): the field-for-field TorqueMapEngineDesc
// -> EngineTorqueMap::Desc resolver, including the crude no-map fuel model's
// active-fuel/eta_i resolution (fuels[0] wins, ignition-based default unless
// indicated_efficiency overrides it). Declared here (defined in powertrain.cpp,
// no longer in an anonymous namespace) so tests can call the REAL production
// resolver instead of maintaining their own hand-duplicated copy - a duplicate
// cannot catch a bug introduced in this function itself.
EngineTorqueMap::Desc to_engine_desc(const TorqueMapEngineDesc& e);
// PHYS-033 P4 I7a C6: the same, with the fuel a feeding tank selects - the
// engine burns the TANK's fuel (its LHV; for a map engine the tank's fuel is
// the map's one fuel, which the loader/Powertrain already enforce). `tank`
// nullptr == the one-argument overload exactly (fuels[0] for the crude
// path). Fixed at construction, never changed at runtime.
EngineTorqueMap::Desc to_engine_desc(const TorqueMapEngineDesc& e, const FuelTankDesc* tank);

class Powertrain {
public:
    // Throws std::runtime_error (naming vehicle_name) on any graph/topology
    // error - plan section 4's validation (unknown ports, double-connected
    // ports, an unconnected non-wheel port, a non-rigid shaft, a lossy gear
    // sharing its input node with another rigid row, a non-positive wheel
    // hub inertia, a cluster whose reduced mass matrix is not invertible, a
    // controller naming an unknown or wrong-typed component, a lockable diff
    // that is not a leaf, more than kMaxStates composed topology states).
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

    // PHYS-033 P4 I4 C3 (plan section 2.4): every control channel this
    // vehicle's powertrain components want, in pt.components array order,
    // duplicate names merged (first occurrence's default wins). Build-time
    // only - called once per vehicle by World::create_vehicle, never per
    // tick/substep (PLAN.md D8 governs the hot path, not this). I4 C5: a
    // lockable differential declares its lock channel (default 0); no
    // shipped vehicle has one, so this returns {} for car_sedan.json/
    // tractor.json today.
    // `test_only_extra` (default empty, merged in AFTER pt.components' own
    // declarations by the same rule) is the least-invasive test hook this
    // commit adds (I4 C3 report): it lets a white-box unit test exercise the
    // declare/merge/resolve/per-tick-copy/ComponentSubstepCtx::channels
    // pipeline end-to-end without adding a throwaway IComponent kind to the
    // closed ComponentParams variant (which would also need ps::io::
    // load_vehicle_json's component-type dispatch to reject it explicitly)
    // and without touching any public header.
    [[nodiscard]] static std::vector<ChannelDecl> declare_control_channels(const PowertrainDesc& pt,
                                                                             const std::vector<ChannelDecl>& test_only_extra = {});

    // Per-substep sequence (plan section 7.2), called in this exact order by
    // VehicleInstance::step_tick:
    void begin_substep(); // Assembly::begin_substep(_rows/_couplings) - zeroes this substep's tau0/d/capacities/couplings
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
    // PHYS-033 P4 I7a C6: a pending set_fuel_tank_mass request is applied
    // here, at the tick boundary, before the snapshot - clamped to [0,
    // capacity] and booked signed (new - old) into fuel_external_added.
    void begin_tick() {
        if (fuel_tank_set_pending_) {
            ledger_.fuel_external_added += fuel_tank_->set_content(fuel_tank_set_value_);
            fuel_tank_set_pending_ = false;
        }
        ledger_tick_start_ = ledger_;
    }
    void end_tick();

    // PHYS-033 P4 I7a C6: the fuel tank feeding this vehicle's engine, if any
    // (energy_store.h). fuel_tank_mass() is 0 without one.
    [[nodiscard]] bool has_fuel_tank() const { return fuel_tank_ != nullptr; }
    [[nodiscard]] real fuel_tank_mass() const { return fuel_tank_ ? fuel_tank_->content() : real(0.0); }
    [[nodiscard]] real fuel_tank_capacity() const { return fuel_tank_ ? fuel_tank_->capacity() : real(0.0); }
    [[nodiscard]] const FuelTank* fuel_tank() const { return fuel_tank_.get(); }
    // Queues a new tank content for the NEXT begin_tick() (a later call
    // before then replaces an earlier one). Throws std::runtime_error when
    // this vehicle has no tank or `kg` is NaN; any other value (negative,
    // above capacity, infinite) is clamped when applied.
    void request_fuel_tank_mass(real kg);

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
    [[nodiscard]] const IEngine& engine() const { return *engine_; }
    // This substep's engine pre_solve() result, cached (mirrors
    // last_clutch_result_/last_gearbox_result_) - PowertrainSnapshot's
    // EngineSoundState fill (commit 6: load/limiter/dfco/starter flags,
    // crank_angle) needs it; only meaningful when has_engine().
    [[nodiscard]] const EngineTorqueMap::PreSolveResult& last_engine_result() const { return last_engine_result_; }
    [[nodiscard]] bool has_clutch() const { return clutch_ != nullptr; }
    [[nodiscard]] bool has_gearbox() const { return gearbox_ != nullptr; }
    [[nodiscard]] const Gearbox& gearbox() const { return *gearbox_; }
    // Speedo ratio (plan section 7.6, generalised by I4 C4 section 2.5 -
    // powertrain.cpp's constructor documents the path rule): |diff.ratio|
    // for a single diff, |product of ratios| along an equal-ratio diff tree,
    // 1 when the exact no-slip form is used.
    [[nodiscard]] real gearbox_output_ratio_magnitude() const;
    [[nodiscard]] real gearbox_out_omega() const { return gearbox_ ? network_.omega(gearbox_out_) : real(0.0); }
    [[nodiscard]] real gearbox_in_omega() const { return gearbox_ ? network_.omega(gearbox_in_) : real(0.0); }
    [[nodiscard]] real engine_omega() const { return engine_ ? network_.omega(engine_crank_) : real(0.0); }
    [[nodiscard]] real driven_wheel_mean_radius() const { return driven_wheel_mean_radius_; }
    // Speedo-formula (plan section 7.6) road-speed proxy, fed to the TCU as
    // Sensors::approx_speed_mps for launch-vs-shift detection; 0 without a
    // gearbox (no r_fd) or with no driven wheels. speedo_radius_ is the mean
    // radius of the hubs reached from gearbox.out (== driven_wheel_mean_
    // radius_ for every single-diff vehicle), see powertrain.cpp.
    [[nodiscard]] real approx_speed_mps() const {
        const real r_fd = gearbox_output_ratio_magnitude();
        return r_fd > real(0.0) ? std::abs(gearbox_out_omega()) / r_fd * speedo_radius_ : real(0.0);
    }
    [[nodiscard]] real speedo_radius() const { return speedo_radius_; }
    [[nodiscard]] real diff_ratio() const { return diff_ratio_; }
    [[nodiscard]] real clutch_engagement() const { return last_clutch_result_.engagement; }
    [[nodiscard]] real clutch_slip() const; // omega(clutch.in) - omega(clutch.out)
    [[nodiscard]] real clutch_torque() const { return has_clutch() ? network_.row_lambda(clutch_row_) : real(0.0); }

    // PHYS-033 P4 I4 C9 (plan section 3.3): MFWD transfer case. nullptr/0 for
    // every vehicle without one (car_sedan.json and every other pre-C9 file) -
    // mirrors clutch_engagement()/clutch_slip()/clutch_torque()'s own pattern.
    [[nodiscard]] bool has_transfer_case() const { return transfer_case_ != nullptr; }
    [[nodiscard]] real awd_engagement() const { return transfer_case_ ? transfer_case_->engagement() : real(0.0); }
    [[nodiscard]] real awd_clutch_torque() const { return transfer_case_ ? network_.row_lambda(transfer_row_) : real(0.0); }
    [[nodiscard]] real awd_clutch_slip() const { return transfer_case_ ? network_.row_velocity(transfer_row_) : real(0.0); }

    // PHYS-033 P4 I4 C10 (plan section 3.4, owner ruling): park pawl.
    // nullptr/0/false for every vehicle without one - same nullable-pointer
    // pattern as transfer_case_ above.
    [[nodiscard]] bool has_park_lock() const { return park_lock_ != nullptr; }
    [[nodiscard]] real park_lock_engaged() const { return (park_lock_ && park_lock_->engaged()) ? real(1.0) : real(0.0); }
    [[nodiscard]] real park_lock_torque() const { return park_lock_ ? network_.row_lambda(park_lock_row_) : real(0.0); }

    // PHYS-033 P4 I4 C11 (plan section 3.5): power take-off. nullptr/0 for
    // every vehicle without one - same nullable-pointer pattern as
    // transfer_case_/park_lock_ above.
    [[nodiscard]] bool has_pto() const { return pto_ != nullptr; }
    [[nodiscard]] real pto_engagement() const { return pto_ ? pto_->engagement() : real(0.0); }
    [[nodiscard]] real pto_mode() const { return pto_ ? (pto_->mode_1000() ? real(1000.0) : real(540.0)) : real(0.0); }
    [[nodiscard]] real pto_rpm() const {
        if (!pto_) return real(0.0);
        constexpr real kPi = 3.14159265358979323846;
        return std::abs(network_.omega(pto_->out_node())) * real(60.0) / (real(2.0) * kPi);
    }
    [[nodiscard]] real pto_clutch_torque() const { return pto_ ? network_.row_lambda(pto_row_) : real(0.0); }
    [[nodiscard]] real pto_clutch_slip() const { return pto_ ? network_.row_velocity(pto_row_) : real(0.0); }
    // The FIRST load_dyno in component order (test/bench only - no shipped
    // vehicle has one), mirrors diff_mesh_torque_debug()'s own "first" rule.
    [[nodiscard]] real dyno_torque() const { return load_dynos_.empty() ? real(0.0) : load_dynos_.front()->torque(); }

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
    // The FIRST differential in pt.components order (I3's only one).
    [[nodiscard]] real diff_mesh_torque_debug() const { return diffs_.empty() ? real(0.0) : diffs_.front()->lambda_mesh(); }
    [[nodiscard]] real synchro_torque_debug() const {
        return has_gearbox() ? network_.row_lambda(gearbox_synchro_row_) : real(0.0);
    }
    [[nodiscard]] real throttle_eff() const { return last_engine_result_.throttle_eff; }
    [[nodiscard]] real engine_torque_demand() const { return last_engine_result_.torque_demand; }
    // I4 C12: 0 without a governor (last_engine_result_'s own fields already
    // default to 0 when desc_.has_governor is false - see EngineTorqueMap::
    // pre_solve). PowertrainSnapshot::governor_setpoint_rpm/hand_throttle.
    [[nodiscard]] real governor_setpoint_rpm() const { return last_engine_result_.governor_setpoint_rpm; }
    // PHYS-033 P4 I7a C5, broadened C5d: nonzero for a fuel map OR the crude
    // no-map estimate; 0 only for an engine with no fuel model at all
    // (pre_solve never touches PreSolveResult::fuel_flow_g_s otherwise).
    // PowertrainSnapshot::fuel_flow_g_s/EngineSoundState::fuel_flow_g_s.
    [[nodiscard]] real fuel_flow_g_s() const { return last_engine_result_.fuel_flow_g_s; }
    // PHYS-033 P4 I7a C3: the hand-lever value itself moved into the
    // throttle_stage.h stage (last_throttle_hand_lever_raw_ below) - this
    // accessor's meaning/callers (PowertrainSnapshot::hand_throttle) are
    // unchanged, only its source.
    [[nodiscard]] real governor_hand_throttle() const { return last_throttle_hand_lever_raw_; }
    // The stage's combined command this substep (EngineSoundState::
    // throttle_command, sound-only, never hashed).
    [[nodiscard]] real throttle_command() const { return last_throttle_command_.command; }
    // The whole ThrottleCommand (command/floor/ceiling) the engine received
    // this substep (tests only - test_throttle_control.cpp's P4).
    [[nodiscard]] const ThrottleCommand& throttle_command_debug() const { return last_throttle_command_; }
    // The TCU's own lagged Outputs the stage/engine will read NEXT substep
    // (tests only). PowertrainDebug::throttle_ceiling reads
    // tcu_request_debug().throttle_ceiling directly (vehicle_instance.h),
    // matching dbg.ignition_cut's own timing - both are the TCU's cached
    // request the stage/engine will apply NEXT substep, not the stage's own
    // already-applied output (last_throttle_command_ above, a different
    // quantity: EngineSoundState::throttle_command only).
    [[nodiscard]] const TcuThrottleLimits& tcu_request_debug() const { return tcu_prev_request_; }
    [[nodiscard]] real idle_integrator() const { return has_engine() ? engine_->idle_integrator() : real(0.0); }
    // Fills out[wheel_index] with the torque of the diff row driving that
    // hub (coef*lambda, plan section 4.5), per diff (I4 C4), 0 for every
    // hub no diff output drives directly (undriven, or a centre diff's
    // outputs, which are carriers, not hubs); out's remaining entries
    // (beyond hub_nodes_.size()) are left at 0.
    // PHYS-033 P4 I4 C8: extended to fold in each diff's lock/LSD/viscous
    // transfer_torque() (added at the left hub, subtracted at the right,
    // Differential::transfer_torque's own convention) - this is now the
    // REAL total shaft torque on each hub, not just the diff row's mesh-
    // torque share (which is all a plain Open diff ever had anyway, so this
    // is bit-identical there: transfer_torque() reads ~0).
    void fill_hub_shaft_torque_debug(std::array<real, 8>& out) const {
        out.fill(real(0.0));
        for (const Differential* d : diffs_) {
            const real coef = d->hub_coef();
            const real lambda = d->lambda_mesh();
            const real transfer = d->transfer_torque();
            for (std::size_t i = 0; i < hub_nodes_.size() && i < out.size(); ++i) {
                if (hub_nodes_[i] == d->left_node()) out[i] = coef * lambda + transfer;
                else if (hub_nodes_[i] == d->right_node()) out[i] = coef * lambda - transfer;
            }
        }
    }

    // PHYS-033 P4 I4 C8 (plan section 3.1): PowertrainSnapshot::diffs' fill,
    // component-array order (same index diffs_ uses / diff_lock_diff_index
    // maps into). bias = signed (T_left-T_right)/(|T_left|+|T_right|) using
    // the same true per-hub torque fill_hub_shaft_torque_debug computes
    // (coef*lambda_mesh +- transfer_torque); locked = kind Locked, or a
    // lockable diff whose bit is set in the network's current composed
    // state (diff_lock_engaged); lock_pending mirrors diff_lock_pending.
    // Returns the number of diffs filled (== diffs_.size(), capped at
    // out.size()).
    std::uint8_t fill_diff_snapshot(std::array<DiffSnapshot, kMaxPowertrainDiffs>& out) const {
        const std::size_t n = std::min(diffs_.size(), out.size());
        for (std::size_t i = 0; i < n; ++i) {
            const Differential& d = *diffs_[i];
            const real t_left = d.hub_coef() * d.lambda_mesh() + d.transfer_torque();
            const real t_right = d.hub_coef() * d.lambda_mesh() - d.transfer_torque();
            const real denom = std::abs(t_left) + std::abs(t_right);
            out[i].bias = denom > real(0.0) ? (t_left - t_right) / denom : real(0.0);
            out[i].locked = d.kind() == DifferentialKind::Locked;
            out[i].lock_pending = false;
            for (std::size_t li = 0; li < locks_.size(); ++li) {
                if (locks_[li].diff_index == i) {
                    out[i].locked = out[i].locked || diff_lock_engaged(li);
                    out[i].lock_pending = diff_lock_pending(li);
                    break;
                }
            }
        }
        return static_cast<std::uint8_t>(n);
    }

    // PHYS-033 P4 I4 C8: PowertrainDebug::diff_transfer_torque's fill, same
    // component-array order as fill_diff_snapshot.
    void fill_diff_transfer_torque_debug(std::array<real, kMaxPowertrainDiffs>& out) const {
        out.fill(real(0.0));
        const std::size_t n = std::min(diffs_.size(), out.size());
        for (std::size_t i = 0; i < n; ++i) out[i] = diffs_[i]->transfer_torque();
    }

    // --- PHYS-033 P4 I4 C4 white-box accessors (tests only) ---------------
    [[nodiscard]] std::size_t diff_count() const { return diffs_.size(); }
    [[nodiscard]] const Differential& diff(std::size_t i) const { return *diffs_[i]; }
    [[nodiscard]] const ShaftNetwork& network() const { return network_; }
    // Every rigid row in NetworkBuilder::add_rigid_row call order, with its
    // topology state - recorded once at construction.
    [[nodiscard]] const std::vector<std::pair<int, RigidRow>>& build_rigid_rows() const { return build_rigid_rows_; }
    // World::reset_vehicle's derived-node order (powertrain.cpp, reset step 4).
    struct ResetDerivation {
        bool is_diff = true; // diff: target = ratio*((w(a)+w(b))/2); gear_pair: target = ratio*w(a)
        NodeId target, a, b;
        real ratio = 1.0;
    };
    [[nodiscard]] const std::vector<ResetDerivation>& reset_derivations() const { return reset_derivations_; }

    // --- PHYS-033 P4 I4 C5: composed topology states / diff locks (plan
    // section 2.6). s = g + n_g * lock_bits; lock bit i is the i-th LOCKABLE
    // diff in component order. The lock bits live ONLY in the network's
    // topology state (hashed there); pending is recomputed every substep from
    // the channel and the locked bit, so it is not state and not hashed. ---
    [[nodiscard]] int gear_state_count() const { return gear_state_count_; } // n_g (1 without a gearbox)
    [[nodiscard]] int composed_state(int gear_state, int lock_bits) const { return gear_state + gear_state_count_ * lock_bits; }
    [[nodiscard]] int state_gear_index(int s) const { return s % gear_state_count_; }
    [[nodiscard]] int state_lock_bits(int s) const { return s / gear_state_count_; }
    [[nodiscard]] std::size_t diff_lock_count() const { return locks_.size(); }
    [[nodiscard]] std::size_t diff_lock_diff_index(std::size_t i) const { return locks_[i].diff_index; } // into diff(i)
    [[nodiscard]] int diff_lock_channel_slot(std::size_t i) const { return locks_[i].channel_slot; }
    [[nodiscard]] bool diff_lock_engaged(std::size_t i) const { return ((state_lock_bits(network_.topology_state()) >> i) & 1) != 0; }
    [[nodiscard]] bool diff_lock_pending(std::size_t i) const { return lock_pending_[i] != 0; }

    // --- PHYS-033 P4 I4 C13: compliant shafts (PowertrainDesc::compliance ==
    // Compliant, ShaftDesc::stiffness > 0). Each is a spring-damper coupling
    // between two distinct nodes a (ports[0]) and b (ports[1]); twist
    // theta = integral(omega_a - omega_b), shaft torque tau_s acts -tau_s on
    // a and +tau_s on b. Per substep (pre_solve_components' tail, after
    // every component's pre_solve and after any topology switch):
    //   trapezoidal (default):  c_impl = c/2 + k*h/4,
    //                           tau_expl = k*theta + c_impl*v_old,
    //                           theta' = theta + h*(v_old + v_new)/2
    //   backward Euler (the first substep after a mode change - a topology
    //   switch or any friction-row stick-flag change in the PREVIOUS
    //   substep, or a reset()):  c_impl = c + k*h, tau_expl = k*theta,
    //                           theta' = theta + h*v_new
    // with c_impl applied implicitly via Assembly::set_coupling and tau_expl
    // via Assembly::torque; tau_s = tau_expl + c_impl*v_new. Ledger (solve()):
    // shaft_pe = sum 1/2*k*theta'^2 (instantaneous), shaft_damping_loss +=
    // h*c*v_d^2 (v_d = (v_old+v_new)/2 trapezoidal, v_new BE), and the
    // spring's discretisation residual h*tau_s*v_new - dPE - dD goes into
    // integrator_loss (trapezoidal: (h/2)*tau_s*(v_new - v_old), any sign;
    // BE: 1/2*k*h^2*v_new^2 >= 0). A Rigid vehicle has none of this: no
    // coupling, no term, no hash byte, no ledger change.
    struct CompliantShaft {
        NodeId a, b;
        real k = 0.0;
        real c = 0.0;
        int shaft_index = 0; // into PowertrainDesc::shafts
        CouplingId coupling{};
        real theta = 0.0;    // STATE (hashed): twist, rad
        real v_old = 0.0;    // this substep's omega_a - omega_b before the solve (scratch)
        real c_impl = 0.0;   // this substep's implicit damping coefficient (scratch)
        real tau_expl = 0.0; // this substep's explicit torque (scratch)
        real tau_last = 0.0; // last substep's total shaft torque tau_s (debug only, not hashed)
        std::uint8_t reset_mode = 0; // reset(): 0 leave (both ends set elsewhere), 1 omega_b = omega_a, 2 omega_a = omega_b
    };
    [[nodiscard]] bool has_compliant_shafts() const { return !compliant_.empty(); }
    [[nodiscard]] const std::vector<CompliantShaft>& compliant_shafts() const { return compliant_; }
    // true when the NEXT substep will integrate the compliant shafts with
    // backward Euler (a mode change happened last substep, or a reset()).
    [[nodiscard]] bool compliant_be_next() const { return be_next_; }
    // true when the LAST completed substep used backward Euler.
    [[nodiscard]] bool compliant_be_last() const { return be_this_; }
    // PowertrainDebug fill: the i-th COMPLIANT shaft (shafts[] order, rigid
    // shafts skipped) in slot i; unused slots 0.
    void fill_shaft_debug(std::array<real, kMaxCompliantShafts>& twist, std::array<real, kMaxCompliantShafts>& torque) const {
        twist.fill(real(0.0));
        torque.fill(real(0.0));
        const std::size_t n = std::min(compliant_.size(), twist.size());
        for (std::size_t i = 0; i < n; ++i) {
            twist[i] = compliant_[i].theta;
            torque[i] = compliant_[i].tau_last;
        }
    }

    // Network omega/stick-sets/topology-state (ShaftNetwork::fold_state_hash)
    // plus every component's own additional state (GearPair/Differential's
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
    // I4 C5: this substep's lock bits from the lock channels (engage within
    // max_mismatch of speed match, else pending; release when < 0.5);
    // updates lock_pending_. Allocation-free.
    [[nodiscard]] int update_lock_bits(const ComponentSubstepCtx& ctx);

    ShaftNetwork network_;
    Assembly assembly_;
    Assembly projection_assembly_; // dedicated scratch for apply_topology_switch, never touches assembly_
    std::vector<std::unique_ptr<IComponent>> components_; // gear_pair(s), differential(s), then brakes (appended last)
    // Non-owning aliases into components_, pt.components order (empty if
    // this vehicle's powertrain has no differential) - the debug reads and
    // white-box accessors use them directly rather than scanning
    // components_ by dynamic_cast every call.
    std::vector<Differential*> diffs_;
    std::vector<std::pair<int, RigidRow>> build_rigid_rows_; // white-box record, build time only
    std::vector<ResetDerivation> reset_derivations_;         // leaves up, built once at construction
    // --- I4 C5 composed states / diff locks --------------------------------
    struct DiffLock {
        std::size_t diff_index = 0; // into diffs_
        NodeId left, right;
        real max_mismatch = 2.0;    // rad/s, |omega_L - omega_R| engage threshold
        int channel_slot = -1;      // into ComponentSubstepCtx::channels
    };
    int gear_state_count_ = 1;                // n_g
    std::vector<DiffLock> locks_;             // lockable diffs, component order (bit i)
    std::vector<std::uint8_t> lock_pending_;  // parallel to locks_, sized once at construction; NOT hashed
    // I4 C6: kind == Locked diffs' L/R node pairs - the row is unconditional/
    // state-independent (not a locks_/lock_pending_ entry, no channel, no
    // bit), but reset() still needs to force L/R to their mean so the
    // post-reset state satisfies this row exactly, same reasoning as the
    // ENGAGED-lock averaging below (reset step 3.5) applied unconditionally.
    std::vector<std::pair<NodeId, NodeId>> always_locked_diffs_;
    std::vector<NodeId> hub_nodes_;
    std::vector<std::uint8_t> driven_; // parallel to hub_nodes_, WheelDesc order
    real driven_wheel_mean_radius_ = 0.0; // mean radius of driven wheels, computed once at construction
    std::vector<real> omega_old_;      // per-node scratch, sized once at construction; filled fresh in solve()
    std::vector<real> topo_switch_scratch_; // per-node scratch for apply_topology_switch, sized once at construction
    PowertrainEnergy ledger_{};
    PowertrainEnergy ledger_tick_start_{};

    // --- engine (nullptr if this vehicle's powertrain has none) -----------
    std::unique_ptr<IEngine> engine_;
    NodeId engine_crank_;
    FrictionRowId engine_friction_row_; // engine-to-ground Coulomb friction (plan section 5.1 row 2)
    EngineTorqueMap::PreSolveResult last_engine_result_{}; // this substep's pre_solve() output, cached
    // PHYS-033 P4 I7a C5, broadened C5d: true iff this vehicle's engine has
    // ANY fuel model (a calibrated fuel_map OR the crude no-map estimate,
    // see ps::drivetrain::has_fuel_model in powertrain_desc.h) - gates
    // ledger_.fuel_mass/fuel_energy_in/engine_heat/
    // fuel_efficiency_violations booking in post_solve_components (indicated_work
    // itself is booked unconditionally - see that function). false (default)
    // for every engine without one, so those ledger fields never move.
    bool engine_has_fuel_model_ = false;
    // PHYS-033 P4 I7a C6: the tank feeding the engine's fuel port (nullptr
    // without one), and a set_fuel_tank_mass request waiting for the next
    // begin_tick(). Only fuel_tank_'s content is simulation state (hashed
    // when present); the pending request is an input, like a control value.
    std::unique_ptr<FuelTank> fuel_tank_;
    bool fuel_tank_set_pending_ = false;
    real fuel_tank_set_value_ = 0.0;
    // PHYS-033 P4 I7a C3: the throttle-command stage between the pedal and
    // the engine (throttle_stage.h) - built from either an explicit
    // ThrottleControlDesc component or (no such component) the IMPLICIT
    // default (the engine's own throttle_input, no curve, a hand lever ==
    // the governor's channel iff the engine has_governor). Replaces the old
    // engine_governor_channel_slot_ (the hand-lever channel slot now lives
    // inside throttle_stage_ itself). last_throttle_command_/
    // last_throttle_hand_lever_raw_ cache this substep's stage output for
    // the same-substep engine pre_solve call and for the accessors above
    // (throttle_command/governor_hand_throttle).
    ThrottleStage throttle_stage_;
    ThrottleCommand last_throttle_command_{};
    real last_throttle_hand_lever_raw_ = 0.0;

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

    // --- speedo path (plan section 7.6 / I4 section 2.5), cached at build
    // time; diff_ratio_ is the FIRST diff's ratio (1 without one). ----------
    real diff_ratio_ = 1.0;
    real speedo_ratio_ = 1.0;
    real speedo_radius_ = 0.0;

    // --- transfer case (I4 C9, plan section 3.3): nullptr/default FrictionRowId
    // if this vehicle's powertrain has no transfer_case component. Non-owning
    // alias into components_ (mirrors diffs_'s pattern); transfer_row_ is
    // cached directly on Powertrain (mirrors clutch_row_/gearbox_synchro_row_
    // above) rather than exposed via a TransferCase::row() accessor, since
    // only Powertrain's own awd_clutch_torque()/awd_clutch_slip() need it. ---
    TransferCase* transfer_case_ = nullptr;
    FrictionRowId transfer_row_{};

    // --- park lock (I4 C10, plan section 3.4): nullptr/default FrictionRowId
    // if this vehicle's powertrain has no park_lock component. Same
    // non-owning-alias-into-components_ pattern as transfer_case_ above. ---
    ParkLock* park_lock_ = nullptr;
    FrictionRowId park_lock_row_{};

    // --- pto (I4 C11, plan section 3.5): nullptr/default FrictionRowId if
    // this vehicle's powertrain has no pto component. Same non-owning-alias-
    // into-components_ pattern as transfer_case_/park_lock_ above. ---
    Pto* pto_ = nullptr;
    FrictionRowId pto_row_{};

    // --- compliant shafts (I4 C13, see CompliantShaft above). Every member
    // below stays empty/false/unused for a Rigid vehicle. ---
    std::vector<CompliantShaft> compliant_;          // shafts[] order, built once at construction
    bool be_next_ = true;                            // STATE (hashed when compliant): next substep is backward Euler
    bool be_this_ = false;                           // this/last substep's scheme (scratch)
    bool topo_switched_this_substep_ = false;        // set in pre_solve_components, read by solve()
    std::vector<std::uint8_t> compliant_stuck_before_; // per friction row, sized once at construction (compliant only)

    // --- load_dyno(s) (I4 C11, plan section 3.5, test/bench only): every
    // load_dyno component, component order. Empty for every shipped vehicle
    // (car_sedan.json, tractor.json) - only a test rig adds one in code. ---
    std::vector<LoadDyno*> load_dynos_;

    // --- TCU (nullptr if this vehicle has no manual_tcu controller) -------
    // PHYS-033 P4 I7a C3: tcu_prev_request_ used to be an EngineTorqueMap::
    // TcuRequest, but that type lost throttle_floor/throttle_ceiling (moved
    // into ThrottleCommand, engine.h - only ignition_cut still goes to the
    // engine directly, see the "which TCU intervention lives where" table in
    // throttle_stage.h); TcuThrottleLimits (namespace scope, above) is the
    // DIFFERENT thing this always was: the TCU's own lagged Outputs.
    std::unique_ptr<ManualTcu> tcu_;
    TcuThrottleLimits tcu_prev_request_{}; // LAGGED one substep, fed into the engine (plan section 7.2)

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
