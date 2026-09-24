// core/src/drivetrain/gearbox.h — the I3 manual/sequential gearbox position
// and phase machine (PHYS-033 P4 I3 plan section 6.3). A new class,
// framework-agnostic like engine_torque_map.h/clutch.h - not yet wired into
// Powertrain (commit 6, "the switch", also wires its state_request output
// into ShaftNetwork::set_topology_state).
//
// Positions: pos in {-m..-1, 0, 1..n} (reverse Rk = -k, N = 0). Topology-
// state INDICES follow the plan's own sorted order R_m..R_1, N, F_1..F_n,
// which this class computes as index = reverse_ratios.size() + position -
// see state_index_for_position()'s own comment.

#pragma once

#include "ps/types.h"

#include <cstdint>
#include <vector>

namespace ps::drivetrain {

enum class GearboxKind : std::uint8_t { Manual, Sequential };

struct GearboxSynchroDesc {
    std::vector<real> forward_capacity_nm; // per forward gear, input-shaft torque; 0 = no synchro (dog only)
    std::vector<real> reverse_capacity_nm; // per reverse gear
    real static_factor = 1.0;
    real max_dog_mismatch_rad_s = 5.0; // at the input shaft
};

class Gearbox {
public:
    struct Desc {
        GearboxKind type = GearboxKind::Manual;
        std::vector<real> forward_ratios;    // > 0, strictly decreasing (1st first). w_in = ratio * w_out
        std::vector<real> reverse_ratios;    // < 0, R1 first (the slowest reverse)
        std::vector<real> forward_efficiency, reverse_efficiency; // (0, 1], same lengths as the ratios above
        real input_inertia_kgm2 = 0.0, output_inertia_kgm2 = 0.0;
        GearboxSynchroDesc synchro;
        real neutral_phase_s = 0.06;       // selector travel through neutral (minimum time dog-out -> synchro)
        real dog_release_torque_nm = 60.0; // |mesh torque at input| must be below this to pull a gear out
        real grind_after_s = 0.25;         // synchronising longer than this -> grind event
        real abort_after_s = 1.0;          // synchronising longer than this -> give up, stay in N
        int initial_gear = 0;              // 0 = N, k > 0 forward, -k reverse Rk
    };

    enum class Phase : std::uint8_t { Idle, Releasing, Selecting, Synchronising };

    struct StepInput {
        real omega1 = 0.0;         // gearbox input shaft omega_old
        real omega2 = 0.0;         // gearbox output shaft omega_old
        real lambda_mesh_prev = 0.0; // last substep's gear-mesh row torque (0 if not currently engaged)
        real h = 0.0;
    };
    struct StepResult {
        int state_index = 0;      // the topology-state index the network should be on THIS substep
        int current = 0, target = 0;
        Phase phase = Phase::Idle;
        real t_phase = 0.0;
        bool grind_active = false;
        bool just_refused = false; // this substep aborted a synchronising attempt back to N
        real synchro_target_ratio = 0.0; // r of the position being synchronised to, 0 if not Synchronising
        real synchro_cap_kinetic = 0.0, synchro_cap_static = 0.0; // 0 unless Synchronising
    };

    explicit Gearbox(Desc desc);

    // Evaluated once per substep, BEFORE the network's topology-state switch
    // decision (plan section 7.2 step 3b uses state_index from this call's
    // result). Advances the phase/position state machine.
    [[nodiscard]] StepResult phase_step(const StepInput& in);

    void set_target(int t) { target_ = t; }
    [[nodiscard]] int target() const { return target_; }
    [[nodiscard]] int current_position() const { return current_; }
    [[nodiscard]] Phase phase() const { return phase_; }
    [[nodiscard]] std::uint32_t grind_events() const { return grind_events_; }

    // Position <-> state-index mapping, sorted order R_m..R_1, N, F_1..F_n:
    // index = reverse_ratios.size() + position (uniform for reverse, N and
    // forward alike - e.g. m=1 reverse gear: R1 -> index 0, N -> index 1,
    // F1 -> index 2, matching the plan's own worked example section 5.2).
    [[nodiscard]] int state_index_for_position(int position) const {
        return static_cast<int>(desc_.reverse_ratios.size()) + position;
    }
    [[nodiscard]] real ratio_for_position(int position) const;
    [[nodiscard]] real efficiency_for_position(int position) const;
    [[nodiscard]] real synchro_capacity_kinetic_for_position(int position) const;

    // For Sequential, the immediate next step toward target() from the walk's
    // own base (see walk_base_ - NOT current_position(), which is
    // transiently 0 mid-shift); for Manual, target() directly.
    [[nodiscard]] int next_step_position() const;

    [[nodiscard]] const Desc& desc() const { return desc_; }
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    // World::reset_vehicle (plan section 4.5): forces this gearbox directly
    // into `gear` with phase Idle - no shift sequence, no grind event
    // (grind_events_ is a lifetime stat, left untouched).
    void reset_to(int gear) {
        current_ = gear;
        target_ = gear;
        phase_ = Phase::Idle;
        t_phase_ = real(0.0);
        grind_flag_ = false;
        grind_active_ = false;
        synchronising_step_ = 0;
        walk_base_ = gear;
    }

private:
    Desc desc_;
    int current_ = 0, target_ = 0;
    Phase phase_ = Phase::Idle;
    real t_phase_ = 0.0;
    bool grind_flag_ = false;   // already fired a grind event this Synchronising episode
    bool grind_active_ = false;
    std::uint32_t grind_events_ = 0;
    int synchronising_step_ = 0; // the position value the CURRENT Selecting/Synchronising episode is chasing
    // For Sequential: the gear this walk STARTED from (captured once, when
    // the shift begins) - current_ itself passes through 0 (neutral) as a
    // transient waypoint mid-shift, so next_step_position() must NOT use
    // current_ directly (it would recompute "one step from neutral", i.e.
    // always gear 1, instead of continuing the walk toward target_).
    int walk_base_ = 0;
};

} // namespace ps::drivetrain
