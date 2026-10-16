// core/src/drivetrain/auto_transmission.h — the clutch-to-clutch gear change and the automatic TCU of an Automatic
// (torque converter) or DualClutch gearbox (automatic gearbox plan A1; descriptors GearboxDesc::shift_clutch/converter
// in powertrain_desc.h and AutomaticTcuDesc in controller_desc.h). Framework-agnostic, owned by the Powertrain like
// the ManualTcu; the converter itself is torque_converter.h, applied by the Powertrain.
//
// The gear set is two friction rows between the gear set's input node (the turbine of an Automatic, the crank shaft
// of a DualClutch) and gearbox.out. Each row carries the ratio of the gear its clutch holds; there is no rigid gear
// row and no topology switch. A gear change hands the input torque from the off-going row to the on-coming row:
//   upshift   torque phase (torque_phase_s): off-going capacity (1-x)*C_h, on-coming x*C_h - the output torque goes
//             from the old ratio to the new one with no gap; inertia phase: the on-coming clutch slips at C_h and pulls
//             the input down to the new gear's speed (engine torque capped at upshift_torque_fraction); closing: the
//             new clutch ramps C_h -> its rated capacity over engage_time_s.
//   downshift the off-going clutch releases over the torque phase; power on, the input flares freely to the new gear's
//             speed and the on-coming clutch engages there; power off, the on-coming clutch pulls the input up at C_h.
// C_h = clamp(1.3 x max(|input torque of the previous substep|, 0.15 x rated), rated) [AS], so the shift clutches never
// tie the box up with more than the torque they hand over. max_inertia_phase_s bounds the inertia phase.
// A DualClutch steps one gear at a time (odd and even gears alternate shafts) and has to preselect a gear that is not
// already waiting on the idle shaft (preselect_time_s); an Automatic may skip gears. A DualClutch launches on clutch
// slip (engagement e from engine rpm), an Automatic through its converter (e = 1).
//
// CVT plan C1 (Desc::cvt): a Cvt uses the same machine with ONE row (row A, the variator) whose ratio is the variator
// state ratio_; the gears are D (1), N and R only (no gear change), so the shift phases are only the N -> D/R engagement.
// Each substep the TCU picks a target ratio (continuous: input rpm target vs throttle and output rpm; stepped: the
// virtual steps in S or manual mode) and the ratio follows it at the variator's rate limit (in ln r). Row capacity =
// min(forward clutch capacity x launch engagement, belt capacity 2 mu F R_p / cos(beta)) with the clamping force F a
// lagged state (CvtBeltDesc). Without a converter the forward clutch launches on slip like a DualClutch's.
//
// DCT launch control (the assist unit's launch_control on a DualClutch, Sensors::launch_hold_open/launch_cap_nm): while
// Armed the launch clutch is held open, during Launch its capacity is the launch controller's torque-following cap.
#pragma once

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <vector>

namespace ps::drivetrain {

class AutoTransmission {
public:
    enum class Phase : std::uint8_t { Idle, Engage, Preselect, Torque, Inertia, Closing };

    struct Desc {
        bool dual_clutch = false;
        std::vector<real> forward_ratios, reverse_ratios;
        ShiftClutchDesc clutch;
        AutomaticTcuDesc tcu;
        int initial_gear = 0;
        bool has_lockup = false; // Automatic with a converter
        real lockup_capacity_nm = 0.0, lockup_static_factor = 1.0;
        real lockup_engage_time_s = 0.6, lockup_release_time_s = 0.15;
        bool cvt = false; // CVT plan C1: forward_ratios = the virtual steps, cvt_desc the variator, tcu.cvt the ratio law
        CvtDesc cvt_desc;
    };

    struct RowCmd {
        real ratio = 1.0;
        real cap_static = 0.0, cap_kinetic = 0.0;
    };

    struct Sensors {
        real engine_omega = 0.0;
        real input_omega = 0.0;  // gear-set input node (turbine / crank)
        real output_omega = 0.0; // gearbox.out
        real throttle = 0.0;     // driver's command (throttle stage, before traction ceilings)
        real gear_request = kGearRequestNone;
        real shift_up_count = 0.0, shift_down_count = 0.0;
        real mode_channel = 0.0;     // kTransmissionModeChannel, >= 0.5 = sport schedule
        real input_torque_prev = 0.0; // |torque the two shift rows carried last substep| (input-referred)
        real engine_torque_demand = 0.0; // CVT plan C1: the engine's demand last substep (clamping-force feed-forward)
        // DCT launch control (assist unit): hold the launch clutch open (Armed) / its kinetic capacity (Launch, < 0 none).
        bool launch_hold_open = false;
        real launch_cap_nm = -1.0;
        bool first_substep = false;
        real h = 0.0;
    };

    struct Outputs {
        RowCmd row[2];
        real lockup_cap_static = 0.0, lockup_cap_kinetic = 0.0;
        real throttle_ceiling = 1.0;
        int current = 0, target = 0;
        Phase phase = Phase::Idle;
        real launch_engagement = 1.0; // DualClutch: the active clutch's engagement e; Automatic: 1
        // CVT plan C1 (0 on other boxes): variator ratio, virtual step (0 = continuous), belt capacity, clamping force.
        real cvt_ratio = 0.0;
        int cvt_step = 0;
        real belt_capacity_nm = 0.0, clamp_force_n = 0.0;
    };

    explicit AutoTransmission(Desc desc);

    [[nodiscard]] Outputs step(const Sensors& s);
    void reset(int gear, bool clutch_locked);

    [[nodiscard]] const Desc& desc() const { return desc_; }
    [[nodiscard]] int current() const { return cur_; }
    [[nodiscard]] int target() const { return tgt_; }
    [[nodiscard]] Phase phase() const { return phase_; }
    [[nodiscard]] int selector() const { return selector_; }
    [[nodiscard]] bool manual_mode() const { return manual_; }
    [[nodiscard]] real lockup_engagement() const { return e_lock_; }
    [[nodiscard]] int preselected() const { return pre_; }
    [[nodiscard]] real launch_engagement() const { return slip_launch() ? e_launch_ : real(1.0); }
    // DualClutch, or a Cvt without a converter: the box launches on clutch slip.
    [[nodiscard]] bool slip_launch() const { return desc_.dual_clutch || (desc_.cvt && !desc_.has_lockup); }
    [[nodiscard]] real cvt_ratio() const { return ratio_; }
    [[nodiscard]] int cvt_step() const { return step_; }
    [[nodiscard]] real clamp_force() const { return clamp_; }
    [[nodiscard]] bool upshifting() const { return upshift_; }
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    [[nodiscard]] real ratio(int gear) const;
    [[nodiscard]] int forward_count() const { return static_cast<int>(desc_.forward_ratios.size()); }
    [[nodiscard]] int reverse_count() const { return static_cast<int>(desc_.reverse_ratios.size()); }

private:
    void choose_target(const Sensors& s, real th);
    void start_shift(int next, const Sensors& s, real th);
    void complete_shift();
    void update_launch(const Sensors& s, real th);
    void update_lockup(const Sensors& s, real th);
    void update_preselect(const Sensors& s, real th);
    void update_cvt(const Sensors& s, real th);
    [[nodiscard]] int nearest_step() const;
    [[nodiscard]] real belt_capacity() const;

    Desc desc_;
    // Shift machine.
    int cur_ = 0, tgt_ = 0, next_ = 0;
    int slot_ = 0; // which row holds cur_
    Phase phase_ = Phase::Idle;
    real t_phase_ = 0.0;
    real c_hold_ = 0.0;
    real e_engage_ = 1.0;
    bool upshift_ = false, power_on_ = false;
    int pre_ = 0; // DualClutch: gear waiting on the idle shaft (0 = none)
    int pre_cand_ = 0;   // DualClutch: gear being preselected ahead of a shift while Idle
    real t_pre_ = 0.0;   // time spent preselecting pre_cand_
    // TCU.
    int selector_ = 0; // 0 N, < 0 R, k > 0 D limited to k
    bool seeded_ = false;
    std::int64_t last_up_ = 0, last_dn_ = 0;
    bool req_valid_ = false;
    int last_req_ = 0;
    bool manual_ = false;
    int manual_gear_ = 1;
    real manual_t_ = 0.0;
    real time_in_gear_ = 0.0;
    bool kickdown_spent_ = false;
    // Lock-up (Automatic).
    bool lock_req_ = false;
    real e_lock_ = 0.0;
    // Launch (DualClutch).
    real e_launch_ = 0.0;
    bool launch_locked_ = false;
    // CVT (hashed only with Desc::cvt).
    real ratio_ = 1.0;  // variator ratio
    int step_ = 0;      // virtual step of the stepped mode, 0 = continuous
    real t_step_ = 0.0; // time on the current step (stepped S dwell)
    real clamp_ = 0.0;  // belt clamping force [N]
    real cos_beta_ = 1.0; // unhashed constant
};

} // namespace ps::drivetrain
