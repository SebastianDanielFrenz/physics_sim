#include "gearbox.h"

#include <cmath>
#include <stdexcept>

namespace ps::drivetrain {
namespace {
template <typename T>
std::uint64_t fnv1a(std::uint64_t h, const T& v) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}
} // namespace

Gearbox::Gearbox(Desc desc) : desc_(std::move(desc)) {
    if (desc_.forward_ratios.size() != desc_.forward_efficiency.size()) {
        throw std::runtime_error("Gearbox: forward_ratios/forward_efficiency size mismatch");
    }
    if (desc_.reverse_ratios.size() != desc_.reverse_efficiency.size()) {
        throw std::runtime_error("Gearbox: reverse_ratios/reverse_efficiency size mismatch");
    }
    if (!desc_.synchro.forward_capacity_nm.empty() && desc_.synchro.forward_capacity_nm.size() != desc_.forward_ratios.size()) {
        throw std::runtime_error("Gearbox: synchro.forward_capacity_nm size mismatch");
    }
    if (!desc_.synchro.reverse_capacity_nm.empty() && desc_.synchro.reverse_capacity_nm.size() != desc_.reverse_ratios.size()) {
        throw std::runtime_error("Gearbox: synchro.reverse_capacity_nm size mismatch");
    }
    const int n = static_cast<int>(desc_.forward_ratios.size());
    const int m = static_cast<int>(desc_.reverse_ratios.size());
    if (desc_.initial_gear < -m || desc_.initial_gear > n) {
        throw std::runtime_error("Gearbox: initial_gear out of range");
    }
    current_ = desc_.initial_gear;
    target_ = desc_.initial_gear;
    walk_base_ = desc_.initial_gear;
}

real Gearbox::ratio_for_position(int position) const {
    if (position > 0) return desc_.forward_ratios[static_cast<std::size_t>(position - 1)];
    if (position < 0) return desc_.reverse_ratios[static_cast<std::size_t>(-position - 1)];
    return real(1.0); // N: unused (no gear row present)
}

real Gearbox::efficiency_for_position(int position) const {
    if (position > 0) return desc_.forward_efficiency[static_cast<std::size_t>(position - 1)];
    if (position < 0) return desc_.reverse_efficiency[static_cast<std::size_t>(-position - 1)];
    return real(1.0);
}

real Gearbox::synchro_capacity_kinetic_for_position(int position) const {
    if (position > 0 && !desc_.synchro.forward_capacity_nm.empty()) {
        return desc_.synchro.forward_capacity_nm[static_cast<std::size_t>(position - 1)];
    }
    if (position < 0 && !desc_.synchro.reverse_capacity_nm.empty()) {
        return desc_.synchro.reverse_capacity_nm[static_cast<std::size_t>(-position - 1)];
    }
    return real(0.0);
}

int Gearbox::next_step_position() const {
    if (target_ == current_ && target_ == walk_base_) return current_;
    if (desc_.type == GearboxType::Sequential) {
        if (target_ == walk_base_) return walk_base_;
        return walk_base_ + (target_ > walk_base_ ? 1 : -1);
    }
    return target_;
}

Gearbox::StepResult Gearbox::phase_step(const StepInput& in) {
    // Retarget (plan section 6.3's "Retarget" row): a change in the CURRENT
    // episode's own step destination while Selecting or Synchronising
    // restarts Synchronising with the new ratio. Comparing next_step_
    // position() (not target_ directly) makes this uniform for Sequential's
    // one-step-at-a-time walk too - see this file's header comment.
    if (phase_ == Phase::Selecting || phase_ == Phase::Synchronising) {
        const int step_now = next_step_position();
        if (step_now != synchronising_step_) {
            phase_ = Phase::Synchronising;
            t_phase_ = real(0.0);
            grind_flag_ = false;
            synchronising_step_ = step_now;
        }
    }

    bool just_refused = false;
    switch (phase_) {
        case Phase::Idle:
            if (target_ != current_) {
                walk_base_ = current_; // capture the walk's starting gear before current_ passes through 0 mid-shift
                phase_ = (current_ == 0) ? Phase::Selecting : Phase::Releasing;
                t_phase_ = real(0.0);
                if (phase_ == Phase::Selecting) synchronising_step_ = next_step_position();
            }
            break;
        case Phase::Releasing:
            if (std::abs(in.lambda_mesh_prev) <= desc_.dog_release_torque_nm) {
                current_ = 0; // dog out
                phase_ = Phase::Selecting;
                t_phase_ = real(0.0);
                synchronising_step_ = next_step_position();
            }
            // else: waits (the driver must lift or clutch) - stays Releasing
            break;
        case Phase::Selecting:
            t_phase_ += in.h;
            if (t_phase_ >= desc_.neutral_phase_s) {
                const int step_now = next_step_position();
                if (step_now == 0) {
                    phase_ = Phase::Idle; // this step's own destination is N: done, no synchro needed
                } else {
                    phase_ = Phase::Synchronising;
                    t_phase_ = real(0.0);
                    grind_flag_ = false;
                    synchronising_step_ = step_now;
                }
            }
            break;
        case Phase::Synchronising: {
            const real r_target = ratio_for_position(synchronising_step_);
            const real delta = in.omega1 - r_target * in.omega2;
            if (std::abs(delta) <= desc_.synchro.max_dog_mismatch_rad_s) {
                current_ = synchronising_step_; // dog in
                phase_ = Phase::Idle;
            } else {
                t_phase_ += in.h;
                if (t_phase_ >= desc_.abort_after_s) {
                    target_ = 0;
                    phase_ = Phase::Idle;
                    grind_active_ = false;
                    just_refused = true;
                } else if (t_phase_ >= desc_.grind_after_s && !grind_flag_) {
                    grind_flag_ = true;
                    grind_active_ = true;
                    ++grind_events_;
                }
            }
            break;
        }
    }
    if (phase_ != Phase::Synchronising) grind_active_ = false;

    StepResult r;
    r.state_index = state_index_for_position(current_);
    r.current = current_;
    r.target = target_;
    r.phase = phase_;
    r.t_phase = t_phase_;
    r.grind_active = grind_active_;
    r.just_refused = just_refused;
    if (phase_ == Phase::Synchronising) {
        r.synchro_target_ratio = ratio_for_position(synchronising_step_);
        r.synchro_cap_kinetic = synchro_capacity_kinetic_for_position(synchronising_step_);
        r.synchro_cap_static = desc_.synchro.static_factor * r.synchro_cap_kinetic;
    }
    return r;
}

std::uint64_t Gearbox::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    h = fnv1a(h, current_);
    h = fnv1a(h, target_);
    h = fnv1a(h, phase_);
    h = fnv1a(h, t_phase_);
    h = fnv1a(h, grind_flag_);
    h = fnv1a(h, synchronising_step_);
    h = fnv1a(h, walk_base_);
    return h;
}

} // namespace ps::drivetrain
