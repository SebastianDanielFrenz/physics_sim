// core/src/vehicle/wheel_instance.h — one wheel: suspension travel
// (measured kinematically via a cylinder shape cast, out/handover_p3.md
// decision 1), tyre slip relaxation state (decision 2) and spin, brakes.
// Internal (PLAN.md D2/D10 pattern: no Jolt type here; owned by
// vehicle_instance.h, itself owned by system.h - never included from a
// public header).

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/tyre_model.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/vehicle/wheel_telemetry.h"

#include <cstdint>
#include <memory>

namespace ps::vehicle {

// PHYS-033 P4 I2 commit 5b (plan section 5.6): wheel spin moved OUT of
// WheelInstance into the drivetrain network's hub node - pre_solve/
// post_solve replace the old single step(), split around
// ps::drivetrain::ShaftNetwork::solve() the way every other drivetrain
// component's pre_solve/post_solve is (core/src/drivetrain/components.h).
// drive_torque_cmd/brake_torque_cmd are gone (the hub's torque now comes
// from the network's own implicit solve, not a value computed once per
// tick outside it).
struct WheelPreSolveInput {
    Pose chassis_pose;
    Motion chassis_motion;
    Vec3 chassis_com_world = Vec3::zero();
    real steer_angle = 0.0;
    real dt = 0.0;
    real partner_compression = 0.0;
    const io::SurfaceTable* surfaces = nullptr;
};

// The hub's implicit torque-law terms this wheel contributes THIS substep
// (plan section 5.6 point 5): tau(w') ~= tau0 - d*(w' - w_old), d >= 0. The
// caller (VehicleInstance::step_tick) forwards these straight into
// Powertrain::add_hub_terms.
struct HubTerms {
    real tau0 = 0.0;
    real d = 0.0;
};

// PHYS-033 I1b (defect 2 fix, decision D-B): the ground-plane contact frame
// at this wheel's measured contact, computed in pass 1 (measure_contact) and
// consumed in pass 2 (step()). Recomputed from the pose every substep, like
// x_ - deliberately NOT folded into fold_state_hash (see wheel_instance.h's
// own top-level comment on why render-only/derived-each-substep state stays
// out of the hash).
struct ContactFrame {
    // Where the contact force acts: the wheel centre at the hit (I1b commit
    // C6), replaced by the true ground contact patch (commit C7, decision
    // D-A) once landed.
    Vec3 point = Vec3::zero();
    Vec3 normal = Vec3::unit_z();   // n: points from the ground toward the wheel
    Vec3 long_dir = Vec3::unit_x(); // t_x = normalize(spin_axis x n): the wheel's rolling direction, in the contact plane
    Vec3 lat_dir = Vec3::unit_y();  // t_y = n x t_x
    // True when `normal` had to be replaced by the suspension axis (a
    // bottomed/self-hit/wall/side-wall contact - see measure_contact's own
    // comment for the exact fallback conditions). Diagnostic only, not
    // hashed.
    bool normal_fallback = false;
};

struct WheelSubstepOutput {
    // R = N*n + (Fx+F_roll)*t_x + Fy*t_y (decision D-B): ONE force vector
    // per wheel, replacing the old separate suspension_force_world (along
    // the chassis-fixed suspension axis) + tyre_force_world (along chassis-
    // fixed forward/lateral axes) - both used to ignore the ground normal
    // entirely (PHYS-033 I1b defect 2).
    Vec3 contact_force_world = Vec3::zero();
    Vec3 contact_point_world = Vec3::zero(); // where contact_force_world (and extra_torque_world below) act
    Vec3 extra_torque_world = Vec3::zero();  // Mz*n + Mx*t_x, a pure couple (no torque-arm needed)
    // PHYS-033 P4 I2 commit 5b (plan section 8): this substep's own
    // contribution to the powertrain energy ledger's wheel_out/tyre_loss
    // terms (0 while airborne or fz <= 0) - VehicleInstance::step_tick
    // forwards these into Powertrain::add_wheel_energy.
    real wheel_out_j = 0.0;
    real tyre_loss_j = 0.0;
};

class WheelInstance {
public:
    WheelInstance(WheelDesc desc, std::unique_ptr<ITyreModel> tyre_model);

    [[nodiscard]] real compression() const { return x_; }

    // Pass 1 (every substep, before pre_solve() below - ARB needs every
    // wheel's fresh measurement first): cylinder shape cast along the
    // suspension axis to find ground contact and this substep's compression.
    void measure_contact(const Pose& chassis_pose, real steer_angle, const IRigidBackend& backend);

    // Pass 2 (plan section 5.6): suspension force from the measured
    // compression, tyre force PREDICTOR from relaxed slip (final alpha,
    // predictor kappa - final only after post_solve), and this wheel's
    // linearised hub torque-law terms for the network solve. `omega_old` is
    // this substep's hub speed BEFORE the network solve (Powertrain::
    // hub_omega, read by the caller before calling ShaftNetwork::solve).
    [[nodiscard]] HubTerms pre_solve(const WheelPreSolveInput& in, real omega_old);
    // Pass 2, after the network's solve: finishes kappa/Fx at the hub's NEW
    // speed (still linear in omega_new, so the wheel's own torque balance
    // and the chassis force stay consistent - plan section 5.6 point 2/3),
    // assembles this substep's contact wrench and energy-ledger deltas.
    [[nodiscard]] WheelSubstepOutput post_solve(real omega_new);

    [[nodiscard]] const WheelTelemetry& telemetry() const { return last_telemetry_; }
    [[nodiscard]] const WheelDesc& desc() const { return desc_; }

    // PLAN.md M1, task 2 of the P3 fix-up phase: the render-facing subset
    // (see wheel_state.h's own header comment for why this is a separate
    // type from WheelTelemetry above). Computed on demand from state
    // already held by this instance - no extra storage duplicating
    // last_telemetry_'s fields.
    [[nodiscard]] WheelState wheel_state() const {
        WheelState s;
        s.steer_angle = last_steer_angle_;
        s.suspension_travel = x_;
        s.spin_angle = spin_angle_;
        s.load = last_telemetry_.fz;
        s.slip_ratio = last_telemetry_.slip_ratio;
        s.slip_angle = last_telemetry_.slip_angle;
        s.surface = last_telemetry_.surface;
        return s;
    }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    // White-box test access (tests/unit/test_wheel_contact_frame.cpp, core/src
    // on its include path per tests/unit/CMakeLists.txt) - see ContactFrame's
    // own comment for why this is not part of fold_state_hash.
    [[nodiscard]] const ContactFrame& contact_frame() const { return contact_frame_; }
    [[nodiscard]] bool hit_ground() const { return hit_ground_; }

private:
    WheelDesc desc_;
    std::unique_ptr<ITyreModel> tyre_;

    real x_ = 0.0;
    real x_prev_ = 0.0;
    bool hit_ground_ = false;
    ContactFrame contact_frame_;
    SurfaceId contact_surface_ = kInvalidSurfaceId;

    SlipRelaxationState relax_;

    // PHYS-033 P4 I2 commit 5b: stashed by pre_solve, consumed by
    // post_solve - one substep's worth of scratch (plan section 5.6 point
    // 4), never allocated (plain members). `pre_airborne_` is
    // !(hit_ground_ && fz > 0) at pre_solve time, i.e. whether the tyre/
    // hub-coupling branch ran at all this substep.
    real pre_dt_ = 0.0;
    real pre_fz_ = 0.0;
    real pre_fx_pred_ = 0.0;
    real pre_C_ = 0.0;     // max(dFx/dkappa, 0), 0 if the kappa predictor clamped
    real pre_dk_dw_ = 0.0; // R * (d kappa_predictor / d slip_velocity)
    RelaxStep pre_rs_{};   // kappa's own relaxation_step_axis result this substep
    real pre_omega_old_ = 0.0;
    real pre_vx_contact_ = 0.0, pre_vy_contact_ = 0.0;
    real pre_fy_ = 0.0, pre_mz_ = 0.0, pre_mx_ = 0.0, pre_froll_ = 0.0;
    bool pre_airborne_ = true;
    Vec3 pre_wheel_center_world_ = Vec3::zero();

    // PLAN.md M1, task 2: render-only state, deliberately NOT folded into
    // fold_state_hash() below (see wheel_instance.cpp's step() for why -
    // both are write-only as far as the simulation itself is concerned,
    // never read back by anything that affects future dynamics, so
    // excluding them cannot hide a worker-count/scheduling-dependent
    // divergence - and this task's own requirement is that every existing
    // scenario's state_hash stays bit-identical).
    real spin_angle_ = 0.0;
    real last_steer_angle_ = 0.0;

    WheelTelemetry last_telemetry_;
};

} // namespace ps::vehicle
