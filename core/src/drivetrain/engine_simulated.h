// core/src/drivetrain/engine_simulated.h — ps::drivetrain::SimulatedEngine
// (PHYS-033 P4 I7b C11): the mean-value engine model (MVEM) behind IEngine,
// next to EngineTorqueMap. Internal (core/src, not a public header); built
// from the public SimulatedEngineDesc (ps/drivetrain/powertrain_desc.h,
// I7b C10).
//
// Model (I7 plan section 3.1), all per crank revolution unless noted:
//   m_thr     = (Cd*A_thr(x_thr) + x_iac*A_bypass) * p_up/sqrt(R*T) * Psi(p_im/p_up)
//               (reverse flow antisymmetric: -(...)*p_im/sqrt(R*T)*Psi(p_up/p_im))
//   m_air_rev = VE(rpm, p_im) * p_im/(R*T_im) * V_disp * c_cycle
//   m_cyl     = m_air_rev * max(omega, 0)/(2*pi)
//   dp_im/dt  = R*T_im/V_im * (m_thr - m_cyl)
//   m_fuel_rev = m_air_rev/(lambda*AFR) * fire * s_starve
//   W_ind_rev = eta_i(rpm, m_air_rev) * f_lambda(lambda) * eta_spark(theta - theta_MBT) * LHV * m_fuel_rev
//   tau       = W_ind_rev/(2*pi)                                       (combustion)
//             - sign(omega)*(p_em - p_im)*V_disp*c_cycle/(2*pi)         (pumping)
//             - sign(omega)*pmep_excess(rpm, p_im)*V_disp*c_cycle/(2*pi) (optional extra pumping)
//             - sign(omega)*(F(rpm) - F(0))                              (smooth friction; F(0) is the
//                                                                         Coulomb row's kinetic capacity)
//             + tau_start                                                (same starter as EngineTorqueMap)
// with p_up = p_em = T_im's ambient (NA engine; set_ambient, ISA default)
// and theta = min(spark_target, theta_kl) when a knock-limit map exists.
// eta_i is GROSS indicated efficiency at MBT spark and lambda = 1 (I7c
// interface request 4): f_lambda and eta_spark act on top of it.
//
// Per substep (plan section 3.3): (1) state machine; (2) engine management
// (request/DFCO/idle PI/limiter - duplicated from EngineTorqueMap on
// purpose, plan section 3.2: refactoring the graph engine to share it would
// reorder its floating-point operations); (3) actuator (electronic:
// backward-Euler first-order lag toward max(g(rpm, request), x_idle);
// mechanical: plate = command, the idle PI drives the bypass valve x_iac);
// (4) one linearly-implicit Euler step of p_im with omega frozen at
// omega_old, plus an equilibrium guard: if f changed sign across the step
// the chord root of f between the two points is taken instead (the tangent
// step overshoots on the concave throttle flow at a small plenum - 119 kPa
// from a 74 kPa WOT step at 0.2 L without it); (5) tau0 at (omega_old, p_im_new) and d = max(-dtau/domega, 0)
// at that frozen air state from analytic table slopes.
//
// Runtime math (plan section 3.5): + - * /, sqrt (set_ambient only) and
// table lookups. Psi is tabulated at construction with ps::math::pow on a
// uniform grid, so its lookup is a direct index, no bracket search and no
// cached index. Zero allocation after construction.
//
// Everything derived at construction (Psi table, the steady-state
// WOT/motoring curves, the steady torque table, the electronic inverse
// plate table, the idle feed-forward) lives in ONE immutable
// SimulatedEngineMaps behind a shared_ptr<const> (I7c interface request 1),
// with a content hash that fold_state_hash folds on EVERY call (request 2).
//
// C11 scope: standalone only - Powertrain does not construct this yet
// (Powertrain::Powertrain throws "not yet integrated (I7b C12)" for a
// simulated engine). Rotary geometry is refused here (C13).

#pragma once

#include "engine.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/table2d.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace ps::drivetrain {

// Immutable model data shared by every SimulatedEngine built from the same
// desc (I7c interface request 1). Built by SimulatedEngine::build_maps.
struct SimulatedEngineMaps {
    // --- copied from the desc (SI) ---------------------------------------
    ThrottleInput throttle_input = ThrottleInput::Electronic;
    real v_disp_m3 = 0.0;          // swept volume per cycle (piston: cylinders*pi/4*bore^2*stroke)
    real c_cycle = 0.0;            // cycles per crank revolution (4-stroke 1/2)
    real v_im_m3 = 0.0;
    real a_throttle_full_m2 = 0.0; // count*pi/4*d^2
    real cd = 0.0;
    real a_bypass_m2 = 0.0;        // mechanical only
    vehicle::Curve1D area_fraction;
    SimulatedEcuDesc ecu;
    SimulatedEngineMapsDesc maps;  // source string cleared (not model data)
    real lhv_j_kg = 0.0, afr_stoich = 0.0; // fuels[0]
    real c_k = 0.0;                // maps.friction_torque_nm(0): the Coulomb row's kinetic capacity
    EngineLimiterDesc limiter;     // runtime only: the steady tables below IGNORE it (like EngineTorqueMap's torque_demand)

    // --- derived at construction ------------------------------------------
    std::vector<real> psi;         // kPsiIntervals + 1 knots, pr = i/kPsiIntervals
    real ref_p_pa = 0.0, ref_t_k = 0.0; // ambient the steady tables below were built at (ISA)
    // All steady values: net crank torque at omega > 0 (Coulomb row
    // included, i.e. tau - c_k), at the steady p_im (bisection root of
    // f(p) = 0), ISA ambient, no limiter, no starter.
    vehicle::Curve1D steady_wot_nm;       // x = rpm; plate 1, fired
    vehicle::Curve1D steady_motoring_nm;  // x = rpm; plate 0, fuel off
    Table2D steady_torque_nm;             // x = rpm (500 .. limiter + 250, step 250), y = plate 0..1 step 0.025; fired, bypass closed
    Table2D inverse_plate;                // electronic only: x = rpm, y = request in [0,1] -> plate target
    real idle_ff = 0.0;                   // idle feed-forward: plate (electronic) / bypass (mechanical)
    real idle_p_im = 0.0;                 // steady manifold pressure at idle_rpm with idle_ff
    std::uint64_t content_hash = 0;
};

class SimulatedEngine final : public IEngine {
public:
    static constexpr real kGasConstant = 287.05;       // J/(kg*K), dry air
    static constexpr real kGamma = 1.4;
    static constexpr real kIsaPressurePa = 101325.0;
    static constexpr real kIsaTemperatureK = 288.15;
    static constexpr int kPsiIntervals = 1000;
    static constexpr real kPsiRegularisationStart = 0.99;

    explicit SimulatedEngine(const SimulatedEngineDesc& desc);
    // Reuse an already-built maps object (must come from build_maps(desc)
    // on an equal desc - not checked beyond non-null).
    SimulatedEngine(const SimulatedEngineDesc& desc, std::shared_ptr<const SimulatedEngineMaps> maps);

    [[nodiscard]] static std::shared_ptr<const SimulatedEngineMaps> build_maps(const SimulatedEngineDesc& desc);

    [[nodiscard]] PreSolveResult pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) override;
    [[nodiscard]] PostSolveResult post_solve(real omega_new, real h) override;

    [[nodiscard]] real idle_integrator() const override { return idle_integrator_; }
    [[nodiscard]] real crank_angle() const override { return crank_angle_; }
    [[nodiscard]] real idle_rpm() const override { return desc_idle_rpm_; }
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override;
    void reset(EngineState state) override;

    // Ambient (intake upstream = exhaust back pressure = p; T_im = T).
    // ISA until called. PHYS-033 P4 I7b C12 feeds C9's Sensors-side
    // ambient in; the steady tables stay at the ISA they were built at.
    void set_ambient(real p_pa, real t_k);

    // --- white-box accessors ------------------------------------------------
    [[nodiscard]] EngineState state() const { return state_; }
    [[nodiscard]] real p_im() const { return p_im_; }
    [[nodiscard]] real x_thr() const { return x_thr_; }
    [[nodiscard]] real x_iac() const { return x_iac_; }
    [[nodiscard]] bool limiter_cut() const { return limiter_cut_; }
    [[nodiscard]] bool dfco_active() const { return dfco_active_; }
    [[nodiscard]] real ambient_pressure() const { return p_amb_; }
    [[nodiscard]] real ambient_temperature() const { return t_amb_; }
    [[nodiscard]] const SimulatedEngineMaps& maps() const { return *maps_; }
    [[nodiscard]] const std::shared_ptr<const SimulatedEngineMaps>& maps_ptr() const { return maps_; }

    // Torque at an arbitrary (omega, p_im) with the given combustion gate
    // (true = fired, the soft-cut taper still applies), starvation factor
    // and starter flag - the same function pre_solve uses, const, for the
    // T6 central-difference check. tau excludes the Coulomb row, like tau0.
    struct TorqueEval {
        real tau = 0.0;          // tau0 as pre_solve would return it
        real dtau_domega = 0.0;  // analytic, at frozen p_im
        real tau_comb = 0.0;     // fired combustion torque (s_starve applied)
        real tau_pump = 0.0;     // pumping incl. pmep_excess
        real tau_fric = 0.0;     // smooth friction (Coulomb part excluded)
        real tau_start = 0.0;
        real m_air_rev = 0.0;    // kg per crank revolution
        real m_fuel_rev = 0.0;   // kg per crank revolution, fired (fire fraction and s_starve applied)
        real fire = 0.0;         // gate * soft-cut taper, before s_starve
        real lambda = 0.0, spark_deg = 0.0, theta_mbt_deg = 0.0;
        real eta_i = 0.0, f_lambda = 0.0, eta_spark = 0.0;
    };
    [[nodiscard]] TorqueEval evaluate_torque(real omega, real p_im, bool fired, real s_starve, bool starter_active) const;

    // Manifold right-hand side f(p) = dp_im/dt and df/dp at (p_im, omega,
    // plate, bypass) - the function the air step integrates.
    struct ManifoldRhs {
        real f = 0.0, dfdp = 0.0;
        real m_thr = 0.0, m_cyl = 0.0;
    };
    [[nodiscard]] ManifoldRhs manifold_rhs(real p_im, real omega, real x_thr, real x_iac) const;

    // Psi(pr) and dPsi/dpr from the tabulated function (pr in [0, 1]).
    [[nodiscard]] real psi(real pr, real* dpsi_dpr = nullptr) const;

    // This substep's work split (post_solve), all h*omega_old*tau_term:
    // indicated (== PostSolveResult::indicated_work_delta), pumping
    // (<= 0 while p_im < p_amb) and smooth friction (<= 0). Unbooked until
    // C12 decides the ledger fields.
    struct WorkSplit {
        real indicated = 0.0, pumping = 0.0, friction = 0.0;
    };
    [[nodiscard]] const WorkSplit& last_work_split() const { return work_split_; }

private:
    void init_state_for(EngineState state);

    std::shared_ptr<const SimulatedEngineMaps> maps_;
    // desc fields read per substep (engine management)
    real desc_idle_rpm_ = 0.0, stall_rpm_ = 0.0, fire_rpm_ = 0.0, start_complete_rpm_ = 0.0;
    real dfco_enable_rpm_ = 0.0, dfco_reenable_rpm_ = 0.0;
    IdleControlDesc idle_;
    StarterDesc starter_;
    real breakaway_torque_nm_ = 0.0;
    real actuator_tau_s_ = 0.0;

    // ambient
    real p_amb_ = kIsaPressurePa, t_amb_ = kIsaTemperatureK;
    real r_t_ = 0.0;         // R*T_amb
    real inv_sqrt_rt_ = 0.0; // 1/sqrt(R*T_amb)

    // actuator coefficient cache (h/(tau+h)), recomputed only when h changes
    real act_h_ = -1.0, act_a_ = 0.0;

    // hashed state (plan section 3.2)
    EngineState state_ = EngineState::Off;
    real idle_integrator_ = 0.0;
    bool limiter_cut_ = false;
    bool dfco_active_ = false;
    real p_im_ = 0.0;
    real x_thr_ = 0.0;
    real x_iac_ = 0.0; // mechanical only (hashed only there)

    // not hashed
    real crank_angle_ = 0.0;
    real omega_old_ = 0.0, tau0_ = 0.0, d_ = 0.0;
    real tau_start_ = 0.0, tau_comb_ = 0.0, tau_pump_ = 0.0, tau_fric_ = 0.0;
    real booked_fuel_mass_kg_ = 0.0;
    WorkSplit work_split_;
};

} // namespace ps::drivetrain
