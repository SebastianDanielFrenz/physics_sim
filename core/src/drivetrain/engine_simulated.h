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
// C12: Powertrain constructs this for a SimulatedEngineDesc component,
// from a desc copy whose fuels list holds only the fuel actually burnt (the
// tank's, else fuels[0]). Rotary geometry is refused here (C13).

#pragma once
#include "turbo_pair.h"

#include "engine.h"
#include "ps/drivetrain/isa_air.h"
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
    real idle_cap_x = 0.0;                // idle authority cap (IdleControlDesc::max_torque_nm), 0 = none
    std::uint64_t content_hash = 0;
};

// PHYS-033 P4 I7b C12: the ambient-dependent derived tables (the ECU's
// steady demand model, the electronic inverse, idle feed-forward). Built at
// ISA into SimulatedEngineMaps (shared, immutable); every SimulatedEngine
// holds its OWN copy, refilled IN PLACE (same axes, indexed writes, no
// allocation) by set_ambient when the ambient changes. Same algorithm as
// build_maps, so a refill at the maps' own ambient reproduces them bit for
// bit.
struct SimulatedSteadyTables {
    real p_pa = 0.0, t_k = 0.0; // ambient the tables below were filled at
    vehicle::Curve1D steady_wot_nm;
    vehicle::Curve1D steady_motoring_nm;
    Table2D steady_torque_nm;
    Table2D inverse_plate; // electronic only (empty for mechanical)
    real idle_ff = 0.0;
    real idle_p_im = 0.0;
    real idle_cap_x = 0.0;
};

class SimulatedEngine final : public IEngine {
public:
    // ps_audio A1 (out/audio_plan.md D-11): these 3 alias the shared
    // header-only constants in ps/drivetrain/isa_air.h (single source of
    // truth with core/src/engine_cycle/map_generator.cpp, which needs the
    // same 3 values and cannot include this ps_core-internal header - see
    // isa_air.h's own header comment). kGamma has no shared consumer yet
    // and stays a plain literal.
    static constexpr real kGasConstant = kIsaAirGasConstantJKgK; // J/(kg*K), dry air
    static constexpr real kGamma = 1.4;
    static constexpr real kIsaPressurePa = kIsaAirPressurePa;
    static constexpr real kIsaTemperatureK = kIsaAirTemperatureK;
    static constexpr int kPsiIntervals = 1000;
    static constexpr real kPsiRegularisationStart = 0.99;
    // Idle manifold-fill compensation (owner 2026-09-29): the idle air path
    // fills the plenum this many times faster than the idle plate alone [A],
    // bringing the ~0.3 s idle manifold lag near the 0.04 s actuator lag.
    static constexpr real kIdleFillGain = 4.0;
    static constexpr real kIdleFillDx = 1e-4; // forward-difference step for df/dx

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
    // Explicit SETUP/OFFLINE setter: materializes complete steady_tables(), which is expensive.
    // Runtime Sensors::ambient instead updates exact physical boundaries immediately and
    // solves only needed idle/pedal/demand operating points, with bounded 24-iteration
    // control bisections. It never builds a full grid, allocates, or quantizes atmosphere.
    // steady_tables() describes the last fully materialized setup/maps-swap cache;
    // ambient_pressure()/temperature() report the CURRENT live physical boundary.
    // Nonfinite/nonpositive ambient is ignored. steady_rebuild_count counts full grids.
    void set_ambient(real p_pa, real t_k);
    [[nodiscard]] const SimulatedSteadyTables& steady_tables() const { return steady_; }
    [[nodiscard]] std::uint64_t steady_rebuild_count() const { return steady_rebuilds_; }

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

    // PHYS-033 P4 I7c C16b: queue an already-built SimulatedEngineMaps
    // (Powertrain::request_replace_engine_maps calls build_maps itself) for a
    // swap at the next apply_pending_maps_swap() call. Caller thread, between
    // World::step() calls only (never a job): besides storing the pointer it
    // PREFILLS pending_steady_ - the per-engine ambient-dependent derived
    // tables - from the NEW maps at the engine's CURRENT ambient, exactly the
    // way a fresh engine built on those maps would hold them after
    // set_ambient(current) (copy of the maps' own ISA tables, then a
    // fill_steady_tables refill iff the ambient differs). That refill is the
    // ~68 ms O(10^6)-evaluation cost; doing it here keeps it off the tick
    // (review of C16b: it used to run inside the vehicle_islands_substep job
    // on the swap tick). A later call before the swap applies replaces an
    // earlier one (last request wins).
    void request_replace_maps(std::shared_ptr<const SimulatedEngineMaps> new_maps);
    [[nodiscard]] bool has_pending_maps_swap() const { return pending_maps_ != nullptr; }
    // Applies a queued swap (no-op, returns nullptr, if none pending): swaps
    // the maps pointer and std::swap's steady_ with the prefilled
    // pending_steady_ (buffer exchange only - no allocation, no free, no
    // refill). MUST be called outside any job (World::step calls it, via
    // vehicle::System::apply_pending_engine_maps_swaps, before running the
    // tick pipeline). Returns the OLD maps: the caller's shared_ptr drop is
    // where a possible deallocation happens, so it must be outside any job
    // too (D9: "the old maps are released on the sim thread outside jobs").
    // If the ambient changes between request and apply, the next pre_solve's
    // set_ambient refills as for any ambient change (pre-existing C12
    // behaviour - callers quantise ambient).
    std::shared_ptr<const SimulatedEngineMaps> apply_pending_maps_swap();

    void replace_turbo_configuration(const TurboPairDesc& desc);

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
        // I7d D5: tau_comb at the SCHEDULED spark with every injected charge fired (no anti-lag retard,
        // spark setting or cylinder cut; the anti-lag lambda applies); equal to tau_comb without anti-lag.
        real tau_comb_unretarded = 0.0;
        // anti-lag cylinder cut: share of the injected fuel (m_fuel_rev) whose spark is cut; tau_comb covers
        // the fired share (1 - cut_fraction) only. 0 without anti-lag.
        real cut_fraction = 0.0;
    };
    // als (I7d D5, nullptr = none): the active anti-lag block - the spark is retarded by retard_deg from
    // the scheduled (knock-limited) advance or set to spark_advance_deg before eta_spark is looked up
    // (below the table's most retarded node eta_spark is extrapolated linearly, floored at 0), lambda is
    // overridden when the block sets one, and cylinder_cut_fraction of the charges are not fired.
    [[nodiscard]] TorqueEval evaluate_torque(real omega, real p_im, bool fired, real s_starve, bool starter_active,
                                             const AntiLagDesc* als = nullptr) const;

    // I7d D5 anti-lag white-box accessors (all 0/false/true defaults without an anti_lag block).
    [[nodiscard]] bool anti_lag_active() const { return als_active_; }
    [[nodiscard]] bool anti_lag_armed() const { return als_armed_; }
    [[nodiscard]] real anti_lag_timer_s() const { return als_timer_s_; }
    [[nodiscard]] real anti_lag_extra_exhaust_w() const { return als_extra_w_; }
    [[nodiscard]] const TurboPair* turbo() const { return turbo_ ? &*turbo_ : nullptr; }

    // Exhaust energy balance of the latest pre_solve (turbo engines only; zeros otherwise), all W:
    // fuel_w = indicated_w + wall_w + sensible_w + chemical_w - clamp_w (manifold_w, the cut charge burnt
    // in the exhaust manifold under anti-lag, is part of sensible_w). port_k is the gas temperature
    // at the exhaust ports handed to the next substep's turbo step. Never hashed (port_k is, as
    // turbo_port_k_).
    struct ExhaustSplit {
        real fuel_w = 0.0;      // booked fuel mass flow x LHV
        real indicated_w = 0.0; // omega_old * tau_comb
        real wall_w = 0.0;      // in-cylinder wall heat
        real sensible_w = 0.0;  // exhaust sensible enthalpy above the intake temperature
        real chemical_w = 0.0;  // unreleased fuel energy (lambda < 1)
        real clamp_w = 0.0;     // sensible enthalpy added by the >= 0 guard (0 in every shipped regime)
        real manifold_w = 0.0;  // anti-lag: cut-cylinder fuel energy released in the exhaust manifold
        real port_k = 0.0;
    };
    [[nodiscard]] const ExhaustSplit& exhaust_split() const { return exhaust_split_; }

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
    void set_live_ambient(real p_pa,real t_k);
    [[nodiscard]] bool live_steady() const { return p_amb_!=steady_.p_pa || t_amb_!=steady_.t_k; }
    [[nodiscard]] real live_steady_torque(real rpm,real plate,bool fired) const;
    [[nodiscard]] real live_inverse_plate(real rpm,real request) const;

    std::optional<TurboPair> turbo_;
    // Turbo feeds (hashed), all written by the previous substep: turbo_air_kg_s_ = the THROTTLE-path mass
    // flow (ManifoldRhs::m_thr, positive part, relaxed by kTurboFlowLagS) - the flow the compressor delivers;
    // turbo_exhaust_kg_s_ = the cylinder exhaust flow; turbo_port_k_ = the exhaust port gas temperature from
    // the exhaust energy balance (0 = no gas yet: the pair treats it as ambient). See engine_simulated.cpp.
    real turbo_air_kg_s_ = 0.0, turbo_exhaust_kg_s_ = 0.0, turbo_port_k_ = 0.0;
    ExhaustSplit exhaust_split_;
    // I7d D5 anti-lag (hashed only when the turbo configuration has an anti_lag block):
    // als_active_ this substep's activity, als_armed_ false once max_active_s is used up in
    // this lift or after a starving substep (re-armed when the command rises to
    // arm_below_command), als_timer_s_ the activity accumulated in this lift. Not hashed:
    // als_extra_w_ (this substep's removed indicated work rate - by the exhaust energy balance
    // the sensible exhaust enthalpy it adds) and the activation counter (sound only).
    bool als_active_ = false, als_armed_ = true;
    real als_timer_s_ = 0.0, als_extra_w_ = 0.0;
    std::uint32_t als_activations_ = 0;
    void clear_anti_lag_state();


    std::shared_ptr<const SimulatedEngineMaps> maps_;
    // PHYS-033 P4 I7c C16b: queued by request_replace_maps, consumed by
    // apply_pending_maps_swap - see both methods' own comments above.
    std::shared_ptr<const SimulatedEngineMaps> pending_maps_;
    // C16b review: steady tables for pending_maps_ at the ambient current at
    // request time, filled by request_replace_maps on the caller thread and
    // exchanged with steady_ by apply_pending_maps_swap.
    SimulatedSteadyTables pending_steady_;
    // desc fields read per substep (engine management)
    real desc_idle_rpm_ = 0.0, stall_rpm_ = 0.0, fire_rpm_ = 0.0, start_complete_rpm_ = 0.0;
    real dfco_enable_rpm_ = 0.0, dfco_reenable_rpm_ = 0.0;
    IdleControlDesc idle_;
    StarterDesc starter_;
    real breakaway_torque_nm_ = 0.0;
    real actuator_tau_s_ = 0.0;

    // ambient
    SimulatedSteadyTables steady_; // full setup cache; non-reference anchor is hashed, values derive from maps/anchor
    std::uint64_t steady_rebuilds_ = 0;
    real live_idle_ff_=0, live_idle_p_im_=0, live_idle_cap_x_=0;
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
