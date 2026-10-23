// engine_cycle/cycle_stepper.h - the 0D crank-angle single-cylinder cycle
// model's state, right-hand side and fixed-step RK4 integrator (PHYS-033 P4
// I7c C15a, out/i7c_plan.md section 3 and section 5 C15a). INTERNAL
// (ps::engine_cycle, not a public header).
//
// Control volumes: cylinder, intake port, exhaust port, each (m, U, m_b)
// (PROVISIONAL P1: U instead of T, see gas.h); the intake runner is an
// incompressible inertial slug (state: its mass flow); the intake plenum (p_im,
// T_im) and the exhaust reservoir (p_amb - PROVISIONAL P6) are boundary
// conditions. ps_audio A5 PIPE MODE (a desc with intake/exhaust pipe blocks):
// no runner slug and no reservoir orifice - both ports exchange gas only with
// the externally set PortSource rates (set_port_sources: the PipeNetwork port
// fluxes of the same step divided by h, held constant over the RK4 step, so
// the port gains exactly what the pipe lost - the A5 CycleEngine). The lumped path
// is untouched (bit-identical). Crank angle theta [deg] is not a state: 0 = gas-exchange TDC,
// 360 = firing TDC, period 720.
//
// The state vector also carries the per-cycle accumulators (work, valve mass
// and enthalpy fluxes, heat) so they are integrated by the same RK4 stages as
// the physics (plan section 3 "Per-point outputs"); mass and energy are
// linear invariants of RK4, so the bookkeeping closes at rounding (G5/G6).
//
// Determinism (PLAN.md D7): ps::math for every transcendental; std::sqrt and
// std::floor are IEEE-exact. No allocation after construction: the immutable
// model is shared through a std::shared_ptr<const CycleModel>, so copying a
// stepper (run_operating_point does) only bumps a reference count.

#pragma once

#include "ps/engine_cycle/burn_law.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/geometry.h"
#include "ps/engine_cycle/valve_flow.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <memory>

namespace ps::engine_cycle {

struct OperatingPoint {
    real rpm = 0.0;
    real p_im_pa = 0.0;
    real t_im_k = 0.0;
    real p_amb_pa = 101325.0;
    // Pipe engines only (EngineCore/CycleEngine): the throttle's upstream
    // supply pressure at t_im_k, i.e. a compressor outlet after the
    // intercooler. 0 = the ambient path through snorkel/airbox. Needed for a
    // plenum target above ambient, which the ambient path cannot reach.
    real p_supply_pa = 0.0;
    real lambda = 1.0;
    // spark_deg_atdc's meaning (burn law, design C): WiebeStart - the start of combustion (Wiebe theta_0) relative to
    // firing TDC, with the model's constant burn_duration (every caller before the law; the live API and the knock
    // replay). On a model whose desc runs the burn law (CycleModelDesc::burn_law == Standard) two more modes make the
    // stepper set its Wiebe from the law at every IVC capture: Spark - the spark [deg ATDC] (Wiebe start = spark +
    // the law's delay); MbtOffset - the spark relative to the cycle's own MBT spark [deg, + = later] (Wiebe start =
    // the MBT start + offset). On a Fixed-law model both read as the pre-law values (Spark: + ignition_delay_deg;
    // MbtOffset: + the constant MBT start).
    enum class SparkMode : std::uint8_t { WiebeStart = 0, Spark = 1, MbtOffset = 2 };
    SparkMode spark_mode = SparkMode::WiebeStart;
    real spark_deg_atdc = 0.0;
    real intake_advance_deg = 0.0;
    real exhaust_retard_deg = 0.0;
};

// State-vector layout.
enum StateIndex : int {
    kCylM = 0, kCylU, kCylMb,
    kIpM, kIpU, kIpMb,
    kRunnerMdot,
    kEpM, kEpU, kEpMb,
    kPhysicalCount,
    // Accumulators (reset at each cycle start by the caller).
    kAccWork = kPhysicalCount,  // integral p dV, whole cycle
    kAccWorkGross,              // integral p dV over [theta_bdc, theta_bdc + 360): compression + expansion
    kAccIvFresh,                // net fresh (unburned-group) mass through the intake valve, into the cylinder
    kAccIvMass,                 // net total mass through the intake valve
    kAccEvOutMass,              // outflow-only mass through the exhaust valve
    kAccEvOutMassT,             // outflow-only integral mdot*T_upstream (mass-averaged T_exh)
    kAccEvEnthalpy,             // net enthalpy through the exhaust valve, out of the cylinder
    kAccIvEnthalpy,             // net enthalpy through the intake valve, into the cylinder
    kAccQComb,                  // combustion heat release
    kAccQWall,                  // wall heat loss (positive = out of the gas)
    kAccFuel,                   // fuel mass added to the cylinder
    kAccRunnerMass,             // runner mass into the intake port
    kAccRunnerEnthalpy,
    kAccResMass,                // mass from the exhaust port into the reservoir
    kAccResEnthalpy,
    kStateCount
};

struct CycleState {
    real y[kStateCount] = {};
};

// ps_audio A5 pipe mode: external source rates INTO a port volume, constant
// over one step (mass [kg/s], total enthalpy [W], burned mass [kg/s]).
struct PortSource {
    real mdot = 0.0;
    real edot = 0.0;
    real mbdot = 0.0;
};

// Test switches (PROVISIONAL P7): physics subsets for G2/G3/G5/G11.
struct CycleSwitches {
    bool valves = true;
    bool combustion = true;
    bool wall_heat = true;
};

// Instantaneous derived quantities at one (theta, state) - also the RHS's
// intermediate values, exposed for tests and diagnostics.
struct CycleInputs {
    real theta_deg = 0.0;
    real v_cyl = 0.0, dv_dt = 0.0;
    CvState cyl, ip, ep;
    real lift_in_m = 0.0, lift_ex_m = 0.0;
    real mdot_iv = 0.0;    // + into the cylinder
    real mdot_ev = 0.0;    // + out of the cylinder
    real mdot_res = 0.0;   // + exhaust port -> reservoir
    real q_wall_w = 0.0;
    real q_comb_w = 0.0;
    real woschni_h = 0.0;
    real wall_area = 0.0;
};

// Immutable, precomputed model (built once from the descriptor).
struct CycleModel {
    CrankGeometry geom;
    GasModel gas;
    PsiPair psi;
    vehicle::Curve1D intake_lift, exhaust_lift, cd_vs_ld;
    real intake_phase_ref = 0.0, exhaust_phase_ref = 0.0;
    real intake_curtain = 0.0;   // n_valves * pi * D (times L gives the curtain area)
    real exhaust_curtain = 0.0;
    real intake_d = 0.0, exhaust_d = 0.0;
    real runner_area = 0.0, runner_length = 0.0;
    real v_ip = 0.0, v_ep = 0.0;
    real reservoir_cd_area = 0.0;
    real wiebe_a = 5.0, wiebe_m = 2.0, burn_duration = 60.0;
    // Burn law (ps/engine_cycle/burn_law.h): on for a Standard desc; ca50_mbt the MBT rule's CA50 [deg ATDC];
    // ignition_delay the Fixed desc's constant delay; v_tdc the clearance volume at firing TDC.
    bool burn_law = false;
    BurnLawParams burn_params;
    real ca50_mbt = 8.0;
    real ignition_delay = 0.0;
    real woschni_c1_gx = 6.18, woschni_c1_cc = 2.28, woschni_c2 = 3.24e-3;
    real t_wall = 450.0, head_area_factor = 1.1;
    real combustion_eff_lean = 0.98;
    real t_backflow = 800.0;
    real lhv = 0.0, afr_st = 0.0;
    // ps_audio A5: the desc carries intake/exhaust pipe blocks - ports couple
    // to a PipeNetwork through PortSource rates (no runner, no reservoir).
    bool pipe_ports = false;
};

class CycleStepper {
public:
    CycleStepper(const drivetrain::CycleModelDesc& desc, real lhv_j_kg, real afr_st);

    // Deterministic initial state at theta = 0 (gas-exchange TDC): cylinder
    // and exhaust port burned gas at p_amb and the backflow temperature,
    // intake port fresh at (p_im, T_im), runner at rest.
    void reset(const OperatingPoint& op);

    // One classical RK4 step of length h [s]; theta advances by 6*rpm*h deg.
    void step(real h);

    // One RK4 step aligned to a cycle of n steps (PROVISIONAL P4): theta runs
    // from 720*k/n to 720*(k+1)/n exactly (k taken mod n); h = cycle time / n.
    void step_aligned(std::uint64_t k, std::uint64_t n);

    // ps_audio A5 (multi-cylinder CycleEngine): the same aligned step for a
    // cylinder firing phase_deg after cylinder 1 - its own crank angle is the
    // global one minus phase_deg, wrapped to [0, 720) at the step start (the
    // step end may exceed 720; every angle-dependent term is 720-periodic or
    // exhausted there). phase_deg == 0 reproduces step_aligned exactly.
    void step_phased(std::uint64_t k, std::uint64_t n, real phase_deg);
    // reset(op), then the crank angle at global angle 0 (= wrap720(-phase_deg))
    // and the cylinder's burned-gas mass for the volume at that angle.
    void reset_phased(const OperatingPoint& op, real phase_deg);
    // Pipe mode only (ignored otherwise): the port source rates used by the
    // following steps, until changed.
    void set_port_sources(const PortSource& intake, const PortSource& exhaust) {
        ip_src_ = intake;
        ep_src_ = exhaust;
    }
    // Pipe mode only: ALSO tie the ports to fixed reservoirs through orifices
    // of the given Cd*A (0 = off) - intake from (op.p_im, op.t_im, fresh),
    // exhaust to (op.p_amb, T_backflow, burned) - evaluated inside the RHS
    // like the lumped exhaust reservoir. The CycleEngine's reservoir_ports
    // reference (no pipe waves; the V8 sabotage).
    void set_port_reservoirs(real intake_cd_area, real exhaust_cd_area) {
        res_ip_cda_ = intake_cd_area;
        res_ep_cda_ = exhaust_cd_area;
    }

    // ---- ps_audio A7 live stepping (never called by CycleEngine or the map
    // generator; the generator path above is untouched) ------------------
    // Crank speed [rpm, >= 0] the RHS uses from now on (dV/dt, the Wiebe rate,
    // Woschni's mean piston speed). 0 freezes the crank: valves stay where
    // they are, gas still flows through an open valve, nothing divides by it.
    void set_speed(real rpm) {
        op_.rpm = rpm;
        deg_per_s_ = 6.0 * rpm;
    }
    // One RK4 step of h [s] from this cylinder's own angle theta0_deg (in
    // [0, 720)) by dtheta_deg >= 0 (the caller integrates the crank from its
    // own omega; dtheta = 6*rpm*h for a constant speed).
    void step_live(real theta0_deg, real dtheta_deg, real h);
    // The next combustion event (call between IVC and the burn start): the
    // fuel is re-derived from the trapped air at IVC for `lambda`; the Wiebe
    // starts at 360 + start_deg_atdc and lasts burn_duration_deg. fire = false
    // is a cut (no fuel, the cylinder only pumps).
    void set_combustion_event(real lambda, real start_deg_atdc, real burn_duration_deg, bool fire);

    [[nodiscard]] const CycleState& state() const { return state_; }
    [[nodiscard]] real crank_deg() const { return theta_; }
    [[nodiscard]] const CycleModel& model() const { return *model_; }
    [[nodiscard]] const OperatingPoint& operating_point() const { return op_; }

    // Test / diagnostic seams.
    void set_switches(const CycleSwitches& s) { switches_ = s; }
    void set_state(const CycleState& s, real theta_deg) { state_ = s; theta_ = theta_deg; }
    void reset_accumulators();
    void reset_peak_pressure() { p_max_ = 0.0; }
    // Fuel and fresh charge for the combustion event (normally captured at IVC).
    void set_charge(real m_air_kg, real m_fuel_kg) { m_air_conv_ = m_air_kg; m_fuel_ = m_fuel_kg; }
    [[nodiscard]] CycleInputs evaluate(real theta_deg, const CycleState& s) const;
    void rhs(real theta_deg, const CycleState& s, CycleState& dydt) const;

    // Values captured at the last IVC crossing (step end at or after IVC).
    [[nodiscard]] real m_air_trapped() const { return m_air_conv_; }
    [[nodiscard]] real m_fuel() const { return m_fuel_; }
    [[nodiscard]] real residual_fraction_ivc() const { return x_b_ivc_; }
    [[nodiscard]] real p_max() const { return p_max_; }
    [[nodiscard]] const Wiebe& wiebe() const { return wiebe_; }
    // Burn law: the angles of the current cycle (set at its IVC capture; the law's reference angles before the first
    // capture), the spark [deg ATDC] and the cycle's MBT spark [deg BTDC] they imply. With the law off (WiebeStart, a
    // Fixed model): the constant duration/delay and the Wiebe start minus the delay.
    [[nodiscard]] const BurnAngles& burn_angles() const { return burn_; }
    [[nodiscard]] real spark_atdc() const { return wiebe_.theta0_deg - 360.0 - burn_.ignition_delay_deg; }
    [[nodiscard]] real mbt_spark_btdc() const;
    // Burn law: the law's angles from the last IVC capture at (rpm, lambda) - the generator's hook and the live
    // engine's per-cycle call (the model's own law parameters; valid only after a capture).
    [[nodiscard]] BurnAngles burn_law_at_ivc(real rpm, real lambda) const;
    [[nodiscard]] real theta_ivc() const { return theta_ivc_; }
    [[nodiscard]] real theta_evo() const { return theta_evo_; }
    // Engine garage P7: the cylinder state (m, U, m_b) and this cylinder's own
    // crank angle at the last IVC capture (the same step end as the values
    // above; theta 0 / zeros before the first capture). Read only by the knock
    // limit's closed-cycle replay (ps/engine_cycle/knock.h); never fed back.
    [[nodiscard]] real ivc_crank_deg() const { return ivc_theta_; }
    [[nodiscard]] real ivc_cyl_m() const { return ivc_m_; }
    [[nodiscard]] real ivc_cyl_u() const { return ivc_u_; }
    [[nodiscard]] real ivc_cyl_mb() const { return ivc_mb_; }
    [[nodiscard]] real lift_intake(real theta_deg) const;
    [[nodiscard]] real lift_exhaust(real theta_deg) const;
    // Woschni IVC reference (0 before the first IVC).
    void set_woschni_reference(real p, real v, real t) { p_ref_ = p; v_ref_ = v; t_ref_ = t; }

private:
    void rk4(real theta0, real theta_mid, real theta1, real h);
    void after_step(real theta_before, real dtheta);
    // Burn law: the Wiebe (start and duration) of the coming combustion from burn_ and the op's spark mode.
    void apply_spark(const BurnAngles& b);
    void add_port_reservoir_rates(const CycleInputs& in, PortSource& ip, PortSource& ep) const;

    std::shared_ptr<const CycleModel> model_;
    OperatingPoint op_{};
    CycleSwitches switches_{};
    CycleState state_{};
    real theta_ = 0.0;
    real deg_per_s_ = 0.0;
    real intake_shift_ = 0.0, exhaust_shift_ = 0.0;
    real theta_ivc_ = 0.0, theta_evo_ = 0.0, theta_spark_ = 0.0;
    real eta_c_ = 0.98;
    Wiebe wiebe_{};
    BurnAngles burn_{};
    bool law_active_ = false; // the op's spark mode is not WiebeStart and the model runs the law
    real m_air_conv_ = 0.0, m_fuel_ = 0.0, x_b_ivc_ = 0.0;
    real p_ref_ = 0.0, v_ref_ = 0.0, t_ref_ = 0.0;
    real p_max_ = 0.0;
    real ivc_theta_ = 0.0, ivc_m_ = 0.0, ivc_u_ = 0.0, ivc_mb_ = 0.0; // P7 IVC snapshot (diagnostic, write-only here)
    PortSource ip_src_{}, ep_src_{};
    real res_ip_cda_ = 0.0, res_ep_cda_ = 0.0;
};

[[nodiscard]] real wrap720(real deg);

} // namespace ps::engine_cycle
