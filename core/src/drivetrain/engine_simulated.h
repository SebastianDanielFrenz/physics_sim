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
// N2O plan N2 (out/n2o_plan.md section 1.2, docs/data_formats/engine.md "nitrous"): while
// Sensors::nitrous is active with N2O flowing (and property tables), the wet charge model replaces
// m_air_rev and the combustion line above. n = mdot_n*2pi/omega N2O and kf kit fuel per rev; the
// air a and the charge temperature T_mix solve the port's mole and adiabatic-mixing energy balances
// (one quadratic, closed form); a_eq = a + k_O2*n; ECU fuel f_ecu = a/(lambda_t*AFR); lambda_c =
// a_eq/(AFR*(f_ecu + kf)); W_ind_rev = eta_i(rpm, a_eq)*eta_spark*(f_lambda(lambda_c)*LHV*f*fire*s
// + dH_dec*n*fire*s); spark = min(schedule - kit retard, theta_kl). m_cyl uses a (plenum mass
// conserved), d carries dn/domega = -n/omega and the implicit da/domega, dfdp the implicit da/dp.
// Without N2O every expression is the pre-N2 one (a null NitrousCharge), bit for bit.
//
// C12: Powertrain constructs this for a SimulatedEngineDesc component,
// from a desc copy whose fuels list holds only the fuel actually burnt (the
// tank's, else fuels[0]). Rotary geometry is refused here (C13).

#pragma once

#include "ps/drivetrain/turbo_envelope.h"
#include "turbo_pair.h"

#include "engine.h"
#include "nitrous_flow.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/limiter_range.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/table2d.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <algorithm>
#include <cstdint>
#include <limits>
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
    // G1 (limiter_range.h): the volumetric-efficiency roll-off past the maps' data, one for the whole engine (every cam
    // profile / lift node carries the same one). Disarmed (the default and every engine whose limiter lies inside its
    // maps) reads exactly as before G1; armed, it is folded into content_hash.
    RpmRolloff rolloff;

    // --- derived at construction ------------------------------------------
    std::vector<real> psi;         // kPsiIntervals + 1 knots, pr = i/kPsiIntervals
    real ref_p_pa = 0.0, ref_t_k = 0.0; // ambient the steady tables below were built at (ISA)
    // All steady values: net crank torque at omega > 0 (Coulomb row
    // included, i.e. tau - c_k), at the steady p_im (bisection root of
    // f(p) = 0), ISA ambient, no limiter, no starter.
    vehicle::Curve1D steady_wot_nm;       // x = rpm; plate 1, fired
    vehicle::Curve1D steady_motoring_nm;  // x = rpm; plate 0, fuel off
    Table2D steady_torque_nm;             // x = rpm (500 .. limiter + 250, step 250, plus idle_rpm as its own node), y = plate 0..1 step 0.025; fired, bypass closed
    Table2D inverse_plate;                // electronic only: x = rpm, y = request in [0,1] -> plate target
    real idle_ff = 0.0;                   // idle feed-forward: plate (electronic) / bypass (mechanical)
    real idle_p_im = 0.0;                 // steady manifold pressure at idle_rpm with idle_ff
    real idle_cap_x = 0.0;                // idle authority cap (IdleControlDesc::max_torque_nm), 0 = none
    // Engine garage P9: the high cam profile's complete model (same desc, that profile's generated maps; its own
    // steady tables) and the ECU's switching rule. Null without cam profiles. Folded into content_hash only when
    // present, so a hot swap of these maps swaps every profile set at once.
    std::shared_ptr<const SimulatedEngineMaps> cam_high;
    CamSwitchingDesc cam_switching;
    // Engine garage P10: a continuous intake lift family (Valvetronic-style). This object is the u = 1 node's complete
    // model; lift_nodes holds the nodes below it (ascending u, same desc on each node's own generated maps, each with its
    // own steady tables); lift_u every node's u (the last 1.0); lift_control the ECU strategy. Empty without a family.
    // Folded into content_hash only when present (every node's hash, the u values and the strategy constants).
    std::vector<std::shared_ptr<const SimulatedEngineMaps>> lift_nodes;
    std::vector<real> lift_u;
    LiftControlDesc lift_control;
    [[nodiscard]] bool has_lift_family() const { return !lift_u.empty(); }
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

// Pedal plan P1 (out/transmission_pedal_plan.md 2.2, Boost B0 addition 1): a turbo engine's full-load envelope F(n) -
// the dyno-SETTLED reachable WOT state (run at throttle 1 with the crank held, engine_dyno.cpp's stepped point, ISA,
// the higher of a rotor-at-rest and a pre-spun run per rpm; the end-of-window value where a point does not settle), never the configured target. Built
// once per (maps, turbo hardware, ECU) at engine construction by full_load_envelope() (memoised process-wide) over
// idle .. just below the limiter in the dyno's 250 rpm steps; Curve1D clamps outside. x = rpm everywhere.
struct FullLoadEnvelope {
    vehicle::Curve1D torque_nm;      // F_ref: settled net crank torque
    vehicle::Curve1D pressure_ratio; // PR_full: settled charge pressure (after the intercooler) / ambient
    vehicle::Curve1D charge_k;       // settled charge temperature
    vehicle::Curve1D exhaust_ratio;  // settled turbine-inlet pressure / ambient
    // F_ref - M_ref: the dyno torque minus the steady model's torque at the same charge/exhaust state (ISA, ECU k = 1).
    // The live refresh after an ambient or friction-estimate change is F = offset + M_live (plan 2.2).
    vehicle::Curve1D model_offset_nm;
    std::uint64_t key = 0; // the memo key it was built for
};

class SimulatedEngine;
// Q5 test hook: drops the process-wide envelope memo so a second build is an independent run (bit-identity tests).
void clear_full_load_envelope_memo();
// Pedal plan P1: the memoised envelope of a turbo engine with an electronic throttle (nullptr otherwise, and for null
// maps). Allocates and runs ~30 held-crank dyno points (cold and pre-spun, 0.6 s for the hyper) on a miss: construction / caller thread
// only, never a job. Implemented in engine_dyno.cpp (it reuses the dyno's stepped point).
[[nodiscard]] std::shared_ptr<const FullLoadEnvelope> full_load_envelope(const SimulatedEngineDesc& desc,
                                                                        const std::shared_ptr<const SimulatedEngineMaps>& maps);

// N2O plan N2 (out/n2o_plan.md section 1.2): the wet-kit flows the engine's charge model
// reads, from Sensors::nitrous. Constant MASS flows: per crank revolution they are
// n = mdot_n_kg_s * 2*pi/omega and kf = mdot_kf_kg_s * 2*pi/omega, so their share of the
// charge falls as 1/omega (the constant-power character of nitrous; dn/domega = -n/omega
// enters d). A null NitrousCharge pointer is the pre-N2 code path, bit for bit.
struct NitrousCharge {
    real mdot_n_kg_s = 0.0;  // N2O mass flow into the intake
    real mdot_kf_kg_s = 0.0; // kit fuel mass flow (the engine's own fuel)
    real h_in_j_kg = 0.0;    // enthalpy of the N2O at the jet (saturated liquid at bottle pressure)
    real retard_deg = 0.0;   // kit/controller retard: spark = min(schedule - retard, theta_kl) (plan D7)
    const OxidizerTables* oxidizer = nullptr; // decomposition heat, O2 fraction, molar mass, gas enthalpy
};

// N2O plan N2 / D7: the cylinder charge the charge model computed - published for the
// knock limit (C17b's autoignition integral) and for sound/HUD. Without N2O only t_in_k,
// t_mix_k (= t_in_k) and air_kg_rev are set. Never hashed.
struct ChargeState {
    real t_in_k = 0.0;              // intake temperature (ambient NA, charge-air temperature turbo)
    real t_mix_k = 0.0;             // charge temperature after the N2O evaporated into the air
    real air_kg_rev = 0.0;          // air trapped per crank revolution (a)
    real n2o_kg_rev = 0.0;          // N2O per crank revolution (n)
    real ecu_fuel_kg_rev = 0.0;     // ECU fuel per rev for the trapped air, a / (lambda_target * AFR_st), before fire/starvation
    real kit_fuel_kg_rev = 0.0;     // kit fuel per rev, before starvation
    real o2_equiv_air_kg_rev = 0.0; // a_eq = a + k_O2 * n
    real lambda_combined = 0.0;     // a_eq / (AFR_st * (ECU + kit fuel)); 0 without N2O (or with no fuel at all)
    bool overflow = false;          // the N2O alone would fill the cylinder: a clamped to 0 (section 1.2 step 9)
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
    // Engine garage P10: the lift actuator lands exactly on its target once within this distance (u units) [AS] - the
    // first-order lag alone never arrives, and u = 1 must be exactly the u = 1 node (bit-identical WOT).
    static constexpr real kLiftSnap = 1e-6;
    // Engine garage P10: the lowest manifold pressure the ECU's min_u pressure search goes down to, as a fraction of
    // ambient [AS] (below every closed-plate manifold pressure the generated grids reach, 20 kPa for the N52).
    static constexpr real kLiftMinPressureFraction = 0.05;

    explicit SimulatedEngine(const SimulatedEngineDesc& desc);
    // Reuse an already-built maps object (must come from build_maps(desc)
    // on an equal desc - not checked beyond non-null).
    SimulatedEngine(const SimulatedEngineDesc& desc, std::shared_ptr<const SimulatedEngineMaps> maps);
    // Pedal plan P1: with an explicit full-load envelope (the two constructors above take full_load_envelope(desc,
    // maps)). nullptr on a turbo engine = no envelope: a part-throttle command then runs the pre-P1 law (only the
    // envelope's own WOT dyno points are built this way - they never leave throttle 1).
    SimulatedEngine(const SimulatedEngineDesc& desc, std::shared_ptr<const SimulatedEngineMaps> maps,
                    std::shared_ptr<const FullLoadEnvelope> envelope);

    [[nodiscard]] static std::shared_ptr<const SimulatedEngineMaps> build_maps(const SimulatedEngineDesc& desc);
    // Engine garage P3 (the dyno's naturally aspirated path): the same model data as build_maps WITHOUT the ISA derived
    // tables (steady_wot_nm, steady_motoring_nm, steady_torque_nm, inverse_plate stay empty, idle_ff/idle_p_im/
    // idle_cap_x stay 0; the grid costs about 70 ms). Valid ONLY for steady_point() below and the const evaluation
    // functions; never hand it to a SimulatedEngine (its idle control and ECU demand read the tables).
    [[nodiscard]] static std::shared_ptr<const SimulatedEngineMaps> build_model_maps(const SimulatedEngineDesc& desc);

    // Engine garage P3: the steady operating point of a NATURALLY ASPIRATED engine at `rpm` (> 0) and throttle
    // `command` (electronic: the torque request, turned into a plate by the runtime's own inverse definition;
    // mechanical: the plate position), fired, no starter, limiter ignored (like the steady tables), bypass closed,
    // at `ambient`. The manifold pressure is the root of f(p) = 0 (steady_p), the torque the evaluate_torque_impl
    // value net of the Coulomb row (tau - c_k) - exactly what steady_net_torque, hence every node of the steady
    // tables, returns: at a table node and ISA the value is bit-identical to steady_wot_nm / steady_torque_nm.
    // No turbo (a turbo engine's torque depends on the charge the pair builds: step it instead, see
    // ps/drivetrain/engine_dyno.h).
    struct SteadyPoint {
        real net_torque_nm = 0.0; // net crank torque, Coulomb row included
        real p_im_pa = 0.0;       // steady manifold pressure
        real plate = 0.0;         // throttle plate position used
        real fuel_g_s = 0.0;      // booked fuel flow at this rpm
        real lambda = 0.0, spark_deg = 0.0;
        real knock_retard_deg = 0.0; // engine garage P7: schedule - theta_kl where the knock limit holds the spark, else 0
    };
    [[nodiscard]] static SteadyPoint steady_point(const SimulatedEngineMaps& m, real rpm, real command, const AmbientSample& ambient);

    [[nodiscard]] PreSolveResult pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) override;
    [[nodiscard]] PostSolveResult post_solve(real omega_new, real h) override;

    [[nodiscard]] real idle_integrator() const override { return idle_integrator_; }
    [[nodiscard]] real crank_angle() const override { return crank_angle_; }
    [[nodiscard]] real idle_rpm() const override { return desc_idle_rpm_; }
    [[nodiscard]] real intake_pressure_pa() const override { return p_im_; }
    // Pedal plan P1 (2.2/2.3): Z, F and the fired floor T_fc at rpm in the current ECU state (the idle controller output
    // and DFCO latch of the last pre_solve, the live ambient and friction estimate). manual_law false on a mechanical
    // linkage.
    [[nodiscard]] PedalEnvelope request_envelope(real rpm) const override;
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
    // Engine garage P9: the high cam profile's steady tables (empty without cam profiles).
    [[nodiscard]] const SimulatedSteadyTables& steady_tables_high_cam() const { return steady_hi_; }
    // Engine garage P9 cam switching (false/0 without cam profiles): the ECU's latched profile request and the
    // blend weight w in [0, 1] (the fraction of the charge drawn through the high-lift lobes; every model lookup
    // is (1 - w) * low + w * high, exactly the single-profile path at w = 0 and w = 1).
    [[nodiscard]] bool cam_high_requested() const { return cam_high_; }
    [[nodiscard]] real cam_blend() const { return cam_w_; }
    // White-box/test hook: force the blend weight (and the latched request: w >= 0.5 -> high). -1 = ECU control.
    // Used by the dyno to hold a profile; never set by World. Not hashed (a held engine is a test harness).
    void set_cam_blend_override(real w) { cam_override_ = w; if (w >= real(0.0)) { cam_w_ = std::clamp(w, real(0.0), real(1.0)); cam_high_ = cam_w_ >= real(0.5); } }
    [[nodiscard]] std::uint64_t steady_rebuild_count() const { return steady_rebuilds_; }
    // Engine garage P10 (continuous intake lift; 1.0 / no-ops without a family): the eccentric-shaft position u the plant
    // runs at (every model lookup is the linear blend of the two lift nodes bracketing u, exactly node k's model at u =
    // u_k, the u = 1 node at u = 1), the ECU's target of the latest pre_solve, and whether that pre_solve set the plate
    // from the vacuum target (true) or a node's own inverse / plate 1 (false).
    [[nodiscard]] real lift_u() const { return u_; }
    [[nodiscard]] real lift_u_target() const { return u_target_; }
    [[nodiscard]] bool lift_vacuum_mode() const { return lift_vacuum_mode_; }
    // Lift node k's steady tables (k < lift node count - 1; the u = 1 node's are steady_tables()).
    [[nodiscard]] const SimulatedSteadyTables& steady_tables_lift_node(std::size_t k) const { return steady_nodes_.at(k); }
    // White-box/test hook: hold u at `u` (clamped to the family's range); -1 = ECU control. Not hashed (a held engine is a
    // test harness, like set_cam_blend_override). No-op without a family.
    void set_lift_override(real u);
    // The ECU's lift strategy at (rpm, request) evaluated stand-alone (no state change): the u target and the plate
    // target pre_solve would command with u already at that target, from the current ambient/friction estimate. For
    // tests. Throws without a family. p_target_pa > 0: the plate holds the manifold at that pressure (pre_solve
    // recomputes the plate for the lift the shaft has NOW) - the vacuum target in the vacuum range (vacuum_mode), the
    // pressure at which the min_u / plate_open_u node makes the target torque where the lift rests on those nodes;
    // -1 where the plate is simply open (plate 1).
    struct LiftTargets {
        real u = 1.0, plate = 1.0, torque_target_nm = 0.0;
        bool vacuum_mode = false;
        real p_target_pa = -1.0;
    };
    [[nodiscard]] LiftTargets lift_targets(real rpm, real request) const;

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

    // Pedal plan P1: also rebuilds the full-load envelope for the new hardware (full_load_envelope, a dyno run on a
    // memo miss) - caller thread, between steps.
    // Q5: with `prepared` (from prepare_turbo_envelope, built by build_turbo_envelope) the envelope is taken from it when it
    // was built for this engine's current maps; otherwise (or without) it is built here as before.
    void replace_turbo_configuration(const TurboPairDesc& desc, const PreparedTurboEnvelope* prepared = nullptr);
    [[nodiscard]] PreparedTurboEnvelope prepare_turbo_envelope(const TurboPairDesc& desc) const;
    // Pedal plan P1 white-box: the envelope in use (nullptr: NA engine, mechanical throttle, or built without one).
    [[nodiscard]] const FullLoadEnvelope* full_load_envelope_ptr() const { return envelope_.get(); }
    // Pedal plan P1 test hook (a held engine is a harness, never set by World; not hashed): -1 = the engine's own law,
    // 0 = the pre-P1 law for every command (two-sided idle PI, max(), boost target ambient + target*plate - the "old
    // split" the fuel comparison runs against), 1 = efficient split, 2 = spooled split (turbo only).
    void set_pedal_law_override(int mode) { pedal_law_override_ = mode; }
    // The charge-pressure target the boost controller chased in the last pre_solve (Pa abs; -1 = the pre-P1 plate law).
    [[nodiscard]] real boost_target_pa() const { return boost_target_pa_; }
    // The manual law's torque target T* of the last pre_solve (NaN outside the 0.01 <= c < 1 branch).
    [[nodiscard]] real pedal_torque_target_nm() const { return pedal_target_nm_; }
    // Torque the steady model makes at the turbo state (charge supply p_charge/t_charge, turbine inlet p_exhaust), plate
    // 1, fired, ECU friction scale k: steady manifold pressure, net of the Coulomb row. Static so the envelope builder can
    // compute M_ref; allocation-free.
    [[nodiscard]] static real boosted_model_torque(const SimulatedEngineMaps& m, real rpm, real p_charge_pa, real t_charge_k,
                                                   real p_exhaust_pa, real k_ecu, real hyd_ref_rpm);

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
        // Engine garage P7: how far the knock limit holds the spark below the schedule (schedule - kit retard),
        // 0 when it does not limit. Diagnostic (the dyno's Knock finding), never read by the dynamics.
        real knock_retard_deg = 0.0;
        real eta_i = 0.0, f_lambda = 0.0, eta_spark = 0.0;
        // I7d D5: tau_comb at the SCHEDULED spark with every injected charge fired (no anti-lag retard,
        // spark setting or cylinder cut; the anti-lag lambda applies); equal to tau_comb without anti-lag.
        real tau_comb_unretarded = 0.0;
        // N2O plan N3: tau_comb with the kit's spark retard removed (the knock-limited schedule; the anti-lag
        // retard/setting stays), same charge, same cut share. tau_comb_no_n2o_retard - tau_comb is the work the
        // kit retard costs (0 where the knock limit already holds the spark below schedule - retard). Equal to
        // tau_comb without N2O. Never read by the dynamics.
        real tau_comb_no_n2o_retard = 0.0;
        // anti-lag cylinder cut: share of the injected fuel (m_fuel_rev) whose spark is cut; tau_comb covers
        // the fired share (1 - cut_fraction) only. 0 without anti-lag.
        real cut_fraction = 0.0;
        // N2O plan N2: the charge (D7 knock seam) and the N2O per revolution. With the charge model
        // m_air_rev is the air a the cylinder draws, m_fuel_rev the BOOKED fuel (ECU fuel * fire + kit fuel,
        // times s_starve - the kit jet flows through a cut cylinder [AS]), f_lambda is f_lambda(lambda_combined)
        // and n2o_burnt_rev = n * fire * s_starve (decomposition is complete in a fired charge [AS]).
        ChargeState charge;
        real n2o_rev = 0.0;
        real n2o_burnt_rev = 0.0;
        real kit_fuel_rev = 0.0; // booked kit fuel per rev (kf * s_starve), a subset of m_fuel_rev
        real burnt_fuel_rev = 0.0; // fuel burnt in the fired charges ((f_ecu + kf) * fire * s_starve); <= m_fuel_rev
    };
    // als (I7d D5, nullptr = none): the active anti-lag block - the spark is retarded by retard_deg from
    // the scheduled (knock-limited) advance or set to spark_advance_deg before eta_spark is looked up
    // (below the table's most retarded node eta_spark is extrapolated linearly, floored at 0), lambda is
    // overridden when the block sets one, and cylinder_cut_fraction of the charges are not fired.
    // nc (N2O plan N2, nullptr = none): the wet charge model with these flows (see NitrousCharge).
    [[nodiscard]] TorqueEval evaluate_torque(real omega, real p_im, bool fired, real s_starve, bool starter_active,
                                             const AntiLagDesc* als = nullptr, const NitrousCharge* nc = nullptr) const;

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
    // N2O plan N3: with the wet charge model the balance reads fuel_w + n2o_w = indicated_w + wall_w + sensible_w +
    // chemical_w - clamp_w, where fuel_w is still the booked fuel x LHV (== the ledger's fuel_energy_in / h) and n2o_w
    // the decomposition power of the N2O that burnt (== n2o_energy_in / h); chemical_w then also holds the kit fuel
    // that passed a cut cylinder unburnt, and the burning fuel releases min(1, lambda_combined) of its energy.
    struct ExhaustSplit {
        real fuel_w = 0.0;      // booked fuel mass flow x LHV
        real indicated_w = 0.0; // omega_old * tau_comb
        real wall_w = 0.0;      // in-cylinder wall heat
        real sensible_w = 0.0;  // exhaust sensible enthalpy above the intake temperature
        real chemical_w = 0.0;  // unreleased fuel energy (lambda < 1)
        real clamp_w = 0.0;     // sensible enthalpy added by the >= 0 guard (0 in every shipped regime)
        real manifold_w = 0.0;  // anti-lag: cut-cylinder fuel energy released in the exhaust manifold
        real n2o_w = 0.0;       // N2O: decomposition power of the burnt N2O (released in full; 0 without N2O)
        real n2o_retard_w = 0.0; // N2O: indicated power the kit's spark retard removes (already inside sensible_w; diagnostic)
        real port_k = 0.0;
    };
    [[nodiscard]] const ExhaustSplit& exhaust_split() const { return exhaust_split_; }

    // Manifold right-hand side f(p) = dp_im/dt and df/dp at (p_im, omega,
    // plate, bypass) - the function the air step integrates.
    struct ManifoldRhs {
        real f = 0.0, dfdp = 0.0;
        real m_thr = 0.0, m_cyl = 0.0;
    };
    // nc (N2O plan N2): the cylinders draw the charge model's air a instead of VE*p/(R*T)*V (plenum mass
    // conserved; dfdp carries da/dp).
    [[nodiscard]] ManifoldRhs manifold_rhs(real p_im, real omega, real x_thr, real x_iac, const NitrousCharge* nc = nullptr) const;

    // N2O plan N2 white-box: the charge of the latest pre_solve (a default ChargeState before the first),
    // and the substeps whose N2O alone would have filled the cylinder (overflow guard; unhashed diagnostic).
    [[nodiscard]] const ChargeState& last_charge() const { return last_charge_; }
    [[nodiscard]] std::uint64_t n2o_overflow_substeps() const { return n2o_overflow_substeps_; }

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
    // rolloff: null = rpm_rolloff(desc); the cam-profile / lift-node sub-builds pass the parent's (their desc copy lists
    // only their own maps).
    [[nodiscard]] static std::shared_ptr<const SimulatedEngineMaps> build_maps_impl(const SimulatedEngineDesc& desc, bool steady_tables,
                                                                                   const RpmRolloff* rolloff = nullptr);
    void init_state_for(EngineState state);
    void set_live_ambient(real p_pa,real t_k);
    // Live control path: the ambient differs from the setup tables' OR the ECU's friction scale is not exactly 1.0
    // (engine thermal plan T1: the setup tables assume k = 1; a cold engine's ECU models the extra friction live).
    [[nodiscard]] bool live_steady() const { return p_amb_!=steady_.p_pa || t_amb_!=steady_.t_k || ecu_k_!=real(1.0); }
    [[nodiscard]] real live_steady_torque(real rpm,real plate,bool fired) const;
    [[nodiscard]] real live_steady_torque_on(const SimulatedEngineMaps& m,real rpm,real plate,bool fired) const;
    [[nodiscard]] real live_inverse_plate(real rpm,real request) const;
    [[nodiscard]] real live_inverse_plate_on(const SimulatedEngineMaps& m,real rpm,real request) const;
    // Engine garage P9: the steady demand-model lookups pre_solve uses, blended over the cam profiles by cam_w_.
    [[nodiscard]] real steady_motoring(real rpm) const;
    [[nodiscard]] real steady_wot(real rpm) const;
    [[nodiscard]] real steady_torque(real rpm, real plate) const;
    [[nodiscard]] real inverse_plate_target(real rpm, real request) const;
    void step_cam_switching(const Sensors& sensors, real rpm, real h);
    // Engine garage P10: the plant's model pair at the current state - (lo, nullptr, 0) for a plain engine, the cam
    // profiles blended by cam_w_ (P9), or the two lift nodes bracketing u_ (P10). Every model evaluation blends over it.
    struct PlantPair {
        const SimulatedEngineMaps* lo = nullptr;
        const SimulatedEngineMaps* hi = nullptr;
        real w = 0.0;
    };
    [[nodiscard]] PlantPair plant_pair() const;
    [[nodiscard]] PlantPair lift_pair_at(real u) const;
    [[nodiscard]] const SimulatedSteadyTables& tables_for(const SimulatedEngineMaps* mm) const;
    [[nodiscard]] const SimulatedEngineMaps& lift_node(std::size_t k) const;
    [[nodiscard]] const SimulatedSteadyTables& lift_node_tables(std::size_t k) const;
    [[nodiscard]] const SimulatedEngineMaps& idle_maps() const;
    [[nodiscard]] const SimulatedSteadyTables& idle_tables() const;
    [[nodiscard]] real node_motoring(std::size_t k, real rpm) const;
    [[nodiscard]] real node_wot(std::size_t k, real rpm) const;
    [[nodiscard]] real node_torque_at_p(std::size_t k, real rpm, real p) const;
    [[nodiscard]] real node_vacuum_torque(std::size_t k, real rpm, real p_target) const;
    [[nodiscard]] real node_pressure_for_torque(std::size_t k, real rpm, real target, real p_lo, real p_hi) const;
    [[nodiscard]] real vacuum_plate(real omega, real p_target, real u, const NitrousCharge* nc) const;
    // Pedal plan P1 helpers (engine_simulated.cpp, "manual pedal law"). All const, allocation-free.
    [[nodiscard]] real steady_idle_torque(real rpm, real plate) const; // the idle model's (idle lift node / low cam) table
    [[nodiscard]] real full_load_torque(real rpm) const;               // F(n)
    // Turbo: net torque at manifold pressure p with the CURRENT charge temperature and turbine-inlet pressure, and the
    // pressure in [p_lo, p_hi] at which it equals target (fixed bisection).
    [[nodiscard]] real boost_torque_at_p(real omega, real p) const;
    [[nodiscard]] real boost_pressure_for_torque(real omega, real target, real p_lo, real p_hi) const;
    // The plate that holds the manifold at p_target from the CURRENT supply (turbo charge or ambient); 1 if it cannot.
    [[nodiscard]] real holding_plate(real omega, real p_target) const;
    int pedal_law_override_ = -1;
    real boost_target_pa_ = -1.0;
    real pedal_target_nm_ = std::numeric_limits<real>::quiet_NaN();
    real theta_idle_last_ = 0.0; // NOT hashed: the idle controller output of the last pre_solve (request_envelope)
    PartLoadBoost part_load_boost_ = PartLoadBoost::Efficient;
    std::shared_ptr<const FullLoadEnvelope> envelope_;
    std::shared_ptr<const FullLoadEnvelope> pending_envelope_; // queued with pending_maps_; keeps the old one after a swap
    // A copy of the construction desc (electronic engines only): what full_load_envelope needs to rebuild the envelope
    // on a turbo-configuration or maps swap. Shared, never written after construction / a swap.
    std::shared_ptr<const SimulatedEngineDesc> envelope_desc_;
    // Engine garage P10 state: u_ (hashed only with a family), its target and plate mode (derived each substep, not
    // hashed), the override hook, and each lower node's steady tables (sized at construction, swapped with the pending
    // set on a hot swap, refilled with the others by set_ambient).
    real u_ = 1.0;
    real u_target_ = 1.0;
    bool lift_vacuum_mode_ = false;
    real u_override_ = -1.0;
    std::size_t idle_node_ = 0;
    real u_act_h_ = -1.0, u_act_a_ = 0.0;
    std::vector<SimulatedSteadyTables> steady_nodes_;
    std::vector<SimulatedSteadyTables> pending_steady_nodes_;
    // Engine garage P9: cam switching state (hashed only with cam profiles) and the override hook.
    bool cam_high_ = false;
    real cam_w_ = 0.0;
    real cam_override_ = -1.0;
    SimulatedSteadyTables steady_hi_;
    SimulatedSteadyTables pending_steady_hi_;

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
    // Boost-aware idle feed-forward cache (pre_solve): the quantised charge pressure/temperature and turbine-inlet
    // pressure plus the ambient feed-forward it was solved for; boost_idle_ff_ is a pure function of them (not hashed).
    static constexpr real kBoostIdleThresholdPa = 1000.0;
    // Pedal plan P1 (2.3) [D]: the DFCO residual band's hysteresis, a fraction of the request span F - T_cut (fuel cut is
    // re-entered only once the target lies this far below the fired floor).
    static constexpr real kPedalDfcoHysteresis = 0.01;
    real boost_idle_p_ = -1.0, boost_idle_t_ = -1.0, boost_idle_exh_ = -1.0, boost_idle_base_ff_ = -1.0, boost_idle_ff_ = 0.0;
    // Engine thermal plan T1 (vault PHYS-063): the oil-viscosity friction scale. plant_k_ = the PLANT's k (the
    // physical friction: tau_fric and its slope in d; T1b: never the Coulomb caps); ecu_k_ = the ECU's quantised estimate
    // (steady torque tables, idle feed-forward/cap, torque demand). Both come from Sensors, one tick old, both 1.0
    // without thermal. Not hashed (derived from the hashed thermal state). eta_c_ = this substep's combustion
    // efficiency min(1, lambda) x (1 - anti-lag cut share), read by post_solve for the released heat.
    real plant_k_ = 1.0, ecu_k_ = 1.0, eta_c_ = 0.0;
    real hyd_ref_rpm_ = 2000.0; // T1b: Sensors::friction_reference_rpm (unused while both k are 1)
    real p_amb_ = kIsaPressurePa, t_amb_ = kIsaTemperatureK;
    real r_t_ = 0.0;         // R*T_amb
    real inv_sqrt_rt_ = 0.0; // 1/sqrt(R*T_amb)

    // actuator coefficient cache (h/(tau+h)), recomputed only when h changes
    real act_h_ = -1.0, act_a_ = 0.0;

    // hashed state (plan section 3.2)
    EngineState state_ = EngineState::Off;
    real idle_integrator_ = 0.0;
    bool limiter_cut_ = false;
    real limiter_rpm_override_ = 0.0; // engine garage P8 (A16): TcuRequest::limiter_rpm of the current substep, NOT hashed
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
    // N2O plan N2 (all unhashed: functions of this substep's Sensors::nitrous and the hashed state).
    // n2o_nc_ is this substep's charge input, n2o_on_ whether the charge model applies (active block with
    // flow and tables); the *_kg_ are post_solve's bookings for this substep.
    NitrousCharge n2o_nc_;
    bool n2o_on_ = false;
    real n2o_in_kg_ = 0.0, n2o_burnt_kg_ = 0.0, kit_fuel_kg_ = 0.0, n2o_dh_j_kg_ = 0.0, burnt_fuel_kg_ = 0.0;
    ChargeState last_charge_;
    std::uint64_t n2o_overflow_substeps_ = 0;
};

} // namespace ps::drivetrain
