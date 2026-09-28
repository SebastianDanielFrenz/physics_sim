// engine_cycle/cycle_engine.h - the multi-cylinder pipe-coupled cycle model
// (ps_audio A5, out/audio_plan.md section 3 A5). Part of target
// ps_engine_cycle, never links ps_core.
//
// n cylinders, each its own 0D CycleStepper in PIPE MODE (cycle_stepper.h),
// phased by the resolved firing angles (firing_order.h), coupled to ONE
// PipeNetwork (pipe_network.h) built from the engine's intake/exhaust pipe
// blocks plus the resolved installation (make_engine_pipe_network_desc below):
//
//   ambient -> [snorkel FV pipe -> airbox volume] -> throttle orifice
//           -> plenum volume -> one runner FV pipe per cylinder -> intake port
//   exhaust port -> one primary FV pipe per cylinder -> merge junction volumes
//           and merge FV pipes (the engine file's DAG) -> flange
//           -> installation waveguide (pipes, catalysts, chambers, branches,
//              radiating outlets) or, without an installation, an open end.
//
// Per step h (D-3 explicit coupling, conservative by construction): every
// port's state goes to the network, the network steps, each port face's
// flux / h becomes the stepper's constant PortSource for its own RK4 step.
// The throttle Cd*A is trimmed once per cycle so that the cycle-mean plenum
// pressure approaches the operating point's p_im (a multiplicative integral
// correction through the steady orifice law); the last cycle is measured
// with the Cd*A frozen.
//
// Deterministic: ps::math only, no unordered containers, no mutable statics,
// allocation-free after construction (run() allocates nothing), plain double.
// Single use: run() once per engine (a map point builds its own engine).
#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/pipe_network.h"
#include "ps/types.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace ps::engine_cycle {

inline constexpr int kCycleEngineMaxCylinders = 16;
// Settling cycles before the measured one (brief A5: 8 [A]); Preview 6 [A].
inline constexpr int kEngineSettlingCyclesFull = 8;
inline constexpr int kEngineSettlingCyclesPreview = 6;
// Merge junction volume per unit connected pipe area [m] (PROVISIONAL P8,
// ASSUMED): h*sum(A)*c/V = h*c/0.1 ~ 0.39 at the Preview step and 900 m/s.
inline constexpr real kJunctionLengthM = 0.1;
// Wave-speed bounds for the pipe CFL checks (audio plan section 4 [A]).
inline constexpr real kIntakeMaxWaveSpeed = 500.0;
inline constexpr real kExhaustMaxWaveSpeed = 900.0;

struct CycleEngineSettings {
    real h_s = kStepFull;  // nominal step; used: 120/(rpm*n), n = ceil(120/(rpm*h_s)) so h <= h_s
    int settling_cycles = kEngineSettlingCyclesFull;  // then one measured cycle
    real dx_m = 0.0;                                  // FV target cell size; 0 = desc.resolution.power_dx_m
    // Reference mode (V8 sabotage): no pipe network; each port tied to a fixed
    // reservoir (plenum at p_im / ambient) through its runner / primary exit
    // area (CycleStepper::set_port_reservoirs). No pipe waves.
    bool reservoir_ports = false;
    real trim_tolerance = 0.02;  // relative plenum-pressure shortfall that counts as unreachable [A]
};

struct CycleEngineResult {
    // Per-cylinder means of run_operating_point's quantities over the measured
    // cycle (t_exh_k mass-weighted over all cylinders, p_max_pa the maximum,
    // steps the engine's total step count).
    PointResult mean;
    real p_plenum_mean_pa = 0.0;  // measured cycle mean (= target in reservoir mode)
    real p_target_pa = 0.0;
    real plenum_error_rel = 0.0;  // (mean - target) / target
    real throttle_cd_area_m2 = 0.0;
    real throttle_cd_area_max_m2 = 0.0;  // min(pi/4 bore^2, stability bound)
    bool throttle_saturated = false;     // at the max and still short of the target by > trim_tolerance
    // Measured-cycle mass closure: delta(network + ports + cylinders) minus the
    // exchange with ambient / waveguide / reservoirs minus the fuel added [kg];
    // scale = sum of the cylinders' trapped air.
    real mass_closure_kg = 0.0;
    real mass_scale_kg = 0.0;
    real air_change_rel = 0.0;  // |air(last) - air(previous cycle)| / air(last), all cylinders
    std::uint64_t steps_per_cycle = 0;
    real h_s = 0.0;
    int cylinders = 0;
    std::array<real, kCycleEngineMaxCylinders> air_kg{};  // per cylinder, measured cycle
    // Trim trace (diagnostic): per cycle, the mean plenum pressure and the
    // throttle Cd*A that cycle ran with (first kTraceCycles cycles).
    static constexpr int kTraceCycles = 16;
    std::array<real, kTraceCycles> trace_plenum_mean_pa{};
    std::array<real, kTraceCycles> trace_cd_area_m2{};
};

// Index layout of the network make_engine_pipe_network_desc builds.
struct EngineNetworkLayout {
    int airbox_volume = -1;  // -1: no snorkel (or no installation) - the throttle draws from ambient
    int plenum_volume = -1;
    int throttle_orifice = -1;
    int intake_port0 = 0;   // intake port of cylinder c (0-based) = intake_port0 + c
    int exhaust_port0 = 0;  // exhaust port of cylinder c = exhaust_port0 + c
    int flange_node = -1;   // waveguide Junction at the flange (-1: none, the flange pipe ends open)
    real throttle_cd_area_geom_m2 = 0.0;  // Cd 1 [AS] * pi/4 * throttle_bore^2
};

// The pipe network of a pipe-bearing engine (PROVISIONAL P8-P10,
// out/audio_a5_progress.md). Plenum and runners start at p_plenum_init_pa,
// the ambient side at `ambient`, the exhaust side at ambient pressure,
// exhaust_backflow_temperature_k and x_b = 1. The throttle orifice is built
// with a tiny Cd*A (the caller sets its own). Throws std::invalid_argument
// on a desc without pipe blocks or a topology it cannot map.
[[nodiscard]] PipeNetworkDesc make_engine_pipe_network_desc(const drivetrain::CycleModelDesc& desc,
                                                            const GasStateDesc& ambient, real p_plenum_init_pa,
                                                            real dx_m, EngineNetworkLayout* layout);

// Firing phase of each cylinder after cylinder 1, in [0, 720) (validates the
// order/angles; an empty order on a 4-cylinder engine means 1-3-4-2).
[[nodiscard]] std::vector<real> engine_firing_phases_deg(const drivetrain::CycleModelDesc& desc);

// ps_audio A7: pipe-network builder with separate intake / exhaust cell
// targets (audio grade: resolution.audio_dx_intake_m / audio_dx_exhaust_m).
// The single-dx overload above is this one with dx_intake == dx_exhaust.
[[nodiscard]] PipeNetworkDesc make_engine_pipe_network_desc(const drivetrain::CycleModelDesc& desc,
                                                            const GasStateDesc& ambient, real p_plenum_init_pa,
                                                            real dx_intake_m, real dx_exhaust_m,
                                                            EngineNetworkLayout* layout);

// ps_audio A7: an end of an engine's network that radiates to the outside -
// an FV pipe end open to ambient (the snorkel mouth; the flange pipe without
// an installation) or a waveguide Open node (each tailpipe outlet).
struct RadiatingEnd {
    enum class Kind : std::uint8_t { PipeEnd, WaveguideOpen };
    Kind kind = Kind::PipeEnd;
    std::string name;
    int index = -1;         // pipe (PipeEnd) or waveguide node (WaveguideOpen)
    bool at_start = false;  // PipeEnd: the pipe's start end
    real area_m2 = 0.0;     // PipeEnd: geometric end area
    bool intake = false;    // on the intake side (upstream of the ports)
};

struct EngineCoreSettings {
    real dx_intake_m = 0.05;
    real dx_exhaust_m = 0.05;
    bool reservoir_ports = false;  // CycleEngineSettings::reservoir_ports
};

// The part of a multi-cylinder pipe engine shared by CycleEngine (the map
// generator: prescribed rpm, whole cycles of aligned steps) and ps_audio's
// live engine (A7: time-varying omega, one step per output sample): the n
// phased CycleSteppers, the PipeNetwork with its layout, the throttle Cd*A
// and the port <-> pipe hand-off of one step. It owns no stepping policy.
class EngineCore {
public:
    // Steppers reset_phased(op, phase); the network built with step h and
    // started at op.p_im_pa (plenum, runners) / op.p_amb_pa; the throttle at
    // the feed-forward Cd*A for op (VE 0.9 [A] through a steady orifice).
    EngineCore(const drivetrain::CycleModelDesc& desc, real lhv_j_kg, real afr_st, const OperatingPoint& op, real h,
               const EngineCoreSettings& settings);
    ~EngineCore();
    EngineCore(const EngineCore&) = delete;
    EngineCore& operator=(const EngineCore&) = delete;

    // D-3 hand-off of one step: every port state to the network, one network
    // step, each port face's flux / h to its stepper as a constant PortSource.
    // A no-op in reservoir mode. The caller then steps the cylinders.
    void exchange();

    [[nodiscard]] int cylinders() const { return cylinders_; }
    [[nodiscard]] real h() const { return h_; }
    [[nodiscard]] real phase_deg(int c) const { return phase_[static_cast<std::size_t>(c)]; }
    [[nodiscard]] const CycleStepper& cylinder(int c) const { return steppers_[static_cast<std::size_t>(c)]; }
    [[nodiscard]] CycleStepper& cylinder(int c) { return steppers_[static_cast<std::size_t>(c)]; }
    [[nodiscard]] const PipeNetwork* network() const { return net_.get(); }
    [[nodiscard]] PipeNetwork* network() { return net_.get(); }
    [[nodiscard]] const EngineNetworkLayout& layout() const { return layout_; }
    [[nodiscard]] real throttle_cd_area() const { return cda_; }
    [[nodiscard]] real throttle_cd_area_max() const { return cda_max_; }
    // Clamped to [1e-9, max]; pushed to the network's throttle orifice.
    void set_throttle_cd_area(real cd_area_m2);
    [[nodiscard]] real intake_volume_m3() const { return v_intake_; }  // plenum + runners
    // Radiating ends in network order: FV pipe ends first (pipe order, start
    // before end), then waveguide Open nodes (node order). Empty in reservoir mode.
    [[nodiscard]] const std::vector<RadiatingEnd>& radiating_ends() const { return rad_; }
    // Volume flow [m^3/s] leaving through radiating end k in the last step:
    // the end cell's u*A (outward) for a pipe end, the waveguide node's
    // outflow for an Open node.
    [[nodiscard]] real radiating_outflow(int k) const;
    [[nodiscard]] real total_mass() const;  // network + every cylinder and port
    // FNV-1a fold: the network (if any), every cylinder's state vector and
    // crank angle, the throttle Cd*A.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const;

private:
    int cylinders_ = 0;
    real h_ = 0.0;
    std::vector<real> phase_;
    std::vector<CycleStepper> steppers_;
    std::unique_ptr<PipeNetwork> net_;
    EngineNetworkLayout layout_{};
    real cda_ = 0.0, cda_max_ = 0.0;
    real v_intake_ = 0.0;
    std::vector<RadiatingEnd> rad_;
};

class CycleEngine {
public:
    // op: rpm, target p_im (plenum), t_im (= ambient temperature), p_amb,
    // lambda, spark, cam shifts. Throws std::invalid_argument on a desc
    // without pipe blocks, more than kCycleEngineMaxCylinders cylinders, or a
    // failed network stability check ("PipeNetwork: ...").
    CycleEngine(const drivetrain::CycleModelDesc& desc, real lhv_j_kg, real afr_st, const OperatingPoint& op,
                const CycleEngineSettings& settings = {});
    ~CycleEngine();
    CycleEngine(const CycleEngine&) = delete;
    CycleEngine& operator=(const CycleEngine&) = delete;

    // settling_cycles + 1 cycles; the result describes the last. Single use
    // (std::logic_error on a second call). Allocation-free.
    CycleEngineResult run();

    [[nodiscard]] int cylinders() const { return core_.cylinders(); }
    [[nodiscard]] real h() const { return core_.h(); }
    [[nodiscard]] std::uint64_t steps_per_cycle() const { return n_; }
    [[nodiscard]] real phase_deg(int c) const { return core_.phase_deg(c); }
    [[nodiscard]] const CycleStepper& cylinder(int c) const { return core_.cylinder(c); }
    // nullptr in reservoir mode.
    [[nodiscard]] const PipeNetwork* network() const { return core_.network(); }
    [[nodiscard]] const EngineNetworkLayout& layout() const { return core_.layout(); }
    // FNV-1a fold: the network (if any), every cylinder's state vector and
    // crank angle, the throttle Cd*A.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const { return core_.fold_state_hash(h); }

private:
    void step_all(std::uint64_t k);

    OperatingPoint op_{};
    CycleEngineSettings settings_{};
    std::uint64_t n_ = 0;
    EngineCore core_;
    bool used_ = false;
};

} // namespace ps::engine_cycle
