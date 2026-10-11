// ps/drivetrain/engine_dyno.h - engine garage P3 (A9, out/engine_garage_design.md 10.1 in the racing_game repo):
// a full-throttle dyno sweep of ONE engine over rpm, from the same runtime engine the car drives.
//
// Per engine kind (docs/data_formats/engine.md "Dyno sweep"):
//   * TorqueMapEngineDesc - the engine's own WOT curve, instant: net crank torque = w - (1 - throttle) * (w - m),
//     w/m the wot / motoring curves (m includes -c_k at rpm 0), i.e. what EngineTorqueMap delivers net of the Coulomb
//     row; exactly w at throttle 1. Governor and rev limiter are NOT applied (they shape the operating line, not
//     what the engine can deliver). Ambient is ignored (the graph engine ignores it).
//   * SimulatedEngineDesc without a turbo - the steady operating point (manifold pressure = root of f(p) = 0),
//     instant, ISA or the given ambient: SimulatedEngine::steady_point, the definition every node of the engine's
//     steady tables is built from. The limiter is not applied.
//   * SimulatedEngineDesc with a turbo - the real runtime engine, SimulatedEngine with its TurboPair, wastegate PID
//     and charge lag, stepped at the car's 960 Hz substep rate with the crank held at the point's speed (the
//     bench_hyper_calibration pattern), until the boost settles or max_settle_s elapse.
// No vehicle pedal map, throttle stage or TCU limit exists at engine level: `throttle` IS the engine's throttle
// command (electronic engines: a torque request; mechanical: the plate).
// Nitrous (engine garage P3b-2): DynoSweepSpec::nitrous adds a wet kit to a SIMULATED engine's sweep, always on the stepped
// path (docs/data_formats/engine.md "Dyno sweep" -> "Nitrous in the sweep").
//
// Units: rpm [rev/min], torque [N*m, NET at the crank, the Coulomb row included], power [kW], boost [bar gauge],
// turbo_rpm [rev/min of one rotor], fuel [g/s], BSFC [g/kWh], lambda [-], spark [deg BTDC], p_intake [kPa abs],
// exhaust [K], settle_s [s of simulated time].
#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/jobs/job_system.h"
#include "ps/types.h"
#include "ps/world/ambient.h"

#include <atomic>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace ps::drivetrain {

// Engine garage P3b-2: the nitrous hardware of a sweep - the bottle and kit descriptors exactly as the vehicle loader produces
// them (the N2oBottleDesc / NitrousKitDesc components of a PowertrainDesc, controller included). nitrous_from_powertrain picks
// them out of a loaded vehicle.
struct DynoNitrous {
    N2oBottleDesc bottle;
    NitrousKitDesc kit;
};

// The bottle and kit components of `pt` (the first of each; fluid links are not consulted), or nullopt when either is missing.
[[nodiscard]] std::optional<DynoNitrous> nitrous_from_powertrain(const PowertrainDesc& pt);

struct DynoSweepSpec {
    // Points to evaluate, in this order (each finite and > 0). Empty = the engine's idle_rpm up to its limiter rpm in
    // 250 rpm steps [AS], the limiter point itself always included last (no duplicate when a step lands on it).
    std::vector<real> rpm;
    // The engine's throttle COMMAND in [0, 1] (see the file header). 1.0 = wide open.
    real throttle = 1.0;
    // Air the engine breathes (ISA sea level by default); passed as Sensors::ambient like World/EngineRig do.
    AmbientSample ambient{};
    // Turbo engines only. Hold at most this long per point (simulated seconds, [C] bench_hyper_calibration used 15 s).
    real max_settle_s = 15.0;
    // Convergence: boost (manifold gauge, bar) and net torque are averaged over consecutive windows of settle_window_s.
    // A window pair passes when |boost(k) - boost(k-1)| / settle_window_s < settle_boost_bar_per_s AND
    // |torque(k) - torque(k-1)| / settle_window_s < settle_torque_rel_per_s * |torque(k)|. A point has settled after TWO
    // consecutive passing pairs (three windows: the earliest exit is 1.5 s with the defaults); the reported values are
    // the means over that last window. A boost rate alone (the design's criterion) exits at the peak of a wastegate
    // overshoot and misses a slowly spooling low-boost rotor - measured on the hyper, docs/data_formats/engine.md.
    // settle_boost_bar_per_s = 0 never converges (always runs max_settle_s).
    real settle_boost_bar_per_s = 0.005;  // [AS] design 10.1
    real settle_torque_rel_per_s = 0.002; // [AS] 0.2 %/s, measured against the 15 s bench loop
    real settle_window_s = 0.5;           // [AS] design 10.1
    // Turbo engines whose configuration has an anti_lag block: request the anti-lag channel on (true) or off
    // (false). It only acts below the block's arm_below_command, so at throttle 1 it changes nothing. Ignored by
    // every other engine.
    bool anti_lag = false;
    // Engine garage P3b-1: the part files the descriptor came from, copied into the findings' PartRef::file. A descriptor
    // does not remember its file, so the caller (the garage) supplies them; empty = unknown. Paths are used as given.
    struct PartFiles {
        std::string engine;               // the engine file (slot "engine")
        std::string turbo;                // the turbo hardware file (slot "turbo")
        std::string turbo_configuration;  // the turbo configuration file (slot "forced_induction")
        std::string nitrous_kit;          // P3b-2: the nitrous kit file (slot "nitrous_kit")
        std::string n2o_bottle;           // P3b-2: the N2O bottle file (slot "n2o_bottle")
    };
    PartFiles part_files;
    // P3b-2: how many turbos the engine's TurboPairDesc extensive values (wastegate_area_m2) are the sum of - the loader
    // multiplies the hardware file's per-turbo value by the configuration's count and a descriptor does not remember it. Only
    // the wastegate Remedy's current/required values divide by it, so they are in the hardware FILE's per-turbo terms.
    int turbo_count = 1;
    // P3b-2: an optional wet kit. Armed and sprayed at the controller's full command; empty (default) = the sweep is
    // bit-identical to before. A simulated engine only (a torque-map engine ignores a kit in the car, so a sweep with one
    // throws std::invalid_argument); the engine is then always stepped (also without a turbo). Rules in engine.md.
    std::optional<DynoNitrous> nitrous;
};

struct DynoPoint {
    real rpm = 0.0;
    real torque_nm = 0.0; // net crank torque; turbo: mean over the settled window of (tau0 - c_k)
    real power_kw = 0.0;  // torque * omega / 1000
    // Manifold gauge pressure (p_intake - ambient) in bar for a turbo engine - EngineSoundState::boost_bar; 0 for an
    // engine without a turbo (also the torque map), as in the car's sound state.
    real boost_bar = 0.0;
    real turbo_rpm = 0.0;     // one rotor; 0 without a turbo
    // NaN where the engine kind has no such quantity:
    real fuel_g_s = 0.0;      // torque map: the fuel map / crude fuel model at the WOT row (NaN without any fuel model)
    real bsfc_g_kwh = 0.0;    // fuel_g_s * 3600 / power_kw; NaN if power <= 0 or fuel is NaN
    real lambda = 0.0;        // NaN for a torque map (no mixture model)
    real spark_deg = 0.0;     // NaN for a torque map
    real p_intake_kpa = 0.0;  // NaN for a torque map (no manifold)
    real exhaust_k = 0.0;     // gas temperature at the exhaust ports; turbo engines only, NaN otherwise
    real charge_boost_bar = 0.0; // compressor outlet (charge) pressure minus ambient, bar; 0 without a turbo
    // Engine garage P3b-2 (a sweep with DynoSweepSpec::nitrous; window means over the settled window):
    real n2o_g_s = 0.0;       // N2O delivered to the engine, g/s; 0 without a kit or while the controller does not spray
    real kit_fuel_g_s = 0.0;  // kit fuel the engine booked, g/s (part of fuel_g_s); 0 likewise
    // The charge's combined lambda with the kit (O2-equivalent air over the stoichiometric demand of ECU + kit fuel); NaN
    // without a kit or while the controller does not spray. `lambda` stays the ECU's own target.
    real lambda_combined = std::numeric_limits<real>::quiet_NaN();
    // true: the settled window met the convergence criterion (always true for the instant kinds). false: the point
    // hit max_settle_s first (e.g. a low-rpm turbo equilibrium that keeps drifting); its values are the last window.
    bool converged = true;
    real settle_s = 0.0;      // simulated seconds the point ran (0 for the instant kinds)
};

struct DynoResult {
    // One per swept rpm, in the order of the spec (or ascending for the default grid). Bit-identical for any worker
    // count.
    std::vector<DynoPoint> points;
    // Engine garage P3b-1: the limit findings measured in the sweep (ps/drivetrain/limit_findings.h), sorted by rpm
    // ascending, then LimitKind value, then emission order (a stable sort; the two axes of one MapGridEdge are
    // emitted p_im first). A finding with a NaN rpm (static) would sort first; run_dyno_sweep produces none.
    // Bit-identical for any worker count, like `points`. Empty when nothing is within a margin of a limit (a torque-map
    // engine below its limiter, a quiet NA engine, ...).
    std::vector<LimitFinding> limits;
    // point_limits[i] = the range of `limits` that belongs to points[i] (same size as `points`): the findings whose rpm
    // equals points[i].rpm. A point without findings has count 0 (first is then the insertion position, still valid to
    // read as an empty range). Two points with the same rpm share one block, and their identical findings are not doubled.
    std::vector<LimitRange> point_limits;
};

// Sweeps `engine` (a TorqueMapEngineDesc or SimulatedEngineDesc; anything else throws std::invalid_argument).
// `maps`: null = the desc's own maps; otherwise they replace a simulated engine's whole maps block exactly like
// World::replace_engine_maps, and complete a maps_pending desc. A desc that is still maps_pending without `maps`
// throws std::runtime_error ("... the engine's maps are pending ..."). A bad spec throws std::invalid_argument.
//
// `pool`: the stepped (turbo) kind runs one job per rpm point on it (nullptr = serially on the calling thread);
// each point builds its own engine from shared immutable maps and writes slot i of the result, so the result is the
// same for any worker count and for pool == nullptr. The instant kinds are microseconds per point and always run
// serially on the calling thread.
//
// `cancel`: read (relaxed) immediately before each point starts, on the thread that is about to run it. Once a point
// sees it set, that point and every point not yet started are skipped and the call throws GenerationCancelled after
// the running points have finished (latency: at most one point per worker, up to max_settle_s of simulated time =
// tens of ms to ~0.1 s of wall time); no partial result is returned. A flag already set on entry throws before any
// work. If every point had already started when the flag is set, the sweep completes normally - the flag is a
// request, not a guarantee (the semantics of MapGenerationControl::cancel, engine garage P1).
[[nodiscard]] DynoResult run_dyno_sweep(const ComponentParams& engine, EngineMapsPtr maps, const DynoSweepSpec& spec,
                                        jobs::JobSystem* pool = nullptr, const std::atomic<bool>* cancel = nullptr);

// Engine garage P3b-1: limits of the DRIVELINE the engine-only sweep does not see. Compares the sweep's peak net crank
// torque (the maximum torque_nm over `dyno.points`, taken at the point with that torque) with every clutch of `powertrain`:
// a clutch holds up to its STATIC capacity capacity_nm * static_factor. Findings (all rpm = the peak point's rpm, slot
// "clutch", field "/capacity_nm"):
//   ClutchCapacity   Near when peak >= static capacity / 1.3, Limiting when peak >= static capacity.
// Each carries a Closed Remedy in the field's own terms (the KINETIC capacity_nm): current = capacity_nm, required_min =
// peak / static_factor (the smallest capacity that holds the peak), required_margin = 1.3 * peak / static_factor (the
// table's safety margin, limit_thresholds.h kDrivelineMargin). With static_factor == 1 these are peak and 1.3 * peak.
// A gearbox input-torque rating does not exist in GearboxDesc, so GearboxInputTorque is not produced. `clutch_file`
// fills PartRef::file (empty = unknown). A sweep without a positive torque, or a powertrain without a clutch, returns {}.
// Sorted like DynoResult::limits.
[[nodiscard]] std::vector<LimitFinding> check_driveline_limits(const DynoResult& dyno, const PowertrainDesc& powertrain,
                                                               const std::string& clutch_file = {});

} // namespace ps::drivetrain
