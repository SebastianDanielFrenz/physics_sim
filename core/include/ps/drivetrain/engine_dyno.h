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
// command (electronic engines: a torque request; mechanical: the plate). Nitrous is out of scope (P3b).
//
// Units: rpm [rev/min], torque [N*m, NET at the crank, the Coulomb row included], power [kW], boost [bar gauge],
// turbo_rpm [rev/min of one rotor], fuel [g/s], BSFC [g/kWh], lambda [-], spark [deg BTDC], p_intake [kPa abs],
// exhaust [K], settle_s [s of simulated time].
#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/jobs/job_system.h"
#include "ps/types.h"
#include "ps/world/ambient.h"

#include <atomic>
#include <vector>

namespace ps::drivetrain {

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
    // true: the settled window met the convergence criterion (always true for the instant kinds). false: the point
    // hit max_settle_s first (e.g. a low-rpm turbo equilibrium that keeps drifting); its values are the last window.
    bool converged = true;
    real settle_s = 0.0;      // simulated seconds the point ran (0 for the instant kinds)
};

struct DynoResult {
    // One per swept rpm, in the order of the spec (or ascending for the default grid). Bit-identical for any worker
    // count. Engine garage P3b appends the limit findings here (and per-point index ranges into them) without
    // changing this vector.
    std::vector<DynoPoint> points;
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

} // namespace ps::drivetrain
