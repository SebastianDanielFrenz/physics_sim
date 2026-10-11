// ps/drivetrain/limit_remedies.h - engine garage P3b-2: the SIMULATED remedies of the limit findings
// (ps/drivetrain/limit_findings.h, RemedyCost::Simulated) and the power-target solve for the nitrous jet.
//
// A sweep (run_dyno_sweep) attaches an UNSOLVED Simulated Remedy placeholder (current filled, the required values NaN) to
// WastegateSaturated, BoostAboveTarget and TurboRotorCap findings; the garage shows a "Calculate" button for it. solve_remedy
// runs a search over settled single-point dyno runs (the same stepped engine as the sweep) and returns the solved Remedy.
// Sketch maps never trigger a solve: nothing here is called by run_dyno_sweep, and a solve is only as good as the maps
// the request carries (a Sketch sweep's solve is a Sketch solve; the garage runs the solve on the maps it shows).
//
// Search method (docs/data_formats/engine.md "Simulated remedies"): the quantity to find is monotone, so after one bracketing
// round the bracket is cut by a FIXED 4-section per round (3 interior points evaluated together, one job each when a pool is
// given). The sequence of evaluated values depends on the request only - never on the pool or its size - so the result is
// bit-identical for any worker count and for no pool. Plain bisection would need fewer evaluations in total (about 10 vs 15)
// but one round per evaluation; with a pool 4-section needs about half the wall time.
#pragma once

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/jobs/job_system.h"
#include "ps/types.h"

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <functional>

namespace ps::drivetrain {

// Everything a solve needs to run settled dyno points: the engine, its maps and the sweep spec (ambient, throttle, settle
// criteria, the nitrous kit, part files, turbo_count). spec.rpm is ignored - the solvers evaluate the finding's rpm / the rpm
// they are given.
struct RemedyRequest {
    ComponentParams engine;
    EngineMapsPtr maps; // null = the desc's own (same rule as run_dyno_sweep)
    DynoSweepSpec spec;
};

struct RemedyOptions {
    // Evaluations of one round run as one job each on this pool (nullptr = serially on the calling thread). Each evaluation is
    // a serial single-point run_dyno_sweep (no nested pool use).
    jobs::JobSystem* pool = nullptr;
    // Read (relaxed) before every round and by every evaluation before its point starts (run_dyno_sweep's cancel semantics):
    // a solve that sees it set throws GenerationCancelled after the running evaluations have finished; nothing partial is
    // returned. A flag set on entry throws before any work.
    const std::atomic<bool>* cancel = nullptr;
    // done / total evaluations, always on the CALLING thread, delivered after each round finished (one call per evaluation).
    // `total` is an UPPER BOUND fixed on entry; when the search ends early a last call (total, total) closes the bar.
    std::function<void(std::size_t done, std::size_t total)> on_progress;
    // The search stops when the bracket is narrower than this fraction of its upper end (the value reported is the upper end,
    // where the condition is satisfied). 0.5 % [AS], the N7 bisection tolerance.
    real tolerance_rel = 0.005;
    // Out: how many settled points the solve ran (nullptr = not reported). Written on success only.
    std::size_t* evaluations_out = nullptr;
};

// Solves the Simulated remedy of `finding` (kinds WastegateSaturated, BoostAboveTarget, TurboRotorCap; any other kind throws
// std::invalid_argument, as does a NaN rpm or an engine that is not a simulated engine with a turbo).
//
// Wastegate kinds: the smallest wastegate area (a factor over the engine's current one, searched in [1, 8]) at which a settled
// point at finding.rpm has NO WastegateSaturated and NO BoostAboveTarget finding - i.e. the mean gate opening < 0.92 [AS, the
// N7 sizing margin] and the charge pressure within the 2 % band over the PID target. TurboRotorCap additionally requires
// no TurboRotorCap finding at that point: the wastegate is tried first, and it clears a rotor cap only when the cap came from
// a gate that ran out of authority.
//
// Result: Remedy{part {"turbo", spec.part_files.turbo, "/wastegate_area_m2"}, current = the engine's wastegate area divided by
// spec.turbo_count (the hardware FILE's per-turbo value), required_min = factor * current, required_margin = required_min *
// 1.10 [AS, the N7b "+10 %"], unit SquareMetre, cost Simulated, solved true}. Already clear at factor 1: required_min =
// current. Not clear at 8x: solved = false, required values NaN; for TurboRotorCap the part is then {"turbo", file, ""} -
// "the turbo hardware", without a number. The solve is for the ONE rpm of the finding; a finding at another rpm of the same
// sweep gets its own solve, and the part needs the largest.
[[nodiscard]] Remedy solve_remedy(const RemedyRequest& request, const LimitFinding& finding, const RemedyOptions& options = {});

enum class PowerTargetPart : std::uint8_t {
    NitrousJet = 0, // the kit's N2O jet diameter (the fuel jet scales with it)
};

struct PowerTargetResult {
    bool solved = false;
    // Part {"nitrous_kit", file, "/nitrous_jet_diameter_mm"}, millimetres, Simulated. required_min is the smallest searched
    // diameter whose dyno-held gain reaches the target; required_margin equals it (a power target has no margin).
    Remedy nitrous_jet;
    // The fuel jet that goes with it: the kit's own fuel/N2O diameter ratio is kept (a bigger N2O jet with the old fuel jet
    // only runs lean). Part {"nitrous_kit", file, "/fuel_jet_diameter_mm"}.
    Remedy fuel_jet;
    real baseline_kw = 0.0;       // the same point without the kit
    real achieved_gain_kw = 0.0;  // power(with the solved jets) - baseline, >= target when solved
    std::size_t evaluations = 0;
};

// The N2O jet diameter at which the kit's dyno-held gain at `rpm` (settled power with the kit minus without, `spec.nitrous`
// required, simulated engine) reaches `target_gain_kw`. Search over [0, 4 x the current diameter]; a target the largest
// candidate cannot reach returns solved = false (required values NaN). target_gain_kw must be finite and > 0, rpm finite and
// > 0 (std::invalid_argument otherwise). The controller's windows apply: a kit that does not spray at `rpm` gains nothing.
[[nodiscard]] PowerTargetResult solve_power_target(const RemedyRequest& request, PowerTargetPart part, real rpm, real target_gain_kw,
                                                   const RemedyOptions& options = {});

} // namespace ps::drivetrain
