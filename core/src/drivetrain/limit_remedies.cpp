// core/src/drivetrain/limit_remedies.cpp - engine garage P3b-2: solve_remedy / solve_power_target
// (ps/drivetrain/limit_remedies.h). Every number comes from settled single-point run_dyno_sweep calls (the engine's own
// stepped model); this file only chooses the values to try and reads the findings / powers back. No physics lives here.
#include "ps/drivetrain/limit_remedies.h"

#include "limit_thresholds.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace ps::drivetrain {
namespace {

constexpr int kSections = 4;                 // [AS] 3 interior points per round, evaluated together
constexpr int kInterior = kSections - 1;
constexpr real kFactorMax = 8.0;             // [AS] the largest wastegate factor searched (the N7b answer is 1.26)
constexpr real kJetCandidates[] = {0.5, 1.0, 2.0, 4.0}; // [AS] bracketing candidates, times the kit's current N2O jet diameter

bool cancelled(const std::atomic<bool>* c) { return c != nullptr && c->load(std::memory_order_relaxed); }

// Rounds a 4-section search needs to take a bracket of relative width `rel0` below `tol` (+1 for the bracket's upper end moving).
std::size_t max_rounds(real rel0, real tol) {
    std::size_t r = 0;
    for (real w = rel0; w > tol && r < 64; w *= real(1.0) / real(kSections)) ++r;
    return r + 1;
}

// Runs fn(i) for i in [0, n): one job each on `pool`, else serially in index order. Rethrows the first captured exception
// (lowest index) after every evaluation finished.
template <class Fn>
void run_round(jobs::JobSystem* pool, std::size_t n, Fn&& fn) {
    std::vector<std::exception_ptr> errors(n);
    const auto one = [&](std::size_t i) {
        try {
            fn(i);
        } catch (...) {
            errors[i] = std::current_exception();
        }
    };
    if (pool == nullptr || n == 1) {
        for (std::size_t i = 0; i < n; ++i) one(i);
    } else {
        pool->parallel_batches(n, one);
    }
    for (const std::exception_ptr& e : errors) {
        if (e) std::rethrow_exception(e);
    }
}

// Progress bookkeeping (calling thread only).
struct Tracker {
    const RemedyOptions& opt;
    std::size_t done = 0, total = 0;
    void evaluated(std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            ++done;
            if (opt.on_progress) opt.on_progress(done, std::max(done, total));
        }
    }
    void finish() {
        if (opt.on_progress && done < total) opt.on_progress(total, total);
        if (opt.evaluations_out != nullptr) *opt.evaluations_out = done;
    }
};

// One settled point of the request's engine at `rpm`, with the descriptor / spec edits applied by the caller.
DynoResult settled_point(const ComponentParams& engine, const RemedyRequest& rq, const DynoSweepSpec& spec, real rpm, const std::atomic<bool>* cancel) {
    DynoSweepSpec sp = spec;
    sp.rpm = {rpm};
    return run_dyno_sweep(engine, rq.maps, sp, nullptr, cancel); // serial: the caller's pool runs the evaluations
}

// Cuts [lo, hi] (P(lo) false, P(hi) true) with 4-section rounds until (hi - lo) <= tol * hi. `round_fn(xs, ok, val)` evaluates the
// three interior points (the caller counts them); `hi_val` follows the value the upper end was evaluated with.
template <class RoundFn>
void refine(real& lo, real& hi, real& hi_val, real tol, std::size_t rounds, const std::atomic<bool>* cancel, RoundFn&& round_fn) {
    for (std::size_t r = 0; r < rounds && (hi - lo) > tol * hi; ++r) {
        if (cancelled(cancel)) throw GenerationCancelled();
        real xs[kInterior];
        for (int j = 0; j < kInterior; ++j) xs[j] = lo + (hi - lo) * real(j + 1) / real(kSections);
        bool ok[kInterior] = {};
        real val[kInterior] = {};
        round_fn(xs, ok, val);
        int t = -1;
        for (int j = 0; j < kInterior; ++j) {
            if (ok[j]) {
                t = j;
                break;
            }
        }
        if (t < 0) {
            lo = xs[kInterior - 1];
        } else {
            if (t > 0) lo = xs[t - 1];
            hi = xs[t];
            hi_val = val[t];
        }
    }
}

bool has_kind(const DynoResult& r, bool (*pred)(LimitKind)) {
    for (const LimitFinding& f : r.limits) {
        if (pred(f.kind)) return true;
    }
    return false;
}
bool is_wastegate_kind(LimitKind k) { return k == LimitKind::WastegateSaturated || k == LimitKind::BoostAboveTarget; }
bool is_wastegate_or_rotor_kind(LimitKind k) { return is_wastegate_kind(k) || k == LimitKind::TurboRotorCap; }

} // namespace

Remedy solve_remedy(const RemedyRequest& rq, const LimitFinding& finding, const RemedyOptions& opt) {
    const bool rotor = finding.kind == LimitKind::TurboRotorCap;
    if (!(finding.kind == LimitKind::WastegateSaturated || finding.kind == LimitKind::BoostAboveTarget || rotor)) {
        throw std::invalid_argument(std::string("solve_remedy: no simulated remedy exists for the finding kind ") + to_string(finding.kind));
    }
    if (!std::isfinite(finding.rpm) || !(finding.rpm > real(0.0))) throw std::invalid_argument("solve_remedy: the finding needs a finite rpm > 0");
    if (!(std::isfinite(opt.tolerance_rel) && opt.tolerance_rel > real(0.0) && opt.tolerance_rel < real(0.5))) {
        throw std::invalid_argument("solve_remedy: tolerance_rel must be in (0, 0.5)");
    }
    const SimulatedEngineDesc* sim = std::get_if<SimulatedEngineDesc>(&rq.engine);
    if (sim == nullptr || !sim->turbo_pair.has_value()) {
        throw std::invalid_argument("solve_remedy: the engine must be a simulated engine with a turbo");
    }
    if (cancelled(opt.cancel)) throw GenerationCancelled();

    const real area0 = sim->turbo_pair->wastegate_area();
    const int count = std::max(1, rq.spec.turbo_count);
    bool (*kinds)(LimitKind) = rotor ? &is_wastegate_or_rotor_kind : &is_wastegate_kind;
    // P(factor): a settled point at finding.rpm with the wastegate area scaled by `factor` has none of the kinds.
    const auto clear_at = [&](real factor) {
        ComponentParams e = rq.engine;
        SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(e);
        d.turbo_pair->wastegate_area_m2 = area0 * factor;
        const DynoResult r = settled_point(e, rq, rq.spec, finding.rpm, opt.cancel);
        return !has_kind(r, kinds);
    };

    Remedy out;
    out.part = PartRef{"turbo", rq.spec.part_files.turbo, "/wastegate_area_m2"};
    out.current = area0 / real(count);
    out.unit = LimitUnit::SquareMetre;
    out.cost = RemedyCost::Simulated;
    out.solved = false;

    Tracker tr{opt};
    const std::size_t rounds = max_rounds(real(0.5), opt.tolerance_rel);
    tr.total = 4 + std::size_t(kInterior) * rounds;

    // Round 0: the factors 1, 2, 4, 8. 1 is the placeholder's own state (normally still not clear); the first clear one
    // brackets the answer.
    const real f0[4] = {1.0, 2.0, 4.0, kFactorMax};
    bool ok0[4] = {};
    run_round(opt.pool, 4, [&](std::size_t i) { ok0[i] = clear_at(f0[i]); });
    tr.evaluated(4);
    int t = -1;
    for (int i = 0; i < 4; ++i) {
        if (ok0[i]) {
            t = i;
            break;
        }
    }
    if (t < 0) {
        if (rotor) out.part.field.clear(); // "the turbo hardware", without a number
        tr.finish();
        return out;
    }
    real lo = t > 0 ? f0[t - 1] : real(1.0);
    real hi = f0[t];
    real unused = 0.0;
    if (t > 0) {
        refine(lo, hi, unused, opt.tolerance_rel, rounds, opt.cancel, [&](const real* xs, bool* ok, real*) {
            run_round(opt.pool, std::size_t(kInterior), [&](std::size_t j) { ok[j] = clear_at(xs[j]); });
            tr.evaluated(std::size_t(kInterior));
        });
    }
    out.required_min = hi * out.current;
    out.required_margin = out.required_min * limits::kWastegateRemedyMargin.value;
    out.solved = true;
    tr.finish();
    return out;
}

PowerTargetResult solve_power_target(const RemedyRequest& rq, PowerTargetPart part, real rpm, real target_gain_kw, const RemedyOptions& opt) {
    if (part != PowerTargetPart::NitrousJet) throw std::invalid_argument("solve_power_target: unknown part");
    if (!(std::isfinite(rpm) && rpm > real(0.0))) throw std::invalid_argument("solve_power_target: rpm must be finite and > 0");
    if (!(std::isfinite(target_gain_kw) && target_gain_kw > real(0.0))) throw std::invalid_argument("solve_power_target: the target gain must be finite and > 0");
    if (!(std::isfinite(opt.tolerance_rel) && opt.tolerance_rel > real(0.0) && opt.tolerance_rel < real(0.5))) {
        throw std::invalid_argument("solve_power_target: tolerance_rel must be in (0, 0.5)");
    }
    if (!rq.spec.nitrous.has_value()) throw std::invalid_argument("solve_power_target: the request needs a nitrous kit (spec.nitrous)");
    if (std::get_if<SimulatedEngineDesc>(&rq.engine) == nullptr) throw std::invalid_argument("solve_power_target: the engine must be a simulated engine");
    const NitrousKitDesc& kit0 = rq.spec.nitrous->kit;
    const real d0 = kit0.nitrous_jet_diameter_mm;
    if (!(d0 > real(0.0))) throw std::invalid_argument("solve_power_target: the kit's N2O jet diameter must be > 0");
    if (cancelled(opt.cancel)) throw GenerationCancelled();

    const auto power_with = [&](real jet_mm) {
        DynoSweepSpec sp = rq.spec;
        sp.nitrous->kit.nitrous_jet_diameter_mm = jet_mm;
        sp.nitrous->kit.fuel_jet_diameter_mm = kit0.fuel_jet_diameter_mm * jet_mm / d0;
        return settled_point(rq.engine, rq, sp, rpm, opt.cancel).points.front().power_kw;
    };
    const auto power_without = [&]() {
        DynoSweepSpec sp = rq.spec;
        sp.nitrous.reset();
        return settled_point(rq.engine, rq, sp, rpm, opt.cancel).points.front().power_kw;
    };

    PowerTargetResult out;
    const auto fill_remedy = [&](Remedy& r, const char* field, real current) {
        r.part = PartRef{"nitrous_kit", rq.spec.part_files.nitrous_kit, field};
        r.current = current;
        r.unit = LimitUnit::Millimetre;
        r.cost = RemedyCost::Simulated;
        r.solved = false;
    };
    fill_remedy(out.nitrous_jet, "/nitrous_jet_diameter_mm", d0);
    fill_remedy(out.fuel_jet, "/fuel_jet_diameter_mm", kit0.fuel_jet_diameter_mm);

    Tracker tr{opt};
    const std::size_t rounds = max_rounds(real(1.0), opt.tolerance_rel);
    tr.total = 5 + std::size_t(kInterior) * rounds;

    // Round 0: the baseline and the four bracketing jets.
    real p[5] = {};
    run_round(opt.pool, 5, [&](std::size_t i) { p[i] = i == 0 ? power_without() : power_with(kJetCandidates[i - 1] * d0); });
    tr.evaluated(5);
    const real base = p[0];
    out.baseline_kw = base;
    int t = -1;
    for (int i = 0; i < 4; ++i) {
        if (p[i + 1] - base >= target_gain_kw) {
            t = i;
            break;
        }
    }
    if (t < 0) {
        out.achieved_gain_kw = p[4] - base; // the best candidate tried, for the caller's message
        tr.finish();
        out.evaluations = tr.done;
        return out;
    }
    real lo = t > 0 ? kJetCandidates[t - 1] * d0 : real(0.0);
    real hi = kJetCandidates[t] * d0;
    real hi_gain = p[t + 1] - base;
    refine(lo, hi, hi_gain, opt.tolerance_rel, rounds, opt.cancel, [&](const real* xs, bool* ok, real* val) {
        real pw[kInterior] = {};
        run_round(opt.pool, std::size_t(kInterior), [&](std::size_t j) { pw[j] = power_with(xs[j]); });
        tr.evaluated(std::size_t(kInterior));
        for (int j = 0; j < kInterior; ++j) {
            val[j] = pw[j] - base;
            ok[j] = val[j] >= target_gain_kw;
        }
    });
    out.solved = true;
    out.achieved_gain_kw = hi_gain;
    out.nitrous_jet.required_min = hi;
    out.nitrous_jet.required_margin = hi;
    out.nitrous_jet.solved = true;
    out.fuel_jet.required_min = kit0.fuel_jet_diameter_mm * hi / d0;
    out.fuel_jet.required_margin = out.fuel_jet.required_min;
    out.fuel_jet.solved = true;
    tr.finish();
    out.evaluations = tr.done;
    return out;
}

} // namespace ps::drivetrain
