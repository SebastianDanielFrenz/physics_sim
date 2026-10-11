// core/src/drivetrain/engine_dyno.cpp - engine garage P3: run_dyno_sweep (ps/drivetrain/engine_dyno.h).
//
// Three methods, one per engine kind; see the public header for what each delivers. Every number a point reports
// comes from the runtime engine's own code: EngineTorqueMap's resolved curves/fuel (powertrain.cpp's
// to_engine_desc), SimulatedEngine::steady_point (the definition of the engine's steady tables), or the stepped
// SimulatedEngine itself (pre_solve/post_solve with TurboPair, wastegate PID, charge lag). No physics lives here.
#include "ps/drivetrain/engine_dyno.h"

#include "engine_params_util.h"
#include "engine_simulated.h"
#include "engine_torque_map.h"
#include "powertrain.h" // to_engine_desc

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace ps::drivetrain {
namespace {

constexpr real kNaN = std::numeric_limits<real>::quiet_NaN();
constexpr real kTwoPiRad = 6.283185307179586; // bench_hyper_calibration's literal, so a point at the same rpm sees the same omega
constexpr real kStepHz = 960.0;               // [C] the car's substep rate (WorldConfig::substep_rate_hz default)
constexpr real kDefaultRpmStep = 250.0;       // [AS] design 10.1

void validate_spec(const DynoSweepSpec& s) {
    const auto bad = [](const char* what) { throw std::invalid_argument(std::string("run_dyno_sweep: ") + what); };
    for (real r : s.rpm) {
        if (!(std::isfinite(r) && r > real(0.0))) bad("every rpm must be finite and > 0");
    }
    if (!(std::isfinite(s.throttle) && s.throttle >= real(0.0) && s.throttle <= real(1.0))) bad("throttle must be in [0, 1]");
    if (!(std::isfinite(s.ambient.pressure_pa) && s.ambient.pressure_pa > real(0.0) && std::isfinite(s.ambient.temperature_k) &&
          s.ambient.temperature_k > real(0.0))) {
        bad("ambient pressure and temperature must be finite and > 0");
    }
    if (!(std::isfinite(s.max_settle_s) && s.max_settle_s > real(0.0))) bad("max_settle_s must be > 0");
    if (!(std::isfinite(s.settle_window_s) && s.settle_window_s > real(0.0))) bad("settle_window_s must be > 0");
    if (!(std::isfinite(s.settle_boost_bar_per_s) && s.settle_boost_bar_per_s >= real(0.0))) bad("settle_boost_bar_per_s must be >= 0");
    if (!(std::isfinite(s.settle_torque_rel_per_s) && s.settle_torque_rel_per_s >= real(0.0))) bad("settle_torque_rel_per_s must be >= 0");
}

// idle .. limiter in kDefaultRpmStep steps, the limiter point last and never duplicated.
std::vector<real> default_rpm_grid(real idle_rpm, real limiter_rpm) {
    if (!(idle_rpm > real(0.0)) || !(limiter_rpm > idle_rpm)) {
        throw std::invalid_argument("run_dyno_sweep: the engine needs 0 < idle_rpm < limiter rpm for the default rpm grid");
    }
    std::vector<real> out;
    for (int k = 0;; ++k) {
        const real r = idle_rpm + real(k) * kDefaultRpmStep;
        if (!(r < limiter_rpm - real(1e-9))) break;
        out.push_back(r);
    }
    out.push_back(limiter_rpm);
    return out;
}

real bsfc_g_kwh(real fuel_g_s, real power_kw) {
    return (power_kw > real(0.0) && std::isfinite(fuel_g_s)) ? fuel_g_s * real(3600.0) / power_kw : kNaN;
}

bool cancelled(const std::atomic<bool>* cancel) { return cancel != nullptr && cancel->load(std::memory_order_relaxed); }

// ---------------------------------------------------------------------------------------------------------------
// Torque map: the WOT curve, instant.
DynoPoint torque_map_point(const EngineTorqueMap::Desc& d, real rpm, real throttle) {
    const real w = d.wot_torque_nm_vs_rpm.evaluate(rpm);
    const real m = d.motoring_torque_nm_vs_rpm.evaluate(rpm);
    // tau0 - c_k = (m + c_k) + theta*(w - m) - c_k, i.e. motoring plus theta of the combustion span. Written as
    // w - (1 - theta)*(w - m) so that theta == 1 returns the curve value w EXACTLY (m + 1*(w - m) can differ by an ulp).
    const real net = w - (real(1.0) - throttle) * (w - m);
    const real omega = rpm * kTwoPiRad / real(60.0);
    DynoPoint p;
    p.rpm = rpm;
    p.torque_nm = net;
    p.power_kw = net * omega / real(1000.0);
    p.lambda = p.spark_deg = p.p_intake_kpa = p.exhaust_k = kNaN;
    // Fuel at the same (rpm, theta_c) the runtime evaluates it: the fuel map at theta, or the crude constant-efficiency
    // model on the combustion torque theta*(w - m).
    if (d.has_fuel_map) {
        p.fuel_g_s = d.fuel_map.value(rpm, throttle);
    } else if (d.has_fuel_model) {
        p.fuel_g_s = std::max(real(0.0), throttle * (w - m) * omega) / (d.eta_i * d.fuel.lhv_j_kg) * real(1000.0);
    } else {
        p.fuel_g_s = kNaN;
    }
    p.bsfc_g_kwh = bsfc_g_kwh(p.fuel_g_s, p.power_kw);
    return p;
}

// ---------------------------------------------------------------------------------------------------------------
// Simulated engine without a turbo: SimulatedEngine::steady_point, instant.
DynoPoint steady_sim_point(const SimulatedEngineMaps& maps, real rpm, const DynoSweepSpec& spec) {
    const SimulatedEngine::SteadyPoint sp = SimulatedEngine::steady_point(maps, rpm, spec.throttle, spec.ambient);
    const real omega = rpm * kTwoPiRad / real(60.0);
    DynoPoint p;
    p.rpm = rpm;
    p.torque_nm = sp.net_torque_nm;
    p.power_kw = sp.net_torque_nm * omega / real(1000.0);
    p.fuel_g_s = sp.fuel_g_s;
    p.bsfc_g_kwh = bsfc_g_kwh(p.fuel_g_s, p.power_kw);
    p.lambda = sp.lambda;
    p.spark_deg = sp.spark_deg;
    p.p_intake_kpa = sp.p_im_pa / real(1000.0);
    p.exhaust_k = kNaN; // no exhaust thermal model without a turbo
    return p;
}

// ---------------------------------------------------------------------------------------------------------------
// Simulated engine with a turbo: the real engine stepped with the crank held.

// Sums of the per-substep samples of ONE settle window (the last PreSolveResult kept whole).
struct WindowStats {
    long n = 0;
    real net_torque = 0.0, fuel_g_s = 0.0, boost_bar = 0.0, charge_boost_bar = 0.0, turbo_rpm = 0.0;
    real lambda = 0.0, spark_deg = 0.0, p_intake_pa = 0.0, exhaust_k = 0.0;
    PreSolveResult last;

    void clear() { *this = WindowStats{}; }
    void add(const PreSolveResult& r, real c_k, real p_amb) {
        ++n;
        // tau0 excludes the engine's Coulomb row (Powertrain carries it as a friction row to ground with kinetic
        // capacity c_k, which opposes the turning crank), so the torque the crank delivers is tau0 - c_k - the
        // quantity bench_hyper_calibration reports as net_Nm.
        net_torque += r.tau0 - c_k;
        fuel_g_s += r.fuel_flow_g_s;
        boost_bar += r.boost_bar;
        charge_boost_bar += (r.p_charge_pa - p_amb) / real(100000.0);
        turbo_rpm += r.turbo_rpm;
        lambda += r.lambda;
        spark_deg += r.spark_advance_deg;
        p_intake_pa += r.p_intake_pa;
        exhaust_k += r.exhaust_port_k;
        last = r;
    }
    [[nodiscard]] real mean(real sum) const { return sum / real(n); }
};

// THE per-point settled-window evaluation: everything a point reports is derived from one window of samples (means in
// `w`, the final substep's PreSolveResult in `w.last`) and the engine instance that produced them. Engine garage P3b
// ("limit findings") hooks in HERE: the wastegate opening / rotor-cap / compressor margin / lambda / fuel-cut
// measurements read engine.turbo(), engine.maps() and w.last (and may extend WindowStats with window maxima) and
// append LimitFinding records to the result next to the DynoPoint this returns.
DynoPoint settled_window_point(const SimulatedEngine& engine, const WindowStats& w, real rpm, real omega, bool converged,
                               real settle_s) {
    (void)engine; // unused until P3b
    DynoPoint p;
    p.rpm = rpm;
    p.torque_nm = w.mean(w.net_torque);
    p.power_kw = p.torque_nm * omega / real(1000.0);
    p.boost_bar = w.mean(w.boost_bar);
    p.charge_boost_bar = w.mean(w.charge_boost_bar);
    p.turbo_rpm = w.mean(w.turbo_rpm);
    p.fuel_g_s = w.mean(w.fuel_g_s);
    p.bsfc_g_kwh = bsfc_g_kwh(p.fuel_g_s, p.power_kw);
    p.lambda = w.mean(w.lambda);
    p.spark_deg = w.mean(w.spark_deg);
    p.p_intake_kpa = w.mean(w.p_intake_pa) / real(1000.0);
    p.exhaust_k = w.mean(w.exhaust_k);
    const bool finite = std::isfinite(p.torque_nm) && std::isfinite(p.boost_bar) && std::isfinite(p.turbo_rpm);
    p.converged = converged && finite;
    p.settle_s = settle_s;
    return p;
}

DynoPoint turbo_point(const SimulatedEngineDesc& desc, const std::shared_ptr<const SimulatedEngineMaps>& maps, real rpm,
                      const DynoSweepSpec& spec) {
    SimulatedEngine engine(desc, maps);
    engine.reset(EngineState::Running); // seeded Running, ignition on, no starter (a held crank never starts the engine)

    const real h = real(1.0) / kStepHz;
    const real omega = rpm * kTwoPiRad / real(60.0);
    const long window_steps = std::max(1L, static_cast<long>(std::floor(spec.settle_window_s * kStepHz + real(0.5))));
    const long max_steps = std::max(window_steps, static_cast<long>(std::floor(spec.max_settle_s * kStepHz + real(0.5))));
    const long windows = max_steps / window_steps; // whole windows; the run ends at a window boundary
    const real window_s = real(window_steps) * h;

    Sensors s;
    s.omega = omega;
    s.ignition = real(1.0);
    s.ambient = spec.ambient;
    s.anti_lag_channel = spec.anti_lag ? real(1.0) : real(0.0);
    const ThrottleCommand throttle{spec.throttle, real(0.0), real(1.0)};
    const real c_k = engine.maps().c_k;

    WindowStats w;
    // Convergence (measured on the hyper, see engine.md "Dyno sweep"): a window pair passes when the boost mean moved
    // less than settle_boost_bar_per_s AND the net-torque mean less than settle_torque_rel_per_s (relative); the point
    // settles after kPassingPairs consecutive passing pairs. One pair alone exits at the peak of a wastegate overshoot
    // (boost rate crosses zero there) and a boost rate alone cannot see a slowly spooling low-boost rotor.
    constexpr int kPassingPairs = 2;
    real prev_boost = kNaN, prev_torque = kNaN;
    int passing = 0;
    bool converged = false;
    long k = 0;
    for (; k < windows; ++k) {
        w.clear();
        for (long i = 0; i < window_steps; ++i) {
            const PreSolveResult r = engine.pre_solve(s, throttle, TcuRequest{}, h);
            (void)engine.post_solve(omega, h);
            w.add(r, c_k, spec.ambient.pressure_pa);
        }
        const real boost = w.mean(w.boost_bar);
        const real torque = w.mean(w.net_torque);
        if (k > 0) {
            const bool boost_ok = std::abs(boost - prev_boost) / window_s < spec.settle_boost_bar_per_s;
            const bool torque_ok = std::abs(torque - prev_torque) / window_s <
                                   spec.settle_torque_rel_per_s * std::max(std::abs(torque), real(1.0));
            passing = (boost_ok && torque_ok) ? passing + 1 : 0;
            if (passing >= kPassingPairs) {
                converged = true;
                ++k;
                break;
            }
        }
        prev_boost = boost;
        prev_torque = torque;
    }
    return settled_window_point(engine, w, rpm, omega, converged, real(k * window_steps) * h);
}

struct Shared {
    const std::atomic<bool>* cancel = nullptr;
    std::atomic<bool> skipped{false}; // some point observed the cancel flag and did not run
};

} // namespace

DynoResult run_dyno_sweep(const ComponentParams& engine, EngineMapsPtr maps, const DynoSweepSpec& spec, jobs::JobSystem* pool,
                          const std::atomic<bool>* cancel) {
    validate_spec(spec);
    // Cancelled before it started: nothing built, no point run (MapGenerationControl::cancel semantics).
    if (cancelled(cancel)) throw GenerationCancelled();

    const ComponentParams resolved = with_engine_maps(engine, maps, "run_dyno_sweep");
    DynoResult result;

    if (const TorqueMapEngineDesc* tm = std::get_if<TorqueMapEngineDesc>(&resolved)) {
        const EngineTorqueMap::Desc d = to_engine_desc(*tm);
        const std::vector<real> rpms = spec.rpm.empty() ? default_rpm_grid(d.idle_rpm, d.limiter.rpm) : spec.rpm;
        result.points.reserve(rpms.size());
        for (real rpm : rpms) {
            if (cancelled(cancel)) throw GenerationCancelled();
            result.points.push_back(torque_map_point(d, rpm, spec.throttle));
        }
        return result;
    }

    const SimulatedEngineDesc* sim = std::get_if<SimulatedEngineDesc>(&resolved);
    if (sim == nullptr) throw std::invalid_argument("run_dyno_sweep: the component is not an engine (torque map or simulated)");

    const std::vector<real> rpms = spec.rpm.empty() ? default_rpm_grid(sim->idle_rpm, sim->limiter.rpm) : spec.rpm;

    if (!sim->turbo_pair.has_value()) {
        // build_model_maps throws SimulatedEngine's "maps are pending" / rotary / geometry messages like the engine would.
        const std::shared_ptr<const SimulatedEngineMaps> m = SimulatedEngine::build_model_maps(*sim);
        result.points.reserve(rpms.size());
        for (real rpm : rpms) {
            if (cancelled(cancel)) throw GenerationCancelled();
            result.points.push_back(steady_sim_point(*m, rpm, spec));
        }
        return result;
    }

    // Turbo: the limiter is a time-domain fuel-cut latch, not what the engine can deliver; moved above the swept range
    // so the limiter point itself is a normal WOT point (the instant kinds ignore it the same way). Only when needed:
    // a descriptor whose limiter already lies above every point is used as it is (bit-identical to the bench).
    SimulatedEngineDesc d = *sim;
    {
        const real top = *std::max_element(rpms.begin(), rpms.end());
        const real band = d.limiter.type == LimiterType::SoftCut ? d.limiter.band_rpm : real(0.0);
        if (!(d.limiter.rpm - band > top)) d.limiter.rpm = top + band + real(1.0);
    }
    const std::shared_ptr<const SimulatedEngineMaps> m = SimulatedEngine::build_maps(d); // throws if maps_pending

    result.points.assign(rpms.size(), DynoPoint{});
    std::vector<std::exception_ptr> errors(rpms.size());
    Shared shared;
    shared.cancel = cancel;
    const auto run_one = [&](std::size_t i) {
        if (cancelled(shared.cancel)) {
            shared.skipped.store(true, std::memory_order_relaxed);
            return;
        }
        try {
            result.points[i] = turbo_point(d, m, rpms[i], spec);
        } catch (...) {
            errors[i] = std::current_exception();
        }
    };
    if (pool == nullptr) {
        for (std::size_t i = 0; i < rpms.size(); ++i) {
            run_one(i);
            if (shared.skipped.load(std::memory_order_relaxed)) break;
        }
    } else {
        pool->parallel_batches(rpms.size(), run_one);
    }
    if (shared.skipped.load(std::memory_order_relaxed)) throw GenerationCancelled();
    for (const std::exception_ptr& e : errors) {
        if (e) std::rethrow_exception(e);
    }
    return result;
}

} // namespace ps::drivetrain
