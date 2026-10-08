// PHYS-033 P4 I7c C16a: cycle model -> runtime maps (ps/drivetrain/engine_maps.h).
//
// Extraction (every grid node, ISA intake state, lambda 1, MBT spark - or,
// with a cycle spark_advance_deg schedule, the scheduled spark plus the
// ignition delay, eta_i then divided by eta_spark(scheduled - theta_mbt) to
// the MBT-equivalent value the runtime multiplies back; C17a follow-up), with
// V_disp = cylinders*pi/4*bore^2*stroke and c_cycle = 1/2 computed exactly as
// SimulatedEngine::build_maps does, so the runtime reproduces the node:
//   VE        = m_air_rev / (p_im/(R*T_ref) * V_disp * c_cycle),  m_air_rev = cylinders * m_air * c_cycle
//               (m_air = net fresh air through the intake valve per cylinder and cycle; R = the
//               runtime's 287.05, not the cycle gas's own R, PROVISIONAL P-B)
//   eta_i     = W_gross * AFR / (LHV * m_air)          (the runtime's fuel accounting: inducted air / AFR, P-C)
//   pmep_exc  = -cylinders * W_pump / V_disp - (p_amb - p_im)   (the runtime's pumping term, signed)
//   F(rpm)    = FMEP(rpm) * V_disp / (4 pi)            (D13 correlation)
// eta_i is resampled onto rpm x m_air_rev: y = the sorted union of every
// node's m_air_rev (bit-identical to the runtime's own ve*air_per_ve at the
// node), each rpm line piecewise-linear over its own nodes, clamped (P-D).
// f_lambda(l) = [W_gross/m_air*l](l) / [..](1); eta_spark(-retard) = the mean
// over the three sweep points of [W_gross/m_air](retard) / [..](0).

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/adaptive_pressure.h"
#include <array>
#include <map>

#include "ps/drivetrain/isa_air.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace ps::drivetrain {

namespace {

constexpr real kPi = 3.14159265358979323846; // the same literal SimulatedEngine uses
constexpr real kCCycle = 0.5;
// ps_audio A1 (out/audio_plan.md D-11): these used to read
// SimulatedEngine::kIsaTemperatureK/kIsaPressurePa/kGasConstant directly
// from core/src/drivetrain/engine_simulated.h - a ps_core-INTERNAL header
// this file (now part of the ps_engine_cycle target, which ps_core links,
// not the reverse) may no longer include. ps/drivetrain/isa_air.h is the
// shared header-only source of truth both sides alias (see that header's
// own comment); SimulatedEngine's own 3 members now alias it too, so this
// is still exactly the runtime's own value, not a duplicated literal.
constexpr real kTref = kIsaAirTemperatureK;
constexpr real kPamb = kIsaAirPressurePa;
constexpr real kRGas = kIsaAirGasConstantJKgK;

// Forced induction (IntakeCycleDesc::compressor_supply, pipe engines only):
// every grid node's throttle draws from a fixed supply of max(ambient,
// kBoostSupplyRatio x the node pressure) at T_ref - a compressor outlet after
// an ideal intercooler, the plate trimming the rest [A]. The airbox path is
// bypassed: it cannot reach a node above ambient, and on a real turbo engine
// it is not what feeds the throttle. The maps stay independent of the turbo
// hardware (swappable at runtime): VE is normalised by the density at T_ref
// and the runtime applies its own charge temperature through air_per_ve. The
// exhaust still ends at ambient here; the runtime adds the turbine back
// pressure through the pumping term (pmep_excess). Engines without the flag
// are unchanged, keys included (map_cache folds the flag only when set).
constexpr real kBoostSupplyRatio = 1.05;
constexpr int kPreviewRpm = 10;
constexpr int kPreviewPim = 6;
// Engine garage P1: Sketch's reduced grid, at most 8 rpm values x 3 p_im rows.
constexpr int kSketchRpm = 8;

struct SweepSite {
    real rpm, p_pa;
};
// The sweeps of Preview and Full (identical, so their results differ only by
// step and cycle settings and the grid).
const std::vector<real> kSparkRetard = {-10.0, -5.0, 0.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 40.0};
const std::vector<SweepSite> kSparkSites = {{2000.0, 50000.0}, {3000.0, 80000.0}, {5000.0, 100000.0}};
const std::vector<real> kLambda = {0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.35, 1.5};
constexpr SweepSite kLambdaSite{3000.0, 80000.0};
// Sketch: one spark-sweep site (the middle one) with 6 retards, a lambda sweep
// of 5 values - [AS], chosen to keep the curve shapes the runtime needs (the
// retard range and the lean/rich ends of Full) with ~1/4 of its sweep points.
const std::vector<real> kSketchSparkRetard = {-10.0, 0.0, 5.0, 10.0, 20.0, 40.0};
const std::vector<SweepSite> kSketchSparkSites = {{3000.0, 80000.0}};
const std::vector<real> kSketchLambda = {0.6, 0.8, 1.0, 1.2, 1.5};
// Position of the reference entry (retard 0 / lambda 1) in a sweep list.
int index_of(const std::vector<real>& v, real x) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i] == x) return static_cast<int>(i);
    }
    throw std::logic_error("engine map generator: sweep list without its reference entry");
}

struct Fnv {
    std::uint64_t h = 14695981039346656037ULL;
    template <typename T>
    void pod(const T& v) {
        unsigned char bytes[sizeof(T)];
        std::memcpy(bytes, &v, sizeof(T));
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    }
    void vec(const std::vector<real>& v) {
        pod(static_cast<std::uint64_t>(v.size()));
        for (real x : v) pod(x);
    }
    void curve(const vehicle::Curve1D& c) {
        vec(c.x);
        vec(c.y);
    }
    void table(const Table2D& t) {
        vec(t.x);
        vec(t.y);
        vec(t.values);
    }
};

void require_ascending(const std::vector<real>& v, const char* what) {
    if (v.size() < 2) throw std::runtime_error(std::string("engine map generator: ") + what + " needs at least 2 values");
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (!(v[i] > real(0.0)) || !(v[i] < real(1e12))) {
            throw std::runtime_error(std::string("engine map generator: ") + what + " values must be > 0 and finite");
        }
        if (i > 0 && !(v[i] > v[i - 1])) {
            throw std::runtime_error(std::string("engine map generator: ") + what + " must be strictly ascending");
        }
    }
}

// Index-rounding subsample to at most m entries (keeps both ends).
std::vector<real> subsample(const std::vector<real>& v, int m) {
    const std::size_t n = v.size();
    if (n <= static_cast<std::size_t>(m)) return v;
    std::vector<real> out;
    const std::size_t mm = static_cast<std::size_t>(m) - 1;
    for (std::size_t k = 0; k <= mm; ++k) {
        const std::size_t idx = (2 * k * (n - 1) + mm) / (2 * mm);
        out.push_back(v[idx]);
    }
    return out;
}

// Sketch's p_im rows: the lowest node, the node nearest ambient and the top
// node; when the node nearest ambient is itself the lowest or the top one, the
// middle node (by index) stands in so the result keeps 3 rows (a grid of fewer
// than 3 nodes keeps all of them). Strictly ascending by construction.
std::vector<real> sketch_p_rows(const std::vector<real>& p) {
    const std::size_t n = p.size();
    if (n <= 3) return p;
    std::size_t nearest = 0;
    for (std::size_t i = 1; i < n; ++i) {
        if (std::abs(p[i] - kPamb) < std::abs(p[nearest] - kPamb)) nearest = i;
    }
    const std::size_t mid = (nearest == 0 || nearest == n - 1) ? (n - 1) / 2 : nearest;
    return {p.front(), p[mid], p.back()};
}

real cam_value(const Table2D& t, real rpm, real p) {
    return t.x.empty() ? real(0.0) : t.value(rpm, p);
}

real mbt_spark_atdc(const CycleModelDesc& d) {
    return engine_cycle::Wiebe::theta0_for_ca50(d.wiebe_a, d.wiebe_m, d.burn_duration_deg, d.ca50_mbt_deg_atdc);
}

real v_disp_of(const CycleModelDesc& d) {
    return real(d.cylinders) * kPi / real(4.0) * d.bore_m * d.bore_m * d.stroke_m;
}

} // namespace

real cycle_friction_torque_nm(const CycleModelDesc& d, real rpm) {
    const FrictionModelDesc& f = d.friction;
    real u = real(0.0);
    switch (f.model) {
    case FrictionModelDesc::Model::None:
        return real(0.0);
    case FrictionModelDesc::Model::PolynomialRpm:
        u = rpm / real(1000.0);
        break;
    case FrictionModelDesc::Model::PolynomialPistonSpeed:
        u = real(2.0) * d.stroke_m * rpm / real(60.0);
        break;
    }
    const real fmep = f.a_pa + f.b_pa * u + f.c_pa * u * u;
    return fmep * v_disp_of(d) / (real(4.0) * kPi);
}

struct EngineMapGenTask::Impl {
    CycleModelDesc desc;
    std::uint64_t reuse_key=0;
    real lhv = 0.0, afr = 0.0;
    MapQuality quality = MapQuality::Full;
    engine_cycle::StepperSettings settings;
    std::unique_ptr<engine_cycle::CycleStepper> proto;
    std::vector<real> rpm_axis, p_axis;
    std::vector<engine_cycle::OperatingPoint> ops;
    std::vector<engine_cycle::PointResult> results;
    std::vector<unsigned char> held;  // 1 = grid node generated with the plenum held (diagnostic)
    std::vector<unsigned char> done;
    // I7c C16b review fix N5: a caught-by-reference std::exception& that is
    // then touched (e.what()) is the TOOL-019 ASan-crash pattern (vault
    // TOOL-019: clang-cl 19.1.5's ASan stack instrumentation moves the catch
    // parameter away from the slot MSVC's EH runtime writes). run_point()
    // therefore catches by `catch (...)` and stores the exception itself
    // (never touching it); finish() rethrows the lowest-index failure
    // unmodified via std::rethrow_exception, so the original exception
    // type/message still reaches the caller, just not reconstructed as a
    // std::string in ASan-unsafe code.
    std::vector<std::exception_ptr> exceptions;
    std::size_t grid_n = 0;
    real mbt_atdc = 0.0;
    bool finished = false;
    // ps_audio A5: pipe-bearing desc -> every point through CycleEngine
    // (generator v2); otherwise the unchanged v1 single-cylinder path.
    bool pipe = false;
    std::uint32_t version = kLumpedGeneratorVersion;
    engine_cycle::CycleEngineSettings engine_settings;
    // The sweep lists of this quality (Sketch: one site, shorter lists).
    std::vector<real> spark_retard;
    std::vector<SweepSite> spark_sites;
    std::vector<real> lambdas;
    int spark_ref = 0, lambda_ref = 0;
};

namespace {
const char* quality_name(MapQuality q) {
    switch (q) {
    case MapQuality::Full: return "Full";
    case MapQuality::Preview: return "Preview";
    case MapQuality::Sketch: return "Sketch";
    }
    throw std::logic_error("engine map generator: unknown MapQuality");
}
} // namespace

EngineMapGenTask::EngineMapGenTask(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality)
    : impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.desc = desc;
    CycleModelDesc invariant=desc;
    invariant.grid_rpm={real(1000),real(2000)};
    invariant.grid_p_im={real(50000),real(100000)};
    s.reuse_key=engine_map_cache_key(invariant,fuel,quality);
    s.lhv = fuel.lhv_j_kg;
    s.afr = fuel.afr_stoich;
    s.quality = quality;
    if (!(s.lhv > real(0.0)) || !(s.afr > real(0.0))) throw std::runtime_error("engine map generator: fuel lhv/afr must be > 0");
    if (!(desc.bore_m > real(0.0)) || !(desc.stroke_m > real(0.0)) || desc.cylinders < 1) {
        throw std::runtime_error("engine map generator: bore, stroke and cylinders must be > 0");
    }
    if (!desc.back_pressure_pa_vs_exh_kg_s.x.empty()) {
        throw std::runtime_error("engine map generator: back_pressure_pa_vs_exh_kg_s is not supported yet (the runtime's "
                                 "exhaust pressure is ambient)");
    }
    require_ascending(desc.grid_rpm, "grid_rpm");
    require_ascending(desc.grid_p_im, "grid_p_im");
    s.proto = std::make_unique<engine_cycle::CycleStepper>(desc, s.lhv, s.afr); // validates the model desc
    // Every quality is handled by name: a new MapQuality value must be added
    // here (the default throws) rather than silently falling into Preview's
    // settings.
    const bool full = quality == MapQuality::Full;
    switch (quality) {
    case MapQuality::Full:
        s.rpm_axis = desc.grid_rpm;
        s.p_axis = desc.grid_p_im;
        s.spark_retard = kSparkRetard;
        s.spark_sites = kSparkSites;
        s.lambdas = kLambda;
        break;
    case MapQuality::Preview:
        s.rpm_axis = subsample(desc.grid_rpm, kPreviewRpm);
        s.p_axis = desc.pressure_sampling.enabled ? desc.grid_p_im : subsample(desc.grid_p_im, kPreviewPim);
        s.spark_retard = kSparkRetard;
        s.spark_sites = kSparkSites;
        s.lambdas = kLambda;
        break;
    case MapQuality::Sketch:
        s.rpm_axis = subsample(desc.grid_rpm, kSketchRpm);
        s.p_axis = desc.pressure_sampling.enabled ? desc.grid_p_im : sketch_p_rows(desc.grid_p_im);
        s.spark_retard = kSketchSparkRetard;
        s.spark_sites = kSketchSparkSites;
        s.lambdas = kSketchLambda;
        break;
    default:
        throw std::logic_error("engine map generator: unknown MapQuality");
    }
    s.spark_ref = index_of(s.spark_retard, real(0.0));
    s.lambda_ref = index_of(s.lambdas, real(1.0));
    // Sketch runs Preview's step and cycle settings (engine garage P1: measure
    // before changing; the dyno preview only needs the shape of the curves).
    s.settings.h_s = full ? engine_cycle::kStepFull : engine_cycle::kStepPreview;
    s.settings.cycles = full ? engine_cycle::kCyclesFull : engine_cycle::kCyclesPreview;
    s.mbt_atdc = mbt_spark_atdc(desc);
    s.pipe = desc.intake.has_value();
    s.version = engine_map_generator_version(desc);
    if (s.pipe) {
        s.engine_settings.h_s = s.settings.h_s;
        s.engine_settings.settling_cycles = full ? engine_cycle::kEngineSettlingCyclesFull : engine_cycle::kEngineSettlingCyclesPreview;
    }
    // C17a follow-up: Tier 1b spark schedule (empty = mbt_tracking) and Tier 2
    // ignition delay (Wiebe start = spark + delay).
    if (!(desc.ignition_delay_deg >= real(0.0)) || !(desc.ignition_delay_deg <= real(60.0))) {
        throw std::runtime_error("engine map generator: ignition_delay_deg must be in [0, 60]");
    }
    const bool scheduled = !desc.spark_advance_deg.x.empty() || !desc.spark_advance_deg.y.empty() ||
                           !desc.spark_advance_deg.values.empty();
    if (scheduled) {
        const std::string err = desc.spark_advance_deg.validate();
        if (!err.empty()) throw std::runtime_error("engine map generator: spark_advance_deg: " + err);
    }

    auto make_op = [&](real rpm, real p, real lambda, real spark_atdc) {
        engine_cycle::OperatingPoint op;
        op.rpm = rpm;
        op.p_im_pa = p;
        op.t_im_k = kTref;
        op.p_amb_pa = kPamb;
        if (s.pipe && desc.intake->compressor_supply) op.p_supply_pa = std::max(kPamb, p * kBoostSupplyRatio);
        op.lambda = lambda;
        op.spark_deg_atdc = spark_atdc;
        op.intake_advance_deg = cam_value(desc.cam.intake_advance_deg, rpm, p);
        op.exhaust_retard_deg = cam_value(desc.cam.exhaust_retard_deg, rpm, p);
        return op;
    };
    for (real rpm : s.rpm_axis) {
        for (real p : s.p_axis) {
            // Scheduled: the node runs at the ECU's spark (deg BTDC -> ATDC)
            // plus the flame-development delay; mbt_tracking: the MBT Wiebe start.
            const real start = scheduled ? -desc.spark_advance_deg.value(rpm, p) + desc.ignition_delay_deg : s.mbt_atdc;
            s.ops.push_back(make_op(rpm, p, real(1.0), start));
        }
    }
    s.grid_n = s.ops.size();
    for (const SweepSite& site : s.spark_sites) {
        for (real retard : s.spark_retard) {
            s.ops.push_back(make_op(site.rpm, site.p_pa, real(1.0), s.mbt_atdc + retard));
        }
    }
    for (real lambda : s.lambdas) s.ops.push_back(make_op(kLambdaSite.rpm, kLambdaSite.p_pa, lambda, s.mbt_atdc));
    s.results.resize(s.ops.size());
    s.done.assign(s.ops.size(), 0);
    s.held.assign(s.ops.size(), 0);
    s.exceptions.resize(s.ops.size());
}

EngineMapGenTask::~EngineMapGenTask() = default;

std::size_t EngineMapGenTask::point_count() const { return impl_->ops.size(); }
std::size_t EngineMapGenTask::grid_point_count() const { return impl_->grid_n; }

std::size_t EngineMapGenTask::reuse_points_from(const EngineMapGenTask& previous) {
    Impl& s=*impl_;const Impl& old=*previous.impl_;
    if(s.reuse_key!=old.reuse_key)throw std::invalid_argument("engine point reuse: engine/fuel/quality mismatch");
    using Key=std::array<real,10>;
    auto key=[](const engine_cycle::OperatingPoint& p,bool grid){return Key{p.rpm,p.p_im_pa,p.t_im_k,p.p_amb_pa,p.p_supply_pa,p.lambda,p.spark_deg_atdc,p.intake_advance_deg,p.exhaust_retard_deg,grid ? real(1):real(0)};};
    std::map<Key,std::size_t> index;
    for(std::size_t i=0;i<old.ops.size();++i)if(old.done[i] && !old.exceptions[i])index.emplace(key(old.ops[i],i<old.grid_n),i);
    std::size_t count=0;
    for(std::size_t i=0;i<s.ops.size();++i) {
        if(s.done[i])continue;
        auto found=index.find(key(s.ops[i],i<s.grid_n));if(found==index.end())continue;
        const auto j=found->second;s.results[i]=old.results[j];s.held[i]=old.held[j];s.done[i]=1;++count;
    }
    return count;
}

void EngineMapGenTask::run_point(std::size_t i) {
    Impl& s = *impl_;
    if (i >= s.ops.size()) throw std::out_of_range("EngineMapGenTask::run_point: index out of range");
    if (s.done[i]) return;
    try {
        if (s.pipe) {
            // One engine per point (PROVISIONAL P7): built here, then run()
            // allocates nothing. The result is the per-cylinder mean, the
            // same per-cylinder quantities the v1 path extracts.
            engine_cycle::CycleEngineResult r = engine_cycle::CycleEngine(s.desc, s.lhv, s.afr, s.ops[i], s.engine_settings).run();
            if (i < s.grid_n && !r.throttle_saturated && r.plenum_error_rel > s.engine_settings.trim_tolerance) {
                // The plenum stays above the node with the throttle closed:
                // the node is below the lowest manifold pressure this engine
                // reaches at this speed (valve overlap pushes exhaust back into
                // the intake - long cams idle at a higher MAP). Hold the plenum
                // instead: feed the throttle from a reservoir at the node
                // pressure itself, so backflow can leave through it. The cycle
                // then reports how the engine breathes at that pressure (little
                // or no net induction), which keeps the runtime above the floor.
                // Only such nodes take this path; every other point is unchanged.
                engine_cycle::OperatingPoint held = s.ops[i];
                held.p_supply_pa = held.p_im_pa;
                r = engine_cycle::CycleEngine(s.desc, s.lhv, s.afr, held, s.engine_settings).run();
                s.held[i] = 1;
            }
            if (i < s.grid_n) {
                if (r.throttle_saturated) {
                    throw std::runtime_error("engine map generator: plenum target " + std::to_string(s.ops[i].p_im_pa) +
                                             " Pa unreachable at " + std::to_string(s.ops[i].rpm) +
                                             " rpm (throttle fully open: mean plenum " + std::to_string(r.p_plenum_mean_pa) +
                                             " Pa)");
                }
                if (!(std::abs(r.plenum_error_rel) <= s.engine_settings.trim_tolerance)) {
                    throw std::runtime_error("engine map generator: plenum target " + std::to_string(s.ops[i].p_im_pa) +
                                             " Pa not settled at " + std::to_string(s.ops[i].rpm) + " rpm (mean plenum " +
                                             std::to_string(r.p_plenum_mean_pa) + " Pa)");
                }
            }
            s.results[i] = r.mean;
        } else {
            s.results[i] = engine_cycle::run_operating_point(*s.proto, s.ops[i], s.settings);
        }
        s.done[i] = 1;
    } catch (...) {
        // N5: never touch the caught exception here (TOOL-019); store it
        // unexamined and let finish() rethrow it on the (single, non-ASan-
        // arming) thread that calls finish().
        s.exceptions[i] = std::current_exception();
        s.done[i] = 2;
    }
}

void EngineMapGenTask::build_check() const {
    const Impl& s = *impl_;
    if (s.pipe && !s.ops.empty()) {
        // Same construction run_point makes for point 0; nothing is run.
        engine_cycle::CycleEngine(s.desc, s.lhv, s.afr, s.ops.front(), s.engine_settings);
    }
}

EngineMapsPtr EngineMapGenTask::finish() {
    Impl& s = *impl_;
    if (s.finished) throw std::runtime_error("EngineMapGenTask::finish: called twice");
    s.finished = true;
    for (std::size_t i = 0; i < s.ops.size(); ++i) {
        if (s.done[i] == 0) throw std::runtime_error("EngineMapGenTask::finish: point " + std::to_string(i) + " was not run");
        if (s.done[i] == 2) {
            // N5: rethrow the ORIGINAL exception unmodified (never
            // reconstructed via e.what() - TOOL-019) for the lowest failing
            // index; its own message still reaches the caller, which is
            // what every existing caller (e.g. vehicle_io.cpp's engine
            // loader) already lets propagate unwrapped.
            std::rethrow_exception(s.exceptions[i]);
        }
    }
    const CycleModelDesc& d = s.desc;
    const std::size_t nr = s.rpm_axis.size(), np = s.p_axis.size();
    const real cyl = real(d.cylinders);
    const real v_disp = v_disp_of(d);
    const real r_t = kRGas * kTref; // the runtime's make_ambient(ISA).r_t

    auto out = std::make_shared<EngineMaps>();
    EngineMaps& em = *out;
    SimulatedEngineMapsDesc& m = em.maps;
    em.quality = s.quality;
    em.generator_version = s.version;

    auto grid_table = [&](Table2D& t) {
        t.x = s.rpm_axis;
        t.y = s.p_axis;
        t.values.assign(nr * np, real(0.0));
    };
    grid_table(m.ve);
    grid_table(m.pmep_excess_pa);
    grid_table(em.t_exh_k);
    grid_table(em.h_exh_w);
    std::vector<real> node_air(nr * np), node_eta(nr * np);
    for (std::size_t ir = 0; ir < nr; ++ir) {
        for (std::size_t ip = 0; ip < np; ++ip) {
            const std::size_t k = ir * np + ip;
            const engine_cycle::PointResult& r = s.results[k];
            const real p = s.p_axis[ip];
            if (!(r.m_air_j_cyc > real(0.0))) {
                throw std::runtime_error("engine map generator: no net fresh air at " + std::to_string(s.rpm_axis[ir]) +
                                         " rpm / " + std::to_string(p) + " Pa");
            }
            const real air_per_ve = p / r_t * v_disp * kCCycle; // same expression order as the runtime
            const real ve = cyl * r.m_air_j_cyc * kCCycle / air_per_ve;
            m.ve.values[k] = ve;
            node_air[k] = ve * air_per_ve; // the runtime's m_air_rev at this node, bit for bit
            node_eta[k] = r.w_gross_j * s.afr / (s.lhv * r.m_air_j_cyc);
            m.pmep_excess_pa.values[k] = -cyl * r.w_pump_j / v_disp - (kPamb - p);
            em.t_exh_k.values[k] = r.t_exh_k;
            em.h_exh_w.values[k] = r.h_exh_j * cyl * s.rpm_axis[ir] / real(120.0);
            if (!(ve > real(0.0) && ve < real(1e3)) || !(node_eta[k] > real(0.0) && node_eta[k] < real(1.0))) {
                throw std::runtime_error("engine map generator: VE/eta_i out of range at " + std::to_string(s.rpm_axis[ir]) +
                                         " rpm / " + std::to_string(p) + " Pa (VE " + std::to_string(ve) + ", eta_i " +
                                         std::to_string(node_eta[k]) + ")");
            }
        }
    }
    m.has_pmep_excess = true;

    // f_lambda
    const int lambda_n = static_cast<int>(s.lambdas.size());
    const int spark_n = static_cast<int>(s.spark_retard.size());
    const int sites_n = static_cast<int>(s.spark_sites.size());
    {
        const std::size_t base = s.grid_n + static_cast<std::size_t>(sites_n * spark_n);
        auto g = [&](int j) {
            const engine_cycle::PointResult& r = s.results[base + static_cast<std::size_t>(j)];
            return r.w_gross_j / r.m_air_j_cyc * s.lambdas[static_cast<std::size_t>(j)];
        };
        const real g_ref = g(s.lambda_ref);
        for (int j = 0; j < lambda_n; ++j) {
            m.f_lambda.x.push_back(s.lambdas[static_cast<std::size_t>(j)]);
            m.f_lambda.y.push_back(j == s.lambda_ref ? real(1.0) : g(j) / g_ref);
        }
    }
    // eta_spark: x = spark - MBT = -retard (ascending), mean of the normalised
    // curves of the sweep sites (three for Preview/Full - summed in site order,
    // so the value is bit-identical to the old (a + b + c) / 3 - one for Sketch)
    {
        std::vector<real> norm(static_cast<std::size_t>(sites_n * spark_n));
        for (int k = 0; k < sites_n; ++k) {
            auto g = [&](int j) {
                const engine_cycle::PointResult& r =
                    s.results[s.grid_n + static_cast<std::size_t>(k * spark_n + j)];
                return r.w_gross_j / r.m_air_j_cyc;
            };
            const real g0 = g(s.spark_ref);
            for (int j = 0; j < spark_n; ++j) {
                norm[static_cast<std::size_t>(k * spark_n + j)] = j == s.spark_ref ? real(1.0) : g(j) / g0;
            }
        }
        real spread = real(0.0);
        for (int j = spark_n - 1; j >= 0; --j) {
            real sum = norm[static_cast<std::size_t>(j)];
            for (int k = 1; k < sites_n; ++k) sum += norm[static_cast<std::size_t>(k * spark_n + j)];
            const real mean = sum / real(sites_n);
            for (int k = 0; k < sites_n; ++k) spread = std::max(spread, std::abs(norm[static_cast<std::size_t>(k * spark_n + j)] - mean));
            m.eta_spark.x.push_back(-s.spark_retard[static_cast<std::size_t>(j)]);
            m.eta_spark.y.push_back(mean);
        }
        em.eta_spark_spread = spread;
        for (unsigned char h : s.held) em.held_plenum_points += h;
    }
    m.theta_mbt_deg.x = {s.rpm_axis.front(), s.rpm_axis.back()};
    m.theta_mbt_deg.y = {s.p_axis.front(), s.p_axis.back()};
    // theta_MBT (spark, deg BTDC): the MBT Wiebe start minus the ignition
    // delay; constant (fixed Wiebe), kept as a map. delay 0 -> exactly -mbt_atdc.
    m.theta_mbt_deg.values.assign(4, d.ignition_delay_deg == real(0.0) ? -s.mbt_atdc
                                                                        : -(s.mbt_atdc - d.ignition_delay_deg));
    // C17a follow-up: with a spark schedule every grid node ran at the
    // scheduled spark; store the MBT-equivalent eta_i = eta_gross(scheduled)
    // / eta_spark(scheduled - theta_mbt), evaluated with exactly the calls the
    // runtime makes, so its eta_i * eta_spark reproduces the node.
    if (!d.spark_advance_deg.x.empty()) {
        for (std::size_t ir = 0; ir < nr; ++ir) {
            for (std::size_t ip = 0; ip < np; ++ip) {
                const std::size_t k = ir * np + ip;
                const real rpm = s.rpm_axis[ir], p = s.p_axis[ip];
                const real x = d.spark_advance_deg.value(rpm, p) - m.theta_mbt_deg.value(rpm, p);
                const real es = m.eta_spark.evaluate(x);
                if (!(es > real(0.0))) {
                    throw std::runtime_error("engine map generator: eta_spark <= 0 at the scheduled spark at " +
                                             std::to_string(rpm) + " rpm / " + std::to_string(p) + " Pa");
                }
                node_eta[k] = node_eta[k] / es;
                if (!(node_eta[k] > real(0.0) && node_eta[k] < real(1.0))) {
                    throw std::runtime_error("engine map generator: MBT-equivalent eta_i out of range at " +
                                             std::to_string(rpm) + " rpm / " + std::to_string(p) + " Pa (" +
                                             std::to_string(node_eta[k]) + ")");
                }
            }
        }
    }

    // eta_i over rpm x m_air_rev: union axis, per-line piecewise-linear resampling.
    for (std::size_t ir = 0; ir < nr; ++ir) {
        for (std::size_t ip = 1; ip < np; ++ip) {
            if (!(node_air[ir * np + ip] > node_air[ir * np + ip - 1])) {
                throw std::runtime_error("engine map generator: air mass not strictly increasing in p_im at " +
                                         std::to_string(s.rpm_axis[ir]) + " rpm (eta_i resampling needs it)");
            }
        }
    }
    std::vector<real> yu = node_air;
    std::sort(yu.begin(), yu.end());
    yu.erase(std::unique(yu.begin(), yu.end()), yu.end());
    m.eta_i.x = s.rpm_axis;
    m.eta_i.y = yu;
    m.eta_i.values.assign(nr * yu.size(), real(0.0));
    for (std::size_t ir = 0; ir < nr; ++ir) {
        const real* xa = &node_air[ir * np];
        const real* ya = &node_eta[ir * np];
        for (std::size_t iy = 0; iy < yu.size(); ++iy) {
            const real q = yu[iy];
            real v;
            if (q <= xa[0]) {
                v = ya[0];
            } else if (q >= xa[np - 1]) {
                v = ya[np - 1];
            } else {
                std::size_t j = 1;
                while (q > xa[j]) ++j; // xa[j-1] < q <= xa[j]
                if (q == xa[j]) {
                    v = ya[j];
                } else {
                    const real t = (q - xa[j - 1]) / (xa[j] - xa[j - 1]);
                    v = ya[j - 1] + t * (ya[j] - ya[j - 1]);
                }
            }
            m.eta_i.values[ir * yu.size() + iy] = v;
        }
    }

    // friction: rpm 0, every desc grid rpm and the midpoints between them
    if (d.friction.model != FrictionModelDesc::Model::None) {
        std::vector<real> xs{real(0.0)};
        real prev = real(0.0);
        for (real r : d.grid_rpm) {
            xs.push_back((prev + r) / real(2.0));
            xs.push_back(r);
            prev = r;
        }
        for (real x : xs) {
            const real f = cycle_friction_torque_nm(d, x);
            if (!(f >= real(0.0))) {
                throw std::runtime_error("engine map generator: the friction correlation is negative at " + std::to_string(x) +
                                         " rpm");
            }
            m.friction_torque_nm.x.push_back(x);
            m.friction_torque_nm.y.push_back(f);
        }
    }
    m.has_theta_kl = false;
    // N4: carry the cycle block's own citation (ps::CycleModelDesc::source,
    // required by the loader's read_cycle_block) through into the generated
    // maps' provenance note, so a generated engine's data source survives -
    // it used to be validated at load time and then silently dropped.
    // s.desc.source is empty only for a CycleModelDesc built directly in
    // C++ (tests/fixtures), never for one that came through the loader.
    m.source = std::string("generated by the ps::engine_cycle map generator v") + std::to_string(s.version) +
               (s.pipe ? " (PHYS-033 P4 I7c C16a; ps_audio A5 CycleEngine: every cylinder, pipes and installation)"
                       : " (PHYS-033 P4 I7c C16a)") +
               ", quality " + quality_name(s.quality) +
               ", from the engine file's cycle block" +
               (s.desc.source.empty() ? std::string() : " (cycle source: " + s.desc.source + ")");
    em.content_hash = engine_maps_content_hash(em);
    return out;
}

namespace {

// State shared by the points of one generate_engine_maps run. Jobs touch only
// the two atomics (a relaxed load of the external cancel flag, one fetch_add per
// finished point): no allocation, no lock inside a job.
struct RunShared {
    EngineMapGenTask* task = nullptr;
    const std::atomic<bool>* cancel = nullptr;
    std::atomic<std::size_t> finished{0}; // points whose run_point returned
    std::atomic<bool> skipped{false};     // some point observed the cancel flag and did not run
    std::atomic<bool> abort{false};       // the progress callback threw: stop starting points
};

void run_one_point(RunShared* sh, std::size_t i) {
    if (sh->abort.load(std::memory_order_relaxed)) return;
    if (sh->cancel != nullptr && sh->cancel->load(std::memory_order_relaxed)) {
        sh->skipped.store(true, std::memory_order_relaxed);
        return;
    }
    sh->task->run_point(i);
    sh->finished.fetch_add(1, std::memory_order_release);
}

} // namespace

static EngineMapsPtr generate_fixed_engine_maps(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality,
                                   jobs::JobSystem* pool, const MapGenerationControl* control,
                                   std::vector<std::unique_ptr<EngineMapGenTask>>* history=nullptr) {
    const std::atomic<bool>* cancel = control != nullptr ? control->cancel : nullptr;
    const std::function<void(std::size_t, std::size_t)>* progress =
        (control != nullptr && control->on_progress) ? &control->on_progress : nullptr;
    // Cancelled before it started: no desc validation, no point, nothing built.
    if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) throw GenerationCancelled();

    auto owned=std::make_unique<EngineMapGenTask>(desc,fuel,quality);
    EngineMapGenTask& task=*owned;
    if(history)for(const auto& previous:*history)task.reuse_points_from(*previous);
    const std::size_t n = task.point_count();
    RunShared sh;
    sh.task = &task;
    sh.cancel = cancel;
    std::size_t reported = 0;
    const auto report_up_to = [&](std::size_t done) {
        // One call per finished point, values 1..total in order (a burst between two polls
        // arrives as consecutive calls), on this - the calling - thread only.
        while (reported < done) {
            ++reported;
            if (progress != nullptr) (*progress)(reported, n);
        }
    };

    if (pool == nullptr) {
        for (std::size_t i = 0; i < n; ++i) {
            run_one_point(&sh, i);
            if (sh.skipped.load(std::memory_order_relaxed)) break;
            report_up_to(sh.finished.load(std::memory_order_acquire));
        }
    } else {
        jobs::JobSystem::WaitGroup group;
        for (std::size_t i = 0; i < n; ++i) {
            RunShared* shp = &sh;
            pool->submit(jobs::JobFn([shp, i]() { run_one_point(shp, i); }), &group);
        }
        std::exception_ptr callback_error;
        if (progress != nullptr && !pool->on_worker_thread()) {
            // Poll the counter the workers bump; the pool's own wait() would spin the
            // same way (a non-worker caller has no queue to help drain).
            while (group.remaining.load(std::memory_order_acquire) > 0) {
                if (!callback_error) {
                    try {
                        report_up_to(sh.finished.load(std::memory_order_acquire));
                    } catch (...) {
                        // The jobs reference this frame: hold the exception until they are done.
                        callback_error = std::current_exception();
                        sh.abort.store(true, std::memory_order_relaxed);
                    }
                }
                std::this_thread::yield();
            }
        } else {
            pool->wait(group);
        }
        if (callback_error) std::rethrow_exception(callback_error);
        report_up_to(sh.finished.load(std::memory_order_acquire));
    }
    if (sh.skipped.load(std::memory_order_relaxed)) throw GenerationCancelled();
    auto result=task.finish();
    if(history)history->push_back(std::move(owned));
    return result;
}

EngineMapsPtr generate_engine_maps(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality,
                                   jobs::JobSystem* pool, const MapGenerationControl* control) {
    const auto& settings=desc.pressure_sampling;
    if(control && control->cancel && control->cancel->load(std::memory_order_relaxed))throw GenerationCancelled();
    if(!settings.enabled)return generate_fixed_engine_maps(desc,fuel,quality,pool,control);
    require_ascending(desc.grid_p_im,"grid_p_im");
    const real lo=settings.min_pa>0 ? settings.min_pa:desc.grid_p_im.front();
    const real hi=settings.max_pa>0 ? settings.max_pa:desc.grid_p_im.back();
    if(!(lo>0 && hi>lo && settings.relative_error>0 && settings.relative_error<=0.1) || settings.max_points<5 || settings.max_points>1024 || settings.max_passes<1 || settings.max_passes>12)
        throw std::runtime_error("adaptive pressure sampling: invalid bounds/tolerance/budget");
    CycleModelDesc sampling=desc;
    sampling.grid_p_im={lo,hi};
    for(real p:desc.grid_p_im)if(p>lo && p<hi)sampling.grid_p_im.push_back(p);
    std::sort(sampling.grid_p_im.begin(),sampling.grid_p_im.end());
    sampling.grid_p_im.erase(std::unique(sampling.grid_p_im.begin(),sampling.grid_p_im.end()),sampling.grid_p_im.end());
    // Adaptive pressure resolution is not capped by Preview's fixed-grid row
    // count. Quality still controls RPM count and cycle accuracy in each task.
    std::size_t offset=0;
    const std::size_t budget=(settings.max_passes+1)*settings.max_points*(desc.grid_rpm.size()+40);
    MapGenerationControl stages;
    stages.cancel=control ? control->cancel:nullptr;
    stages.on_progress=[&](std::size_t done,std::size_t){if(control && control->on_progress)control->on_progress(std::min(offset+done,budget-1),budget);};
    std::vector<std::unique_ptr<EngineMapGenTask>> history;
    auto run=[&](const CycleModelDesc& d){auto m=generate_fixed_engine_maps(d,fuel,quality,pool,&stages,&history);offset+=d.grid_rpm.size()*d.grid_p_im.size()+40;return m;};
    auto current=run(sampling);
    for(int pass=0;pass<settings.max_passes;++pass) {
        if(stages.cancel && stages.cancel->load(std::memory_order_relaxed))throw GenerationCancelled();
        CycleModelDesc probe=sampling;
        for(std::size_t j=1;j<sampling.grid_p_im.size();++j)
            for(int q=1;q<4;++q)probe.grid_p_im.push_back(sampling.grid_p_im[j-1]+(sampling.grid_p_im[j]-sampling.grid_p_im[j-1])*real(q)/real(4));
        std::sort(probe.grid_p_im.begin(),probe.grid_p_im.end());
        probe.grid_p_im.erase(std::unique(probe.grid_p_im.begin(),probe.grid_p_im.end()),probe.grid_p_im.end());
        if(probe.grid_p_im.size()>settings.max_points)throw std::runtime_error("adaptive pressure sampling: point budget exhausted before validation converged");
        auto tested=run(probe);
        std::vector<real> failing;
        for(real p:probe.grid_p_im) {
            if(std::binary_search(sampling.grid_p_im.begin(),sampling.grid_p_im.end(),p))continue;
            real error=0;
            auto check=[&](const Table2D& actual,const Table2D& old){error=std::max(error,pressure_error(actual,old,p));};
            check(tested->maps.ve,current->maps.ve);
            check(tested->maps.pmep_excess_pa,current->maps.pmep_excess_pa);
            check(tested->t_exh_k,current->t_exh_k);check(tested->h_exh_w,current->h_exh_w);
            // eta_i is on fresh-air mass, not pressure; compare on each pressure
            // probe's actual mass so the generated efficiency curve also refines.
            const real rt=real(287.05)*real(288.15);
            for(real rpm:tested->maps.ve.x) {
                const real air=tested->maps.ve.value(rpm,p)*p/rt*v_disp_of(desc)*real(0.5);
                const real a=tested->maps.eta_i.value(rpm,air),b=current->maps.eta_i.value(rpm,air);
                error=std::max(error,std::abs(a-b)/std::max(real(1e-6),std::abs(a)));
            }
            if(error>settings.relative_error*real(0.5))failing.push_back(p);
        }
        if(failing.empty()) {
            auto result=std::make_shared<EngineMaps>(*tested);
            reduce_pressure_axis(result->maps.ve,settings.relative_error*real(0.5));
            reduce_pressure_axis(result->maps.pmep_excess_pa,settings.relative_error*real(0.5));
            reduce_pressure_axis(result->t_exh_k,settings.relative_error*real(0.5));
            reduce_pressure_axis(result->h_exh_w,settings.relative_error*real(0.5));
            // Keep every air-mass knot. Its union axis is much larger than a
            // pressure axis, and reducing it is unnecessary for generation.
            result->generator_version=engine_map_generator_version(desc);
            result->content_hash=engine_maps_content_hash(*result);
            if(control && control->on_progress)control->on_progress(budget,budget);
            return result;
        }
        sampling.grid_p_im.insert(sampling.grid_p_im.end(),failing.begin(),failing.end());
        std::sort(sampling.grid_p_im.begin(),sampling.grid_p_im.end());
        current=run(sampling);
    }
    throw std::runtime_error("adaptive pressure sampling: pass budget exhausted before validation converged");
}

void validate_engine_map_generation(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality) {
    EngineMapGenTask task(desc, fuel, quality); // desc/fuel/grid/spark-schedule validation
    task.build_check();                          // point 0's build-time checks (pipe engines)
    // finish()'s friction-axis sign check (the correlation's own values do not depend on any point).
    if (desc.friction.model != FrictionModelDesc::Model::None) {
        real prev = real(0.0);
        const auto check = [&](real x) {
            if (!(cycle_friction_torque_nm(desc, x) >= real(0.0))) {
                throw std::runtime_error("engine map generator: the friction correlation is negative at " + std::to_string(x) +
                                         " rpm");
            }
        };
        check(real(0.0));
        for (real r : desc.grid_rpm) {
            check((prev + r) / real(2.0));
            check(r);
            prev = r;
        }
    }
}

std::uint64_t engine_maps_content_hash(const EngineMaps& em) {
    Fnv f;
    f.pod(em.generator_version);
    f.pod(static_cast<int>(em.quality));
    const SimulatedEngineMapsDesc& m = em.maps;
    f.table(m.ve);
    f.table(m.eta_i);
    f.curve(m.f_lambda);
    f.curve(m.eta_spark);
    f.table(m.theta_mbt_deg);
    f.curve(m.friction_torque_nm);
    f.pod(m.has_pmep_excess);
    if (m.has_pmep_excess) f.table(m.pmep_excess_pa);
    f.pod(m.has_theta_kl);
    if (m.has_theta_kl) f.table(m.theta_kl_deg);
    f.table(em.t_exh_k);
    f.table(em.h_exh_w);
    f.pod(em.eta_spark_spread);
    return f.h;
}

} // namespace ps::drivetrain
