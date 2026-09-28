// PHYS-033 P4 I7c C16a: cycle model -> runtime maps (ps/drivetrain/engine_maps.h).
//
// Extraction (every grid node, ISA intake state, lambda 1, MBT spark), with
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

#include "ps/drivetrain/isa_air.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
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

constexpr int kPreviewRpm = 10;
constexpr int kPreviewPim = 6;

constexpr real kSparkRetard[] = {-10.0, -5.0, 0.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 40.0};
constexpr int kSparkN = 10;
constexpr int kSparkRefIndex = 2; // retard 0
struct SweepSite {
    real rpm, p_pa;
};
constexpr SweepSite kSparkSites[] = {{2000.0, 50000.0}, {3000.0, 80000.0}, {5000.0, 100000.0}};
constexpr int kSparkSites_n = 3;
constexpr real kLambda[] = {0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.35, 1.5};
constexpr int kLambdaN = 9;
constexpr int kLambdaRefIndex = 4;
constexpr SweepSite kLambdaSite{3000.0, 80000.0};

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
    real lhv = 0.0, afr = 0.0;
    MapQuality quality = MapQuality::Full;
    engine_cycle::StepperSettings settings;
    std::unique_ptr<engine_cycle::CycleStepper> proto;
    std::vector<real> rpm_axis, p_axis;
    std::vector<engine_cycle::OperatingPoint> ops;
    std::vector<engine_cycle::PointResult> results;
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
};

EngineMapGenTask::EngineMapGenTask(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality)
    : impl_(std::make_unique<Impl>()) {
    Impl& s = *impl_;
    s.desc = desc;
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
    s.settings.h_s = quality == MapQuality::Full ? engine_cycle::kStepFull : engine_cycle::kStepPreview;
    s.settings.cycles = quality == MapQuality::Full ? engine_cycle::kCyclesFull : engine_cycle::kCyclesPreview;
    s.rpm_axis = quality == MapQuality::Full ? desc.grid_rpm : subsample(desc.grid_rpm, kPreviewRpm);
    s.p_axis = quality == MapQuality::Full ? desc.grid_p_im : subsample(desc.grid_p_im, kPreviewPim);
    s.mbt_atdc = mbt_spark_atdc(desc);

    auto make_op = [&](real rpm, real p, real lambda, real spark_atdc) {
        engine_cycle::OperatingPoint op;
        op.rpm = rpm;
        op.p_im_pa = p;
        op.t_im_k = kTref;
        op.p_amb_pa = kPamb;
        op.lambda = lambda;
        op.spark_deg_atdc = spark_atdc;
        op.intake_advance_deg = cam_value(desc.cam.intake_advance_deg, rpm, p);
        op.exhaust_retard_deg = cam_value(desc.cam.exhaust_retard_deg, rpm, p);
        return op;
    };
    for (real rpm : s.rpm_axis) {
        for (real p : s.p_axis) s.ops.push_back(make_op(rpm, p, real(1.0), s.mbt_atdc));
    }
    s.grid_n = s.ops.size();
    for (int k = 0; k < kSparkSites_n; ++k) {
        for (int j = 0; j < kSparkN; ++j) {
            s.ops.push_back(make_op(kSparkSites[k].rpm, kSparkSites[k].p_pa, real(1.0), s.mbt_atdc + kSparkRetard[j]));
        }
    }
    for (int j = 0; j < kLambdaN; ++j) s.ops.push_back(make_op(kLambdaSite.rpm, kLambdaSite.p_pa, kLambda[j], s.mbt_atdc));
    s.results.resize(s.ops.size());
    s.done.assign(s.ops.size(), 0);
    s.exceptions.resize(s.ops.size());
}

EngineMapGenTask::~EngineMapGenTask() = default;

std::size_t EngineMapGenTask::point_count() const { return impl_->ops.size(); }
std::size_t EngineMapGenTask::grid_point_count() const { return impl_->grid_n; }

void EngineMapGenTask::run_point(std::size_t i) {
    Impl& s = *impl_;
    if (i >= s.ops.size()) throw std::out_of_range("EngineMapGenTask::run_point: index out of range");
    try {
        s.results[i] = engine_cycle::run_operating_point(*s.proto, s.ops[i], s.settings);
        s.done[i] = 1;
    } catch (...) {
        // N5: never touch the caught exception here (TOOL-019); store it
        // unexamined and let finish() rethrow it on the (single, non-ASan-
        // arming) thread that calls finish().
        s.exceptions[i] = std::current_exception();
        s.done[i] = 2;
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
    em.generator_version = kGeneratorVersion;

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

    // f_lambda
    {
        const std::size_t base = s.grid_n + static_cast<std::size_t>(kSparkSites_n * kSparkN);
        auto g = [&](int j) {
            const engine_cycle::PointResult& r = s.results[base + static_cast<std::size_t>(j)];
            return r.w_gross_j / r.m_air_j_cyc * kLambda[j];
        };
        const real g_ref = g(kLambdaRefIndex);
        for (int j = 0; j < kLambdaN; ++j) {
            m.f_lambda.x.push_back(kLambda[j]);
            m.f_lambda.y.push_back(j == kLambdaRefIndex ? real(1.0) : g(j) / g_ref);
        }
    }
    // eta_spark: x = spark - MBT = -retard (ascending), mean of the three normalised curves
    {
        real norm[kSparkSites_n][kSparkN];
        for (int k = 0; k < kSparkSites_n; ++k) {
            auto g = [&](int j) {
                const engine_cycle::PointResult& r =
                    s.results[s.grid_n + static_cast<std::size_t>(k * kSparkN + j)];
                return r.w_gross_j / r.m_air_j_cyc;
            };
            const real g0 = g(kSparkRefIndex);
            for (int j = 0; j < kSparkN; ++j) norm[k][j] = j == kSparkRefIndex ? real(1.0) : g(j) / g0;
        }
        real spread = real(0.0);
        for (int j = kSparkN - 1; j >= 0; --j) {
            const real mean = (norm[0][j] + norm[1][j] + norm[2][j]) / real(3.0);
            for (int k = 0; k < kSparkSites_n; ++k) spread = std::max(spread, std::abs(norm[k][j] - mean));
            m.eta_spark.x.push_back(-kSparkRetard[j]);
            m.eta_spark.y.push_back(mean);
        }
        em.eta_spark_spread = spread;
    }
    // theta_MBT: constant (fixed Wiebe), kept as a map
    m.theta_mbt_deg.x = {s.rpm_axis.front(), s.rpm_axis.back()};
    m.theta_mbt_deg.y = {s.p_axis.front(), s.p_axis.back()};
    m.theta_mbt_deg.values.assign(4, -s.mbt_atdc);
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
    m.source = std::string("generated by the ps::engine_cycle map generator v") + std::to_string(kGeneratorVersion) +
               " (PHYS-033 P4 I7c C16a), quality " + (s.quality == MapQuality::Full ? "Full" : "Preview") +
               ", from the engine file's cycle block" +
               (s.desc.source.empty() ? std::string() : " (cycle source: " + s.desc.source + ")");
    em.content_hash = engine_maps_content_hash(em);
    return out;
}

EngineMapsPtr generate_engine_maps(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality,
                                   jobs::JobSystem* pool) {
    EngineMapGenTask task(desc, fuel, quality);
    const std::size_t n = task.point_count();
    if (pool == nullptr) {
        for (std::size_t i = 0; i < n; ++i) task.run_point(i);
    } else {
        pool->parallel_for(n, 1, [&task](std::size_t b, std::size_t e) {
            for (std::size_t i = b; i < e; ++i) task.run_point(i);
        });
    }
    return task.finish();
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
