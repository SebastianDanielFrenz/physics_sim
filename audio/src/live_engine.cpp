// ps_audio/live_engine.cpp - see live_engine.h.
#include "ps_audio/live_engine.h"

#include "ps/engine_cycle/wiebe.h"
#include "ps_audio/engine_input.h"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ps_audio {

using ps::drivetrain::CycleModelDesc;
using namespace ps::engine_cycle;

namespace {

constexpr real kPi = 3.14159265358979323846264338327950288;
constexpr real kRadToRpm = 60.0 / (2.0 * kPi);

[[noreturn]] void fail(const std::string& what) { throw std::invalid_argument("LiveEngine: " + what); }

real wrap720(real deg) {
    real r = deg - 720.0 * std::floor(deg / 720.0);
    if (r >= 720.0) r -= 720.0;
    return r;
}

const LiveEngineSettings& checked(const LiveEngineSettings& s) {
    if (s.substeps != 1 && s.substeps != 2 && s.substeps != 4 && s.substeps != 8 && s.substeps != 16)
        fail("substeps must be 1, 2, 4, 8 or 16");
    if (!(s.sample_rate_hz > 0.0)) fail("sample_rate_hz must be > 0");
    if (!(s.p_amb_pa > 0.0 && s.t_amb_k > 0.0)) fail("ambient pressure and temperature must be > 0");
    if (!(s.initial_p_intake_pa > 0.0)) fail("initial_p_intake_pa must be > 0");
    if (!(s.plenum_filter_tau_s > 0.0 && s.plenum_pi_tau_s > 0.0)) fail("plenum loop time constants must be > 0");
    if (!(s.min_throttle_cd_area_m2 > 0.0)) fail("min_throttle_cd_area_m2 must be > 0");
    if (s.threads < 1) fail("threads must be >= 1");
    return s;
}

OperatingPoint start_op(const LiveEngineSettings& s) {
    OperatingPoint op;
    op.rpm = s.initial_rpm > 0.0 ? s.initial_rpm : 0.0;
    op.p_im_pa = s.initial_p_intake_pa;
    op.t_im_k = s.t_amb_k;
    op.p_amb_pa = s.p_amb_pa;
    op.lambda = 1.0;
    return op;
}

EngineCoreSettings core_settings(const CycleModelDesc& d, const LiveEngineSettings& s) {
    EngineCoreSettings c;
    c.dx_intake_m = s.dx_intake_m > 0.0 ? s.dx_intake_m : d.resolution.audio_dx_intake_m;
    c.dx_exhaust_m = s.dx_exhaust_m > 0.0 ? s.dx_exhaust_m : d.resolution.audio_dx_exhaust_m;
    // ps_audio A8 (F3): only the audio-path live engine's FV Open ends get
    // the radiation loss - the map generator (CycleEngine) never sets this,
    // so map content/hashes are unaffected.
    c.radiation = true;
    c.threads = s.threads;
    return c;
}

}  // namespace

LiveEngine::LiveEngine(const CycleModelDesc& desc, real lhv_j_kg, real afr_st, const LiveEngineSettings& settings)
    : settings_(checked(settings)),
      core_(desc, lhv_j_kg, afr_st, start_op(settings), 1.0 / (settings.sample_rate_hz * settings.substeps),
            core_settings(desc, settings)),
      h_(1.0 / (settings.sample_rate_hz * settings.substeps)) {
    for (int n = settings_.substeps; n > 1; n /= 2) ++stages_;
    sources_ = static_cast<int>(core_.radiating_ends().size());
    if (sources_ < 1) fail("the engine network has no radiating end");
    if (sources_ > kLiveMaxSources) fail("more than " + std::to_string(kLiveMaxSources) + " radiating ends");
    rho_amb_ = settings_.p_amb_pa / (core_.cylinder(0).model().gas.fresh.r * settings_.t_amb_k);
    p_f_ = settings_.initial_p_intake_pa;
    ign_delay_ = desc.ignition_delay_deg;
    burn_deg_ = desc.burn_duration_deg;
    sob_mbt_ = Wiebe::theta0_for_ca50(desc.wiebe_a, desc.wiebe_m, desc.burn_duration_deg, desc.ca50_mbt_deg_atdc);
    const std::size_t nc = static_cast<std::size_t>(core_.cylinders());
    fresh_acc_prev_.assign(nc, 0.0);
    fresh_last_.assign(nc, 0.0);
    events_.assign(nc, 0);
    firing_.assign(static_cast<std::size_t>(kFiringLog), FiringEvent{});
    if (settings_.min_throttle_cd_area_m2 > core_.throttle_cd_area_max())
        fail("min_throttle_cd_area_m2 exceeds the throttle's maximum Cd*A");
    // First combustion event (charges trapped before the first IVC seen here).
    for (int c = 0; c < core_.cylinders(); ++c)
        core_.cylinder(c).set_combustion_event(1.0, sob_mbt_, burn_deg_, true);
}

real LiveEngine::plenum_pressure_pa() const {
    const PipeNetwork* net = core_.network();
    return net->volume_state(core_.layout().plenum_volume).p;
}

void LiveEngine::physics_step(const LiveControl& c, real* p_out) {
    const real omega = c.omega_rad_s > 0.0 ? c.omega_rad_s : 0.0;
    const real rpm = omega * kRadToRpm;
    const real dtheta = 6.0 * rpm * h_;
    const int nc = core_.cylinders();

    if(auto* net=core_.network()) {
        const auto& layout=core_.layout();
        net->set_orifice_supply(layout.throttle_orifice,
            ps::engine_cycle::GasStateDesc{c.p_charge_pa,c.t_charge_k,real(0.0)});
    }
    // Plenum loop (throttle Cd*A), then the D-3 port <-> pipe exchange.
    if (c.p_intake_pa > 0.0) {
        if (settings_.plenum_loop) {
            // Pure backstop (ps_audio A8b): these isfinite holds keep a
            // non-finite network reading from reaching set_orifice_cd_area's
            // invariant check. A10 added them for a NaN that A8b traced to a
            // runtime CFL violation in an intake runner (a supersonic backflow
            // jet at overlap under the overrun vacuum), fixed at the source by
            // PipeNetwork's CFL sub-cycling - with it the network stays finite
            // (test_engine_rig_audio.cpp "A8b B1", test_pipe_network.cpp P9),
            // so they did not engage in those runs; they are not part of the model.
            const real p_net = plenum_pressure_pa();
            if (std::isfinite(p_net)) p_f_ += h_ / settings_.plenum_filter_tau_s * (p_net - p_f_);
            if (!std::isfinite(p_f_)) p_f_ = settings_.initial_p_intake_pa;
            // The integral runs only while the crank turns: a stopped engine
            // pumps nothing, and winding the plate shut against the slow refill
            // of the plenum left a self-sustained intake tone (V13 diag).
            if (omega > 0.0) {
                const real e = (c.p_intake_pa - p_f_) / c.p_intake_pa;
                real cda = core_.throttle_cd_area() * (1.0 + h_ / settings_.plenum_pi_tau_s * e);
                if (!std::isfinite(cda)) cda = core_.throttle_cd_area();
                if (cda < settings_.min_throttle_cd_area_m2) cda = settings_.min_throttle_cd_area_m2;
                if (cda > core_.throttle_cd_area_max()) cda = core_.throttle_cd_area_max();
                core_.set_throttle_cd_area(cda);
            }
        }
    } else {
        const real th = c.throttle < 0.0 ? 0.0 : (c.throttle > 1.0 ? 1.0 : c.throttle);
        const real lo = settings_.min_throttle_cd_area_m2;
        core_.set_throttle_cd_area(lo + th * (core_.throttle_cd_area_max() - lo));
    }
    core_.exchange();

    const bool fuel_off = (c.flags & (kInputFuelCut | kInputDfco)) != 0u;
    const bool spark_off = (c.flags & kInputIgnitionCut) != 0u;
    const real load = p_f_ / settings_.p_amb_pa;
    // A9b: three passes. Pass 1 (serial, engine-wide): speed and the ignition-cut
    // charge reset. Pass 2: the live step, one cylinder per worker - cylinders
    // are independent within a step and step_live keeps all state per cylinder,
    // so any thread count is bit-identical. Pass 3 (serial, in cylinder order):
    // IVC and firing-TDC bookkeeping into shared per-engine state.
    for (int k = 0; k < nc; ++k) {
        CycleStepper& cyl = core_.cylinder(k);
        cyl.set_speed(rpm);
        if (dtheta > 0.0) {
            // Ignition cut: a charge whose start of burn falls in this step does not burn.
            const real th0 = wrap720(theta_ - core_.phase_deg(k));
            const real d_sob = wrap720(cyl.wiebe().theta0_deg - th0);
            if (spark_off && d_sob > 0.0 && d_sob <= dtheta && cyl.m_fuel() > 0.0) {
                cyl.set_charge(cyl.m_air_trapped(), 0.0);
                ++cuts_;
            }
        }
    }
    {
        struct CylinderJob final : StepJob {
            LiveEngine& eng;
            int nc;
            real dtheta;
            CylinderJob(LiveEngine& e, int n, real d) : eng(e), nc(n), dtheta(d) {}
            void run(int worker, int workers) override {
                for (int k = worker; k < nc; k += workers) {
                    const real th0 = wrap720(eng.theta_ - eng.core_.phase_deg(k));
                    eng.core_.cylinder(k).step_live(th0, dtheta, eng.h_);
                }
            }
        };
        StepTeam* team = core_.step_team();
        if (team != nullptr && team->size() > 1) {
            CylinderJob job(*this, nc, dtheta);
            team->run(job);
        } else {
            for (int k = 0; k < nc; ++k) {
                const real th0 = wrap720(theta_ - core_.phase_deg(k));
                core_.cylinder(k).step_live(th0, dtheta, h_);
            }
        }
    }
    if (dtheta > 0.0) {
        for (int k = 0; k < nc; ++k) {
            CycleStepper& cyl = core_.cylinder(k);
            const std::size_t kk = static_cast<std::size_t>(k);
            const real th0 = wrap720(theta_ - core_.phase_deg(k));
            const real d_ivc = wrap720(cyl.theta_ivc() - th0);
            if (d_ivc > 0.0 && d_ivc <= dtheta) {
                // IVC: close the cycle's fresh-air account and set the next combustion event.
                const real acc = cyl.state().y[kAccIvFresh];
                fresh_last_[kk] = acc - fresh_acc_prev_[kk];
                fresh_acc_prev_[kk] = acc;
                const CombustionPerturbation pert =
                    combustion_variation(settings_.variation, settings_.seed, k, events_[kk], load,
                                         settings_.variation_scale);
                ++events_[kk];
                cyl.set_combustion_event(c.lambda, c.start_of_burn_deg_atdc + pert.start_shift_deg,
                                         burn_deg_ * pert.burn_scale, !fuel_off);
                if (fuel_off) ++cuts_;
            }
            const real d_tdc = wrap720(360.0 - th0);
            if (d_tdc > 0.0 && d_tdc <= dtheta) {
                FiringEvent& ev = firing_[static_cast<std::size_t>(firing_count_ % kFiringLog)];
                ev.t_s = t_ + h_ * (d_tdc / dtheta);
                ev.cylinder = k;
                ev.omega_rad_s = c.omega_input_rad_s >= 0.0 ? c.omega_input_rad_s : omega;
                ++firing_count_;
            }
        }
    }
    theta_ = wrap720(theta_ + dtheta);
    t_ += h_;

    // PROVISIONAL monopole radiation at 1 m (A8 replaces it).
    const real g = rho_amb_ / (4.0 * kPi) / h_;
    for (int s = 0; s < sources_; ++s) {
        const std::size_t ss = static_cast<std::size_t>(s);
        const real q = core_.radiating_outflow(s);
        p_out[s] = g * (q - q_prev_[ss]);
        q_prev_[ss] = q;
    }
}

void LiveEngine::step_sample(const LiveControl& c, real* out) {
    if (settings_.substeps == 1) {
        physics_step(c, out);
        return;
    }
    // Substeps in time order, then each half-band stage halves the count in
    // place (substeps 2 = the single-stage push2 of the A7 design, unchanged).
    int n = settings_.substeps;
    for (int k = 0; k < n; ++k) physics_step(c, sub_[static_cast<std::size_t>(k)].data());
    for (int st = 0; st < stages_; ++st) {
        n /= 2;
        for (int j = 0; j < n; ++j) {
            const std::size_t j0 = static_cast<std::size_t>(2 * j);
            for (int s = 0; s < sources_; ++s) {
                const std::size_t ss = static_cast<std::size_t>(s);
                sub_[static_cast<std::size_t>(j)][ss] =
                    dec_[ss][static_cast<std::size_t>(st)].push2(sub_[j0][ss], sub_[j0 + 1][ss]);
            }
        }
    }
    for (int s = 0; s < sources_; ++s) out[s] = sub_[0][static_cast<std::size_t>(s)];
}

std::uint64_t LiveEngine::fold_state_hash(std::uint64_t h) const {
    h = core_.fold_state_hash(h);
    const auto fold = [&h](real v) {
        unsigned char b[sizeof(real)];
        std::memcpy(b, &v, sizeof(real));
        for (unsigned char x : b) { h ^= x; h *= 1099511628211ull; }
    };
    fold(theta_);
    fold(t_);
    fold(p_f_);
    for (int s = 0; s < sources_; ++s) fold(q_prev_[static_cast<std::size_t>(s)]);
    return h;
}

StabilityProbeResult probe_stability(const CycleModelDesc& desc, real lhv_j_kg, real afr_st,
                                     const LiveEngineSettings& base, real bound) {
    StabilityProbeResult r;
    r.rpm = 1000.0;
    r.p_im_pa = 30000.0;
    if (!desc.grid_rpm.empty()) {
        r.rpm = desc.grid_rpm.front();
        for (real v : desc.grid_rpm) if (v < r.rpm) r.rpm = v;
    }
    if (!desc.grid_p_im.empty()) {
        r.p_im_pa = desc.grid_p_im.front();
        for (real v : desc.grid_p_im) if (v < r.p_im_pa) r.p_im_pa = v;
    }
    real air[2] = {0.0, 0.0};
    for (int run = 0; run < 2; ++run) {
        LiveEngineSettings s = base;
        s.substeps = run + 1;
        s.initial_rpm = r.rpm;
        s.initial_p_intake_pa = r.p_im_pa;
        s.plenum_loop = false;
        s.variation_scale = 0.0;
        LiveEngine eng(desc, lhv_j_kg, afr_st, s);
        LiveControl c;
        c.omega_rad_s = r.rpm / kRadToRpm;
        c.p_intake_pa = r.p_im_pa;
        c.start_of_burn_deg_atdc = eng.mbt_start_of_burn_deg();
        const real cycle_s = 120.0 / r.rpm;
        const std::int64_t n = static_cast<std::int64_t>(std::ceil(2.0 * cycle_s * s.sample_rate_hz));
        std::array<real, kLiveMaxSources> out{};
        for (std::int64_t i = 0; i < n; ++i) eng.step_sample(c, out.data());
        real sum = 0.0;
        for (int k = 0; k < eng.cylinders(); ++k) sum += eng.last_fresh_air_kg(k);
        air[run] = sum;
        if (!std::isfinite(sum) || !std::isfinite(eng.plenum_pressure_pa())) r.finite = false;
    }
    r.air_h_kg = air[0];
    r.air_h2_kg = air[1];
    r.rel_diff = air[1] != 0.0 ? std::fabs(air[0] - air[1]) / std::fabs(air[1]) : 1.0;
    r.substeps = (!r.finite || !(r.rel_diff <= bound)) ? 2 : 1;
    return r;
}

}  // namespace ps_audio
