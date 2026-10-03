// ps_audio/voice.cpp - see voice.h.
#include "ps_audio/voice.h"

#include "ps/engine_cycle/combustion_variation.h"
#include "ps/math/transcendental.h"

#include <cmath>
#include <stdexcept>

namespace ps_audio {

using ps::drivetrain::CycleModelDesc;

namespace {

constexpr real kPi = 3.14159265358979323846264338327950288;
constexpr real kRadToDeg = 180.0 / kPi;
constexpr real kDegToRad = kPi / 180.0;
constexpr real kTwoPi = 2.0 * kPi;

StabilityProbeResult resolve_substeps(const CycleModelDesc& desc, real lhv, real afr, const VoiceSettings& s) {
    if (s.substeps == 1 || s.substeps == 2 || s.substeps == 4 || s.substeps == 8 || s.substeps == 16) {
        StabilityProbeResult r;
        r.substeps = s.substeps;
        return r;
    }
    if (s.substeps != 0) throw std::invalid_argument("Voice: substeps must be 0 (probe), 1, 2, 4, 8 or 16");
    return probe_stability(desc, lhv, afr, s.engine, s.stability_bound);
}

LiveEngineSettings engine_settings(const VoiceSettings& s, int substeps) {
    LiveEngineSettings e = s.engine;
    e.substeps = substeps;
    return e;
}

// Signed angle difference wrapped to [-360, 360).
real wrap_pm360(real d) {
    while (d >= 360.0) d -= 720.0;
    while (d < -360.0) d += 720.0;
    return d;
}

real lerp(real a, real b, real f) { return a + (b - a) * f; }

}  // namespace

Voice::Voice(const CycleModelDesc& desc, real lhv_j_kg, real afr_st, const VoiceSettings& settings)
    : s_(settings),
      probe_(resolve_substeps(desc, lhv_j_kg, afr_st, settings)),
      engine_(desc, lhv_j_kg, afr_st, engine_settings(settings, probe_.substeps)),
      tm_(settings.time_map),
      fs_(settings.engine.sample_rate_hz),
      h_(1.0 / settings.engine.sample_rate_hz) {
    if (s_.control_interval < 1) throw std::invalid_argument("Voice: control_interval must be >= 1");
    if (!(s_.pause_fade_s > 0.0 && s_.crank_tau_s > 0.0 && s_.crank_max_rel >= 0.0))
        throw std::invalid_argument("Voice: pause_fade_s and crank_tau_s must be > 0, crank_max_rel >= 0");
    ctl_.p_intake_pa = s_.engine.initial_p_intake_pa;
    ctl_.start_of_burn_deg_atdc = engine_.mbt_start_of_burn_deg();
    // ps_audio A8: one independent white-noise stream per source, seeded
    // from the same LiveEngineSettings::seed combustion_variation.h uses -
    // deterministic, and a different seed (or source index) gives an
    // independent stream (SplitMix64's own avalanche, no shared state).
    for (int s = 0; s < engine_.source_count(); ++s) {
        flow_noise_[static_cast<std::size_t>(s)].rng =
            s_.engine.seed ^ (0xa24baed4963ee407ull * (static_cast<std::uint64_t>(s) + 1u));
    }
}

void Voice::flow_noise_control_point() {
    const FlowNoiseSettings& fs = s_.flow_noise;
    const int ns = engine_.source_count();
    for (int s = 0; s < ns; ++s) {
        FlowNoiseState& fn = flow_noise_[static_cast<std::size_t>(s)];
        const real area = engine_.source(s).area_m2;
        if (!(area > 0.0)) {
            fn.gain_pa = 0.0;
            continue;
        }
        const real q = engine_.core().radiating_outflow(s);
        const real u = std::fabs(q) / area;
        fn.gain_pa = fs.level_coeff_pa * ps::math::pow(u, fs.level_exponent);
        if (!(fn.gain_pa > 0.0)) {
            fn.gain_pa = 0.0;
            continue;
        }
        const real d = std::sqrt(4.0 * area / kPi);  // hydraulic diameter of a circular outlet
        real f_centre = d > 0.0 ? fs.strouhal * u / d : fs.min_hz;
        if (f_centre < fs.min_hz) f_centre = fs.min_hz;
        if (f_centre > fs.max_hz) f_centre = fs.max_hz;
        const real nyq_guard = 0.45 / h_;  // stay well clear of Nyquist
        real f_lo = 0.5 * f_centre;
        real f_hi = 2.0 * f_centre;
        if (f_lo < fs.min_hz) f_lo = fs.min_hz;
        if (f_hi > fs.max_hz) f_hi = fs.max_hz;
        if (f_hi > nyq_guard) f_hi = nyq_guard;
        if (f_hi <= f_lo) f_hi = f_lo * 1.5;
        fn.a_hp = ps::math::exp(-kTwoPi * f_lo * h_);
        fn.a_lp = ps::math::exp(-kTwoPi * f_hi * h_);
    }
}

real Voice::flow_noise_sample(int s) {
    FlowNoiseState& fn = flow_noise_[static_cast<std::size_t>(s)];
    if (!(fn.gain_pa > 0.0)) return 0.0;
    const std::uint64_t bits = ps::engine_cycle::splitmix64_next(fn.rng);
    const real x = 2.0 * ps::engine_cycle::unit_uniform(bits) - 1.0;
    // One-pole highpass (leaky differentiator) then one-pole lowpass: a
    // bandpass whose pole magnitudes are always in (0, 1) by construction
    // (a plain exponential of a non-negative rate), so it stays stable
    // across a control point's abrupt coefficient update - unlike a
    // resonant (high-Q) biquad, which a fast Strouhal-frequency swing could
    // ring or overshoot.
    const real hp = fn.a_hp * (fn.hp_y1 + x - fn.hp_x1);
    fn.hp_x1 = x;
    fn.hp_y1 = hp;
    const real lp = fn.a_lp * fn.lp_y1 + (1.0 - fn.a_lp) * hp;
    fn.lp_y1 = lp;
    real pmod = 1.0 + s_.flow_noise.pressure_mod * (frame_[static_cast<std::size_t>(s)] / s_.flow_noise.pressure_ref_pa);
    if (pmod < 0.0) pmod = 0.0;
    if (pmod > 2.0) pmod = 2.0;
    return fn.gain_pa * pmod * lp;
}

void Voice::control_point() {
    const real t_audio = static_cast<real>(n_) * h_;
    EngineInput in;
    while (queue_.pop(in)) {
        if (hist_n_ > 0 && !(in.t_sim_s > hist_[static_cast<std::size_t>(hist_n_ - 1)].t_sim_s)) continue;
        if (hist_n_ == kHistory) {
            for (int i = 1; i < kHistory; ++i) hist_[static_cast<std::size_t>(i - 1)] = hist_[static_cast<std::size_t>(i)];
            --hist_n_;
            if (lower_ >= 0) --lower_;
        }
        hist_[static_cast<std::size_t>(hist_n_++)] = in;
        if (frozen_) tm_.on_arrival_paused(in.t_sim_s, t_audio);
        else tm_.on_arrival(in.t_sim_s, t_audio);
    }
    const bool want_pause = pause_req_.load(std::memory_order_acquire);
    if (want_pause && !pausing_) pausing_ = true;
    if (!want_pause && pausing_) {
        pausing_ = false;
        if (frozen_) {
            frozen_ = false;
            tm_.reset_to_newest();
            target_valid_ = false;
        }
    }
}

void Voice::interpolate() {
    if (hist_n_ == 0) return;  // no snapshot yet: the engine stays at its start inputs
    const real t = tm_.read_time();
    int lo = 0;
    while (lo + 1 < hist_n_ && hist_[static_cast<std::size_t>(lo + 1)].t_sim_s <= t) ++lo;
    const EngineInput& a = hist_[static_cast<std::size_t>(lo)];
    const bool has_b = lo + 1 < hist_n_ && t > a.t_sim_s;
    const EngineInput& b = has_b ? hist_[static_cast<std::size_t>(lo + 1)] : a;
    const real f = has_b ? (t - a.t_sim_s) / (b.t_sim_s - a.t_sim_s) : 0.0;
    ctl_.omega_rad_s = lerp(a.omega_rad_s, b.omega_rad_s, f);
    ctl_.p_intake_pa = (a.p_intake_pa > 0.0 && b.p_intake_pa > 0.0) ? lerp(a.p_intake_pa, b.p_intake_pa, f) : a.p_intake_pa;
    ctl_.p_charge_pa=lerp(a.p_charge_pa,b.p_charge_pa,f);
    ctl_.t_charge_k=lerp(a.t_charge_k,b.t_charge_k,f);
    ctl_.throttle = lerp(a.throttle, b.throttle, f);
    ctl_.lambda = lerp(a.lambda, b.lambda, f);
    if (a.spark_given) {
        const real sp = b.spark_given ? lerp(a.spark_deg_atdc, b.spark_deg_atdc, f) : a.spark_deg_atdc;
        ctl_.start_of_burn_deg_atdc = sp + engine_.ignition_delay_deg();
    } else {
        ctl_.start_of_burn_deg_atdc = engine_.mbt_start_of_burn_deg();
    }
    ctl_.flags = a.flags;
    // Crank target: re-seated whenever the read point crosses a snapshot carrying a crank angle.
    if (a.t_sim_s != lower_t_) {
        lower_t_ = a.t_sim_s;
        lower_ = lo;
        if (a.crank_angle_deg >= 0.0 && t >= a.t_sim_s) {
            target_theta_ = a.crank_angle_deg + a.omega_rad_s * kRadToDeg * (t - a.t_sim_s);
            target_valid_ = true;
        } else if (a.crank_angle_deg < 0.0) {
            target_valid_ = false;
        }
    }
}

void Voice::set_audio_bank(AudioBankPtr bank) {
    if(!bank || bank->channels!=source_count()) throw std::invalid_argument("Audio bank topology mismatch");
    retained_banks_.push_back(bank);
    requested_bank_.store(std::move(bank),std::memory_order_release);
}

void Voice::render(float* out, int frames) {
    const int ns = engine_.source_count();
    const std::uint64_t ci = static_cast<std::uint64_t>(s_.control_interval);
    const real fade_step = h_ / s_.pause_fade_s;
    const bool flow_noise_on = s_.flow_noise.enabled && s_.flow_noise.level_coeff_pa > 0.0;
    const auto requested=requested_bank_.load(std::memory_order_acquire);
    if(requested && requested!=active_bank_) {
        previous_bank_=active_bank_;previous_playback_=bank_playback_;
        active_bank_=requested;bank_playback_=BankPlayback{};bank_fade_=real(0.0);
    }
    for (int f = 0; f < frames; ++f) {
        const bool cp = n_ % ci == 0;
        if (cp) control_point();
        float* o = out + static_cast<std::size_t>(f) * static_cast<std::size_t>(ns);
        if (frozen_) {
            for (int s = 0; s < ns; ++s) o[s] = 0.0f;
            ++n_;
            continue;
        }
        const real rate = tm_.rate();
        tm_.advance(h_);
        interpolate();

        // Crank advance: omega (pitch), plus the capped phase correction while locked.
        real omega = ctl_.omega_rad_s > 0.0 ? ctl_.omega_rad_s : 0.0;
        if (target_valid_) target_theta_ += omega * kRadToDeg * rate * h_;
        if (target_valid_ && omega > 0.0 && tm_.locked()) {
            const real err = wrap_pm360(target_theta_ - engine_.crank_deg());
            real corr = err / s_.crank_tau_s;               // deg/s
            const real cap = s_.crank_max_rel * omega * kRadToDeg;
            if (corr > cap) corr = cap;
            if (corr < -cap) corr = -cap;
            omega += corr * kDegToRad;
        }
        omega_applied_ = omega;
        LiveControl c = ctl_;
        c.omega_rad_s = omega;
        c.omega_input_rad_s = ctl_.omega_rad_s > 0.0 ? ctl_.omega_rad_s : 0.0;
        engine_.step_sample(c, frame_.data());
        if(active_bank_) {
            std::array<float,kLiveMaxSources> current{},previous{};
            const real rpm=omega*real(60.0/6.2831853071795864769);
            bank_playback_.frame(*active_bank_,rpm,c.p_intake_pa,current.data());
            if(previous_bank_) previous_playback_.frame(*previous_bank_,rpm,c.p_intake_pa,previous.data());
            bank_fade_=std::min(real(1.0),bank_fade_+real(1.0)/(fs_*real(0.12)));
            const real wanted=c.flags==0?real(1.0):real(0.0);
            bank_weight_+=(wanted-bank_weight_)/(fs_*real(0.01));
            for(int source=0;source<ns;++source) {
                const real before=previous_bank_?previous[static_cast<std::size_t>(source)]:frame_[static_cast<std::size_t>(source)];
                const real baked=before*(real(1.0)-bank_fade_)+current[static_cast<std::size_t>(source)]*bank_fade_;
                frame_[static_cast<std::size_t>(source)]=static_cast<float>(frame_[static_cast<std::size_t>(source)]*(real(1.0)-bank_weight_)+baked*bank_weight_);
            }
        }
        if (flow_noise_on && cp) flow_noise_control_point();

        if (pausing_) {
            gain_ -= fade_step;
            if (gain_ <= 0.0) {
                gain_ = 0.0;
                frozen_ = true;
            }
        } else if (gain_ < 1.0) {
            gain_ += fade_step;
            if (gain_ > 1.0) gain_ = 1.0;
        }
        for (int s = 0; s < ns; ++s) {
            real v = frame_[static_cast<std::size_t>(s)];
            if (flow_noise_on) v += flow_noise_sample(s);
            o[s] = static_cast<float>(gain_ * v);
        }
        ++n_;
    }
}

std::vector<float> render_to_buffer(Voice& voice, std::size_t frames) {
    const std::size_t ns = static_cast<std::size_t>(voice.source_count());
    std::vector<float> out(frames * ns, 0.0f);
    constexpr std::size_t kChunk = 256;
    std::size_t done = 0;
    while (done < frames) {
        const std::size_t n = frames - done < kChunk ? frames - done : kChunk;
        voice.render(out.data() + done * ns, static_cast<int>(n));
        done += n;
    }
    return out;
}

}  // namespace ps_audio
