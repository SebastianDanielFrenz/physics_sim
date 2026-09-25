#include "haptics_core.h"

#include <algorithm>
#include <cmath>

namespace ps_godot {

namespace {
constexpr float kPi = 3.14159265358979323846f;

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// 0 at/below t0, 1 at/above t1, linear ramp between. t1 <= t0 degenerates
// to a hard step at t0 (guarded, never divides by zero/negative).
float threshold_saturate(float x, float t0, float t1) {
    if (x <= t0) return 0.0f;
    if (t1 <= t0) return x > t0 ? 1.0f : 0.0f;
    if (x >= t1) return 1.0f;
    return (x - t0) / (t1 - t0);
}

// Speed fade for the slip sources: 1 when the speed is unknown (< 0),
// otherwise a linear ramp from lo to hi (see HapticsWheelInput).
float speed_fade(float speed_mps, float lo, float hi) {
    if (speed_mps < 0.0f) return 1.0f;
    return threshold_saturate(speed_mps, lo, hi);
}

// Exponential approach toward `target` over time constant `tau` (s).
float exp_decay_toward(float current, float target, float dt, float tau) {
    if (tau <= 0.0f || dt <= 0.0f) return target;
    const float alpha = 1.0f - std::exp(-dt / tau);
    return current + (target - current) * std::min(alpha, 1.0f);
}

// Stage index with hysteresis (HapticsTuning::stage_up_thresholds/
// stage_hysteresis - see haptics_core.h's own comment on the scheme).
int compute_stage(float raw, int prev_stage, const HapticsTuning& t) {
    int stage = std::clamp(prev_stage, 0, static_cast<int>(t.stage_levels.size()) - 1);
    while (stage < static_cast<int>(t.stage_up_thresholds.size()) && raw >= t.stage_up_thresholds[static_cast<std::size_t>(stage)]) {
        ++stage;
    }
    while (stage > 0) {
        const float down_threshold = t.stage_up_thresholds[static_cast<std::size_t>(stage - 1)] - t.stage_hysteresis;
        if (raw < down_threshold) {
            --stage;
        } else {
            break;
        }
    }
    return stage;
}
} // namespace

float HapticsCore::Highpass1::step(float x, float dt, float fc) {
    if (!init) {
        prev_x = x;
        prev_y = 0.0f;
        init = true;
        return 0.0f;
    }
    if (dt <= 0.0f || fc <= 0.0f) {
        prev_x = x;
        return prev_y;
    }
    const float rc = 1.0f / (2.0f * kPi * fc);
    const float alpha = rc / (rc + dt);
    const float y = alpha * (prev_y + x - prev_x);
    prev_x = x;
    prev_y = y;
    return y;
}

void HapticsCore::reset() {
    debug_ = HapticsSourceDebug{};
    accel_hp_x_ = Highpass1{};
    accel_hp_y_ = Highpass1{};
    accel_hp_z_ = Highpass1{};
    wheel_filters_.fill(WheelFilter{});
    landing_pulse_level_ = 0.0f;
    prev_in_contact_.fill(false);
    prev_contact_init_ = false;
    engine_grind_level_ = 0.0f;
    engine_stall_level_ = 0.0f;
    prev_stalled_ = false;
    weak_stage_ = 0;
    strong_stage_ = 0;
    weak_smoothed_ = 0.0f;
    strong_smoothed_ = 0.0f;
}

HapticsOutput HapticsCore::update(const HapticsInput& in) {
    const float dt = in.dt_s;
    const int wheel_count = std::clamp(in.wheel_count, 0, kMaxHapticsWheels);

    // --- Source 1: body-G TRANSIENT (STRONG) - vector high-pass, then magnitude ---
    const float hp_x = accel_hp_x_.step(in.accel_x, dt, tuning_.body_g_highpass_fc_hz);
    const float hp_y = accel_hp_y_.step(in.accel_y, dt, tuning_.body_g_highpass_fc_hz);
    const float hp_z = accel_hp_z_.step(in.accel_z, dt, tuning_.body_g_highpass_fc_hz);
    const float hp_mag = std::sqrt(hp_x * hp_x + hp_y * hp_y + hp_z * hp_z);
    const float body_g_intensity = threshold_saturate(hp_mag, tuning_.body_g_threshold_mps2, tuning_.body_g_saturate_mps2);

    // --- Per-wheel pass: load noise (source 3), light slip (source 4),
    //     sustained grip-limit (source 2), landing edge detection (source 5) ---
    float total_contact_load = 0.0f;
    for (int i = 0; i < wheel_count; ++i) {
        if (in.wheels[static_cast<std::size_t>(i)].in_contact) {
            total_contact_load += std::max(0.0f, in.wheels[static_cast<std::size_t>(i)].normal_load_n);
        }
    }
    const float load_denom = std::max(total_contact_load, 1.0f);

    float load_noise_intensity = 0.0f;   // source 3, WEAK - dominant wheel (max), a single kerb tap is enough
    float load_noise_rms_raw = 0.0f;     // source 3 debug - dominant wheel's pre-threshold RMS(load_rel)
    float light_slip_intensity = 0.0f;   // source 4, WEAK - load-share-weighted sum
    float grip_limit_intensity = 0.0f;   // source 2, STRONG (sustained) - load-share-weighted sum
    float surface_texture_intensity = 0.0f; // source 7, WEAK - load-share-weighted sum
    bool landing_edge = false;

    for (int i = 0; i < wheel_count; ++i) {
        const HapticsWheelInput& w = in.wheels[static_cast<std::size_t>(i)];
        WheelFilter& wf = wheel_filters_[static_cast<std::size_t>(i)];

        // Source 3: high-pass of Fz relative to a slowly-tracked "static"
        // load, RMS'd over a short window.
        const float hp_fz = wf.load_hp.step(w.normal_load_n, dt, tuning_.load_highpass_fc_hz);
        if (!wf.init) {
            wf.static_load = w.normal_load_n;
            wf.init = true;
        } else {
            wf.static_load = exp_decay_toward(wf.static_load, w.normal_load_n, dt, tuning_.load_static_tau_s);
        }
        const float load_rel = hp_fz / std::max(wf.static_load, 200.0f);
        {
            const float window = std::max(tuning_.load_rms_window_s, 1e-3f);
            const float alpha = dt > 0.0f ? std::min(dt / window, 1.0f) : 0.0f;
            wf.ac_ms2 += (load_rel * load_rel - wf.ac_ms2) * alpha;
        }
        const float rms = std::sqrt(std::max(wf.ac_ms2, 0.0f));
        load_noise_rms_raw = std::max(load_noise_rms_raw, rms);
        // Fades in with ground speed (owner bug, 2026-09-25: a launch's own
        // load-transfer transient was maxing this source out under ~1 m/s -
        // see this file's header comment on source 3).
        const float load_noise_fade = speed_fade(w.ground_speed_mps, tuning_.load_noise_speed_fade_lo_mps, tuning_.load_noise_speed_fade_hi_mps);
        const float wheel_load_noise = load_noise_fade * threshold_saturate(rms, tuning_.load_noise_threshold, tuning_.load_noise_saturate);
        load_noise_intensity = std::max(load_noise_intensity, wheel_load_noise);

        const float load_weight = w.in_contact ? (std::max(0.0f, w.normal_load_n) / load_denom) : 0.0f;

        // Source 7: surface texture / rumble strip - exact name match against
        // the tuning table, fades in with ground speed the same way, weighted
        // by this wheel's load share (this file's header comment, source 7).
        if (w.in_contact) {
            for (const SurfaceTextureRule& rule : tuning_.surface_texture_rules) {
                if (rule.name[0] == '\0') continue;
                if (rule.name != w.surface_name) continue;
                const float texture_fade = speed_fade(w.ground_speed_mps, tuning_.surface_texture_speed_fade_lo_mps, tuning_.surface_texture_speed_fade_hi_mps);
                surface_texture_intensity += rule.strength * texture_fade * load_weight;
                break;
            }
        }

        // Slip means sliding only while something moves over the ground
        // (HapticsWheelInput::ground_speed_mps).
        const float ratio_fade = speed_fade(std::max(w.ground_speed_mps, w.wheel_surface_speed_mps),
                                            tuning_.slip_speed_fade_lo_mps, tuning_.slip_speed_fade_hi_mps);
        const float angle_fade = speed_fade(w.ground_speed_mps, tuning_.slip_speed_fade_lo_mps, tuning_.slip_speed_fade_hi_mps);

        // Source 4: light slip buzz, well below the real grip-limit peak.
        const float ratio_light = ratio_fade * threshold_saturate(std::fabs(w.slip_ratio), tuning_.slip_ratio_threshold, tuning_.slip_ratio_saturate);
        const float angle_light = angle_fade * threshold_saturate(std::fabs(w.slip_angle_rad), tuning_.slip_angle_threshold_rad, tuning_.slip_angle_saturate_rad);
        const float wheel_light_slip = w.in_contact ? std::max(ratio_light, angle_light) : 0.0f;
        light_slip_intensity += wheel_light_slip * load_weight;

        // Source 2: sustained grip-limit - utilisation = |slip| / peak,
        // whichever slip mode is further past its own peak dominates
        // (combined-slip-style "whichever runs out first").
        float wheel_grip_limit = 0.0f;
        if (w.in_contact) {
            const float ratio_u = tuning_.peak_slip_ratio > 0.0f ? std::fabs(w.slip_ratio) / tuning_.peak_slip_ratio : 0.0f;
            const float angle_u = tuning_.peak_slip_angle_rad > 0.0f ? std::fabs(w.slip_angle_rad) / tuning_.peak_slip_angle_rad : 0.0f;
            wheel_grip_limit = std::max(
                ratio_fade * threshold_saturate(ratio_u, 1.0f, tuning_.grip_limit_saturate_utilization),
                angle_fade * threshold_saturate(angle_u, 1.0f, tuning_.grip_limit_saturate_utilization));
        }
        grip_limit_intensity += wheel_grip_limit * load_weight;

        // Source 5: landing edge (any wheel, no-contact -> contact).
        const bool prev_contact = prev_contact_init_ ? prev_in_contact_[static_cast<std::size_t>(i)] : w.in_contact;
        if (!prev_contact && w.in_contact) landing_edge = true;
        prev_in_contact_[static_cast<std::size_t>(i)] = w.in_contact;
    }
    prev_contact_init_ = true;
    light_slip_intensity = clamp01(light_slip_intensity);
    grip_limit_intensity = clamp01(grip_limit_intensity);
    surface_texture_intensity = clamp01(surface_texture_intensity);

    // --- Source 5 continued: pulse level (set on the edge, exponential decay otherwise) ---
    if (landing_edge) {
        const float vertical_spike = std::fabs(in.accel_z);
        const float pulse = threshold_saturate(vertical_spike, tuning_.landing_accel_threshold_mps2, tuning_.landing_accel_saturate_mps2);
        landing_pulse_level_ = std::max(landing_pulse_level_, pulse);
    } else if (dt > 0.0f && tuning_.landing_pulse_decay_s > 0.0f) {
        landing_pulse_level_ *= std::exp(-dt / tuning_.landing_pulse_decay_s);
    }

    // --- Source 6: engine (optional, small) - grind/stall only; deliberately
    //     no rev-limiter buzz here (owner requirement, 2026-09-25 - see this
    //     file's header comment on source 6: only a physically-derived
    //     engine-motion channel may rumble near the limiter, and none exists
    //     in this codebase yet). ---
    float engine_weak = 0.0f;

    if (in.engine.grind_active) {
        engine_grind_level_ = tuning_.engine_grind_weak;
    } else if (dt > 0.0f && tuning_.engine_grind_decay_s > 0.0f) {
        engine_grind_level_ *= std::exp(-dt / tuning_.engine_grind_decay_s);
    }
    engine_weak = std::max(engine_weak, engine_grind_level_);

    if (in.engine.stalled && !prev_stalled_) {
        engine_stall_level_ = tuning_.engine_stall_pulse;
    } else if (dt > 0.0f && tuning_.engine_stall_decay_s > 0.0f) {
        engine_stall_level_ *= std::exp(-dt / tuning_.engine_stall_decay_s);
    }
    prev_stalled_ = in.engine.stalled;
    engine_weak = std::max(engine_weak, engine_stall_level_);

    // --- Combine: STRONG = transient-or-sustained (max), + a fraction of
    //     whatever else is also happening; WEAK = texture/light-slip/engine
    //     the same way ("weighted sum, clamped, or max plus a fraction of
    //     the sum" - owner's own phrasing). max(), not sum(), between
    //     source 1 and source 5 specifically to avoid double-counting the
    //     same physical landing event. ---
    const float strong_max = std::max({body_g_intensity, landing_pulse_level_, grip_limit_intensity});
    const float strong_sum = body_g_intensity + landing_pulse_level_ + grip_limit_intensity;
    const float strong_raw = clamp01(strong_max + 0.25f * std::max(0.0f, strong_sum - strong_max));

    const float weak_max = std::max({load_noise_intensity, light_slip_intensity, engine_weak, surface_texture_intensity});
    const float weak_sum = load_noise_intensity + light_slip_intensity + engine_weak + surface_texture_intensity;
    const float weak_raw = clamp01(weak_max + 0.30f * std::max(0.0f, weak_sum - weak_max));

    // --- Stage quantisation (hysteresis) + attack/release smoothing ---
    weak_stage_ = compute_stage(weak_raw, weak_stage_, tuning_);
    strong_stage_ = compute_stage(strong_raw, strong_stage_, tuning_);
    const float weak_target = tuning_.stage_levels[static_cast<std::size_t>(weak_stage_)];
    const float strong_target = tuning_.stage_levels[static_cast<std::size_t>(strong_stage_)];

    weak_smoothed_ = exp_decay_toward(weak_smoothed_, weak_target, dt, weak_target > weak_smoothed_ ? tuning_.attack_s : tuning_.release_s);
    strong_smoothed_ = exp_decay_toward(strong_smoothed_, strong_target, dt, strong_target > strong_smoothed_ ? tuning_.attack_s : tuning_.release_s);

    HapticsOutput out;
    out.weak = clamp01(weak_smoothed_ * tuning_.strength);
    out.strong = clamp01(strong_smoothed_ * tuning_.strength);
    if (!tuning_.enabled || in.paused) {
        out.weak = 0.0f;
        out.strong = 0.0f;
    }

    // --- Debug readout (haptics_core.h's own top comment) - every raw
    //     per-source value computed above, plus the final combined/output
    //     values, snapshotted for a caller/report to inspect. ---
    debug_.source1_body_g = body_g_intensity;
    debug_.source2_grip_limit = grip_limit_intensity;
    debug_.source3_load_noise = load_noise_intensity;
    debug_.source3_load_noise_rms_raw = load_noise_rms_raw;
    debug_.source4_light_slip = light_slip_intensity;
    debug_.source5_landing = landing_pulse_level_;
    debug_.source6_engine = engine_weak;
    debug_.source7_surface_texture = surface_texture_intensity;
    debug_.strong_raw = strong_raw;
    debug_.weak_raw = weak_raw;
    debug_.strong_out = out.strong;
    debug_.weak_out = out.weak;

    return out;
}

} // namespace ps_godot
