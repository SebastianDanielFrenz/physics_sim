// adapters/godot/tests/test_haptics.cpp — synthetic-input test for the
// multi-stage gamepad rumble core (haptics_core.h/.cpp), same pattern as
// test_input_map.cpp: plain main(), no test framework, one line per check,
// failure count as the process exit code (CTest + smoke_test.ps1 both read
// pass/fail from that alone).

#include "../src/haptics_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

int g_failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what);
    }
}

bool close(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// The adapter wiring (ps_simulation.cpp/input_map.cpp) calls
// HapticsCore::update once per Godot render frame - this dt is only used
// to exercise the core with a realistic, fixed cadence; nothing here
// depends on it matching any particular real frame rate.
constexpr float kDt = 1.0f / 120.0f;

ps_godot::HapticsInput make_input(float dt) {
    ps_godot::HapticsInput in;
    in.dt_s = dt;
    return in;
}

// Runs `frames` calls of core.update(builder(frame_index)) and returns the
// full output sequence (for tests that need more than just the last
// value) - a plain template so every call site can pass a lambda directly
// without pulling in <functional> ceremony.
template <typename Builder>
std::vector<ps_godot::HapticsOutput> run_frames(ps_godot::HapticsCore& core, int frames, Builder builder) {
    std::vector<ps_godot::HapticsOutput> out;
    out.reserve(static_cast<std::size_t>(frames));
    for (int i = 0; i < frames; ++i) {
        out.push_back(core.update(builder(i)));
    }
    return out;
}

} // namespace

using namespace ps_godot;

int main() {
    // --- Steady cornering at constant 8 m/s^2 lateral, no load noise, no
    //     slip: decays to 0 after the high-pass settles -------------------
    {
        HapticsCore core;
        const int frames = static_cast<int>(5.0f / kDt); // 5 s
        auto out = run_frames(core, frames, [](int) {
            HapticsInput in = make_input(kDt);
            in.accel_y = 8.0f; // steady lateral G, chassis frame
            in.wheel_count = 0;
            return in;
        });
        const HapticsOutput& last = out.back();
        check(close(last.strong, 0.0f, 1e-3f), "steady 8 m/s^2 lateral: strong decays to ~0 once the high-pass settles");
        check(close(last.weak, 0.0f, 1e-3f), "steady 8 m/s^2 lateral: weak stays ~0 (no wheel/engine data)");
    }

    // --- A step in lateral G produces a strong-motor pulse that decays ---
    {
        HapticsCore core;
        // Settle at 0 first.
        const int settle_frames = static_cast<int>(1.0f / kDt);
        for (int i = 0; i < settle_frames; ++i) {
            HapticsInput in = make_input(kDt);
            in.wheel_count = 0;
            (void)core.update(in);
        }
        // Step to 10 m/s^2 and hold; track the peak reached shortly after
        // the step and the value several seconds later.
        float peak = 0.0f;
        const int post_step_frames = static_cast<int>(5.0f / kDt);
        float value_after_5s = 0.0f;
        for (int i = 0; i < post_step_frames; ++i) {
            HapticsInput in = make_input(kDt);
            in.accel_y = 10.0f;
            in.wheel_count = 0;
            const HapticsOutput out = core.update(in);
            peak = std::max(peak, out.strong);
            value_after_5s = out.strong;
        }
        check(peak > 0.3f, "step in lateral G: strong motor pulses noticeably above baseline right after the step");
        check(value_after_5s < 0.02f, "step in lateral G: strong motor has decayed back to ~0 by 5 s after a HELD step");
    }

    // --- Load noise on one wheel produces weak output; the same mean load
    //     without noise produces ~0 -----------------------------------------
    {
        HapticsCore core_noisy;
        const int frames = static_cast<int>(3.0f / kDt); // let static_load (tau=1s) and RMS window settle
        auto out = run_frames(core_noisy, frames, [](int i) {
            HapticsInput in = make_input(kDt);
            const float t = static_cast<float>(i) * kDt;
            in.wheels[0].normal_load_n = 4000.0f + 800.0f * std::sin(2.0f * 3.14159265f * 10.0f * t); // 10 Hz kerb-like noise
            in.wheels[0].in_contact = true;
            in.wheel_count = 1;
            return in;
        });
        check(out.back().weak > 0.05f, "load noise on one wheel: weak motor is clearly non-zero");

        HapticsCore core_quiet;
        auto out2 = run_frames(core_quiet, frames, [](int) {
            HapticsInput in = make_input(kDt);
            in.wheels[0].normal_load_n = 4000.0f; // same mean, zero noise
            in.wheels[0].in_contact = true;
            in.wheel_count = 1;
            return in;
        });
        check(out2.back().weak < 0.01f, "same mean load with no noise: weak motor is ~0");
    }

    // --- Slip ratio 0.5 on the driven wheels produces weak output; 0.02
    //     produces none --------------------------------------------------
    {
        HapticsCore core_high;
        const int frames = static_cast<int>(1.0f / kDt);
        auto out = run_frames(core_high, frames, [](int) {
            HapticsInput in = make_input(kDt);
            for (int w = 0; w < 2; ++w) {
                in.wheels[w].normal_load_n = 4000.0f;
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_ratio = 0.5f;
            }
            in.wheel_count = 2;
            return in;
        });
        check(out.back().weak > 0.5f, "slip ratio 0.5 on driven wheels: weak motor is clearly non-zero");

        HapticsCore core_low;
        auto out2 = run_frames(core_low, frames, [](int) {
            HapticsInput in = make_input(kDt);
            for (int w = 0; w < 2; ++w) {
                in.wheels[w].normal_load_n = 4000.0f;
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_ratio = 0.02f;
            }
            in.wheel_count = 2;
            return in;
        });
        check(out2.back().weak < 0.02f, "slip ratio 0.02: weak motor produces none");
    }

    // --- Sustained grip-limit source (STRONG, owner refinement 2026-09-24):
    //     a constant slip angle 2x beyond peak on both fronts at constant
    //     lateral G gives a SUSTAINED strong-motor output that does not
    //     decay; the same G with slip below peak gives ~0 after the
    //     high-pass settles ----------------------------------------------
    {
        const HapticsTuning tuning{}; // default peak_slip_angle_rad ~4.5 deg (PHYS-036)
        HapticsCore core_past_peak(tuning);
        const float past_peak_angle = 2.0f * tuning.peak_slip_angle_rad;
        const int total_frames = static_cast<int>(5.0f / kDt);
        const int sample_a = static_cast<int>(3.0f / kDt);
        const int sample_b = total_frames - 1;
        float strong_at_a = 0.0f, strong_at_b = 0.0f;
        for (int i = 0; i < total_frames; ++i) {
            HapticsInput in = make_input(kDt);
            in.accel_y = 8.0f; // constant lateral G, same as the steady-cornering test
            for (int w = 0; w < 2; ++w) { // both fronts
                in.wheels[w].normal_load_n = 4000.0f;
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_angle_rad = past_peak_angle;
            }
            in.wheel_count = 2;
            const HapticsOutput out = core_past_peak.update(in);
            if (i == sample_a) strong_at_a = out.strong;
            if (i == sample_b) strong_at_b = out.strong;
        }
        check(strong_at_a > 0.15f, "2x-past-peak slip angle: strong motor is clearly non-zero once settled");
        check(close(strong_at_a, strong_at_b, 0.01f), "2x-past-peak slip angle: strong motor is SUSTAINED (does not decay between 3s and 5s)");

        HapticsCore core_below_peak(tuning);
        const float below_peak_angle = 0.5f * tuning.peak_slip_angle_rad;
        float strong_below_final = 0.0f;
        for (int i = 0; i < total_frames; ++i) {
            HapticsInput in = make_input(kDt);
            in.accel_y = 8.0f;
            for (int w = 0; w < 2; ++w) {
                in.wheels[w].normal_load_n = 4000.0f;
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_angle_rad = below_peak_angle;
            }
            in.wheel_count = 2;
            strong_below_final = core_below_peak.update(in).strong;
        }
        check(strong_below_final < 0.05f, "below-peak slip angle at the same G: strong motor is ~0 once the high-pass settles");
    }

    // --- Stage hysteresis: an input hovering at a stage threshold does not
    //     toggle the output stage every frame -----------------------------
    {
        HapticsCore core;
        const HapticsTuning& t = core.tuning();
        // Raw light-slip intensity 0.28 and 0.32 straddle stage_up_thresholds[1]
        // (0.30, the stage1->2 boundary) - without hysteresis this would
        // flip the stage every other frame; with it, once stage 2 is
        // reached it should stay there (down-threshold 0.30-0.06=0.24 is
        // never crossed by 0.28).
        auto ratio_for_intensity = [&](float intensity) {
            return t.slip_ratio_threshold + intensity * (t.slip_ratio_saturate - t.slip_ratio_threshold);
        };
        const float ratio_lo = ratio_for_intensity(0.28f);
        const float ratio_hi = ratio_for_intensity(0.32f);
        const int frames = static_cast<int>(1.0f / kDt);
        std::vector<float> tail;
        for (int i = 0; i < frames; ++i) {
            HapticsInput in = make_input(kDt);
            in.wheels[0].normal_load_n = 4000.0f;
            in.wheels[0].in_contact = true;
            in.wheels[0].slip_ratio = (i % 2 == 0) ? ratio_hi : ratio_lo;
            in.wheel_count = 1;
            const HapticsOutput out = core.update(in);
            if (i >= frames - 20) tail.push_back(out.weak);
        }
        float tail_min = tail.front(), tail_max = tail.front();
        for (float v : tail) { tail_min = std::min(tail_min, v); tail_max = std::max(tail_max, v); }
        check((tail_max - tail_min) < 0.02f, "stage hysteresis: output settles to one constant stage despite the raw input hovering across the boundary");
    }

    // --- Paused -> 0; disabled -> 0; strength 0.5 halves the output ------
    {
        auto settle_grip_limit = [](HapticsCore& core, int frames) {
            HapticsOutput last{};
            for (int i = 0; i < frames; ++i) {
                HapticsInput in = make_input(kDt);
                in.wheels[0].normal_load_n = 4000.0f;
                in.wheels[0].in_contact = true;
                in.wheels[0].slip_angle_rad = 2.0f * HapticsTuning{}.peak_slip_angle_rad;
                in.wheel_count = 1;
                last = core.update(in);
            }
            return last;
        };

        const int settle = static_cast<int>(2.0f / kDt);
        HapticsCore core_full;
        const HapticsOutput baseline = settle_grip_limit(core_full, settle);
        check(baseline.strong > 0.1f, "paused/disabled/strength setup: baseline sustained strong output is non-zero before the gate is applied");

        // Paused, same core (state preserved) -> output forced to 0.
        {
            HapticsInput in = make_input(kDt);
            in.wheels[0].normal_load_n = 4000.0f;
            in.wheels[0].in_contact = true;
            in.wheels[0].slip_angle_rad = 2.0f * HapticsTuning{}.peak_slip_angle_rad;
            in.wheel_count = 1;
            in.paused = true;
            const HapticsOutput out = core_full.update(in);
            check(out.strong == 0.0f && out.weak == 0.0f, "paused: output is forced to exactly 0");
        }

        HapticsTuning disabled_tuning;
        disabled_tuning.enabled = false;
        HapticsCore core_disabled(disabled_tuning);
        const HapticsOutput disabled_out = settle_grip_limit(core_disabled, settle);
        check(disabled_out.strong == 0.0f && disabled_out.weak == 0.0f, "disabled: output is forced to exactly 0");

        HapticsTuning half_tuning;
        half_tuning.strength = 0.5f;
        HapticsCore core_half(half_tuning);
        const HapticsOutput half_out = settle_grip_limit(core_half, settle);
        check(close(half_out.strong, baseline.strong * 0.5f, 1e-3f), "strength 0.5 halves the settled strong output");
    }

    // --- Deterministic: the same input sequence gives bit-identical output ---
    {
        auto build = [](int i) {
            HapticsInput in = make_input(kDt);
            const float t = static_cast<float>(i) * kDt;
            in.accel_x = 2.0f * std::sin(0.7f * t);
            in.accel_y = (t > 1.0f) ? 9.0f : 0.0f;
            in.accel_z = 0.3f * std::cos(1.3f * t);
            in.wheels[0].normal_load_n = 4000.0f + 500.0f * std::sin(20.0f * t);
            in.wheels[0].in_contact = (i % 37) != 0; // occasional landing edges
            in.wheels[0].slip_ratio = 0.2f * std::sin(0.3f * t);
            in.wheels[0].slip_angle_rad = 0.1f * std::sin(0.2f * t + 1.0f);
            in.wheels[1].normal_load_n = 3800.0f;
            in.wheels[1].in_contact = true;
            in.wheels[1].slip_angle_rad = 0.15f;
            in.wheel_count = 2;
            in.engine.rpm = 5000.0f + 1000.0f * std::sin(0.5f * t);
            in.engine.grind_active = (i % 53) == 0;
            return in;
        };
        HapticsCore core_a, core_b;
        const int frames = static_cast<int>(4.0f / kDt);
        bool identical = true;
        for (int i = 0; i < frames; ++i) {
            const HapticsInput in = build(i);
            const HapticsOutput oa = core_a.update(in);
            const HapticsOutput ob = core_b.update(build(i));
            if (oa.weak != ob.weak || oa.strong != ob.strong) { identical = false; break; }
        }
        check(identical, "determinism: two fresh cores fed the same input sequence produce bit-identical output");
    }

    // --- Standstill: large relaxed slip on a car at rest (e.g. stalled in
    //     gear) is carcass deflection, not sliding - no rumble. A burnout at
    //     rest (wheel surface moving) still rumbles. Owner bug 2026-09-24. --
    {
        auto build_rest = [](float ground, float wheel_surface) {
            return [ground, wheel_surface](int) {
                HapticsInput in = make_input(kDt);
                for (int w = 0; w < 4; ++w) {
                    in.wheels[w].normal_load_n = 3700.0f;
                    in.wheels[w].in_contact = true;
                    in.wheels[w].slip_ratio = 0.6f;
                    in.wheels[w].slip_angle_rad = 0.3f;
                    in.wheels[w].ground_speed_mps = ground;
                    in.wheels[w].wheel_surface_speed_mps = wheel_surface;
                }
                in.wheel_count = 4;
                return in;
            };
        };
        const int frames = static_cast<int>(2.0f / kDt);
        HapticsCore core_rest;
        auto rest = run_frames(core_rest, frames, build_rest(0.0f, 0.0f));
        check(rest.back().strong == 0.0f && rest.back().weak == 0.0f,
              "standstill with large relaxed slip and nothing moving: no rumble at all");

        HapticsCore core_burnout;
        auto burnout = run_frames(core_burnout, frames, build_rest(0.0f, 10.0f));
        check(burnout.back().strong > 0.15f,
              "burnout at rest (wheel surface 10 m/s, ground 0): strong motor still rumbles from the slip ratio");

        HapticsCore core_moving;
        auto moving = run_frames(core_moving, frames, build_rest(5.0f, 5.0f));
        check(moving.back().strong >= burnout.back().strong,
              "same slip while moving at 5 m/s: slip angle adds back in, at least as strong as the burnout");
    }

    // --- Bug A pin (owner, 2026-09-25: "accelerating in 3rd gear is enough
    //     to get the controller to vibrate"). Real ps_run telemetry traced
    //     this to a LAUNCH transient (a fast front-wheel Fz swing under
    //     ~2 m/s), not steady WOT accel in any particular gear - see this
    //     file's header comment and haptics_core.h's source-3 doc comment.
    //     This pins the actual owner-reported symptom: already-at-speed,
    //     already-settled full-throttle acceleration in 3rd (representative
    //     measured numbers: ~20 m/s cruise, accel_x ~1.7 m/s^2, front Fz
    //     ~3300-3700N with only small ripple - no launch transient, no
    //     wheelspin) produces ~0 from both motors. ------------------------
    {
        HapticsCore core;
        const int frames = static_cast<int>(3.0f / kDt);
        auto out = run_frames(core, frames, [](int i) {
            HapticsInput in = make_input(kDt);
            const float t = static_cast<float>(i) * kDt;
            in.accel_x = 1.7f; // steady WOT acceleration, ISO chassis-frame forward
            const float fz_ripple = 60.0f * std::sin(2.0f * 3.14159265f * 4.0f * t); // small road-noise ripple, NOT a launch swing
            in.wheels[0].normal_load_n = 3500.0f + fz_ripple; // FL
            in.wheels[1].normal_load_n = 3500.0f - fz_ripple; // FR
            in.wheels[2].normal_load_n = 3300.0f;             // RL
            in.wheels[3].normal_load_n = 3300.0f;              // RR
            for (int w = 0; w < 4; ++w) {
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_ratio = 0.03f; // small, well below peak_slip_ratio (0.10)
                in.wheels[w].ground_speed_mps = 20.0f; // already-at-speed cruise, well past every fade threshold
                in.wheels[w].wheel_surface_speed_mps = 20.0f;
            }
            in.wheel_count = 4;
            return in;
        });
        const HapticsOutput& last = out.back();
        check(last.strong < 0.02f, "steady 3rd-gear WOT accel (already settled, no launch transient): strong motor ~0");
        check(last.weak < 0.02f, "steady 3rd-gear WOT accel (already settled, no launch transient): weak motor ~0");
    }

    // --- Bug B (owner, 2026-09-25: "didn't feel the curbs on the asphalt
    //     straight at all") - source 7, surface texture / rumble strip.
    //     Kerb at speed -> clear rumble; kerb at ~0 speed -> none (not
    //     rolling over the strip yet); asphalt (or any unmatched surface)
    //     -> none regardless of speed. ---------------------------------------
    {
        auto build_surface = [](const std::array<char, 16>& name, float speed) {
            return [name, speed](int) {
                HapticsInput in = make_input(kDt);
                in.wheels[0].normal_load_n = 4000.0f;
                in.wheels[0].in_contact = true;
                in.wheels[0].ground_speed_mps = speed;
                in.wheels[0].wheel_surface_speed_mps = speed;
                in.wheels[0].surface_name = name;
                in.wheel_count = 1;
                return in;
            };
        };
        const std::array<char, 16> kerb_name{'k', 'e', 'r', 'b'};
        const std::array<char, 16> asphalt_name{'a', 's', 'p', 'h', 'a', 'l', 't'};
        const int frames = static_cast<int>(1.0f / kDt);

        HapticsCore core_kerb_moving;
        auto kerb_moving = run_frames(core_kerb_moving, frames, build_surface(kerb_name, 10.0f));
        check(kerb_moving.back().weak > 0.1f, "kerb at speed: weak motor rumbles clearly");

        HapticsCore core_kerb_still;
        auto kerb_still = run_frames(core_kerb_still, frames, build_surface(kerb_name, 0.0f));
        check(kerb_still.back().weak < 0.01f, "kerb at ~0 speed: no rumble (not rolling over the strip yet)");

        HapticsCore core_asphalt;
        auto asphalt = run_frames(core_asphalt, frames, build_surface(asphalt_name, 10.0f));
        check(asphalt.back().weak < 0.01f, "asphalt at speed: no rumble from the surface-texture source");
    }

    // --- Regression: a rev limiter must not cause rumble on its own (owner
    //     requirement, 2026-09-25) - HapticsEngineInput no longer even has a
    //     limiter field, so a caller cannot wire one in; this pins that a
    //     high/pinned rpm alone (no grind, no stall) contributes nothing
    //     from source 6. ------------------------------------------------
    {
        HapticsCore core;
        const int frames = static_cast<int>(1.0f / kDt);
        auto out = run_frames(core, frames, [](int) {
            HapticsInput in = make_input(kDt);
            in.engine.rpm = 6800.0f; // pinned at/above a typical limiter rpm
            in.wheel_count = 0;
            return in;
        });
        check(out.back().weak == 0.0f, "engine pinned at limiter rpm, no grind/stall: weak motor stays exactly 0");
    }

    // --- Owner requirement, 2026-09-25: "good asphalt doesn't cause
    //     rumble" - steady cruise on the asphalt straight at 50/100/150
    //     km/h, part throttle in whichever gear that speed lands in, is
    //     silent. Representative numbers from the demo's own
    //     --m1-haptics-cruise scripted drive (PS_HAPTICS_LOG=1) on
    //     data/tracks/test_ground's real (triangulated-mesh) asphalt
    //     straight - see haptics_core.h's own top comment (source 3) for
    //     the measured settled-cruise RMS ceiling this pins against. -----
    {
        auto build_cruise = [](float speed_mps, float slip_ratio) {
            return [speed_mps, slip_ratio](int i) {
                HapticsInput in = make_input(kDt);
                const float t = static_cast<float>(i) * kDt;
                // Small ambient ripple representative of a real (mesh, not
                // mathematically flat) asphalt surface at steady speed -
                // comfortably under the measured settled-cruise ceiling.
                const float fz_ripple = 40.0f * std::sin(2.0f * 3.14159265f * 6.0f * t);
                const std::array<char, 16> asphalt_name{'a', 's', 'p', 'h', 'a', 'l', 't'};
                for (int w = 0; w < 4; ++w) {
                    in.wheels[w].normal_load_n = 3500.0f + (w < 2 ? fz_ripple : -fz_ripple);
                    in.wheels[w].in_contact = true;
                    in.wheels[w].slip_ratio = slip_ratio;
                    in.wheels[w].ground_speed_mps = speed_mps;
                    in.wheels[w].wheel_surface_speed_mps = speed_mps;
                    in.wheels[w].surface_name = asphalt_name;
                }
                in.wheel_count = 4;
                return in;
            };
        };
        const int settle_frames = static_cast<int>(3.0f / kDt);
        for (float kmh : {50.0f, 100.0f, 150.0f}) {
            HapticsCore core;
            auto out = run_frames(core, settle_frames, build_cruise(kmh / 3.6f, 0.04f));
            const HapticsOutput& last = out.back();
            char label[128];
            std::snprintf(label, sizeof(label), "asphalt steady cruise at %.0f km/h (part throttle): both motors ~0", kmh);
            check(last.strong < 0.02f && last.weak < 0.02f, label);
        }
    }

    // --- Normal-traction slip (0.04, well within a FWD car's ordinary
    //     ~0.03-0.05 under-throttle band) is silent from source 4 - the
    //     threshold that used to sit INSIDE this band (owner bug,
    //     2026-09-25) is now clearly above it (haptics_core.h's own top
    //     comment, source 4). --------------------------------------------
    {
        HapticsCore core;
        const int frames = static_cast<int>(2.0f / kDt);
        auto out = run_frames(core, frames, [](int) {
            HapticsInput in = make_input(kDt);
            for (int w = 0; w < 4; ++w) {
                in.wheels[w].normal_load_n = 3500.0f;
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_ratio = 0.04f;
                in.wheels[w].ground_speed_mps = 15.0f;
                in.wheels[w].wheel_surface_speed_mps = 15.0f;
            }
            in.wheel_count = 4;
            return in;
        });
        check(out.back().weak < 0.01f, "normal-traction slip_ratio 0.04 under throttle: weak motor stays silent");
        check(out.back().strong < 0.01f, "normal-traction slip_ratio 0.04 under throttle: strong motor stays silent");
    }

    // --- Source 7 generalised (owner requirement, 2026-09-25): dirt/grass
    //     texture is felt (asphalt/kerb already pinned by the "Bug B" test
    //     above) - every data/surfaces/surfaces.json name now has a table
    //     entry (haptics_core.h's own HapticsTuning::surface_texture_rules
    //     field comment). ----------------------------------------------
    {
        auto build_surface = [](const std::array<char, 16>& name, float speed) {
            return [name, speed](int) {
                HapticsInput in = make_input(kDt);
                in.wheels[0].normal_load_n = 4000.0f;
                in.wheels[0].in_contact = true;
                in.wheels[0].ground_speed_mps = speed;
                in.wheels[0].wheel_surface_speed_mps = speed;
                in.wheels[0].surface_name = name;
                in.wheel_count = 1;
                return in;
            };
        };
        const int frames = static_cast<int>(1.0f / kDt);
        const std::array<char, 16> dirt_name{'d', 'i', 'r', 't'};
        const std::array<char, 16> grass_name{'g', 'r', 'a', 's', 's'};
        const std::array<char, 16> field_name{'f', 'i', 'e', 'l', 'd'};
        const std::array<char, 16> low_mu_name{'l', 'o', 'w', '_', 'm', 'u'};

        HapticsCore core_dirt;
        check(run_frames(core_dirt, frames, build_surface(dirt_name, 10.0f)).back().weak > 0.05f,
              "dirt texture at speed: weak motor is clearly non-zero");
        HapticsCore core_grass;
        check(run_frames(core_grass, frames, build_surface(grass_name, 10.0f)).back().weak > 0.05f,
              "grass texture at speed: weak motor is clearly non-zero");
        HapticsCore core_field;
        check(run_frames(core_field, frames, build_surface(field_name, 10.0f)).back().weak > 0.05f,
              "field texture at speed: weak motor is clearly non-zero");
        // low_mu is smooth (haptics_core.h's surface table): no texture
        // rumble while rolling without slip, exactly like asphalt.
        HapticsCore core_low_mu;
        const HapticsOutput low_mu_out = run_frames(core_low_mu, frames, build_surface(low_mu_name, 10.0f)).back();
        check(low_mu_out.weak == 0.0f && low_mu_out.strong == 0.0f, "low_mu at speed without slip: silent (smooth surface)");
    }

    // --- Grip limit still rumbles even on asphalt (owner requirement,
    //     2026-09-25 kept: "loss of grip (wheelspin/lockup/slide past
    //     peak) does rumble") - a wheelspin-level slip ratio well past
    //     peak_slip_ratio on asphalt produces sustained STRONG rumble
    //     despite this task's silencing fixes above. -------------------
    {
        const std::array<char, 16> asphalt_name{'a', 's', 'p', 'h', 'a', 'l', 't'};
        HapticsCore core;
        const int frames = static_cast<int>(2.0f / kDt);
        auto out = run_frames(core, frames, [&](int) {
            HapticsInput in = make_input(kDt);
            for (int w = 0; w < 2; ++w) {
                in.wheels[w].normal_load_n = 3500.0f;
                in.wheels[w].in_contact = true;
                in.wheels[w].slip_ratio = 0.35f; // well past peak_slip_ratio (0.10) - wheelspin
                in.wheels[w].ground_speed_mps = 15.0f;
                in.wheels[w].wheel_surface_speed_mps = 20.0f;
                in.wheels[w].surface_name = asphalt_name;
            }
            in.wheel_count = 2;
            return in;
        });
        check(out.back().strong > 0.3f, "wheelspin-level slip on asphalt: strong motor rumbles clearly (grip loss is still felt)");
    }

    if (g_failures == 0) {
        std::printf("test_haptics: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_haptics: %d check(s) FAILED\n", g_failures);
    return g_failures;
}
