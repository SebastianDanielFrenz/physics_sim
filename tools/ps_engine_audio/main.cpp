// main.cpp - ps_audio A10 (out/audio_plan.md section 6): tools/ps_engine_audio,
// an engine-only offline/live listening-test host. Loads an engine file with
// ps::io::load_engine_json, builds a ps::EngineRig (the engine on a dyno,
// no vehicle), steps it at 240 Hz alongside a ps_audio::Voice fed from its
// EngineSoundState every tick, and renders a deterministic WAV file. This
// file is the ONE place in the project that links both ps_core and ps_audio
// (see CMakeLists.txt) - the EngineSoundState -> EngineInput glue itself
// (ps_audio A11) moved out to tools/common/engine_sound_glue.h so
// adapters/godot's ps_godot_adapter (a second binary linking both ps_core
// and ps_audio) can share it instead of duplicating the mapping.
//
// Usage:
//   ps_engine_audio <engine.json> --wav out.wav --script events.json
//                    [--installation file.json] [--seconds S] [--seed N]
//                    [--map-workers N] [--engine-cache DIR]
//                    [--dyno-brake-nm X] [--full-scale-pa P] [--live]
//                    [--dx-intake-mm D] [--dx-exhaust-mm D] [--substeps 1|2|4|8|16]
//                    [--voice-threads N]
#include "ps/drivetrain/engine_rig.h"
#include "ps/engine_cycle/cycle_engine.h" // RadiatingEnd, for the printed source list
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "engine_sound_glue.h"
#include "ps_audio/engine_input.h"
#include "ps_audio/source.h"
#include "ps_audio/voice.h"

#include "headless_env.h"
#include "script.h"
#include "wav_writer.h"

#include "miniaudio.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace {

struct Args {
    std::string engine_path;
    std::string wav_path;
    std::string script_path;
    std::string installation_path; // empty = engine's own default_installation
    double seconds = -1.0;         // <= 0: derive from the script
    std::uint64_t seed = 0;
    int map_workers = 0;
    std::string engine_cache_dir;
    double dyno_brake_nm = -1.0; // < 0: no dyno (free rev)
    double dyno_viscous_nms = -1.0; // > 0: a viscous dyno (N*m*s/rad) instead
    double hitch_every_s = -1.0;   // > 0: diag - withhold the voice input for hitch_ticks ticks this often
    int hitch_ticks = 3;
    std::string input_csv_path;    // diag: replay recorded EngineInputs (one row per 240 Hz tick) instead of the rig
    double plenum_pi_tau_s = -1.0;     // < 0: LiveEngineSettings default
    double plenum_filter_tau_s = -1.0; // < 0: LiveEngineSettings default
    std::string diag_csv_path;         // empty: none; else one row per tick (voice plenum loop)
    bool live = false;
    double full_scale_pa = ps_audio::Calibration{}.pa_full_scale; // Pa at 1 m mapped to digital full scale
    // Listening-test overrides of the voice's pipe cell sizes (0 = the engine's
    // resolution.audio_dx_*) and physics steps per output sample (0 = the
    // stability probe). Bypass the loader's 1-substep CFL floor on purpose; the
    // PipeNetwork's own build-time CFL check still rejects an unsteppable size.
    double dx_intake_mm = 0.0;
    double dx_exhaust_mm = 0.0;
    int substeps = 0;
    int voice_threads = 1;  // ps_audio A9b: physics-step worker count (1 = serial)
};

[[noreturn]] void usage_error(const std::string& msg) {
    std::fprintf(stderr,
                  "ps_engine_audio: %s\n\n"
                  "usage: ps_engine_audio <engine.json> --wav out.wav --script events.json\n"
                  "                       [--installation file.json] [--seconds S] [--seed N]\n"
                  "                       [--map-workers N] [--engine-cache DIR]\n"
                  "                       [--dyno-brake-nm X] [--full-scale-pa P] [--live]\n"
                  "                       [--dx-intake-mm D] [--dx-exhaust-mm D] [--substeps 1|2|4|8|16]\n"
                  "                       [--voice-threads N]\n",
                  msg.c_str());
    std::exit(1);
}

Args parse_args(int argc, char** argv) {
    Args a;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) usage_error(std::string(flag) + " requires a value");
            return argv[++i];
        };
        if (arg == "--wav") {
            a.wav_path = next("--wav");
        } else if (arg == "--script") {
            a.script_path = next("--script");
        } else if (arg == "--installation") {
            a.installation_path = next("--installation");
        } else if (arg == "--seconds") {
            a.seconds = std::stod(next("--seconds"));
        } else if (arg == "--seed") {
            a.seed = static_cast<std::uint64_t>(std::stoull(next("--seed")));
        } else if (arg == "--map-workers") {
            a.map_workers = std::stoi(next("--map-workers"));
        } else if (arg == "--engine-cache") {
            a.engine_cache_dir = next("--engine-cache");
        } else if (arg == "--dyno-brake-nm") {
            a.dyno_brake_nm = std::stod(next("--dyno-brake-nm"));
        } else if (arg == "--dyno-viscous-nms") {
            a.dyno_viscous_nms = std::stod(next("--dyno-viscous-nms"));
        } else if (arg == "--hitch-every-s") {
            a.hitch_every_s = std::stod(next("--hitch-every-s"));
        } else if (arg == "--input-csv") {
            a.input_csv_path = next("--input-csv");
        } else if (arg == "--hitch-ticks") {
            a.hitch_ticks = std::stoi(next("--hitch-ticks"));
        } else if (arg == "--full-scale-pa") {
            a.full_scale_pa = std::stod(next("--full-scale-pa"));
            if (!(a.full_scale_pa > 0.0)) usage_error("--full-scale-pa must be > 0");
        } else if (arg == "--dx-intake-mm") {
            a.dx_intake_mm = std::stod(next("--dx-intake-mm"));
            if (!(a.dx_intake_mm > 0.0)) usage_error("--dx-intake-mm must be > 0");
        } else if (arg == "--dx-exhaust-mm") {
            a.dx_exhaust_mm = std::stod(next("--dx-exhaust-mm"));
            if (!(a.dx_exhaust_mm > 0.0)) usage_error("--dx-exhaust-mm must be > 0");
        } else if (arg == "--substeps") {
            a.substeps = std::stoi(next("--substeps"));
            if (a.substeps != 1 && a.substeps != 2 && a.substeps != 4 && a.substeps != 8 && a.substeps != 16)
                usage_error("--substeps must be 1, 2, 4, 8 or 16");
        } else if (arg == "--voice-threads") {
            a.voice_threads = std::stoi(next("--voice-threads"));
            if (a.voice_threads < 1) usage_error("--voice-threads must be >= 1");
        } else if (arg == "--plenum-pi-tau-s") {
            a.plenum_pi_tau_s = std::stod(next("--plenum-pi-tau-s"));
        } else if (arg == "--plenum-filter-tau-s") {
            a.plenum_filter_tau_s = std::stod(next("--plenum-filter-tau-s"));
        } else if (arg == "--diag-csv") {
            a.diag_csv_path = next("--diag-csv");
        } else if (arg == "--live") {
            a.live = true;
        } else if (!arg.empty() && arg[0] == '-') {
            usage_error("unknown option '" + arg + "'");
        } else {
            positional.push_back(arg);
        }
    }
    if (positional.size() != 1) usage_error("expected exactly one positional <engine.json> argument");
    a.engine_path = positional[0];
    if (a.wav_path.empty()) usage_error("--wav is required");
    if (a.script_path.empty()) usage_error("--script is required");
    return a;
}

// Prints the Voice's own resolved radiating-end list (the authoritative
// source, not a guess from the input CycleModelDesc) - one line per
// ps::engine_cycle::RadiatingEnd.
void print_source_list(const ps_audio::Voice& voice) {
    std::fprintf(stderr, "ps_engine_audio: %d radiating source(s):\n", voice.source_count());
    for (int k = 0; k < voice.source_count(); ++k) {
        const ps::engine_cycle::RadiatingEnd& s = voice.source(k);
        const char* kind = s.kind == ps::engine_cycle::RadiatingEnd::Kind::PipeEnd ? "PipeEnd" : "WaveguideOpen";
        std::fprintf(stderr, "  [%d] %s '%s' index=%d %s area=%.6f m^2 %s\n", k, kind, s.name.c_str(), s.index,
                      s.at_start ? "(start)" : "(end)", s.area_m2, s.intake ? "intake" : "exhaust");
    }
}

// --live playback: a fixed, already-rendered float buffer played back
// through miniaudio via a simple frame-cursor callback. Deliberately NOT a
// true real-time interactive session (Runner + device thread driving the
// physics live) - the render is fully deterministic and offline first, then
// (optionally) played back, so --live can never change the WAV's content or
// this tool's determinism guarantees.
struct PlaybackState {
    const float* data = nullptr;
    std::size_t total_frames = 0;
    int channels = 0;
    std::size_t cursor_frames = 0;
};

void playback_callback(ma_device* device, void* output, const void* /*input*/, ma_uint32 frame_count) {
    auto* st = static_cast<PlaybackState*>(device->pUserData);
    float* out = static_cast<float*>(output);
    const std::size_t remaining = st->total_frames - st->cursor_frames;
    const std::size_t to_copy = std::min<std::size_t>(frame_count, remaining);
    if (to_copy > 0) {
        std::copy_n(st->data + st->cursor_frames * static_cast<std::size_t>(st->channels),
                    to_copy * static_cast<std::size_t>(st->channels), out);
        st->cursor_frames += to_copy;
    }
    if (to_copy < frame_count) {
        std::fill_n(out + to_copy * static_cast<std::size_t>(st->channels),
                    (frame_count - to_copy) * static_cast<std::size_t>(st->channels), 0.0f);
    }
}

void play_buffer_live(const std::vector<float>& interleaved, int channels, int sample_rate_hz) {
    PlaybackState st;
    st.data = interleaved.data();
    st.total_frames = interleaved.empty() ? 0 : interleaved.size() / static_cast<std::size_t>(channels);
    st.channels = channels;

    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = static_cast<ma_uint32>(channels);
    cfg.sampleRate = static_cast<ma_uint32>(sample_rate_hz);
    cfg.dataCallback = playback_callback;
    cfg.pUserData = &st;

    ma_device device;
    if (ma_device_init(nullptr, &cfg, &device) != MA_SUCCESS) {
        throw std::runtime_error("ps_engine_audio --live: ma_device_init failed");
    }
    if (ma_device_start(&device) != MA_SUCCESS) {
        ma_device_uninit(&device);
        throw std::runtime_error("ps_engine_audio --live: ma_device_start failed");
    }
    std::fprintf(stderr, "ps_engine_audio: playing back %.2f s live (ctrl-C to stop early)...\n",
                  static_cast<double>(st.total_frames) / sample_rate_hz);
    while (st.cursor_frames < st.total_frames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // Let the tail actually leave the device before tearing it down.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ma_device_uninit(&device);
}

} // namespace

int main(int argc, char** argv) {
    ps_tools::install_headless_crt_handlers();
    try {
        const Args args = parse_args(argc, argv);
        const ps_tools::EngineAudioScript script = ps_tools::load_engine_audio_script(args.script_path);

        // Kept alive for the rest of main() (both load_engine_json calls
        // below plus EngineRig's own internal load use the same
        // map_options, which only stores a raw non-owning pointer).
        std::unique_ptr<ps::jobs::JobSystem> map_worker_pool;
        ps::io::EngineMapOptions map_options;
        if (args.map_workers > 0) {
            map_worker_pool = std::make_unique<ps::jobs::JobSystem>(args.map_workers);
            map_options.pool = map_worker_pool.get();
        }
        if (!args.engine_cache_dir.empty()) map_options.cache_dir = args.engine_cache_dir;

        const std::string* installation_override = args.installation_path.empty() ? nullptr : &args.installation_path;

        // Resolve the engine's SimulatedEngineDesc directly (for Voice's
        // CycleModelDesc/fuel) - a second, independent load from the one
        // EngineRig performs internally. Both loads are pure/deterministic
        // (ps::io::load_engine_json has no side effects beyond the shared,
        // opt-in .psmaps disk cache), so doing it twice costs generation
        // time but never correctness; EngineRig does not expose its
        // resolved desc, and duplicating a few hundred lines of loader
        // logic in this tool would be worse.
        const ps::drivetrain::ComponentParams params = ps::io::load_engine_json(args.engine_path, map_options, installation_override);
        const auto* sim = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&params);
        if (sim == nullptr) {
            throw std::runtime_error("ps_engine_audio: '" + args.engine_path +
                                      "' is not a \"kind\": \"simulated\" engine (ps_audio only renders the live "
                                      "cycle model, not a torque-map engine)");
        }
        if (!sim->cycle.has_value()) {
            throw std::runtime_error("ps_engine_audio: '" + args.engine_path +
                                      "' has no \"cycle\" block (ps_audio::Voice needs the live cycle model, not "
                                      "pre-generated maps alone)");
        }
        if (sim->fuels.empty()) {
            throw std::runtime_error("ps_engine_audio: '" + args.engine_path + "' has no fuel");
        }

        ps::EngineRig::Config rig_config;
        if (args.dyno_brake_nm > 0.0) {
            rig_config.dyno.enabled = true;
            rig_config.dyno.desc.mode = ps::drivetrain::LoadDynoMode::Brake;
            rig_config.dyno.desc.inertia_kgm2 = 0.02; // [AS] small test-dyno rotor
            rig_config.dyno.desc.torque_nm = args.dyno_brake_nm;
        } else if (args.dyno_viscous_nms > 0.0) {
            rig_config.dyno.enabled = true;
            rig_config.dyno.desc.mode = ps::drivetrain::LoadDynoMode::Viscous;
            rig_config.dyno.desc.inertia_kgm2 = 0.02; // [AS] small test-dyno rotor
            rig_config.dyno.desc.viscous_nm_s_rad = args.dyno_viscous_nms;
        }
        ps::EngineRig rig(args.engine_path, map_options, installation_override, rig_config);

        ps_audio::VoiceSettings voice_settings;
        voice_settings.engine.seed = args.seed;
        voice_settings.engine.dx_intake_m = args.dx_intake_mm * 1e-3;
        voice_settings.engine.dx_exhaust_m = args.dx_exhaust_mm * 1e-3;
        voice_settings.substeps = args.substeps;
        voice_settings.engine.threads = args.voice_threads;
        if (args.plenum_pi_tau_s > 0.0) voice_settings.engine.plenum_pi_tau_s = args.plenum_pi_tau_s;
        if (args.plenum_filter_tau_s > 0.0) voice_settings.engine.plenum_filter_tau_s = args.plenum_filter_tau_s;
        ps_audio::Voice voice(*sim->cycle, sim->fuels[0].lhv_j_kg, sim->fuels[0].afr_stoich, voice_settings);
        print_source_list(voice);

        const int sample_rate_hz = static_cast<int>(voice.sample_rate_hz());
        const int sources = voice.source_count();
        // Voice::render is per-source (out[f * source_count() + s], see
        // voice.h) - this tool mixes every source down to mono with equal
        // weight (a simple, clearly-labelled listening-test mixdown, not a
        // physically modelled spatialisation; PROVISIONAL, see progress
        // file). --live and the WAV file always hear the same mix.
        const int channels = 1;
        const double tick_hz = 240.0;
        if (std::fmod(static_cast<double>(sample_rate_hz), tick_hz) != 0.0) {
            throw std::runtime_error("ps_engine_audio: sample_rate_hz must be an integer multiple of the 240 Hz "
                                      "physics tick");
        }
        const int frames_per_tick = static_cast<int>(sample_rate_hz / tick_hz);

        double duration_s = args.seconds > 0.0 ? args.seconds : script.duration_s;
        if (duration_s <= 0.0) {
            const double last_t = script.events.empty() ? 0.0 : script.events.back().t;
            duration_s = last_t + 3.0; // fixed tail so the engine's final state is actually heard
        }
        std::vector<ps_audio::EngineInput> replay;
        if (!args.input_csv_path.empty()) {
            std::ifstream in_csv(args.input_csv_path);
            if (!in_csv) throw std::runtime_error("cannot open --input-csv " + args.input_csv_path);
            std::string line;
            std::getline(in_csv, line);  // header
            while (std::getline(in_csv, line)) {
                ps_audio::EngineInput e;
                int spark_given = 0;
                unsigned flags = 0;
                std::istringstream ls(line);
                char c = 0;
                ls >> e.t_sim_s >> c >> e.omega_rad_s >> c >> e.crank_angle_deg >> c >> e.p_charge_pa >> c >> e.t_charge_k >> c >>
                    e.p_intake_pa >> c >> e.throttle >> c >> e.lambda >> c >> e.spark_deg_atdc >> c >> spark_given >> c >> flags;
                if (!ls) continue;
                e.spark_given = spark_given != 0;
                e.flags = flags;
                replay.push_back(e);
            }
            std::fprintf(stderr, "ps_engine_audio: replaying %zu recorded inputs\n", replay.size());
        }
        const long long total_ticks = replay.empty() ? static_cast<long long>(std::ceil(duration_s * tick_hz))
                                                     : static_cast<long long>(replay.size());

        std::vector<float> rendered;
        rendered.reserve(static_cast<std::size_t>(total_ticks) * static_cast<std::size_t>(frames_per_tick));
        // Voice::render's own per-source interleaved buffer for one tick's
        // worth of frames, reused every tick (no per-tick allocation once
        // warmed up beyond this vector's own capacity, reached on tick 0).
        std::vector<float> per_source_block(static_cast<std::size_t>(frames_per_tick) * static_cast<std::size_t>(sources),
                                             0.0f);
        std::vector<float> mixed_block(static_cast<std::size_t>(frames_per_tick), 0.0f);
        // Voice output is Pa at 1 m; ps_audio::Calibration maps pa_full_scale to digital
        // full scale (no AGC). Without it 1 Pa was full scale and every engine clipped.
        const float mix_gain = static_cast<float>(1.0 / args.full_scale_pa) /
                               (sources > 0 ? static_cast<float>(sources) : 1.0f);

        const auto wall_start = std::chrono::steady_clock::now();

        std::FILE* diag = nullptr;
#if defined(_MSC_VER)
        if (!args.diag_csv_path.empty() && fopen_s(&diag, args.diag_csv_path.c_str(), "w") != 0) diag = nullptr;
#else
        if (!args.diag_csv_path.empty()) diag = std::fopen(args.diag_csv_path.c_str(), "w");
#endif
        if (diag != nullptr)
            std::fprintf(diag, "t,rpm,throttle,p_intake_in_pa,p_charge_in_pa,voice_plenum_pa,voice_plenum_filtered_pa,"
                               "voice_throttle_cd_area_m2,block_rms_pa,hitches,crank_err_max_deg,crank_sat_samples\n");
        std::vector<ps_audio::EngineInput> withheld;
        withheld.reserve(64);
        std::size_t next_event = 0;
        for (long long tick = 0; tick < total_ticks; ++tick) {
            const double t = static_cast<double>(tick) / tick_hz;
            while (next_event < script.events.size() && script.events[next_event].t <= t) {
                const ps_tools::ScriptEvent& e = script.events[next_event];
                if (e.starter) rig.set_starter(*e.starter);
                if (e.ignition) rig.set_ignition(*e.ignition);
                if (e.throttle) rig.set_throttle(*e.throttle);
                if (e.dyno_scale) rig.set_dyno_torque_scale(*e.dyno_scale);
                ++next_event;
            }

            if (replay.empty()) rig.step();
            const ps_audio::EngineInput cur = replay.empty()
                                                  ? ps_tools::make_engine_input(rig.engine_sound_state(), rig.sim_time_s())
                                                  : replay[static_cast<std::size_t>(tick)];
            {
                const ps_audio::EngineInput& in = cur;
                const long long period = args.hitch_every_s > 0.0 ? std::llround(args.hitch_every_s * tick_hz) : 0;
                if (period > 0 && tick > 0 && tick % period < args.hitch_ticks) {
                    withheld.push_back(in);  // diag: the input arrives late, all at once
                } else {
                    for (const auto& w : withheld) voice.push_input(w);
                    withheld.clear();
                    voice.push_input(in);
                }
            }
            voice.render(per_source_block.data(), frames_per_tick);
            for (int f = 0; f < frames_per_tick; ++f) {
                float sum = 0.0f;
                for (int s = 0; s < sources; ++s) {
                    sum += per_source_block[static_cast<std::size_t>(f) * static_cast<std::size_t>(sources) +
                                              static_cast<std::size_t>(s)];
                }
                mixed_block[static_cast<std::size_t>(f)] = sum * mix_gain;
            }
            rendered.insert(rendered.end(), mixed_block.begin(), mixed_block.end());
            if (diag != nullptr) {
                double ss = 0.0;
                for (float v : per_source_block) ss += static_cast<double>(v) * v;
                std::fprintf(diag, "%.6f,%.3f,%.5f,%.2f,%.2f,%.2f,%.2f,%.6e,%.5f,%llu,%.2f,%llu\n", cur.t_sim_s,
                             cur.omega_rad_s * 60.0 / 6.283185307179586, cur.throttle, cur.p_intake_pa,
                             cur.p_charge_pa, voice.engine().plenum_pressure_pa(), voice.engine().plenum_filtered_pa(),
                             voice.engine().throttle_cd_area(), std::sqrt(ss / per_source_block.size()),
                             static_cast<unsigned long long>(voice.published_hitch_count()), voice.take_max_crank_error_deg(),
                             static_cast<unsigned long long>(voice.take_crank_saturated_samples()));
            }
        }
        if (diag != nullptr) std::fclose(diag);

        const auto wall_end = std::chrono::steady_clock::now();
        const double wall_s = std::chrono::duration<double>(wall_end - wall_start).count();
        const double render_speed = wall_s > 0.0 ? duration_s / wall_s : 0.0;
        std::fprintf(stderr, "ps_engine_audio: rendered %.3f s of sim time in %.3f s wall time (%.2fx real time)\n",
                      duration_s, wall_s, render_speed);
        std::fprintf(stderr, "ps_engine_audio: %d source(s) mixed, sample rate %d Hz, %d channel(s)\n",
                      voice.source_count(), sample_rate_hz, channels);

        std::vector<std::int16_t> pcm16(rendered.size());
        for (std::size_t i = 0; i < rendered.size(); ++i) {
            const float v = std::max(-1.0f, std::min(1.0f, rendered[i]));
            pcm16[i] = static_cast<std::int16_t>(std::lround(v * 32767.0f));
        }
        ps_tools::write_wav_pcm16(args.wav_path, sample_rate_hz, channels, pcm16);
        std::fprintf(stderr, "ps_engine_audio: wrote '%s' (%zu frames)\n", args.wav_path.c_str(),
                      rendered.size() / static_cast<std::size_t>(channels));

        if (args.live) {
            play_buffer_live(rendered, channels, sample_rate_hz);
        }
    } catch (const std::exception& e) {
        // catch by const&, read via what() only here at the top level -
        // same accepted pattern as ps_run's own top-level handler
        // (CLAUDE.md's "Windows ASan caveat": a top-level catch-and-print
        // like this one is a known, accepted gap under the asan preset,
        // never exercised by any automated test since this tool is not
        // CTest-registered and ci.ps1 never invokes it - a manual
        // listening-test host, run by hand).
        std::fprintf(stderr, "ps_engine_audio: error: %s\n", e.what());
        return 1;
    }
    return 0;
}
