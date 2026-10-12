#include "vehicle_voice.h"

#include "engine_sound_glue.h"

#include <cstdio>
#include <cstdlib>
#include <variant>

namespace {
// MSVC-family compilers deprecate fopen/getenv (warnings are errors here).
std::FILE* open_for_write(const std::string& path, const char* mode = "w") {
#if defined(_MSC_VER)
    std::FILE* f = nullptr;
    return fopen_s(&f, path.c_str(), mode) == 0 ? f : nullptr;
#else
    return std::fopen(path.c_str(), mode);
#endif
}
std::string env_value(const char* name) {
#if defined(_MSC_VER)
    char* v = nullptr;
    std::size_t n = 0;
    std::string out;
    if (_dupenv_s(&v, &n, name) == 0 && v != nullptr) out = v;
    std::free(v);
    return out;
#else
    const char* v = std::getenv(name);
    return v != nullptr ? std::string(v) : std::string();
#endif
}
} // namespace

namespace ps_godot {

std::unique_ptr<VehicleVoice> VehicleVoice::try_create(const ps::vehicle::VehicleDesc& desc, std::string& out_reason,
                                                         const ps_audio::VoiceSettings& settings, ps::real dx_scale) {
    const ps::drivetrain::SimulatedEngineDesc* sim = nullptr;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
            sim = s;
            break; // a vehicle has at most one engine component
        }
    }
    if (sim == nullptr) {
        out_reason = "no \"kind\": \"simulated\" engine component (a torque-map engine has no live cycle model)";
        return nullptr;
    }
    if (!sim->cycle.has_value()) {
        out_reason = "the simulated engine has no \"cycle\" block (maps-only, no live pipe-network model to render)";
        return nullptr;
    }
    if (sim->fuels.empty()) {
        out_reason = "the simulated engine has no fuel";
        return nullptr;
    }
    if (sim->has_cam_profiles()) {
        // Engine garage P9: the live voice renders ONE cylinder model; switching it is open point G10.
        out_reason = "the simulated engine switches cam profiles (engine garage P9) - the live voice has no cam switching yet (open point G10)";
        return nullptr;
    }
    out_reason = "ok";

    // ps_audio A11 live dx (see vehicle_voice.h's header comment for the
    // measurement): a MUTABLE COPY of the engine's own cycle desc, scaled
    // here only - the loaded VehicleDesc (physics, map generation, the
    // offline WAV tools) is never touched.
    ps::drivetrain::CycleModelDesc live_cycle = *sim->cycle;
    live_cycle.resolution.audio_dx_intake_m *= dx_scale;
    live_cycle.resolution.audio_dx_exhaust_m *= dx_scale;

    // VehicleVoice's private ctor is only reachable through this factory,
    // so std::make_unique cannot be used (it needs a public ctor) -
    // new+unique_ptr is the ordinary workaround.
    ps_audio::VoiceSettings live_settings = settings;
    // The sound network follows the already-integrated MVEM manifold.
    // Avoid adding a second slow throttle response solely in the renderer.
    live_settings.engine.plenum_pi_tau_s = 0.015;
    live_settings.engine.plenum_filter_tau_s = 0.003;

    // The V8 exceeds a single audio worker's budget; reuse the existing
    // deterministic step team rather than altering the engine physics.
    if (live_cycle.cylinders >= 8 && live_settings.engine.threads == 1)
        live_settings.engine.threads = 4;
    return std::unique_ptr<VehicleVoice>(
        new VehicleVoice(live_cycle, sim->fuels[0].lhv_j_kg, sim->fuels[0].afr_stoich, live_settings));
}

VehicleVoice::VehicleVoice(const ps::drivetrain::CycleModelDesc& cycle, ps::real lhv_j_kg, ps::real afr_stoich,
                            const ps_audio::VoiceSettings& settings)
    : voice_(cycle, lhv_j_kg, afr_stoich, settings),
      // 4 x 256 frames = 21 ms of ring (default 8 = 43 ms): the ring stays
      // full, so its whole length is latency between an rpm change and the
      // speaker (owner 2026-09-29). The render thread refills a 256-frame
      // block in ~3.5 ms (0.66 core), far inside the ring.
      runner_(std::vector<ps_audio::Voice*>{&voice_}, ps_audio::RunnerSettings{256, 4, 500}) {
    output_=std::make_shared<ps_audio::RunnerOutput>(runner_,static_cast<int>(sample_rate_hz()));
    const std::string record_path = env_value("PS_VOICE_INPUT_CSV");
    const std::string output_path = env_value("PS_VOICE_OUTPUT_RAW");
    if (!output_path.empty() && (output_file_ = open_for_write(output_path, "wb")) != nullptr) {
        output_reads_file_ = open_for_write(output_path + ".reads.csv");
        if (output_reads_file_ != nullptr) std::fprintf(output_reads_file_, "steady_us,frames\n");
    }
    if (!record_path.empty() && (record_file_ = open_for_write(record_path)) != nullptr)
        std::fprintf(record_file_, "t_sim_s,omega_rad_s,crank_angle_deg,p_charge_pa,t_charge_k,p_intake_pa,throttle,lambda,"
                                   "spark_deg_atdc,spark_given,flags\n");
}

VehicleVoice::~VehicleVoice() {
    output_->invalidate();
    runner_.stop();
    if (record_file_ != nullptr) std::fclose(record_file_);
    if (output_file_ != nullptr) std::fclose(output_file_);
    if (output_reads_file_ != nullptr) std::fclose(output_reads_file_);
}

void VehicleVoice::push_tick(const ps::drivetrain::EngineSoundState& ess, ps::real t_sim_s) {
    const ps_audio::EngineInput in = ps_tools::make_engine_input(ess, t_sim_s);
    voice_.push_input(in);
    if (record_file_ != nullptr) {
        // Written as it goes and flushed once per 240 ticks, so a killed process keeps its data.
        std::fprintf(record_file_, "%.9f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%u\n", in.t_sim_s, in.omega_rad_s,
                     in.crank_angle_deg, in.p_charge_pa, in.t_charge_k, in.p_intake_pa, in.throttle, in.lambda,
                     in.spark_deg_atdc, in.spark_given ? 1 : 0, static_cast<unsigned>(in.flags));
        if (++recorded_rows_ % 240u == 0u) std::fflush(record_file_);
    }
}

void VehicleVoice::record_output(const float* samples, std::size_t frames) {
    if (output_file_ == nullptr) return;
    std::fwrite(samples, sizeof(float), frames * static_cast<std::size_t>(channel_count()), output_file_);
    if (output_reads_file_ != nullptr) {
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
        std::fprintf(output_reads_file_, "%lld,%zu\n", static_cast<long long>(us), frames);
    }
}

} // namespace ps_godot
