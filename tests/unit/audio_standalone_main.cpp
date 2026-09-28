// ps_audio_standalone - link-time guard for ps_audio (ps_audio A7): a plain
// main() linking ps_audio WHOLE-ARCHIVE plus ps_engine_cycle, never ps_core
// (tests/unit/CMakeLists.txt). Anything in ps_audio that starts needing
// ps_core's implementation (JobSystem, World, io) fails this target's link.
// It also renders a short block: a Voice on the [AS] pipe I4 fixture (coarse
// cells so the guard stays fast), a few snapshots, one Runner block.
#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"

#include "ps_audio/runner.h"
#include "ps_audio/voice.h"

#include <cmath>
#include <cstdio>
#include <vector>

int main() {
    using namespace ps;
    const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    ps_audio::VoiceSettings vs;
    vs.substeps = 1;
    vs.engine.dx_intake_m = 0.05;
    vs.engine.dx_exhaust_m = 0.05;
    vs.engine.initial_rpm = 2000.0;
    vs.engine.initial_p_intake_pa = 50000.0;
    ps_audio::Voice voice(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    for (int i = 0; i < 8; ++i) {
        ps_audio::EngineInput in;
        in.t_sim_s = i / 240.0;
        in.omega_rad_s = 2000.0 * 6.283185307179586 / 60.0;
        in.p_intake_pa = 50000.0;
        voice.push_input(in);
    }
    ps_audio::Runner runner({&voice});
    int blocks = 0;
    for (int b = 0; b < 4; ++b) blocks += runner.render_block() ? 1 : 0;
    std::vector<float> buf(static_cast<std::size_t>(runner.block_frames() * runner.channel_count()));
    std::size_t frames = 0;
    double peak = 0.0;
    bool finite = true;
    for (;;) {
        const std::size_t n = runner.read(buf.data(), static_cast<std::size_t>(runner.block_frames()));
        if (n == 0) break;
        frames += n;
        for (std::size_t i = 0; i < n * static_cast<std::size_t>(runner.channel_count()); ++i) {
            if (!std::isfinite(buf[i])) finite = false;
            peak = std::fabs(buf[i]) > peak ? std::fabs(buf[i]) : peak;
        }
    }
    std::printf("audio_standalone: %d sources, %d blocks, %zu frames, peak %.4g Pa, t_engine %.4f s\n",
                voice.source_count(), blocks, frames, peak, voice.engine().time_s());
    if (!finite || frames != static_cast<std::size_t>(4 * runner.block_frames()) || !(peak > 0.0)) {
        std::printf("audio_standalone: FAIL\n");
        return 1;
    }
    std::printf("audio_standalone: OK\n");
    return 0;
}
