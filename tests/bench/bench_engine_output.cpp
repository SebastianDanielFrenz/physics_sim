#include "vehicle_voice.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include <cstdio>
#include <cmath>
#include <vector>
int main(int argc,char** argv) {
    if(argc<2) return 2;
    ps::jobs::JobSystem jobs(10); ps::io::EngineMapOptions opt; opt.pool=&jobs;
    const auto desc=ps::io::load_vehicle_json(argv[1],opt);
    for(double rpm : {950.,3000.,7500.}) {
        std::string reason;
        auto voice=ps_godot::VehicleVoice::try_create(desc,reason);
        if(!voice) { std::printf("%s\n",reason.c_str());return 3; }
        auto& runner=voice->runner();
        std::vector<float> buf(256*static_cast<std::size_t>(runner.channel_count()));
        ps::drivetrain::EngineSoundState state;
        state.engine_state=ps::drivetrain::EngineState::Running;
        state.omega=rpm*6.283185307179586/60; state.p_intake_pa=rpm<1000 ? 50000 : 271000;
        state.p_charge_pa=281325; state.t_charge_k=294.25; state.lambda=.83;
        state.spark_advance_deg=18; state.throttle_position=rpm<1000 ? .1 : 1;
        bool finite=true;
        std::uint64_t warm_ns=0;
        for(int b=0;b<90;++b) {
            const double t=b*256./48000.; state.crank_angle=std::fmod(t*state.omega,12.566370614359172);
            voice->push_tick(state,t);
            if(!runner.render_block()) return 4;
            if(runner.read(buf.data(),256)!=256) return 5;
            for(float x:buf) finite=finite && std::isfinite(x);
            if(b==29) warm_ns=runner.render_ns();
        }
        const double us_per_frame=(runner.render_ns()-warm_ns)/1000./(60*256);
        std::printf("rpm=%.0f live_us_per_frame=%.3f budget_us=20.833 realtime_ratio=%.3f max_block_ms=%.3f finite=%d\n",rpm,us_per_frame,us_per_frame/20.833333333,runner.max_block_render_ns()/1.e6,finite);
        if(!finite) return 6;
    }
}
