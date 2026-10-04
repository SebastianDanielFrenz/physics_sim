#include "engine_bank_generator.h"
#include "engine_sound_glue.h"
#include "drivetrain/engine_simulated.h"
#include "ps/io/vehicle_io.h"
#include "ps_audio/voice.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <set>
#include <stdexcept>
#include <thread>
namespace ps_tools {

// Generator identity (cache key). Planes: nominal rows 950/2500/4500/6500/8000
// rpm moved to rpm = 48000*120/round(48000*120/nominal) so one engine cycle is
// a whole number of frames; pedals 0.12/1.0; MVEM settle 1920 x 1/960 s; voice
// warm-up 240 ticks x 200 frames (1 s); loop = max(12, round(1.0 s * rpm/120))
// cycles; seam crossfade = whole cycles covering >= 2400 frames (50 ms);
// band-limit Blackman sinc 97 taps, -6 dB at 24000/ratio - 1500 Hz with ratio =
// next row / this row (top row 1.1); live dx x2, plenum PI 0.015 s / filter
// 0.003 s (the demo voice's settings).
const char* const kEngineAudioBankSettings =
    "engine-bank-v2:48000:950,2500,4500,6500,8000@whole-cycle-frames:0.12,1:mvem1920/960:warm240x200:"
    "loop1.0s-min12:xfade2400-whole-cycles:fir48-blackman-stop24000/ratio-1500-top1.1:dx2:plenum0.015/0.003";

namespace {
constexpr int kRate = 48000;
constexpr int kTickFrames = 200;  // 48000 / 240
constexpr int kWarmTicks = 240;
constexpr int kMinCycles = 12;
constexpr double kLoopSeconds = 1.0;
constexpr int kCrossfadeMinFrames = 2400;
constexpr int kFilterHalfTaps = 48;
constexpr double kTopRowRatio = 1.1;
constexpr double kTransitionHz = 1500.0;
constexpr double kNominalRpm[] = {950, 2500, 4500, 6500, 8000};
constexpr double kPedals[] = {0.12, 1.0};

void fold_dependencies(const std::filesystem::path& path,std::uint64_t& hash,std::set<std::string>& visited) {
    const auto absolute=std::filesystem::absolute(path).lexically_normal();
    if(!visited.insert(absolute.string()).second) return;
    std::ifstream in(absolute); if(!in) throw std::runtime_error("Missing audio-bank dependency: "+absolute.string());
    std::string content((std::istreambuf_iterator<char>(in)),{});
    for(unsigned char c:content) { hash^=c;hash*=1099511628211ULL; }
    const auto j=nlohmann::json::parse(content);
    auto walk=[&](auto&& self,const nlohmann::json& value)->void {
        if(value.is_string()) { const auto ref=value.get<std::string>();
            if(ref.size()>5 && ref.substr(ref.size()-5)==".json") fold_dependencies(absolute.parent_path()/ref,hash,visited);
        } else if(value.is_array() || value.is_object()) for(const auto& child:value) self(self,child);
    };walk(walk,j);
}

double wrap720(double d) {
    d = std::fmod(d, 720.0);
    return d < 0.0 ? d + 720.0 : d;
}

struct PlaneResult {
    std::vector<float> loop;
    double pressure = 0.0, crank0 = 0.0, removed = 0.0;
    int channels = 0;
};

// One plane: a steady-state recording of the live voice at the row's exact
// rpm, turned into a crank-aligned loop.
PlaneResult record_plane(const ps::drivetrain::SimulatedEngineDesc& desc, int row, int load, const std::function<bool()>& cancelled) {
    const int rows = static_cast<int>(std::size(kNominalRpm));
    auto fpc_of = [](int r) { return static_cast<int>(std::lround(kRate * 120.0 / kNominalRpm[r])); };
    const int fpc = fpc_of(row);
    const double rpm = kRate * 120.0 / fpc;
    const double ratio = row + 1 < rows ? (kRate * 120.0 / fpc_of(row + 1)) / rpm : kTopRowRatio;
    const double pedal = kPedals[load];
    const int cycles = std::max(kMinCycles, static_cast<int>(std::lround(kLoopSeconds * rpm / 120.0)));
    const int crossfade = fpc * std::max(1, (kCrossfadeMinFrames + fpc - 1) / fpc);

    ps::drivetrain::SimulatedEngine engine(desc);
    ps::drivetrain::Sensors sensors;sensors.omega=rpm*6.2831853071795864769/60.0;sensors.ignition=1;
    ps::drivetrain::PreSolveResult pre;
    // Below limiter while baking the high-speed texture; actual live cut flags remain authoritative.
    for(int i=0;i<1920;++i) { pre=engine.pre_solve(sensors,{pedal,0,1},{},1.0/960); (void)engine.post_solve(sensors.omega,1.0/960); }
    PlaneResult out;
    out.pressure = pre.p_intake_pa;
    ps_audio::VoiceSettings settings;settings.engine.threads=1;settings.engine.initial_rpm=rpm;
    settings.engine.initial_p_intake_pa=pre.p_intake_pa;settings.engine.plenum_pi_tau_s=0.015;settings.engine.plenum_filter_tau_s=0.003;
    auto cycle=*desc.cycle;cycle.resolution.audio_dx_intake_m*=2;cycle.resolution.audio_dx_exhaust_m*=2;
    ps_audio::Voice voice(cycle,desc.fuels[0].lhv_j_kg,desc.fuels[0].afr_stoich,settings);
    const int ch = voice.source_count();
    out.channels = ch;
    ps_audio::EngineInput input;input.omega_rad_s=sensors.omega;input.p_intake_pa=pre.p_intake_pa;
    input.p_charge_pa=pre.p_charge_pa;input.t_charge_k=pre.t_charge_k;input.lambda=pre.lambda;
    input.throttle=pre.throttle_position;input.spark_given=true;input.spark_deg_atdc=-pre.spark_advance_deg;
    // No crank_angle_deg: the voice runs open-loop at exactly omega, so the
    // crank of every recorded frame follows from the one read after warm-up.
    std::vector<float> block(static_cast<std::size_t>(kTickFrames*ch));
    const std::size_t need = static_cast<std::size_t>(2 * kFilterHalfTaps + cycles * fpc + crossfade);
    const int rec_ticks = static_cast<int>((need + kTickFrames - 1) / kTickFrames);
    std::vector<float> rec;
    rec.reserve(static_cast<std::size_t>(rec_ticks) * kTickFrames * static_cast<std::size_t>(ch));
    double crank_end_warm = 0.0;
    for(int tick=0;tick<kWarmTicks+rec_ticks;++tick) {
        if(cancelled && cancelled()) throw std::runtime_error("Audio-bank generation cancelled");
        input.t_sim_s=static_cast<double>(tick)/240.0;voice.push_input(input);voice.render(block.data(),kTickFrames);
        if(tick>=kWarmTicks) rec.insert(rec.end(),block.begin(),block.end());
        if(tick==kWarmTicks-1) crank_end_warm=voice.engine().crank_deg();
    }
    const double deg_per_frame = 720.0 / fpc;
    const double predicted = wrap720(crank_end_warm + static_cast<double>(rec_ticks) * kTickFrames * deg_per_frame);
    double err = std::abs(predicted - voice.engine().crank_deg());
    err = std::min(err, 720.0 - err);
    if (err > 1e-3) throw std::runtime_error("Audio-bank recording crank drifted from omega*t (" + std::to_string(err) + " deg)");
    rec.resize(need * static_cast<std::size_t>(ch));
    ps_audio::CrankLoopSpec spec;
    spec.frames_per_cycle = fpc;
    spec.cycles = cycles;
    spec.crossfade_frames = crossfade;
    spec.filter_half_taps = kFilterHalfTaps;
    spec.cutoff_hz = 0.5 * kRate / ratio - kTransitionHz;
    spec.rate_hz = kRate;
    ps_audio::CrankLoopStats st;
    out.loop = ps_audio::build_crank_loop(rec, ch, spec, &st);
    out.removed = st.removed_energy_fraction;
    // Loop frame 0 = recording frame kFilterHalfTaps (frame j sits at crank_end_warm + (j+1)*deg_per_frame).
    out.crank0 = wrap720(crank_end_warm + (kFilterHalfTaps + 1) * deg_per_frame);
    return out;
}
}  // namespace

std::uint64_t engine_audio_bank_key(const std::string& engine_path,const std::string& turbo_configuration_path,const std::string& settings) {
    std::uint64_t key=14695981039346656037ULL; std::set<std::string> visited;
    for(unsigned char c:settings) { key^=c;key*=1099511628211ULL; }
    fold_dependencies(engine_path,key,visited);
    if(!turbo_configuration_path.empty()) fold_dependencies(turbo_configuration_path,key,visited);
    return key;
}

ps_audio::AudioBankPtr prepare_engine_audio_bank(const std::string& engine_path,const std::string& turbo_configuration_path,const std::string& cache_dir, const std::function<bool()>& cancelled, EngineAudioBankStats* stats) {
    const auto t0=std::chrono::steady_clock::now();
    auto finish=[&](const ps_audio::AudioBankPtr& bank,bool hit) {
        if(stats) { stats->cache_hit=hit;stats->bytes=bank->samples.size()*sizeof(float);
            stats->seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count(); }
        return bank;
    };
    const std::uint64_t key=engine_audio_bank_key(engine_path,turbo_configuration_path);
    std::ostringstream hex;hex<<std::hex<<key;
    const auto file=std::filesystem::path(cache_dir)/"audio"/(hex.str()+".psab");
    // Any unreadable file (PSAB0001, another key, damage) is a silent miss.
    try { return finish(ps_audio::read_audio_bank(file.string(),key),true); } catch(...) {}
    if(cancelled && cancelled()) throw std::runtime_error("Audio-bank generation cancelled");
    ps::io::EngineMapOptions options;options.cache_dir=cache_dir;
    auto loaded=ps::io::load_engine_json(engine_path,options);
    auto desc=std::get<ps::drivetrain::SimulatedEngineDesc>(loaded);
    if(!turbo_configuration_path.empty()) desc.turbo_pair=ps::io::load_turbo_configuration_json(turbo_configuration_path,cache_dir);
    if(!desc.cycle) throw std::runtime_error("Audio bank requires a simulated cycle engine");
    const int rows=static_cast<int>(std::size(kNominalRpm));
    const int planes=rows*2;
    std::vector<PlaneResult> results(static_cast<std::size_t>(planes));
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(planes));
    // Planes are independent and each is deterministic: one thread per plane
    // up to the hardware, results stored by plane index (order-independent).
    // Half the hardware threads: the demo generates while the game, its sim
    // thread and the live voice's step team keep running.
    const unsigned hw=std::max(1u,std::thread::hardware_concurrency()/2);
    const int workers=static_cast<int>(std::min<unsigned>(hw,static_cast<unsigned>(planes)));
    std::atomic<int> next{0};
    auto work=[&] {
        for(int p=next++;p<planes;p=next++) {
            try { results[static_cast<std::size_t>(p)]=record_plane(desc,p/2,p%2,cancelled); }
            catch(...) { errors[static_cast<std::size_t>(p)]=std::current_exception(); }
        }
    };
    std::vector<std::thread> pool;
    for(int w=1;w<workers;++w) pool.emplace_back(work);
    work();
    for(auto& t:pool) t.join();
    for(const auto& e:errors) if(e) std::rethrow_exception(e);
    auto bank=std::make_shared<ps_audio::AudioBank>();bank->key=key;bank->rate=kRate;bank->channels=results[0].channels;
    for(int r=0;r<rows;++r) {
        const int fpc=static_cast<int>(std::lround(kRate*120.0/kNominalRpm[r]));
        bank->rpms.push_back(kRate*120.0/fpc);
        bank->frames_per_cycle.push_back(fpc);
        bank->cycles.push_back(static_cast<std::int32_t>(results[static_cast<std::size_t>(r*2)].loop.size()/static_cast<std::size_t>(fpc*bank->channels)));
    }
    if(stats) stats->removed_energy_fraction.clear();
    for(const auto& plane:results) {
        if(plane.channels!=bank->channels) throw std::runtime_error("Audio bank channel topology changed");
        bank->pressures.push_back(plane.pressure);
        bank->crank0_deg.push_back(plane.crank0);
        bank->samples.insert(bank->samples.end(),plane.loop.begin(),plane.loop.end());
        if(stats) stats->removed_energy_fraction.push_back(plane.removed);
    }
    bank->finalize();
    std::filesystem::create_directories(file.parent_path());
    std::ostringstream thread;thread<<std::this_thread::get_id();
    const auto temp=file.string()+"."+thread.str()+".tmp";
    ps_audio::write_audio_bank(*bank,temp);
    std::error_code error;std::filesystem::rename(temp,file,error);
    if(error) { std::filesystem::remove(file,error);error.clear();std::filesystem::rename(temp,file,error); }
    if(error) throw std::runtime_error("Cannot publish generated audio bank: "+error.message());
    return finish(bank,false);
}
}
