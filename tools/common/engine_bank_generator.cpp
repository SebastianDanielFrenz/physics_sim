#include "engine_bank_generator.h"
#include "engine_sound_glue.h"
#include "drivetrain/engine_simulated.h"
#include "ps/io/vehicle_io.h"
#include "ps_audio/voice.h"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <set>
#include <stdexcept>
#include <thread>
namespace ps_tools {
namespace {
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
}
ps_audio::AudioBankPtr prepare_engine_audio_bank(const std::string& engine_path,const std::string& turbo_configuration_path,const std::string& cache_dir, const std::function<bool()>& cancelled) {
    std::uint64_t key=14695981039346656037ULL; std::set<std::string> visited;
    // Generator version, grid, sample rate and warmup are part of cache identity.
    const std::string bank_settings="engine-bank-v1:48000:950,2500,4500,6500,8000:0.12,1:24000:2s";
    for(unsigned char c:bank_settings) { key^=c;key*=1099511628211ULL; }
    fold_dependencies(engine_path,key,visited);
    if(!turbo_configuration_path.empty()) fold_dependencies(turbo_configuration_path,key,visited);
    std::ostringstream hex;hex<<std::hex<<key;
    const auto file=std::filesystem::path(cache_dir)/"audio"/(hex.str()+".psab");
    try { return ps_audio::read_audio_bank(file.string(),key); } catch(...) {}
    ps::io::EngineMapOptions options;options.cache_dir=cache_dir;
    auto loaded=ps::io::load_engine_json(engine_path,options);
    auto desc=std::get<ps::drivetrain::SimulatedEngineDesc>(loaded);
    if(!turbo_configuration_path.empty()) desc.turbo_pair=ps::io::load_turbo_configuration_json(turbo_configuration_path,cache_dir);
    if(!desc.cycle) throw std::runtime_error("Audio bank requires a simulated cycle engine");
    auto bank=std::make_shared<ps_audio::AudioBank>();bank->key=key;bank->rpms={950,2500,4500,6500,8000};
    for(auto rpm:bank->rpms) for(double pedal:{0.12,1.0}) {
        if(cancelled && cancelled()) throw std::runtime_error("Audio-bank generation cancelled");
        ps::drivetrain::SimulatedEngine engine(desc);
        ps::drivetrain::Sensors sensors;sensors.omega=rpm*6.2831853071795864769/60.0;sensors.ignition=1;
        ps::drivetrain::PreSolveResult pre;
        // Below limiter while baking the high-speed texture; actual live cut flags remain authoritative.
        for(int i=0;i<1920;++i) { pre=engine.pre_solve(sensors,{pedal,0,1},{},1.0/960); (void)engine.post_solve(sensors.omega,1.0/960); }
        bank->pressures.push_back(pre.p_intake_pa);
        ps_audio::VoiceSettings settings;settings.engine.threads=1;settings.engine.initial_rpm=rpm;
        settings.engine.initial_p_intake_pa=pre.p_intake_pa;settings.engine.plenum_pi_tau_s=0.015;settings.engine.plenum_filter_tau_s=0.003;
        auto cycle=*desc.cycle;cycle.resolution.audio_dx_intake_m*=2;cycle.resolution.audio_dx_exhaust_m*=2;
        ps_audio::Voice voice(cycle,desc.fuels[0].lhv_j_kg,desc.fuels[0].afr_stoich,settings);
        if(bank->channels==0) bank->channels=voice.source_count();
        if(bank->channels!=voice.source_count()) throw std::runtime_error("Audio bank channel topology changed");
        ps_audio::EngineInput input;input.omega_rad_s=sensors.omega;input.p_intake_pa=pre.p_intake_pa;
        input.p_charge_pa=pre.p_charge_pa;input.t_charge_k=pre.t_charge_k;input.lambda=pre.lambda;
        input.throttle=pre.throttle_position;input.spark_given=true;input.spark_deg_atdc=-pre.spark_advance_deg;
        std::vector<float> block(static_cast<std::size_t>(200*bank->channels));
        for(int tick=0;tick<360;++tick) {
            if(cancelled && cancelled()) throw std::runtime_error("Audio-bank generation cancelled");
            input.t_sim_s=static_cast<double>(tick)/240.0;voice.push_input(input);voice.render(block.data(),200);
            if(tick>=240) bank->samples.insert(bank->samples.end(),block.begin(),block.end());
        }
    }
    // Each plane contains the final half-second, retaining all radiating channels.
    std::filesystem::create_directories(file.parent_path());
    std::ostringstream thread;thread<<std::this_thread::get_id();
    const auto temp=file.string()+"."+thread.str()+".tmp";
    ps_audio::write_audio_bank(*bank,temp);
    std::error_code error;std::filesystem::rename(temp,file,error);
    if(error) { std::filesystem::remove(file,error);error.clear();std::filesystem::rename(temp,file,error); }
    if(error) throw std::runtime_error("Cannot publish generated audio bank: "+error.message());
    return bank;
}
}
