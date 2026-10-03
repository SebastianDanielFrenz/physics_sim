#pragma once
#include "ps_audio/audio_bank.h"
#include <string>
#include <functional>
namespace ps_tools {
ps_audio::AudioBankPtr prepare_engine_audio_bank(const std::string& engine_path,const std::string& turbo_configuration_path,const std::string& cache_dir, const std::function<bool()>& cancelled = {});
}
