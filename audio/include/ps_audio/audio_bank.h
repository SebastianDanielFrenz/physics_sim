#pragma once
#include "ps/types.h"
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace ps_audio {
inline constexpr int kBankMaxPlanes=32;
struct AudioBank {
    int rate=48000, channels=0, frames=24000;
    std::uint64_t key=0;
    std::vector<ps::real> rpms, pressures;
    std::vector<float> samples; // rpm-major, two load planes, frame-major/channel-minor; Pa
    float sample(int plane,int channel,double phase) const;
};
using AudioBankPtr=std::shared_ptr<const AudioBank>;
AudioBankPtr read_audio_bank(const std::string& path,std::uint64_t expected_key);
void write_audio_bank(const AudioBank& bank,const std::string& path);
class BankPlayback {
public:
    std::array<double,kBankMaxPlanes> phase{};
    void frame(const AudioBank& bank,double rpm,double pressure,float* out);
};
}
