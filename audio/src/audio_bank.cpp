#include "ps_audio/audio_bank.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>
namespace ps_audio {
namespace {
std::uint64_t checksum(const AudioBank& b) {
    std::uint64_t h=14695981039346656037ULL;
    auto fold=[&](const void* p,std::size_t n) { const auto* bytes=static_cast<const unsigned char*>(p);
        for(std::size_t i=0;i<n;++i) { h^=bytes[i];h*=1099511628211ULL; } };
    fold(b.rpms.data(),b.rpms.size()*sizeof(ps::real)); fold(b.pressures.data(),b.pressures.size()*sizeof(ps::real));
    fold(b.samples.data(),b.samples.size()*sizeof(float)); return h;
}
void validate(const AudioBank& b) {
    if(b.rate!=48000 || b.channels<1 || b.channels>8 || b.frames<4800 || b.frames>96000 || b.rpms.size()<2 || b.rpms.size()*2>kBankMaxPlanes ||
       b.pressures.size()!=b.rpms.size()*2 || b.samples.size()!=b.rpms.size()*2*static_cast<std::size_t>(b.frames)*static_cast<std::size_t>(b.channels))
        throw std::runtime_error("Invalid audio bank dimensions");
    double previous=0;
    for(auto rpm:b.rpms) { if(!std::isfinite(rpm) || rpm<=previous) throw std::runtime_error("Invalid audio bank RPM axis"); previous=rpm; }
    for(std::size_t i=0;i<b.pressures.size();i+=2) if(!std::isfinite(b.pressures[i]) || !std::isfinite(b.pressures[i+1]) || b.pressures[i]<=0 || b.pressures[i+1]<=b.pressures[i])
        throw std::runtime_error("Invalid audio bank pressure axis");
    for(float v:b.samples) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite audio bank sample");
}
}
AudioBankPtr read_audio_bank(const std::string& path,std::uint64_t expected_key) {
    std::ifstream in(path,std::ios::binary); if(!in) throw std::runtime_error("Missing audio bank");
    char magic[8]{}; in.read(magic,8); if(std::memcmp(magic,"PSAB0001",8)!=0) throw std::runtime_error("Audio bank version mismatch");
    auto b=std::make_shared<AudioBank>(); std::uint64_t sum=0; std::uint32_t n=0;
    in.read(reinterpret_cast<char*>(&b->key),8);in.read(reinterpret_cast<char*>(&sum),8);
    in.read(reinterpret_cast<char*>(&b->rate),4);in.read(reinterpret_cast<char*>(&b->channels),4);in.read(reinterpret_cast<char*>(&b->frames),4);in.read(reinterpret_cast<char*>(&n),4);
    if(!in || b->key!=expected_key || n<2 || n*2>kBankMaxPlanes || b->frames<4800 || b->frames>96000 || b->channels<1 || b->channels>8)
        throw std::runtime_error("Audio bank key/dimension mismatch");
    b->rpms.resize(n); b->pressures.resize(n*2); b->samples.resize(static_cast<std::size_t>(n)*2*static_cast<std::size_t>(b->frames)*static_cast<std::size_t>(b->channels));
    in.read(reinterpret_cast<char*>(b->rpms.data()),static_cast<std::streamsize>(b->rpms.size()*sizeof(ps::real)));
    in.read(reinterpret_cast<char*>(b->pressures.data()),static_cast<std::streamsize>(b->pressures.size()*sizeof(ps::real)));
    in.read(reinterpret_cast<char*>(b->samples.data()),static_cast<std::streamsize>(b->samples.size()*sizeof(float)));
    if(!in || checksum(*b)!=sum) throw std::runtime_error("Audio bank checksum mismatch"); validate(*b); return b;
}
void write_audio_bank(const AudioBank& b,const std::string& path) {
    validate(b); std::ofstream out(path,std::ios::binary); if(!out) throw std::runtime_error("Cannot save audio bank");
    const auto sum=checksum(b);const auto n=static_cast<std::uint32_t>(b.rpms.size());
    out.write("PSAB0001",8);out.write(reinterpret_cast<const char*>(&b.key),8);out.write(reinterpret_cast<const char*>(&sum),8);
    out.write(reinterpret_cast<const char*>(&b.rate),4);out.write(reinterpret_cast<const char*>(&b.channels),4);out.write(reinterpret_cast<const char*>(&b.frames),4);out.write(reinterpret_cast<const char*>(&n),4);
    out.write(reinterpret_cast<const char*>(b.rpms.data()),static_cast<std::streamsize>(b.rpms.size()*sizeof(ps::real)));
    out.write(reinterpret_cast<const char*>(b.pressures.data()),static_cast<std::streamsize>(b.pressures.size()*sizeof(ps::real)));
    out.write(reinterpret_cast<const char*>(b.samples.data()),static_cast<std::streamsize>(b.samples.size()*sizeof(float)));
    if(!out) throw std::runtime_error("Audio bank write failed");
}
float AudioBank::sample(int plane,int channel,double p) const {
    // 25 ms overlap hides loop joins; phase wraps to the end of the overlap.
    constexpr int overlap=1200;
    const int a=static_cast<int>(p); const int next=std::min(a+1,frames-1);const double f=p-a;
    auto at=[&](int frame) { return samples[(static_cast<std::size_t>(plane)*static_cast<std::size_t>(frames)+static_cast<std::size_t>(frame))*static_cast<std::size_t>(channels)+static_cast<std::size_t>(channel)]; };
    float v=static_cast<float>(at(a)*(1-f)+at(next)*f);
    if(a>=frames-overlap) { const double head=p-(frames-overlap);const int i=static_cast<int>(head);
        const double hf=head-i;const float hv=static_cast<float>(at(i)*(1-hf)+at(std::min(i+1,overlap))*hf);
        const double blend=head/overlap;v=static_cast<float>(v*(1-blend)+hv*blend); }
    return v;
}
void BankPlayback::frame(const AudioBank& b,double rpm,double pressure,float* out) {
    std::size_t lo=0;while(lo+1<b.rpms.size() && b.rpms[lo+1]<rpm) ++lo;
    const std::size_t hi=std::min(lo+1,b.rpms.size()-1);
    const double mix=hi==lo?0:std::clamp((rpm-b.rpms[lo])/(b.rpms[hi]-b.rpms[lo]),0.0,1.0);
    auto load=[&](std::size_t row) { return std::clamp((pressure-b.pressures[row*2])/(b.pressures[row*2+1]-b.pressures[row*2]),0.0,1.0); };
    const double a=load(lo),d=load(hi);
    for(int c=0;c<b.channels;++c) {
        const double l=b.sample(static_cast<int>(lo*2),c,phase[lo*2])*(1-a)+b.sample(static_cast<int>(lo*2+1),c,phase[lo*2+1])*a;
        const double h=b.sample(static_cast<int>(hi*2),c,phase[hi*2])*(1-d)+b.sample(static_cast<int>(hi*2+1),c,phase[hi*2+1])*d;
        out[c]=static_cast<float>((l*(1-mix)+h*mix)*std::clamp(rpm/400.0,0.0,1.0));
    }
    for(std::size_t i=0;i<b.rpms.size()*2;++i) {
        phase[i]+=std::max(0.0,rpm)/b.rpms[i/2];
        while(phase[i]>=b.frames) phase[i]-=b.frames-1200;
    }
}
}
