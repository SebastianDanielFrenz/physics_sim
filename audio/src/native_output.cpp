#include "ps_audio/native_output.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
namespace ps_audio {
class RunnerOutput::Source final : public OutputSource {
public:
    Source(std::shared_ptr<RunnerOutput> bridge,std::uint64_t owner):bridge_(std::move(bridge)),owner_(owner) {}
    ~Source() override {
        std::lock_guard<std::mutex> lock(bridge_->mutex_);
        if(bridge_->runner_) bridge_->runner_->release_consumer(owner_);
        if(bridge_->owner_==owner_) bridge_->owner_=0;
    }
    std::size_t read(float* out,std::size_t frames) override {
        std::lock_guard<std::mutex> lock(bridge_->mutex_);
        return bridge_->runner_ ? bridge_->runner_->read_owned(out,frames,owner_) : 0;
    }
    int channels() const override { return bridge_->channels_; }
    int sample_rate() const override { return bridge_->rate_; }
    SynthesisStats synthesis_stats() const override {
        std::lock_guard<std::mutex> lock(bridge_->mutex_);
        const auto* r=bridge_->runner_;
        return r ? SynthesisStats{r->blocks_rendered(),r->render_ns(),r->max_block_render_ns(),r->rejected_reads(),r->block_frames()} : SynthesisStats{};
    }
    bool connected() const override {
        std::lock_guard<std::mutex> lock(bridge_->mutex_); return bridge_->runner_!=nullptr;
    }
private:
    std::shared_ptr<RunnerOutput> bridge_;
    std::uint64_t owner_;
};
std::shared_ptr<OutputSource> RunnerOutput::acquire() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!runner_ || owner_) return {};
    owner_=runner_->claim_consumer();
    if(!owner_) return {};
    try { return std::make_shared<Source>(shared_from_this(),owner_); }
    catch(...) { runner_->release_consumer(owner_); owner_=0; throw; }
}
void RunnerOutput::invalidate() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(runner_ && owner_) runner_->release_consumer(owner_);
    runner_=nullptr; owner_=0;
}
namespace {
std::size_t target(const std::shared_ptr<OutputSource>& s,int ms) {
    if(!s || s->channels()<1 || s->channels()>16 || s->sample_rate()<8000 || s->sample_rate()>192000 || ms<5 || ms>50)
        throw std::invalid_argument("OutputPump: source, 8-192 kHz, 1-16 channels, latency 5-50 ms required");
    return static_cast<std::size_t>(s->sample_rate())*static_cast<std::size_t>(ms)/1000;
}
}
OutputPump::OutputPump(std::shared_ptr<OutputSource> s,int ms,float gain)
    : source_(std::move(s)),channels_(source_ ? static_cast<std::size_t>(source_->channels()) : 0),
      target_frames_(target(source_,ms)),max_frames_(static_cast<std::size_t>(source_->sample_rate())/20),ring_(max_frames_*channels_),scratch_(256*channels_) {
    set_gain(gain);
}
OutputPump::~OutputPump() { stop(); }
void OutputPump::start() {
    if(thread_.joinable()) return;
    stop_=false; thread_=std::thread([this]{loop();});
}
void OutputPump::stop() { stop_=true; if(thread_.joinable()) thread_.join(); }
void OutputPump::set_gain(float gain) {
    if(!std::isfinite(gain) || gain<0 || gain>64) throw std::invalid_argument("OutputPump: gain must be finite, 0-64");
    gain_=gain;
}
void OutputPump::loop() {
    float current_gain=0; // short startup fade; later gain changes use the same 10 ms smoothing
    while(!stop_.load() && source_->connected()) {
        const auto queued=ring_.readable()/channels_;
        const auto required=std::min(max_frames_,std::max(target_frames_,device_frames_.load()+static_cast<std::size_t>(source_->sample_rate())/500));
        if(queued>=required) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
        const auto requested=std::min(std::size_t(256),required-queued);
        const auto got=source_->read(scratch_.data(),requested);
        if(got<requested) { ++source_short_; source_missing_.fetch_add(requested-got); }
        const float goal=gain_.load();
        const float smoothing=1.f/(.01f*static_cast<float>(source_->sample_rate()));
        for(std::size_t f=0;f<got;++f) {
            current_gain += smoothing*(goal-current_gain);
            for(std::size_t c=0;c<channels_;++c) {
                auto& sample=scratch_[f*channels_+c]; sample=std::isfinite(sample) ? sample*current_gain : 0;
            }
        }
        ring_.write(scratch_.data(),got*channels_);
        if(got<requested) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
std::size_t OutputPump::read(float* out,std::size_t frames) {
    device_frames_.store(frames);
    const auto got=ring_.read(out,frames*channels_)/channels_;
    delivered_.fetch_add(got);
    if(got<frames) { ++output_short_; output_missing_.fetch_add(frames-got); }
    return got;
}
OutputPumpStats OutputPump::stats() const {
    return {source_short_.load(),source_missing_.load(),output_short_.load(),output_missing_.load(),delivered_.load(),std::min(max_frames_,ring_.readable_snapshot()/channels_)};
}
}
