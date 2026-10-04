#pragma once
#include "ps_audio/runner.h"
#include <memory>
#include <mutex>
namespace ps_audio {
struct SynthesisStats { std::uint64_t blocks=0,render_ns=0,max_block_ns=0,rejected_reads=0; int block_frames=0; };
struct OutputSource {
    virtual ~OutputSource() = default;
    virtual std::size_t read(float*,std::size_t)=0;
    virtual int channels() const=0;
    virtual int sample_rate() const=0;
    virtual bool connected() const=0;
    virtual SynthesisStats synthesis_stats() const { return {}; }
};
// Owner calls invalidate BEFORE destroying Runner. Leases can safely outlive it.
class RunnerOutput : public std::enable_shared_from_this<RunnerOutput> {
public:
    RunnerOutput(Runner& runner,int rate): runner_(&runner),rate_(rate),channels_(runner.channel_count()) {}
    ~RunnerOutput() { invalidate(); }
    std::shared_ptr<OutputSource> acquire();
    void invalidate();
    bool connected() const { std::lock_guard<std::mutex> lock(mutex_); return runner_!=nullptr; }
private:
    class Source;
    mutable std::mutex mutex_;
    Runner* runner_;
    int rate_,channels_;
    std::uint64_t owner_=0;
};
struct OutputPumpStats {
    std::uint64_t source_short_reads=0, source_missing_frames=0;
    std::uint64_t output_short_reads=0, output_missing_frames=0, frames_delivered=0;
    std::size_t queued_frames=0;
};
// One producer drains the exclusive source independently of rendering.
// Device callback is the ONLY consumer of read(); stop it before destruction.
class OutputPump {
public:
    OutputPump(std::shared_ptr<OutputSource>,int target_latency_ms=12,float gain=1);
    ~OutputPump();
    void start();
    void stop();
    std::size_t read(float*,std::size_t);
    void set_gain(float);
    OutputPumpStats stats() const;
    SynthesisStats synthesis_stats() const { return source_->synthesis_stats(); }
    bool source_connected() const { return source_->connected(); }
private:
    void loop();
    std::shared_ptr<OutputSource> source_;
    const std::size_t channels_,target_frames_,max_frames_;
    std::atomic<std::size_t> device_frames_{0};
    SpscFloatRing ring_;
    std::vector<float> scratch_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<float> gain_{1};
    std::atomic<std::uint64_t> source_short_{0},source_missing_{0},output_short_{0},output_missing_{0},delivered_{0};
};
}
