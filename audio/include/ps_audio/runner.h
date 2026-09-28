// ps_audio/runner.h - the audio render thread (ps_audio A7, out/audio_plan.md
// section 4 "Thread model"): ONE thread renders every voice in 256-frame
// blocks into one lock-free SPSC float ring; the device side (A10) or a test
// pulls with read(). No device code here.
//
// Ring layout: interleaved frames of channel_count() floats, channel order =
// voice order, then each voice's sources in order. Pressure at 1 m [Pa] per
// source (PROVISIONAL placeholder radiation; A8 calibrates).
//
// render_block() renders one block on the caller's thread (tests, offline
// render) - never while the thread runs. The Runner does not own the voices;
// they must outlive it. Allocation-free after construction on both sides.
#pragma once

#include "ps_audio/spsc.h"
#include "ps_audio/voice.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

namespace ps_audio {

struct RunnerSettings {
    int block_frames = 256;
    int ring_blocks = 8;          // ring capacity in blocks
    int idle_sleep_us = 500;      // thread sleep while the ring is full
};

class Runner {
public:
    Runner(const std::vector<Voice*>& voices, const RunnerSettings& settings = {});
    ~Runner();
    Runner(const Runner&) = delete;
    Runner& operator=(const Runner&) = delete;

    void start();  // spawns the render thread (no-op if running)
    void stop();   // joins it

    // Consumer (device) side: up to `frames` frames into out (frames *
    // channel_count() floats); returns the frames read.
    std::size_t read(float* out, std::size_t frames);
    [[nodiscard]] std::size_t frames_available() const { return ring_.readable() / channels_u(); }

    // Synchronous: render one block of every voice into the ring (false when
    // the ring has no room). Only while the thread is not running.
    bool render_block();

    [[nodiscard]] int channel_count() const { return channels_; }
    [[nodiscard]] int block_frames() const { return s_.block_frames; }
    [[nodiscard]] std::uint64_t blocks_rendered() const { return blocks_.load(std::memory_order_acquire); }
    [[nodiscard]] bool running() const { return running_.load(std::memory_order_acquire); }

private:
    [[nodiscard]] std::size_t channels_u() const { return static_cast<std::size_t>(channels_); }
    void thread_main();

    RunnerSettings s_;
    std::vector<Voice*> voices_;
    int channels_ = 0;
    SpscFloatRing ring_;
    std::vector<float> voice_buf_;  // one voice's block (block_frames * its sources)
    std::vector<float> block_;      // interleaved block of all channels
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
    std::atomic<std::uint64_t> blocks_{0};
};

}  // namespace ps_audio
