// ps_audio/runner.cpp - see runner.h.
#include "ps_audio/runner.h"

#include <chrono>
#include <stdexcept>

namespace ps_audio {

namespace {

int count_channels(const std::vector<Voice*>& voices) {
    int n = 0;
    for (const Voice* v : voices) {
        if (v == nullptr) throw std::invalid_argument("Runner: null voice");
        n += v->source_count();
    }
    if (n < 1) throw std::invalid_argument("Runner: no voices");
    return n;
}

const RunnerSettings& checked(const RunnerSettings& s) {
    if (s.block_frames < 1 || s.ring_blocks < 2 || s.idle_sleep_us < 0)
        throw std::invalid_argument("Runner: block_frames >= 1, ring_blocks >= 2, idle_sleep_us >= 0");
    return s;
}

}  // namespace

Runner::Runner(const std::vector<Voice*>& voices, const RunnerSettings& settings)
    : s_(checked(settings)),
      voices_(voices),
      channels_(count_channels(voices)),
      ring_(static_cast<std::size_t>(settings.block_frames) * static_cast<std::size_t>(settings.ring_blocks) *
            static_cast<std::size_t>(channels_)) {
    int max_src = 0;
    for (const Voice* v : voices_) max_src = v->source_count() > max_src ? v->source_count() : max_src;
    voice_buf_.assign(static_cast<std::size_t>(s_.block_frames) * static_cast<std::size_t>(max_src), 0.0f);
    block_.assign(static_cast<std::size_t>(s_.block_frames) * channels_u(), 0.0f);
}

Runner::~Runner() { stop(); }

void Runner::start() {
    if (running_.load(std::memory_order_acquire)) return;
    stop_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this] { thread_main(); });
}

void Runner::stop() {
    if (!thread_.joinable()) return;
    stop_.store(true, std::memory_order_release);
    thread_.join();
    running_.store(false, std::memory_order_release);
}

std::size_t Runner::read(float* out, std::size_t frames) {
    return read_owned(out,frames,0);
}
std::uint64_t Runner::claim_consumer() {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    if(consumer_owner_) return 0;
    consumer_owner_=next_owner_++; return consumer_owner_;
}
void Runner::release_consumer(std::uint64_t owner) {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    if(consumer_owner_==owner) consumer_owner_=0;
}
std::size_t Runner::read_owned(float* out,std::size_t frames,std::uint64_t owner) {
    std::lock_guard<std::mutex> lock(consumer_mutex_);
    if(owner!=consumer_owner_) { ++rejected_reads_; return 0; }
    const std::size_t n = ring_.read(out, frames * channels_u());
    return n / channels_u();
}

bool Runner::render_block() {
    const std::size_t bf = static_cast<std::size_t>(s_.block_frames);
    if (ring_.writable() < bf * channels_u()) return false;
    const auto render_begin=std::chrono::steady_clock::now();
    std::size_t ch0 = 0;
    for (Voice* v : voices_) {
        const std::size_t ns = static_cast<std::size_t>(v->source_count());
        v->render(voice_buf_.data(), s_.block_frames);
        for (std::size_t f = 0; f < bf; ++f)
            for (std::size_t s = 0; s < ns; ++s) block_[f * channels_u() + ch0 + s] = voice_buf_[f * ns + s];
        ch0 += ns;
    }
    ring_.write(block_.data(), bf * channels_u());
    const auto elapsed=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-render_begin).count());
    render_ns_.fetch_add(elapsed);
    auto maximum=max_block_ns_.load();
    while(elapsed>maximum && !max_block_ns_.compare_exchange_weak(maximum,elapsed)) {}
    blocks_.fetch_add(1, std::memory_order_acq_rel);
    return true;
}

void Runner::thread_main() {
    while (!stop_.load(std::memory_order_acquire)) {
        if (!render_block()) std::this_thread::sleep_for(std::chrono::microseconds(s_.idle_sleep_us));
    }
}

}  // namespace ps_audio
