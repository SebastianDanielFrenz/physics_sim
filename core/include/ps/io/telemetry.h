// ps/io/telemetry.h — named channel registry with CSV output through an IO
// thread/queue (PLAN.md section 5: "Telemetry: any component can register
// named channels; runner writes CSV"; D8: "telemetry/file IO go through a
// queue to an IO thread").

#pragma once

#include "ps/types.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ps::io {

class TelemetryRegistry {
public:
    TelemetryRegistry() = default;
    ~TelemetryRegistry();
    TelemetryRegistry(const TelemetryRegistry&) = delete;

    // Registers a channel by name, returns its stable column index. Call
    // during setup (World/scenario construction), not per tick - this may
    // allocate and is not itself a hot-path job.
    std::size_t register_channel(const std::string& name);
    [[nodiscard]] const std::vector<std::string>& channel_names() const { return names_; }

    // Opens csv_path and starts the IO thread that drains recorded rows to
    // it (header row = "tick,sim_time,<channel names...>"). No-op if
    // already started.
    void start_csv(const std::string& csv_path);

    // Flushes and stops the IO thread, closing the file. Safe to call more
    // than once / never having started.
    void stop();

    // Enqueues one row (values.size() must equal channel_names().size());
    // never blocks on file IO itself. Called once per tick by World.
    void record_row(std::uint64_t tick, real sim_time, std::vector<real> values);

private:
    struct Row {
        std::uint64_t tick;
        real sim_time;
        std::vector<real> values;
    };

    void io_thread_main(std::string csv_path);

    std::vector<std::string> names_;

    std::mutex queue_m_;
    std::condition_variable queue_cv_;
    std::deque<Row> queue_;
    bool stop_requested_ = false;
    std::thread io_thread_;
    bool running_ = false;
};

} // namespace ps::io
