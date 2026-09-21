#include "ps/io/telemetry.h"

#include <cstdio>
#include <fstream>

namespace ps::io {

TelemetryRegistry::~TelemetryRegistry() { stop(); }

std::size_t TelemetryRegistry::register_channel(const std::string& name) {
    names_.push_back(name);
    return names_.size() - 1;
}

void TelemetryRegistry::start_csv(const std::string& csv_path) {
    if (running_) return;
    stop_requested_ = false;
    running_ = true;
    io_thread_ = std::thread([this, csv_path] { io_thread_main(csv_path); });
}

void TelemetryRegistry::stop() {
    if (!running_) return;
    {
        std::lock_guard<std::mutex> lock(queue_m_);
        stop_requested_ = true;
    }
    queue_cv_.notify_all();
    if (io_thread_.joinable()) io_thread_.join();
    running_ = false;
}

void TelemetryRegistry::record_row(std::uint64_t tick, real sim_time, std::vector<real> values) {
    {
        std::lock_guard<std::mutex> lock(queue_m_);
        queue_.push_back(Row{tick, sim_time, std::move(values)});
    }
    queue_cv_.notify_one();
}

void TelemetryRegistry::io_thread_main(std::string csv_path) {
    std::ofstream out(csv_path, std::ios::trunc);
    out << "tick,sim_time";
    for (const auto& n : names_) out << ',' << n;
    out << '\n';

    // Single producer (World, via record_row) that only ever calls stop()
    // strictly after its last record_row() - so once stop_requested_ is
    // observed true with an empty batch, no more rows can arrive.
    for (;;) {
        std::deque<Row> batch;
        bool stopping;
        {
            std::unique_lock<std::mutex> lock(queue_m_);
            queue_cv_.wait(lock, [this] { return !queue_.empty() || stop_requested_; });
            batch.swap(queue_);
            stopping = stop_requested_;
        }
        for (const auto& row : batch) {
            out << row.tick << ',' << row.sim_time;
            for (real v : row.values) out << ',' << v;
            out << '\n';
        }
        if (!batch.empty()) out.flush();
        if (stopping && batch.empty()) break;
    }
}

} // namespace ps::io
