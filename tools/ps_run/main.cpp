// ps_run — headless scenario runner (PLAN.md D6/section 4): scenario file
// -> CSV, a pure function with no other side effects (no wall clock reads
// beyond telemetry profiling inside World, no network, no global state
// carried between runs) so tools/sweep can fan many of these out safely.

#include "headless_env.h"
#include "ps/io/scenario.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/version.h"
#include "ps/world/world.h"

#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>

namespace {

std::string default_csv_path(const std::string& scenario_path, const std::string& scenario_name) {
    std::filesystem::path p(scenario_path);
    p = p.parent_path() / (scenario_name + ".csv");
    return p.string();
}

} // namespace

int main(int argc, char** argv) {
    ps_tools::install_headless_crt_handlers();

    if (argc < 2) {
        std::fprintf(stderr, "ps_run %s\nusage: ps_run <scenario.json> [output.csv] [--workers N]\n", ps::version_string());
        return 2;
    }

    const std::string scenario_path = argv[1];
    std::string csv_override;
    int worker_override = -1;
    int map_workers = 0; // I7c C16a: > 0 = generate engine maps on a JobSystem with that many workers at load
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--workers" && i + 1 < argc) {
            worker_override = std::atoi(argv[++i]);
        } else if (arg == "--map-workers" && i + 1 < argc) {
            map_workers = std::atoi(argv[++i]);
        } else if (csv_override.empty()) {
            csv_override = arg;
        }
    }

    try {
        // The map pool lives only for the load; generated maps are
        // bit-identical for any worker count (I7c C16a test M3).
        std::unique_ptr<ps::jobs::JobSystem> map_pool;
        if (map_workers > 0) map_pool = std::make_unique<ps::jobs::JobSystem>(map_workers);
        ps::io::EngineMapOptions map_options;
        map_options.pool = map_pool.get();
        ps::io::Scenario scenario = ps::io::load_scenario(scenario_path, map_options);
        map_pool.reset();
        if (worker_override >= 0) scenario.world_config.job_workers = worker_override;

        ps::io::BuiltScenario built = ps::io::build_scenario(scenario);
        ps::World& world = *built.world;

        const std::string csv_path = !csv_override.empty() ? csv_override
            : (!scenario.telemetry_csv.empty() ? scenario.telemetry_csv : default_csv_path(scenario_path, scenario.name));
        world.telemetry().start_csv(csv_path);

        const std::uint64_t num_ticks = static_cast<std::uint64_t>(scenario.duration_s * scenario.world_config.tick_rate_hz + 0.5);
        std::size_t next_control_event = 0;
        for (std::uint64_t i = 0; i < num_ticks; ++i) {
            // PLAN.md P3 scope item 7: apply any due control_events before
            // stepping, in time order (scenario.control_events is sorted by
            // load_scenario).
            while (next_control_event < scenario.control_events.size() &&
                   scenario.control_events[next_control_event].time_s <= world.sim_time()) {
                const auto& ev = scenario.control_events[next_control_event];
                world.set_control(ev.channel, ev.value);
                ++next_control_event;
            }
            world.step();
        }

        world.telemetry().stop();

        std::printf("ps_run: scenario '%s' -> %s (%llu ticks, %d workers, state_hash=0x%016llx)\n",
                    scenario.name.c_str(), csv_path.c_str(),
                    static_cast<unsigned long long>(num_ticks),
                    world.job_system().worker_count(),
                    static_cast<unsigned long long>(world.state_hash()));
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ps_run: error: %s\n", e.what());
        return 1;
    }
}
