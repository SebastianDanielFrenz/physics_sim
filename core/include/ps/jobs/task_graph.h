// ps/jobs/task_graph.h — small DAG of named barrier stages (PLAN.md D8's
// "task graph with barriers").
//
// All nodes whose dependencies are satisfied run in the same level, each
// as its own job on the shared JobSystem; a barrier waits for the whole
// level before the next one starts. World's tick pipeline (PLAN.md section
// 5: freeze state -> scene queries -> vehicle islands -> force elements ->
// ordered gather -> [solver slot] -> backend step -> terrain -> snapshot)
// is expressed as one such graph, built once and re-run every tick so
// building it is not itself a per-tick allocation.

#pragma once

#include "ps/jobs/job_system.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace ps::jobs {

class TaskGraph {
public:
    using StageFn = std::function<void()>;

    // Returns this stage's node index, usable as a dependency id in a later
    // add_stage call. Stages are otherwise run in the order dependencies
    // allow; with no dependencies given, a stage depends on nothing and may
    // run in the very first level.
    std::size_t add_stage(std::string name, StageFn fn, std::vector<std::size_t> depends_on = {});

    // Runs every stage exactly once, in dependency order (topological
    // levels; each level is a barrier). Safe to call every tick - iterates
    // its own already-built node list, no rebuilding.
    void run(JobSystem& job_system) const;

    [[nodiscard]] const std::vector<std::string>& stage_names() const { return names_; }
    [[nodiscard]] std::size_t stage_count() const { return names_.size(); }

private:
    std::vector<std::string> names_;
    std::vector<StageFn> fns_;
    std::vector<std::vector<std::size_t>> deps_;
};

} // namespace ps::jobs
