#include "ps/jobs/task_graph.h"

#include <cassert>

namespace ps::jobs {

std::size_t TaskGraph::add_stage(std::string name, StageFn fn, std::vector<std::size_t> depends_on) {
    for ([[maybe_unused]] std::size_t d : depends_on) {
        assert(d < names_.size() && "dependency must reference an already-added stage");
    }
    names_.push_back(std::move(name));
    fns_.push_back(std::move(fn));
    deps_.push_back(std::move(depends_on));
    return names_.size() - 1;
}

void TaskGraph::run(JobSystem& job_system) const {
    const std::size_t n = names_.size();
    if (n == 0) return;

    std::vector<std::size_t> indegree(n, 0);
    std::vector<std::vector<std::size_t>> dependents(n);
    for (std::size_t i = 0; i < n; ++i) {
        indegree[i] = deps_[i].size();
        for (std::size_t d : deps_[i]) dependents[d].push_back(i);
    }

    std::vector<bool> done(n, false);
    std::vector<std::size_t> ready;
    ready.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (indegree[i] == 0) ready.push_back(i);
    }

    std::size_t completed = 0;
    while (completed < n) {
        assert(!ready.empty() && "task graph has a dependency cycle");
        std::vector<std::size_t> level;
        level.swap(ready);

        JobSystem::WaitGroup group;
        for (std::size_t node : level) {
            job_system.submit([this, node] { fns_[node](); }, &group);
        }
        job_system.wait(group);

        for (std::size_t node : level) {
            done[node] = true;
            ++completed;
            for (std::size_t dep : dependents[node]) {
                if (--indegree[dep] == 0) ready.push_back(dep);
            }
        }
    }
}

} // namespace ps::jobs
