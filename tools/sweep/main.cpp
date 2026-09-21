// tools/sweep — fans N scenarios out over local processes (PLAN.md D9: "any
// launcher... can distribute runs later without touching the library",
// since ps_run is a pure function). Each scenario runs as its own ps_run
// child process; --jobs caps how many run concurrently.

#include "headless_env.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#endif

namespace {

std::filesystem::path find_ps_run(const char* argv0) {
#if defined(_WIN32)
    const char* exe_name = "ps_run.exe";
#else
    const char* exe_name = "ps_run";
#endif
    const std::filesystem::path dir = std::filesystem::path(argv0).parent_path();
    // Same directory sweep.exe lives in (typical after `cmake --install`),
    // then the CMake build layout's sibling target directory
    // (tools/sweep/<sweep exe> and tools/ps_run/<ps_run exe> under the same
    // build tree), tried in that order.
    for (const std::filesystem::path& candidate : {dir / exe_name, dir / ".." / "ps_run" / exe_name}) {
        if (std::filesystem::exists(candidate)) return candidate;
    }
    return dir / exe_name;
}

#if defined(_WIN32)
struct Proc { PROCESS_INFORMATION pi{}; std::string scenario; };

Proc launch(const std::filesystem::path& ps_run, const std::string& scenario) {
    std::string cmd = "\"" + ps_run.string() + "\" \"" + scenario + "\"";
    std::vector<char> cmdline(cmd.begin(), cmd.end());
    cmdline.push_back('\0');
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    Proc p;
    p.scenario = scenario;
    if (!CreateProcessA(nullptr, cmdline.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si, &p.pi)) {
        std::fprintf(stderr, "sweep: failed to launch ps_run for %s (error %lu)\n", scenario.c_str(), GetLastError());
        p.pi.hProcess = nullptr;
    }
    return p;
}

int run_all(const std::filesystem::path& ps_run, const std::vector<std::string>& scenarios, int jobs) {
    std::vector<Proc> running;
    std::size_t next = 0;
    int failures = 0;

    auto reap_one = [&]() {
        std::vector<HANDLE> handles;
        handles.reserve(running.size());
        for (auto& p : running) handles.push_back(p.pi.hProcess);
        const DWORD idx = WaitForMultipleObjects(static_cast<DWORD>(handles.size()), handles.data(), FALSE, INFINITE);
        const std::size_t i = static_cast<std::size_t>(idx - WAIT_OBJECT_0);
        DWORD code = 0;
        GetExitCodeProcess(running[i].pi.hProcess, &code);
        if (code != 0) {
            std::fprintf(stderr, "sweep: FAILED (exit %lu): %s\n", code, running[i].scenario.c_str());
            ++failures;
        } else {
            std::printf("sweep: ok: %s\n", running[i].scenario.c_str());
        }
        CloseHandle(running[i].pi.hProcess);
        CloseHandle(running[i].pi.hThread);
        running.erase(running.begin() + static_cast<long>(i));
    };

    while (next < scenarios.size() || !running.empty()) {
        while (next < scenarios.size() && static_cast<int>(running.size()) < jobs) {
            running.push_back(launch(ps_run, scenarios[next]));
            ++next;
        }
        if (!running.empty()) reap_one();
    }
    return failures;
}
#else
struct Proc { pid_t pid = -1; std::string scenario; };

Proc launch(const std::filesystem::path& ps_run, const std::string& scenario) {
    Proc p;
    p.scenario = scenario;
    const pid_t pid = fork();
    if (pid == 0) {
        execl(ps_run.c_str(), ps_run.c_str(), scenario.c_str(), static_cast<char*>(nullptr));
        _exit(127); // exec failed
    }
    p.pid = pid;
    return p;
}

int run_all(const std::filesystem::path& ps_run, const std::vector<std::string>& scenarios, int jobs) {
    std::vector<Proc> running;
    std::size_t next = 0;
    int failures = 0;

    auto reap_one = [&]() {
        int status = 0;
        const pid_t done = waitpid(-1, &status, 0);
        for (std::size_t i = 0; i < running.size(); ++i) {
            if (running[i].pid == done) {
                const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
                if (!ok) { std::fprintf(stderr, "sweep: FAILED: %s\n", running[i].scenario.c_str()); ++failures; }
                else { std::printf("sweep: ok: %s\n", running[i].scenario.c_str()); }
                running.erase(running.begin() + static_cast<long>(i));
                return;
            }
        }
    };

    while (next < scenarios.size() || !running.empty()) {
        while (next < scenarios.size() && static_cast<int>(running.size()) < jobs) {
            running.push_back(launch(ps_run, scenarios[next]));
            ++next;
        }
        if (!running.empty()) reap_one();
    }
    return failures;
}
#endif

} // namespace

int main(int argc, char** argv) {
    ps_tools::install_headless_crt_handlers();

    std::vector<std::string> scenarios;
    int jobs = 4;
    std::string ps_run_override;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--jobs" && i + 1 < argc) {
            jobs = std::atoi(argv[++i]);
        } else if (arg == "--ps-run" && i + 1 < argc) {
            ps_run_override = argv[++i];
        } else {
            scenarios.push_back(arg);
        }
    }

    if (scenarios.empty()) {
        std::fprintf(stderr, "usage: sweep <scenario1.json> [scenario2.json ...] [--jobs N] [--ps-run path]\n");
        return 2;
    }
    if (jobs < 1) jobs = 1;

    const std::filesystem::path ps_run = ps_run_override.empty() ? find_ps_run(argv[0]) : std::filesystem::path(ps_run_override);
    if (!std::filesystem::exists(ps_run)) {
        std::fprintf(stderr, "sweep: ps_run not found at %s (pass --ps-run to override)\n", ps_run.string().c_str());
        return 2;
    }

    std::printf("sweep: running %zu scenario(s) over up to %d concurrent process(es)\n", scenarios.size(), jobs);
    const int failures = run_all(ps_run, scenarios, jobs);
    std::printf("sweep: %zu scenario(s), %d failure(s)\n", scenarios.size(), failures);
    return failures == 0 ? 0 : 1;
}
