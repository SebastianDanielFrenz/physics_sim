// Custom Catch2 entry point (replaces Catch2::Catch2WithMain) so every
// Catch2-based test executable installs the headless CRT handlers
// (tools/common/headless_env.h) before running a single test - otherwise a
// failed assert/JPH_ASSERT could pop a blocking modal dialog under the
// Windows debug CRT and hang a headless/CI run forever instead of failing
// fast (see PLAN.md: every tool/test must run headless).

#include "headless_env.h"

#define CATCH_CONFIG_RUNNER
#include <catch2/catch_session.hpp>

int main(int argc, char* argv[]) {
    ps_tools::install_headless_crt_handlers();
    return Catch::Session().run(argc, argv);
}
