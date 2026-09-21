// tools/common/headless_env.h — headless-safe CRT setup for tools/tests.
//
// PLAN.md requires every tool/test to run headless (CI-style, no
// interaction). On Windows, an assert()/abort() under the debug CRT
// (_DEBUG, MSVC/clang-cl -MTd) otherwise pops a blocking modal dialog
// ("Debug Error! abort() has been called") that has nothing to click in a
// CI/agent session, hanging the process forever instead of failing fast
// with a visible message. install_headless_crt_handlers() routes CRT
// assert/error/warn reports and the abort message to stderr instead and
// suppresses the OS's own "just-in-time debugger" crash dialog, so a
// failure always prints and the process always exits non-zero.
//
// Deliberately NOT included by any ps_core public (or internal) header -
// this is tools/tests plumbing only, never linked into the library itself.

#pragma once

namespace ps_tools {

// No-op on non-Windows platforms. Call once, first thing in main().
void install_headless_crt_handlers();

} // namespace ps_tools
