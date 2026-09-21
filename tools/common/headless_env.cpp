#include "headless_env.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <crtdbg.h>
#include <cstdlib>
#endif

namespace ps_tools {

void install_headless_crt_handlers() {
#if defined(_WIN32)
    // Suppress the OS-level "Windows has stopped working" / WER dialog on
    // an unhandled crash (e.g. a raw JPH_BREAKPOINT with no debugger
    // attached - see core/src/backend/jolt/jolt_backend.cpp's
    // ensure_jolt_registered, which routes Jolt's own asserts through this
    // same stderr path instead).
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);

    // Route the debug CRT's assert()/_ASSERTE()/error/warn reports to
    // stderr instead of a modal MessageBox.
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);

    // abort() itself (not routed through _CrtSetReportMode): write its
    // message to stderr and let the OS report the fault non-interactively,
    // rather than the default "Debug Error!" modal dialog.
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
}

} // namespace ps_tools
