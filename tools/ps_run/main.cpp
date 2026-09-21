// ps_run — headless scenario runner (CLI -> CSV). Placeholder for Phase 0:
// prints the ps_core version and exits. Scenario loading, stepping and CSV
// telemetry are P1 work (PLAN.md section 4, D6).

#include "ps/version.h"

#include <cstdio>

int main() {
    std::printf("ps_run %s\n", ps::version_string());
    return 0;
}
