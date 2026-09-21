// Phase 0 smoke test: ps_core builds, its version API works, and the Jolt
// dependency actually links and initialises. Real unit tests for math,
// world stepping etc. arrive in P1.

#include "ps/version.h"

#include <catch2/catch_test_macros.hpp>

#include <cstring>

TEST_CASE("version_string reports the expected shape", "[hello]") {
    const char* v = ps::version_string();
    REQUIRE(v != nullptr);
    REQUIRE(std::strlen(v) > 0);
    // "MAJOR.MINOR.PATCH" — two dots.
    int dots = 0;
    for (const char* p = v; *p; ++p) {
        if (*p == '.') ++dots;
    }
    REQUIRE(dots == 2);
}

TEST_CASE("Jolt backend self-check succeeds", "[hello][jolt]") {
    REQUIRE(ps::jolt_backend_self_check());
}
