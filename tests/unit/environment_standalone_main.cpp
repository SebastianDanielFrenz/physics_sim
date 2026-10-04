// tests/unit/environment_standalone_main.cpp - link-time guard for ps_environment
// and ps_environment_io (weather plan W1), the same pattern as
// engine_cycle_standalone_main.cpp: a plain main() (NOT Catch2, whose support
// library assumes ps_core) linked WHOLE-ARCHIVE against ps_environment and
// ps_environment_io without ps_core. If anything in either library ever needs
// a ps_core symbol (JobSystem, World, ...) this target fails to LINK.
//
// It builds a calendar and sun from a code-made descriptor, applies a merge
// patch, loads the shipped climate file and checks a few values, exit code 0 on
// success.

#include "environment_fixtures.h"

#include "ps/environment/calendar.h"
#include "ps/environment/climate_io.h"
#include "ps/environment/env_time.h"
#include "ps/environment/json_extends.h"
#include "ps/environment/sun.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::fprintf(stderr, "environment_standalone: FAILED: %s\n", what);
        ++failures;
    }
}

} // namespace

int main() {
    using namespace ps::environment;

    // Clock -> calendar -> sun at an astronomical equinox-ish day.
    const ClimateDesc desc = ps::test_env::real_year(50.15);
    const Calendar cal(desc);
    const Sun sun(desc);
    EnvClock clock;
    clock.scale = 1.0;
    const double noon = 80.0 * 86400.0 + 43200.0;   // day of year 81 at local noon
    const SunState s = sun.at(cal, noon);
    check(s.elevation_rad > 30.0 * kDegToRad && s.elevation_rad < 45.0 * kDegToRad, "equinox noon elevation at 50 N");
    check(s.is_day, "noon is day");
    check(std::fabs(clock.env_time(240 * 10) - 10.0) < 1e-12, "clock closed form");
    check(clear_sky_ghi_w_m2(s.elevation_rad) > 500.0, "clear-sky irradiance at noon");

    // Merge patch (RFC 7396 example 7).
    nlohmann::json target = nlohmann::json::parse(R"({"a":{"b":"c"}})");
    ps::io::json_merge_patch(target, nlohmann::json::parse(R"({"a":{"b":"d","c":null}})"));
    check(target == nlohmann::json::parse(R"({"a":{"b":"d"}})"), "merge patch");

#ifdef PS_SOURCE_DIR
    const std::filesystem::path shipped =
        std::filesystem::path(PS_SOURCE_DIR) / "data" / "climates" / "test_fixed_summer.json";
    const ClimateDesc loaded = ps::io::load_climate_json(shipped);
    check(loaded.seasons.size() == 4, "shipped climate has 4 seasons");
    const Calendar loaded_cal(loaded);
    check(loaded_cal.days_per_year() == 124, "shipped climate: 4 x 31 days");
#endif

    if (failures == 0) std::printf("environment_standalone: OK\n");
    return failures == 0 ? 0 : 1;
}
