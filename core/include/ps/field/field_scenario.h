// ps/field/field_scenario.h - a headless field scenario (physics_sim.field_scenario/1,
// docs/data_formats/field_scenario.md): a climate, a seed, soils, crops, a small
// grid, a number of years and a list of test-only sow/harvest rules. Used by
// ps_field_run and by the validation tests. Paths are relative to the scenario file.
//
// Operations apply BETWEEN days: before the step of the day whose real day of
// year equals the rule's `doy` (and whose game year index is first_year +
// k * every_years). They are plain operations on the field; the field never
// harvests or kills a crop by itself.

#pragma once

#include "ps/environment/climate_desc.h"
#include "ps/field/field.h"
#include "ps/field/weather_record.h"
#include "ps/types.h"

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

namespace ps::io {

struct FieldOperationRule {
    std::string op;              // "sow" or "harvest"
    int crop_id = 0;             // sow only (1-based palette index)
    ps::real density_scale = 1.0;
    int doy = 1;                 // real day of year
    int first_year = 0;
    int last_year = 1 << 30;
    int every_years = 1;
};

struct FieldScenario {
    environment::ClimateDesc climate;
    std::uint64_t seed = 0;
    field::DailyWeatherConfig weather;
    field::FieldConfig field;
    int years = 1;
    bool irrigate_to_capacity = false;       // test-only
    std::vector<FieldOperationRule> rules;
};

struct HarvestEvent {
    std::int64_t day = 0;
    int year = 0;
    int doy = 0;
    field::HarvestResult result;             // field mean
};

struct FieldRunSummary {
    std::vector<HarvestEvent> harvests;
    ps::real max_balance_residual_mm = 0.0;  // worst cell
    ps::real total_precip_mm = 0.0;          // cell 0 ledger
    std::uint64_t state_hash = 0;
};

// Throws std::runtime_error naming the file and the field.
FieldScenario load_field_scenario(const std::filesystem::path& path);

// Runs scenario.years game years from day 0. `csv` (optional) gets one row per day.
FieldRunSummary run_field_scenario(const FieldScenario& scenario, std::ostream* csv = nullptr);

} // namespace ps::io
