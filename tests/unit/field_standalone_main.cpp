// tests/unit/field_standalone_main.cpp - link-time guard for ps_field and
// ps_field_io (farming plan F1a), the same pattern as
// environment_standalone_main.cpp: a plain main() (NOT Catch2) linked
// WHOLE-ARCHIVE against ps_field_io (and so ps_field, ps_environment_io,
// ps_environment) without ps_core. If anything in them ever needs a ps_core
// symbol (JobSystem, World, ...) this target fails to LINK.
//
// It loads the shipped soil, crop and climate, builds a one-cell field, sows
// winter wheat and steps 40 days from the weather record; exit code 0 on
// success.

#include "ps/environment/climate_io.h"
#include "ps/field/field.h"
#include "ps/field/field_io.h"
#include "ps/field/weather_record.h"

#include <cmath>
#include <cstdio>
#include <filesystem>

namespace {
int failures = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        std::fprintf(stderr, "field_standalone: FAILED: %s\n", what);
        ++failures;
    }
}
} // namespace

int main() {
    const std::filesystem::path root = PS_SOURCE_DIR;
    const auto climate = ps::io::load_climate_json(root / "data/climates/farm_de_home.json");
    ps::field::FieldConfig cfg;
    cfg.soils.push_back(ps::io::load_soil_json(root / "data/soils/silt_loam_loess.json"));
    cfg.crops.push_back(ps::io::load_crop_json(root / "data/crops/winter_wheat.json"));
    ps::field::Field field(cfg);
    ps::field::DailyWeatherSource src(climate, 3);
    field.sow_all(1);
    field.advance_to(39, src);
    check(field.last_day() == 39, "advanced to day 39");
    const ps::field::CellView v = field.view(0);
    check(std::isfinite(v.d_root_mm) && v.d_root_mm >= 0.0, "finite non-negative depletion");
    check(std::fabs(field.water_balance_residual_mm(0)) < 1e-9, "water balance closes");
    return failures == 0 ? 0 : 1;
}
