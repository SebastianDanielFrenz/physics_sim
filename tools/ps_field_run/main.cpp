// ps_field_run - headless field scenario runner (farming plan F1a).
// usage: ps_field_run <scenario.json> [out.csv]
// Runs scenario.years game years on a small grid and writes one CSV row per
// day (default: <scenario name>.csv next to the scenario), then prints the
// harvests and the water-balance residual.

#include "headless_env.h"
#include "ps/field/field_scenario.h"

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
    ps_tools::install_headless_crt_handlers();
    if (argc < 2) {
        std::fprintf(stderr, "usage: ps_field_run <scenario.json> [out.csv]\n");
        return 2;
    }
    try {
        const std::filesystem::path scenario_path = argv[1];
        const ps::io::FieldScenario sc = ps::io::load_field_scenario(scenario_path);
        std::filesystem::path out = argc > 2 ? std::filesystem::path(argv[2])
                                             : scenario_path.parent_path() / (scenario_path.stem().string() + ".csv");
        std::ofstream csv(out);
        if (!csv) {
            std::fprintf(stderr, "ps_field_run: cannot write %s\n", out.string().c_str());
            return 1;
        }
        const ps::io::FieldRunSummary sum = ps::io::run_field_scenario(sc, &csv);
        for (const ps::io::HarvestEvent& h : sum.harvests)
            std::printf("harvest day %lld (year %d, doy %d): grain %.3f t/ha dm, straw %.3f t/ha dm, bbch %.1f\n",
                        static_cast<long long>(h.day), h.year, h.doy, h.result.grain_dm_t_ha, h.result.straw_dm_t_ha,
                        h.result.bbch);
        std::printf("water balance residual (worst cell): %.3e mm; precipitation %.1f mm; state_hash=0x%016llx\n",
                    sum.max_balance_residual_mm, sum.total_precip_mm, static_cast<unsigned long long>(sum.state_hash));
        if (sc.field.nutrients_enabled)
            std::printf("nutrient balance residual (worst cell): N %.3e, P2O5 %.3e, K2O %.3e, CaO %.3e kg/ha\n", sum.max_n_residual,
                        sum.max_p_residual, sum.max_k_residual, sum.max_cao_residual);
        std::printf("wrote %s\n", out.string().c_str());
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ps_field_run: %s\n", e.what());
        return 1;
    }
}
