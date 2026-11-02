// tests/support/engine_map_test_cache.h - one shared engine-map cache and one process-wide generation pool for the
// tests that only USE a generated-map engine (a car_hyper variant, the demo pipe engine) and do not assert anything
// about generation itself (test-cost audit 2026-11-02, lever L1).
//
// Without it each such test loaded its engine with no pool and no cache, so the Full maps of the same engine were
// generated again on one thread in every test process (car_hyper_n2o: 33 s in L4a, H4, "Turbo hybrid ...", ...).
// With it the first process of a build generates them on the pool and writes <PS_TEST_MAP_CACHE_DIR>/<key>.psmaps;
// every later load is a cache hit (~1 s for the hyper car, 0.2-0.4 ms per engine file).
//
// Safety of the shared directory: the key (engine_map_cache_key) covers every generator input plus the generator
// version, a hit is bit-identical to a fresh generation (test_engine_map_cache H1), the file is written to a temp name
// unique per process, thread and call and then renamed (H4, core/src/engine_cycle/map_cache.cpp), and a damaged or
// foreign file is a silent miss (H2). Several ctest processes writing the same entry at once are therefore safe; the
// loser of the rename race overwrites with identical bytes. The turbo-table cache under <dir>/turbo is rewritten
// non-atomically by load_turbo_configuration_json, but its reader re-derives the table and compares before accepting
// a hit, so a half-written file is also just a miss.
//
// Tests of generation itself (test_engine_map_cache, test_engine_map_generator, test_engine_map_control,
// test_wot_generation, test_engine_map_gen_v2, ...) must NOT use this: they have to generate cold. The directory is
// per build tree (PS_TEST_MAP_CACHE_DIR, set by tests/unit and tests/validation CMakeLists.txt), so presets and
// compilers never share entries; a generator change that does not bump kGeneratorVersion does not invalidate it -
// delete <build>/test_map_cache then.
#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <filesystem>
#include <string>

#ifndef PS_TEST_MAP_CACHE_DIR
#error "PS_TEST_MAP_CACHE_DIR must be defined by the test target (tests/unit and tests/validation CMakeLists.txt)"
#endif

namespace ps_test {

// The pool the generator runs on when the cache misses. Process-wide so a test that loads several engines does not
// start one pool each; idle workers sleep on a 1 ms condition-variable wait, they do not spin.
inline ps::jobs::JobSystem& shared_map_pool() {
    static ps::jobs::JobSystem pool(10);
    return pool;
}

inline const std::string& shared_map_cache_dir() {
    static const std::string dir = [] {
        std::error_code ec;
        std::filesystem::create_directories(PS_TEST_MAP_CACHE_DIR, ec); // a failure only means no caching: loads still work
        return std::string(PS_TEST_MAP_CACHE_DIR);
    }();
    return dir;
}

// Pass as EngineMapOptions (load_engine_json, EngineRig) or as VehicleLoadOptions::maps (load_vehicle_json).
inline ps::io::EngineMapOptions shared_map_options(ps::drivetrain::MapQuality quality = ps::drivetrain::MapQuality::Full) {
    ps::io::EngineMapOptions o;
    o.quality = quality;
    o.pool = &shared_map_pool();
    o.cache_dir = shared_map_cache_dir();
    return o;
}

} // namespace ps_test
