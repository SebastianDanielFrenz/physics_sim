# Offline turbo stage maps (experimental)

`tools/turbo_sim/meanline.py` generates `physics_sim.turbo_stage_maps/1`
from `physics_sim.turbo_geometry/1`. Python 3.10+ standard library only.
Generation runs offline; vehicle loading reads completed JSON and does no
stage simulation. Engine cycle maps remain a separate generator/cache.

```powershell
python tools/turbo_sim/meanline.py tools/turbo_sim/standard_geometry.json out/turbo_sim/map.json --adaptive --speed-lines 9 --flow-points 13 --expansion-points 9
python tools/turbo_sim/benchmark_maps.py tools/turbo_sim/standard_geometry.json out/turbo_sim/verification.json
python tools/turbo_sim/build_examples.py
python -m unittest discover -s tools/turbo_sim -p test_meanline.py -v
```

Add `"performance_map": "relative/path/to/map.json"` to a
`physics_sim.turbo/1` hardware file to opt in. Paths resolve relative to that
hardware file, not the installation or engine. `build_examples.py` creates
complete, separately selectable compact/standard/large experimental
hardware and twin installations under `out/turbo_sim/examples`.
Default driving hardware is unchanged. Hardware dimensions/units must match
the generated geometry; count is applied by the installation loader.

Map axes:

- `corrected_rpm`: shaft RPM corrected by `sqrt(T_in/T_reference)`;
  compressor and turbine have separate reference temperatures.
- `compressor_corrected_flow_kg_s`: **per unit**, actual flow times
  `sqrt(T_in/T_reference)/(P_in/P_reference)`.
- `turbine_expansion_ratio`: inlet total pressure / outlet static pressure.
  Runtime interpolates on `sqrt(1-ER^(-(gamma-1)/gamma))`, the nondimensional
  isentropic spouting velocity, rather than directly on ER.

Point arrays are row-major (`rpm_index * axis_size + flow_or_ER_index`).
Compressor points include status, total pressure ratio/temperature, specific
work, power, efficiency, Mach values, incidence risk and balance residuals.
Turbine points include mass flow, specific work/torque, power, outlet total
temperature, efficiency, choking and energy residual.

`ok` compressor corners are required for interpolation. Choked, no-positive-
head and `transonic_unmodelled` points are retained as explicit coverage data.
Outside the compressor's coverage the runtime reads the map's own
continuation (stage seam S0, see "Runtime mask boundary transition") and
publishes `turbo_stage_map_fallback=true` and
`turbo_compressor_stage_extended=true`; it does **not** clamp shaft RPM to the
last line. `turbo_compressor_stage_used` and `turbo_turbine_stage_used`
identify actual consumers in `PreSolveResult` / `EngineSoundState`. Outside
the turbine's coverage the legacy authored approximation remains.
The current garage HUD does not yet display these new diagnostics.

The turbine's per-flow torque is interpolated, then multiplied by actual
flow and shaft angular speed. Torque is nonzero at rest even though power
is zero; startup integrates its first-step kinetic energy. Turbine and
wastegate mass flow are solved at their common upstream pressure.
The exhaust model is compressible when the generated turbine is used.
The legacy fallback keeps its old approximation, clearly diagnosed.

`source`, `provenance`, geometry/gas inputs, solver version and coverage are
stored in every map. Manufacturer wheel diameters do not make assumed
internal geometry manufacturer-accurate. The first turbine solver is an
impulse-stage approximation, **not** a validated radial reaction stage.
Solver `meanline-4` offers compressor `shock_model: "inlet_normal_entropy"`.
It applies perfect-gas normal-shock entropy from inlet relative Mach inside
the outlet continuity solve, preserving Euler work/total enthalpy. The full
normal-shock orientation is an explicit assumption, not resolved blade-flow
physics. Default absent/`unsupported` preserves explicit transonic rejection.
Supersonic outlet/passage choking remains unsupported. Points report shock
pressure retention and entropy. Old `meanline-3` maps remain loadable.
Resolved surge, housing/volute losses, variable gas properties and rotor
damage are not implemented.

Cache: SHA-256 identity of solver, resolved stage geometry/gas and operating
point, independent of the sweep grid, stored as atomic per-point JSON.
Completed points survive cancellation. Adaptive full-map cache additionally
keys the grid, provenance and sampling settings; corruption is a miss.
`content_hash` covers the canonical map body. Native state hashing hashes
the consumed JSON, including provenance; it does not trust a supplied hash.
Output files are atomically replaced only on successful completion.

Adaptive cell-center audits measure sampled interpolation error, **not** a
global bound. Requested tolerance, measured errors, boundary cells and
`budget_limited` remain in `sampling`. Generation retaining a budget-limited
map never certifies its interpolation accuracy. The verification tool
compares approximate visually digitized manufacturer chart points with
explicit reference-condition and digitization uncertainty.

See vault **PHYS-068** for design rationale, validation results and remaining
work. Physics sources: [NASA nozzle flow](https://www.grc.nasa.gov/www/k-12/BGP/mflchk.html),
[finite-blade slip equation](https://www.mdpi.com/1996-1073/8/5/4317),
[Garrett reference hardware/maps](https://www.garrettmotion.com/racing-and-performance/performance-catalog/turbo/gtx3582r-gen-ii/).

Authored installations can set `compressible_exhaust: true` in the
`physics_sim.turbo_configuration/1` file. This replaces ambient-density
incompressible exhaust pressure with a common compressible nozzle inverse
for turbine plus wastegate effective area. Choked flow scales linearly with
upstream pressure. Turbine efficiency/head remain authored; no generated
compressor/turbine map is selected by this option. Absent/false retains legacy
replay behavior; the true option is included in engine state hashing.
Normal-shock reference: https://www.grc.nasa.gov/WWW/BGH/normal.html .

### Runtime mask boundary transition
Inside coverage (every corner of the bilinear cell valid) the compressor uses the valid corners' bilinear values, unchanged. Since stage seam S0 (2026-10-13) any point whose cell has an unsupported corner reads `TurboStageMaps::compressor_extended`, the map's own continuation, instead of blending the authored head/work law in (that law was 2.5-4x below the map near the choke edge: charge and Euler work jumped where the blend weight came in, and at a fixed flow the charge could FALL with rotor speed - standard map 1.659 -> 1.569 bar at 100-105k rpm, 0.77 kg/s):

- along a speed line, valid knots interpolate linearly in flow; beyond the line's last valid knot (its choke edge) the pressure ratio falls linearly to 1 at the line's first invalid knot (the generator's "choked / no positive head" verdict; one more knot at the last spacing when the axis ends valid) and stays 1; the specific Euler work is held at the edge value (the impeller still turns the flow in choke; the lost head becomes charge heat through the work). Below a line's first valid knot both are held flat;
- between two speed lines with data, the two line values blend linearly in corrected speed - at the same flow up to the smaller of the two edge flows (the covered bilinear rule, so the value is continuous across the coverage boundary), choke-aligned beyond it: the edge flow and the choke knot interpolate in speed and each line is read at the same relative position of its own interval (the beta-line idea, applied only outside coverage; reading both lines at one flow there makes a speed-density engine's charge saw-tooth between lines, measured 1.775 / 1.864 / 2.241 bar at 110 / 115 / 120k on the standard map);
- below the lowest / above the highest line with data: compressor affinity from that line (flow f N_line/N; PR^kappa - 1 and work scale with (N/N_line)^2, kappa = (gamma - 1)/gamma of the map's air).

The extension is continuous in (speed, flow), equals the covered values at the boundary and, on a map whose lines are ordered, never falls with speed at a fixed flow (tests/unit/test_turbo_stage_seam.cpp). It is never certified: the fallback flag stays set. Garage check: `drivetrain::check_turbo_stage_coverage` (BuildRule `TurboStageMapCoverage`, part of `check_buildable`; and `EngineMapOptions::warnings_out` at load) reports when the engine's air flow at the target boost needs points outside the certified cells (Near) or above the map's highest speed line (Limiting) - docs/data_formats/engine.md "Buildability rules". Compressor/turbine usage and fallback flags describe each step. The shipped example generator refines up to five passes with 129 speed / 257 flow budgets and refuses a budget-limited audit; the 3% target is sampled at cell centers, not a global error guarantee.

### C++ generator (ps::drivetrain::meanline, 2026-10-14)
`ps/drivetrain/turbo_meanline.h` is a C++ port of `meanline.py` (solver `meanline-4`): `compressor_point`, `turbine_point`, `generate` (optional JobSystem; every point in its own slot, bit-identical for any worker count) and `adaptive_generate` (the same cell-centre refinement rule). `ps/io/turbo_stage_io.h` reads a geometry profile or a stage map's embedded geometry (`parse_turbo_geometry_json`), a map's axes (`parse_turbo_stage_sampling_json`), writes the map JSON (`turbo_stage_maps_to_json` / atomic `write_turbo_stage_maps_json`; `content_hash` is `fnv1a64:<hex>`, a `generator` block names the port and its `cache_key`) and parses map JSON exactly like the vehicle loader (`turbo_stage_maps_from_json`, the same function `performance_map` uses). `ml::to_stage_maps(result, units)` gives the in-memory `TurboStageMaps`. Python stays the reference: `tests/unit/test_turbo_meanline.cpp` [turbo_meanline] compares against `tests/fixtures/turbo_meanline/` (written by its `make_reference.py`): statuses identical, values within 1e-9 relative (measured worst 1.9e-15), adaptive axes bitwise. The shipped standard recipe (5 passes, 25 x 133 x 9) takes 137 ms on one thread, 54 ms on 4 workers (Linux release) vs 6.4 s in Python.

Runtime extension: `drivetrain::turbo_stage_demand(engine)` (engine_rules.h; the numbers `check_turbo_stage_coverage` reads) -> `ml::demand_points` -> `ml::extend_sampling_for_demand(current axes, points, ExtensionPolicy)` (current knots kept bitwise, speed lines every `speed_step_rpm` over the demand band to `speed_margin` above it, flow knots to `flow_margin` beyond it; policy values are the caller's config) -> `ml::generate` -> `turbo_stage_maps_to_json`, cached under `ml::cache_key(geometry, axes, nullptr)`. Sampling only moves the certified boundary onto the solver's own choke line (a bilinear cell needs four valid corners, so the boundary lags it by one line spacing). A demand beyond the choke line stays uncovered: measured for racing_game's car_hyper_n2o at 1.8 bar, steps 13-15 of 16 become covered with 500 rpm lines, step 16 (0.5336 kg/s per turbo) is above the 0.5186 kg/s the compressor passes at pressure ratio 2.7765.
