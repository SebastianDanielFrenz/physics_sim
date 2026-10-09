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
Outside coverage the current runtime uses the legacy authored approximation
and publishes `turbo_stage_map_fallback=true`; it does **not** clamp shaft RPM
to the last line. `turbo_compressor_stage_used` and `turbo_turbine_stage_used`
identify actual consumers in `PreSolveResult` / `EngineSoundState`.
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
