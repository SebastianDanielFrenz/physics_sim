# `physics_sim.machine/1` — machine parts, and `physics_sim.tool_power/1`

`data/machines/*.json`. Loader `ps::io::load_machine_json(path, sockets)` and `load_tool_power_json(path)` (`ps/io/machine_io.h`,
`core/src/io/machine_io.cpp`). Descriptors `ps::machine::MachineDesc`, `ToolDesc` (`ps/machine/tool.h`). `"extends"` works as in
every part file. An unknown key is rejected, and every block of numbers starts its `source` with `[C]`, `[D]`, `[M]` or `[AS]`.

A machine part says how a tool behaves. It names sockets and never carries a coordinate or a mass:

| What | Owner |
|---|---|
| draft coefficients, depth and speed range, soil law, PTO, operation name | the machine part |
| sockets, working width (the distance between the two edge sockets), colliders | the model (`farm.geometry/1` in the game) |
| identity, label, price, mass, stores | the game's part (`farm.vehicle/2`), which links the machine part by path |
| mass in the physics | `BodyDesc::mass`, as for every vehicle |

The caller binds socket names to chassis-local positions: `load_machine_json(path, sockets)`, or in a vehicle file

```json
"machine": { "ref": "../../../../data/machines/cultivator_3m.json",
             "sockets": { "socket_work_L": [-1.55, 1.495, -0.6], "socket_work_R": [-1.55, -1.495, -0.6] } }
```

`"wheels": []` is legal only with a `machine` block ([linkage.md](linkage.md)).

## Top level

| Key | Type | Required | Meaning |
|---|---|---|---|
| `format` | string | yes | `"physics_sim.machine/1"` |
| `extends` | string | no | parent file, relative to this one |
| `name` | string | yes | the machine's name |
| `source` | string | yes | free text |
| `tools` | array | yes | at least one tool |

## Tool

| Key | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | |
| `kind` | string | no | `"soil"` (default) or `"applicator"`, see "Applicators" |
| `operation` | string | yes | carried through to the game, which turns work into its own records. `ps_core` never reads it |
| `edge_left`, `edge_right` | string | yes | sockets of the model; the width is their distance |
| `range` | object | yes | below |
| `draft` | object | yes | below |
| `soil` | object | yes | below |
| `pto` | object | no | a rotor on the tractor's PTO shaft |

**`range`**: `source`, `depth_min_m` (shallower does not count as worked), `depth_nominal_m` (the gauge's first setting),
`depth_max_m` (the tines' length, draft stops growing), `speed_min_km_h`, `speed_max_km_h`, `overspeed_quality_per_km_h`,
`quality_floor`. Quality is 1 up to `speed_max_km_h`, then falls by the rate to the floor. Below the range only time is lost, so
`speed_min_km_h` only feeds the shop's sweep.

**`draft`**, the ASABE D497 form

```
D = F_i (A + B S + C S^2) W T  x  moisture(wetness)  x  (1 + compaction_gain * compaction)    newtons
```

`S` in km/h, `W` the width in the coefficients' unit, `T` the depth in cm, or 1 with `depth_term` false.

| Key | Meaning |
|---|---|
| `source` | see "Sources" |
| `width_unit` | `"metre"` or `"tool"` |
| `tool_spacing_m` | only with `"tool"`: `W` = width over this |
| `depth_term` | whether `T` is the depth |
| `primary`, `secondary` | `{a, b, c}`, N per width unit, per cm with a depth term. Untilled and freshly tilled; between them by `SoilSample::tilled` |
| `texture_factor` | `{fine, medium, coarse}`, `F_i` |
| `moisture_factor` | `{wetness: [...], factor: [...]}`, a curve over `SoilSample::wetness` (0 wilting point, 1 field capacity) |
| `compaction_gain` | the draft's share added at compaction 1 |
| `lateral_ratio` | side force over draft, sliding sideways |
| `creep_speed_m_s` | the forces build up from standstill over this speed |

**`soil`**: what the ground does to the tool across its travel, forces per metre of width.

| Key | Meaning |
|---|---|
| `source` | |
| `suction_ratio` | pulled in, over draft |
| `bearing_n_m2` | soil under the tool, per metre of depth |
| `gauge_n_m2` | a roller or gauge wheel past the set depth; 0 none |
| `hard_n_m2`, `hard_friction` | ground it cannot enter (`workable` false): it scrapes over it and works nothing |
| `damping_n_s_m2` | per metre per second of the edge's vertical speed |

**`pto`**: `source`, `nominal_rpm` (the tractor must offer it), `inertia_kgm2` (at the tractor's shaft), `no_load_kw`, `kw_per_m`
(rotor running, per metre of width), `specific_energy_mj_m3` (per cubic metre of soil worked), `min_speed_fraction` (below this
share of nominal the tool does no work). Rotor power is `no_load + kw_per_m W + e W d v`, torque is that over the shaft's speed.

## Applicators

`"kind": "applicator"` is a tool that puts something on the ground without touching it: a sprayer boom, a fertiliser or lime
spreader, a slurry distributor. A soil tool works when it is in the ground (`depth >= depth_min`); a sprayer never is, so it has a
kind of its own. Every part without `kind` is a soil tool, exactly as before: nothing in the state hash changes for it.

An applicator has `name`, `kind`, `operation`, `edge_left`, `edge_right` and a `range` with `source`, `speed_min_km_h`,
`speed_max_km_h`, `overspeed_quality_per_km_h` and `quality_floor`. It has no `draft`, `soil` or `pto` block and no depth keys in
`range`: the loader rejects them. Its width is the distance of its two sockets, as for a soil tool.

It works while **all three** hold in a tick: `World::set_tool_enabled(vehicle, tool, true)` (off at spawn), the soil source's
sample under the middle of the edge is `workable`, and the forward speed of that point is above zero (standing or reversing
writes nothing). Then it writes a `WorkQuad` with `depth_m` 0 and the speed range's `quality`, like a soil tool; the first tick
after the switch only starts the strip. It puts no force on the vehicle: no draft, no soil force, `ToolState::draft_n` and
`vertical_n` stay 0. It does not ask the ground's height.

`World::set_tool_enabled` is for between steps and refuses a soil tool (`std::invalid_argument`). `ToolState::enabled` mirrors the
switch. The switch is part of the state hash for an applicator and not for any other tool.

The game decides what comes out (a product, a rate, a tank): the part says only where and how fast.

## How a tool runs

A tool lives on the vehicle that carries it, `vehicle::System` steps it inside the island's substep. `tick_begin` asks the ground's
height under the edge and `ISoilSource::sample` at the edge's midpoint; `substep` turns depth and velocity into a force and a
moment at the edge, on the shadow chassis, which reaches the tractor through the hitch rows; `tick_end` appends one `WorkQuad`.
Nothing in the tool knows about a tractor's power: lugging, wheel slip and stalling come from the tractor's own drivetrain under
that force.

```cpp
world.set_soil_source(soil);                         // ps::machine::ISoilSource, read from the jobs: no locks, no allocation
world.vehicle_tool_count(implement);
world.tool_desc(implement, 0).operation;
world.tool_state(implement, 0);                      // working, depth, speed, draft, vertical, quality, worked area, PTO
world.set_tool_gauge_depth(implement, 0, 0.10);
```

Without a soil source, or on a `workable` false sample, a tool does no work.

## Work

Between steps `world.drain_work(quads)` appends the quads since the last drain, in (tick, vehicle, tool) order. Each tool holds
4096, a fuller buffer drops the newest and counts them in `world.work_dropped()`. A standing tool writes none.

`ps::machine::work_coverage(quad, grid, out)` cuts a quad into cell shares by ascending cell; `work_area(quad)` is its area. A
quad that crosses itself, a tool pivoting, counts as two triangles. `ps::machine::WorkLedger` gathers shares per (cell, vehicle,
tool) with coverage-weighted depth and quality, and `take()` hands them over in ascending order.

Apply the ledger once per pass, not per tick: `ps::field` applies a thin strip as `f x coverage` and a repeat as
`1 - (1 - f)^coverage`, so a thousand strips applied one by one do not add up to one pass.

```cpp
machine::WorkLedger ledger(grid);
std::vector<machine::WorkQuad> quads;
world.drain_work(quads);
for (const machine::WorkQuad& q : quads) ledger.add(q);
quads.clear();
// at the end of the pass, or of the game's chunk
for (const machine::WorkCell& w : ledger.take()) {
    // effect from world.tool_desc(w.vehicle, w.tool).operation, w.depth_m, w.quality
    field.apply(w.cell, effect_with_coverage(w.coverage));
}
```

## `physics_sim.tool_power/1`

`data/machines/power_estimate.json`, the shop's "needs about N kW". `tool_power_sweep(tool, spec, speed_steps)` runs the tool's
speed range at its nominal depth over the spec's soils:

```
engine_kw = (drawbar_kw / tractive_ratio + pto_kw) / pto_share,  needed_kw = max(engine_kw) * reserve_factor
```

| Key | Meaning |
|---|---|
| `format` | `"physics_sim.tool_power/1"` |
| `source` | tagged |
| `tractive_ratio` | drawbar power over PTO-equivalent power |
| `pto_share` | PTO power over net engine power |
| `reserve_factor` | the engine is not bought to run at its limit |
| `soils` | `{texture, wetness, tilled, compaction}` cases |

`check_tool_limits(tool, sweep, ToolTractorFit{engine_kw, pto_torque_nm}, policy)` grades it against a tractor: `ToolPower` (24)
against the rated power, `PtoShaftTorque` (23) against the PTO clutch at the shaft, at the `hitch_*_share` thresholds of
[limit_policy.md](limit_policy.md).

## Sources

ASABE D497 itself was not read: it is paywalled. The draft coefficients are those of the University of Arizona Extension table,
seen only as search-engine extracts, so every shipped draft block is `[AS]` and says so. The rotary-power table and the field
speed table of D497 were not found. The moisture curve is an assumption: the literature found disagrees on the sign of the effect
below field capacity. The EP496 ratios of `power_estimate.json` come from search-engine extracts too.
