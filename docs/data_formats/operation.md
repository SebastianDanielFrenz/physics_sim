# physics_sim.operation/1 - the field operation effect record (F1c)

One plain-data record per field operation. It holds no machine type: a data file produces it today, a machine tool (M1)
will produce the same record later. `ps::field::OperationEffect` (core/include/ps/field/management_desc.h) is the C++
form, `ps::io::load_operation_json(path, palette)` the loader, `Field::apply(cell, effect)` / `apply_all(effect)` the
consumer. A record whose members all sit at their defaults does nothing and leaves the field bit-identical.

Names inside a file (crop, herbicide, material, method) are resolved against the palettes of the field the record is
applied to (`OperationPalette`, index + 1 = id). Every number of the `tillage`, `harvest` and `seed` blocks needs a source
entry starting with [C], [D], [M] or [AS] in the block's own `source` object; unknown keys are rejected.

```json
{ "format": "physics_sim.operation/1", "name": "plough",
  "coverage": 1.0,
  "tillage": { "depth_m": 0.25, "mix_surface": 0.0, "mix_upper": 0.0, "mix_lower": 0.0, "inversion": 0.9,
               "residue_chop": 0.0, "loosening": 0.8, "seedbed_fineness": 0.25, "crop_kill": 1.0,
               "weed_kill": { "annual_grass": 0.95, "annual_broadleaf": 0.95, "perennial": 0.6, "volunteer": 0.95 },
               "source": { "...": "[AS] ..." } },
  "harvest": { "straw_carried": 0.0, "straw_chopped": 0.9, "grain_loss": 0.015, "source": { } },
  "straw_pickup": 0.0,
  "seed":  { "crop": "winter_wheat", "seeds_per_m2": 350, "depth_m": 0.035, "quality": 0.9, "residue_tolerance": 0.6, "source": { } },
  "spray": { "product": "grass_herbicide", "dose": 1.0 },
  "deposit": { "material": "can_27", "method": "broadcast", "rate_t_ha": 0.3 } }
```

## Members

`coverage` - share of the cell the pass covered (a double pass is 2). Every fraction below is "of the covered area";
the effect on the cell is the area-weighted share `f x coverage` for a coverage up to 1 and `1 - (1 - f)^coverage` for repeated passes.

`tillage` (all optional; `depth_m = 0` means the tool does not engage the soil):

| key | meaning |
|---|---|
| `depth_m` | working depth of the tool pass |
| `mix_surface`, `mix_upper`, `mix_lower` | share of the surface / upper (0-0.10 m) / lower (0.10-0.30 m) layer stirred through the worked depth |
| `inversion` | share of the worked soil turned over: surface material and the upper layer go under, the bottom comes up |
| `residue_chop` | share of the loose surface straw chopped (it decomposes faster) |
| `loosening` | share of compaction and consolidation removed in the worked layers (settles again with rain and time) |
| `seedbed_fineness` | tilth the tool leaves in workable soil (0 clods, 1 fine) |
| `crop_kill` | share of a standing crop destroyed (it goes into the residue pools) |
| `weed_kill` | object with the kinds `annual_grass`, `annual_broadleaf`, `perennial`, `volunteer`: share of the emerged weeds destroyed |

Workability follows from the root-zone wetness (soil `management` block). A pass in wet or very dry soil does less, leaves
a worse seedbed and, when wet, damages the structure (smearing: higher runoff curve number, shallower rooting, slow recovery).
Moving soil moves its lime, its surface ammoniacal N and its residue between the layers; nothing is created or lost.

`harvest` - `straw_carried` share of the harvestable straw that leaves with the grain, `straw_chopped` share that stays chopped
and spread (the rest lies in a swath on the surface), `grain_loss` share of the grain dropped on the ground (seeds volunteers).
The grain always leaves; stubble and roots stay. The record returns a `HarvestResult` through `apply_all`.

`straw_pickup` - share of the loose surface straw taken off (baler, rake).

`seed` - `crop` (name, 1-based id in the palette), `seeds_per_m2`, `depth_m`, `quality` (share placed in contact with soil),
`residue_tolerance` (0: residue in the row blocks emergence, 1: the tool cuts through it). The crop starts from this
operation: emergence needs moisture, temperature and a seedbed; too deep, too dry, too thin or too late is an outcome.

`spray` - `product` (herbicide name) and `dose` relative to the label rate.

`deposit` - the F1b deposit (fertiliser, manure, lime); see field_material.md.

Order inside one record: crop and weed kill, harvest, straw pickup, tillage, deposit, spraying, sowing.

## Shipped records (data/operations/)

plough, cultivator, stubble_cultivator, seedbed_harrow, drill, direct_drill, weeder, combine_straw_chopped,
combine_straw_off, spray_grass_herbicide, spray_broadleaf_herbicide, spray_total_herbicide. All [AS]: parameter sets of tool
passes from recalled tool behaviour, not measured. Scenario use: `{"op": "operation", "file": "../operations/plough.json",
"doy": 255}` with optional overrides `coverage`, `seeds_per_m2`, `depth_m`, `dose`, `straw_carried`, `straw_chopped`.
