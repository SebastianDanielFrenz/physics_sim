# `physics_sim.soil/1` - soil for the field model (farming F1a)

Loader: `ps::io::load_soil_json(path)` / `parse_soil_json`
(`core/include/ps/field/field_io.h`, library `ps_field_io`). Descriptor:
`ps::field::SoilDesc` (`ps/field/soil_desc.h`). Files: `data/soils/*.json`
(`silt_loam_loess.json`, `sandy_loam.json`). A field's palette holds one or
more soils; `FieldConfig::cell_soil` names a soil per grid cell.

The soil is a two-bucket column (model overview in `ps/field/field.h`): the root-zone
bucket (depth = the rooting depth of the cell, `fallow_root_depth_m` without a
crop) and the layer below it down to `depth_m`.

**Source tags.** Every number (and boolean) in a block must have an entry of the
same name in that block's `source` object, and the text must start with
`[C]` cited, `[D]` derived, `[M]` measured or `[AS]` assumed. The loader
refuses a file with a parameter without a tag. String members (`name`,
`texture`) are exempt.

| Block / key | Unit | Notes |
|---|---|---|
| `format` | | `"physics_sim.soil/1"` |
| `extends` | | merge-patch parent (as for climate files); the override must itself be tagged |
| `name` | | non-empty |
| `texture` | | informational |
| `water.theta_fc` | m3/m3 | field capacity, in (theta_wp, 1) |
| `water.theta_wp` | m3/m3 | permanent wilting point, > 0 |
| `water.depth_m` | m | bottom of the water bucket; the roots cannot go deeper |
| `water.fallow_root_depth_m` | m | depth of the "root zone" bucket without a crop, below `depth_m` |
| `evaporation.depth_m` | m | Ze, the surface layer that dries by evaporation (FAO-56 ch. 7), <= `fallow_root_depth_m` |
| `evaporation.rew_mm` | mm | readily evaporable water (stage 1), below the total evaporable water TEW = 1000 (theta_fc - 0.5 theta_wp) Ze (FAO-56 eq 73) |
| `runoff.curve_number_ii` | | SCS CN for average antecedent moisture, in (30, 100) |
| `runoff.cn_normal_wetness` | | relative root-zone wetness (1 - Dr/TAW) at which CN II applies, in (0, 1); CN I (dry) and CN III (wet) follow CN_I = CN_II/(2.281 - 0.01281 CN_II), CN_III = CN_II/(0.427 + 0.00573 CN_II), interpolated linearly in the wetness |
| `runoff.ia_ratio` | | initial abstraction as a fraction of the retention S = 25.4 (1000/CN - 10) mm, in [0, 1] |
| `snow.melt_mm_per_k_day` | mm/(K day) | degree-day melt factor, >= 0 |
| `snow.melt_base_c` | degC | melt starts above this daily mean |

Derived (not in the file): TAW per metre = 1000 (theta_fc - theta_wp) mm/m,
TEW (above), total TAW = TAW per metre x `depth_m`.

Rejections: wrong `format`; unknown key; a missing required key; a number given
as a string; a parameter without a source tag; theta_fc <= theta_wp;
`fallow_root_depth_m` >= `depth_m`; `evaporation.depth_m` > `fallow_root_depth_m`;
REW >= TEW; curve number outside (30, 100); a negative melt factor.
`tests/unit/test_field_io.cpp` I2, I3, I5.
