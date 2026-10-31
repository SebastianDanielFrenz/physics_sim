# physics_sim.herbicide/1 - a herbicide (F1c)

`ps::field::HerbicideDesc`, loader `ps::io::load_herbicide_json`; scenario key `"herbicides"` (up to 8); applied by an operation
record's `spray` member. Shipped (data/herbicides/, [AS], label-rate figures illustrative): grass_herbicide (group 1, ACCase),
broadleaf_herbicide, total_herbicide (non-selective).

Keys: `name`, `moa` (mode-of-action group; resistance is tracked per group, at most 4 distinct per field), `label_rate`
(information; doses are relative), `targets` (per weed kind: `ed50_rel` dose that kills half, `slope` of the log-logistic curve,
`stage_gdd_full` and `stage_gdd_zero` - efficacy complete up to that thermal time since emergence, zero beyond, linear between;
a kind that is absent is not affected), `resistance.ed50_factor` (ED50 multiplier of the resistant biotype), `residual`
(`half_life_days`, `germ_kill`: share of emerging seedlings killed at full residual activity), `crops` (per crop name or `"default"`:
`max_bbch` safe application window, `damage_in`/`damage_out` share of the stand lost at the label dose inside/outside the window),
`damage.dose_exponent` (crop damage ~ dose^exponent). Every number carries a source tag.

Kill of a weed group: `1 - (1 - r) S(dose) - r S(dose / ed50_factor)`-style two-biotype mixture with the susceptible share `1 - r`;
the survivors raise `r` (selection): `r' = r (1 - kr) / ((1 - r)(1 - ks) + r (1 - kr))`. Seed produced by the survivors inherits `r'`.
