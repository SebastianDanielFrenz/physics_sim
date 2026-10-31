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

## Optional `fungicide` block (F1d part 4)

A product with a `fungicide` block is a fungicide: it acts on the pathogen pressure of the growing crop, not on weeds (give it an
empty `targets` object and zero crop damage). Keys, all with source tags: `efficacy_max` (in [0, 1): the largest protection, a
fungicide never buys back the whole loss), `dose_half` (dose relative to the label rate that gives half of it), `half_life_days`
(the protection fades with it), and the optional string `family` (the crop family it protects; empty or absent = every family).
A spray of dose d on a cell with a crop sets `p = efficacy_max d/(d + dose_half)` (times the coverage share) and adds it as
`1 - (1 - old)(1 - p)`, capped at `efficacy_max`; on bare soil it does nothing. The protection (the product's `h_act`, decaying
with `half_life_days` instead of the residual half-life) divides the crop's untreated pathogen loss by `1 - protection` and slows
the build of next season's inoculum. Shipped: data/herbicides/cereal_fungicide.json ([AS]: efficacy_max 0.6, dose_half 0.3,
half-life 200 d, family cereal; operation data/operations/spray_cereal_fungicide.json): at the label dose it buys back about a
third of the take-all loss when applied right after emergence (test P6: 37 %).
