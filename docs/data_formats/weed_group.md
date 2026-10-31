# physics_sim.weed_group/1 - a group of weeds (F1c)

`ps::field::WeedGroupDesc`, loader `ps::io::load_weed_group_json`. One group per species or functional type; a field carries up
to 8 (scenario key `"weeds"`). Every number carries a source entry ([C], [D], [M] or [AS]) in its block's `source` object;
unknown keys are rejected. Shipped (data/weeds/, all [AS], recalled biology, no measured fit): black_grass (annual grass),
cleavers (annual broadleaf), couch_grass (perennial), volunteer_wheat (volunteer: shed grain of the crop named in `volunteer_crop`).

Blocks: `kind` (annual_grass, annual_broadleaf, perennial, volunteer); `seedbank` (`seeds_init_m2`, `clumps_init_m2` for
perennials, initial shares per layer surface/upper/lower, decay per day per layer); `germination` (soil-temperature window
`t_min_c`/`t_opt_c`/`t_max_c`, `rate_per_day`, wetness window, `emerge_*` share of germinants that reach the light by layer,
`seedling_g`); `growth` (`t_base_c`, `growth_per_gdd`, `w_max_g` per plant, `area_max_g_m2` monoculture cap, `self_thin_per_day`,
`shade`, `n_conc_pct`, `c_frac`, cold kill); `development` (`gdd_flower`, `gdd_ripe`, `gdd_death`, `seeds_per_g`, `shed_per_day`);
`perennial` (dormancy temperature, senescence, regrowth); `competition` (Cousens 1985 hyperbola `I`, `A` in percent, `rel_emerge_decay`:
`loss % = I d / (1 + I d / A)`, d = plants per m2, I falls for weeds that emerge after the crop); `resistance` (`init` share resistant to each
mode of action, 1e-6 [AS] after Powles and Yu 2010, order of magnitude).

Model: germination flushes follow soil temperature and the wetness of the seed layer; plants grow logistically in thermal time,
shaded by the crop; seed is set between `gdd_flower` and `gdd_ripe` and shed on the ground; cultivation buries the seed (inversion) or
brings it up. Weeds take mineral nitrogen, no water and no P or K (a stated simplification); their carbon goes into the residue
pools when they die or are killed, the seed carbon is not tracked. The crop's growth is multiplied by the Cousens factor
(`weed_factor` in the management view, the crop's `stress_weed`).
