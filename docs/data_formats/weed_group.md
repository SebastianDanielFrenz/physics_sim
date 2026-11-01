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
brings it up. Weeds take mineral nitrogen and water (the `water` block) and no P or K (a stated simplification); their carbon goes into the residue
pools when they die or are killed, the seed carbon is not tracked. The crop's growth is multiplied by the Cousens factor
(`weed_factor` in the management view, the crop's `stress_weed`).

## `water` block (F1d part 6)

The group transpires from the root-zone bucket the crop draws on, and shades the soil. Per day and group: `cover = 1 - exp(-cover_per_g_m2 x
biomass)`, demand `kc_tr x cover x (1 - shade x crop cover) x ET0`, reduced by the FAO-56 stress factor of the group (`p_depletion`, the share of
the root-zone TAW that can be used before stress). The demands of all groups add up, are capped by `(Kc_max - Kcb) x ET0 - soil evaporation` and
by the Priestley-Taylor wet-surface limit of the weather record (`e_wet_mm`, when given), and are cut with the crop's transpiration and the soil
evaporation when the bucket cannot give the day's total. The soil evaporation is multiplied by `1 - combined weed cover`. Their transpiration is part
of the water ledger's `transpiration` and is also reported cumulatively (`ManagementView::weed_transpiration_mm`). `kc_tr` 0 switches the water use
off (the shading stays).

| key | unit | meaning |
|---|---|---|
| `kc_tr` | - | transpiration coefficient at full cover (FAO-56 basal coefficient of the weed canopy), >= 0 |
| `p_depletion` | - | share of TAW usable before stress, in (0, 1) |
| `cover_per_g_m2` | m2/g | canopy cover = 1 - exp(-this x biomass in g/m2), > 0 |

The direct competition factor (Cousens, `competition` block) stays as before and is not reduced where water is involved: it is the empirical
total of what a weed density costs, so a field with both effects counts water twice to some extent (documented limitation; the tests that isolate the
water effect switch the Cousens factor off).

