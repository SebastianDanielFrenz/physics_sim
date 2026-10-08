# Adaptive engine pressure sampling

The cycle block accepts an optional `pressure_sampling` object:

```json
"pressure_sampling": {
  "enabled": true,
  "min_kpa": 25,
  "max_kpa": 600,
  "relative_error": 0.02,
  "max_points": 512,
  "max_passes": 8
}
```

Pressures are absolute. Omitted or zero bounds use the endpoints of
`grid_p_im_kpa`. Existing grid points are initial seeds inside the domain;
different engines can have different seeds and bounds. Sampling is opt-in,
so existing authored grids and cache keys retain their legacy behavior.

Each interval is evaluated at its quarter, half and three-quarter pressures
over the selected RPM rows. The generator compares those actual cycle results
with interpolation of the previous maps, including efficiency at the resulting
fresh-air mass. Intervals exceeding tolerance receive additional nodes.
Completed operating-point results are retained throughout generation and
reused on subsequent passes; refinement never repeats their cycle simulation.

VE, pumping, exhaust temperature, exhaust enthalpy and efficiency are reduced
independently against all computed validation samples. Efficiency retains its
fresh-air-mass coordinate; it is not converted into a pressure table. For signed
or nearly zero values, the error denominator has a floor of 1% of the table's
peak absolute magnitude. Half the tolerance is allocated to refinement and half
to final table reduction. This is sampled interpolation control, not a proof of
accuracy between arbitrarily narrow unsampled features or of physical validity
outside the engine model's calibration.

Budget exhaustion throws an explicit convergence error. Cancellation propagates
between and during passes. Adaptive settings and an adaptive generator version
participate in cache identity. Cached tables retain their independently selected
axes. The cold generation can be more expensive, while warm cache hits reuse the
completed maps.
