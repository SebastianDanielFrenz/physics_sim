# C6 fuel tank - progress (agent worktree agent-a2bbb2e7f76cdd7b0, base 3f16d65)

## Done
- Read brief, plan backups, C5d code (engine_torque_map, powertrain, vehicle_io, world, vehicle_instance).
- Steps 1-6 coded (not yet built): desc, loader, energy_store.h, starvation, Powertrain wiring, World API + tank_kg telemetry + snapshot.

## Next
1. Public: FuelTankDesc (powertrain_desc.h, in ComponentParams), FluidLinkDesc + PowertrainDesc::fluid.
2. Loader: "fuel_tank" component, "fluid" array, rejections (exact messages in T7).
3. core/src/drivetrain/energy_store.h: IEnergyStore/FuelTank.
4. Engine: Sensors::fuel_available_kg (+inf default), starvation scale s in pre_solve.
5. Powertrain: tank member, fuel override, debit, hash, pending set at begin_tick.
6. World/VehicleInstance: fuel_tank_mass/set_fuel_tank_mass, snapshot fuel_mass_kg, telemetry tank_kg.
7. Tests T1-T9 + G, sabotages, docs.

## Design decisions (reasons)
- Starvation input: `EngineTorqueMap::Sensors::fuel_available_kg` (default +infinity = never starves,
  so every existing direct pre_solve call site is unchanged). Documented as a physical supply limit,
  not an ECU sensor. Powertrain sets it to m_tank only when a tank exists.
- s decision: demand_mass = h*mdot_demand; starve iff demand_mass > available; then
  s = available/demand_mass and booked = available EXACTLY (so the emptying substep leaves
  m_tank == 0.0 exactly); otherwise s = 1 and booked = h*mdot (the pre-C6 expression).
- Bit-exactness at s == 1: each combustion term is multiplied by s individually, keeping the
  existing addition order; the build has FP contraction off (/clang:-ffp-contract=off), so x*1.0 == x
  and nothing else changes.
- torque_demand stays UNSCALED (it is the ECU's request, which a dry tank does not change);
  torque_estimate and published theta_c (load) use the scaled tau_comb (physical combustion).
  throttle_eff stays the throttle.

## Measured numbers
(none yet)

## Open questions
(none yet)
