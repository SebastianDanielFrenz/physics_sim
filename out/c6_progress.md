# C6 fuel tank - progress (agent worktree agent-a2bbb2e7f76cdd7b0, base 3f16d65)

## Done
- Public: FuelTankDesc (ComponentParams), FluidLinkDesc + PowertrainDesc::fluid, PowertrainSnapshot::fuel_mass_kg,
  World::fuel_tank_mass / set_fuel_tank_mass.
- Loader: "fuel_tank" component (inline only), "fluid" array, every rejection pinned in T7.
- core/src/drivetrain/energy_store.h: IEnergyStore / FuelTank.
- Engine: Sensors::fuel_available_kg (+inf default), starvation fraction s in pre_solve.
- Powertrain: tank member, fuel override (tank fuel else fuels[0]), exact debit, hash only with a tank,
  pending set applied in begin_tick (clamped, signed into fuel_external_added), backstops.
- World: vehicle<i>.fuel.tank_kg telemetry only with a tank.
- Tests T1-T9, T3b, T7b in tests/unit/test_fuel_tank.cpp (11 cases); sabotages T1-T5 run and reverted.
- Docs: vehicle.md (fuel_tank, fluid links, rejections, starvation, runtime access, telemetry), engine.md one line.
- ctest [fuel]|[io]|[drivetrain] 286/286, ps_determinism_tests 9/9, validation [drivetrain] pass (4 pre-existing mayfail).

## Next
- Final commit (removes this file from the index), handback.

## Measured numbers
- T1: bit-identical every tick over vehicle_powertrain.json; plain hash 0x5380949669212c71; tank burnt 0.026931 kg (30 -> 29.9731).
- T2 (auto-clutch launch): worst |(m0-m)-(fuel_mass-ext)| 1.4603e-12 kg (bound 3e-11); burnt 0.0089508 kg.
- T3 crude: empty at substep 792 (0.825 s) at 6795.34 rpm; stall t_meas 2.49792 s vs t_pred 2.49668 s (+0.050 %),
  bounds [1.30138, 4.58992]; dry cranking 396.306 rpm (fire 250, start_complete 550); integrator 0.108081 = clamp;
  restart 0.344792 s after refuel.
- T5 map: empty at substep 734 at 6750.13 rpm; stall 2.4875 vs 2.48664 s (+0.035 %), bounds [1.30147, 4.55749];
  cranking 396.812 rpm; integrator 0.108081; restart 0.344792 s.
- T3b idle: empty at 1.09896 s at 799.57 rpm, integrator ~0 at empty; stalled after 0.25625 s (t_pred 0.255351,
  bounds [0.220251, 0.288724]); integrator peak 0.108081 = clamp; reset to 0 on Stalled.
- T4: E5 0.00113634 kg, E10 0.00116072 kg, ratio 1.02145 = LHV_E5/LHV_E10 to 1e-12; E5 tank == plain car_sedan.
- T8: crude empty at WOT tick 279, map 258; worst balance ratio 7.457e-16.
- T9: hashes equal 1 vs 10 workers; debug_alloc own 0, external (Jolt QuadTree) 714.
- G: vehicle_step_steer 0xbab69b300eba41a3, vehicle_powertrain 0x5380949669212c71,
  terrain_multi_interest 0xd27f5f629161cee4, vehicle_tractor_i4 0xac386af7129b9ffe (ps_run release, 10 workers).
- Sabotages:
  - T1 s=0.9999999999999999 when a finite tank covers demand (engine_torque_map.cpp): first mismatching tick 0.
  - T2 withdraw(delta*1.001) (powertrain.cpp): first failing check err 7.58e-10 kg vs bound 3e-11, worst 8.95e-6 kg; 3120 failed.
  - T3 tau_comb unscaled (engine_torque_map.cpp): T3 and T3b never stall (k_stall -1 within 20 s / 10 s).
  - T4 tank fuel override dropped (powertrain.cpp): ratio 1.0 vs expected 1.02145 (|r/e-1| = 0.0210).
  - T5 map engines never starve (engine_torque_map.cpp): emptying substep books 1.2915e-5 kg vs 7.9393e-6 held
    (excess 4.976e-6 kg), then 12.40 g/s flows with load 1.0 from an empty tank; 32499 failed.

## Open questions / deviations
- 1e400: nlohmann v3.12 throws out_of_range.406 "number overflow parsing '1e400'" before the loader's isfinite
  check; T7 pins the parser message, the loader check stays as a backstop no JSON text reaches.
- T2/T8/T9 launch with assist.auto_clutch = 1 from an open clutch: a gear-1 clutch dump from rest stalls the engine.
- T9 after the refuel sets starter 1 and brake 1 (the dry engine stayed Stalled otherwise).

## Design decisions (reasons)
- Starvation input: Sensors::fuel_available_kg, default +infinity (never starves) so every direct pre_solve call
  site is unchanged; Powertrain sets it to the tank content only with a tank.
- Starve iff h*mdot_demand > available: s = available/demand, booked = available exactly (the emptying substep
  leaves m_tank == 0.0 exactly); else s = 1, booked = h*mdot (the pre-C6 expression). s = 1 when demand == 0.
- Bit-exactness at s == 1: each combustion term multiplied by s individually, existing addition order kept;
  FP contraction is off, so x*1.0 == x.
- torque_demand stays unscaled (the ECU's request); torque_estimate and the published theta_c (load) use the
  scaled tau_comb. throttle_eff stays the throttle.
- Fuel choice fixed at construction (tank fuel, else fuels[0]), not hashed.
- Idle integrator: winds up to its upper clamp (0.108081 for i4_2l_na) while starving at idle and while cranking
  dry; bounded by the existing clamp, reset to 0 on entering Stalled - no extra anti-windup added.
