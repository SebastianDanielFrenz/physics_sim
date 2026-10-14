// tests/unit/test_abs_esc.cpp - assists I5 part 2 (ABS) and I6 (ESC), docs/next_steps_brief.md 2.2b: two new blocks of the
// assist unit (electronics kind `assist_unit`, keys `abs` and `esc`), channels assist.abs (0/1) and assist.esc (mode code
// 0 off / 1 on / 2 sport), registered only on a car whose unit carries the block.
//
// Both are sensor-only like TC: wheel peripheral speeds (hub omega x loaded radius, projected through the wheel's steer
// angle and the gyro yaw rate as TC does), the steering channel and an ideal IMU (gyro yaw rate, longitudinal
// acceleration). No ground speed, contact load, surface or tyre slip is read.
//
// Law under test (VehicleInstance::step_tick, docs/data_formats/electronics.md "abs" / "esc"):
//   reference speed v_ref: integrates the IMU longitudinal acceleration each substep; while any service brake command is
//   applied it is lifted to the fastest wheel's ground-equivalent speed, otherwise it relaxes toward the slowest one.
//   ABS (per wheel i, service brake only, the handbrake is never modulated):
//     slip_i = (v_ref - v_i) / max(v_ref, slip_floor); e = slip_i - target_slip
//     integral_i = clamp(integral_i + ki e dt, 0, 1); scale_i* = clamp(1 - kp max(0, e) - integral_i, 0, 1)
//     scale_i rate-limited (release_rate down, apply_rate up); the rear axle runs min(scale_RL, scale_RR) (select-low);
//     scale 1 and integral 0 while v_ref < min_speed or the wheel's command <= 0.02.
//     The corner's friction-row capacity = (corner_max_nm x min(1, brake + esc_i) x scale_i) + handbrake share.
//   ESC: r_t = v delta / (L (1 + (v/v_ch)^2)), |r_t| <= a_lat_max / v (delta = mean front road-wheel angle, L wheelbase);
//     e = r - r_t, deadband db = deadband + fraction |r_t|; inactive in mode off or below min_speed.
//     Oversteer (|r| > |r_t| same sign, or opposite signs) brakes the FRONT wheel(s) on the side y with sign(y) = sign(-e);
//     understeer brakes the REAR wheel on that side (braking a wheel at lateral offset y gives a yaw moment +y F).
//     esc_i = clamp(gain (|e| - db), 0, max_brake), rate-limited at brake_rate; the engine torque fraction
//     = clamp(1 - torque_gain (|e| - db), 0, 1) through the TC throttle ceiling (min with TC's own fraction).
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS] unless marked):
//   L1  data/electronics/assist_unit_abs_esc.json loads: grade street, abs block with the AbsDesc defaults, esc modes
//       off/on/sport, default on, sport overrides deadband 0.10 rad/s / fraction 0.20 / max_brake 0.6. Rejections name the
//       field: unknown key in abs or esc, target_slip outside (0, 1), esc without abs, esc modes without "on", max_brake > 1,
//       a mode_params entry for an unlisted mode; drive_modes abs_default not a bool, esc_default not a mode name
//   L2  pure functions: esc_mode_requested(0, 0.49, 0.5, 1, 1.49, 1.5, 2, 7, NaN) = off off on on on sport sport sport off;
//       resolve_esc_mode on a unit without sport maps sport -> on, a unit without off maps off -> sport if offered else on
//   C1  channels assist.abs (default 1) and assist.esc (default 1 = on) registered only on a car whose unit carries the
//       block (car_sedan + the unit: both; plain car_sedan and car_hyper: neither); snapshot assist_fitted_bits bit 8 = ABS,
//       bit 9 = ESC (0 elsewhere), assist_active_bits bit 8/9 = intervening this tick
//   A1  ABS-1 [C test_vehicle_braking's acceptance, PHYS-019]: car_sedan + the unit, settled, hubs rolling with the chassis
//       (reset_vehicle hubs_roll_with_chassis), 100 km/h, full brake, steer 0, ESC off: stop (speed < 0.3 m/s) distance
//       within +-3 % of the load-transfer-corrected peak-mu estimate [D]: a = sum_axles 2 Dx(Fz_corner(a)) / m with
//       Fz_front_axle = m g x_rear / L + m a h / L, Dx(Fz) = Fz (pdx1 + pdx2 dfz) lambda_mux, every input read from the
//       loaded desc (60/40 static split from the wheel attachments, h = the settled chassis CoM height above the ground),
//       fixed-point to convergence. ABS-off run (channel 0) stops at least 5 % longer (negative control). abs active bit set
//       on >= 50 % of the braking ticks above 5 m/s
//   A2  steerable: same start, full brake, steer channel 0.3 from t = 0.1 s, ESC off: heading change after 1.5 s (or at the
//       stop if earlier) >= 5 deg with ABS on, and >= 3x the ABS-off run's |heading change|
//   A3  split-mu: left wheels asphalt, right wheels low_mu (0.1), 100 km/h, full brake, steer 0, ESC off, local 20 m-cell
//       ground: |heading change| over the whole stop <= 20 deg and <= 0.5x the ABS-off run's; the car stops
//   E1  ESC-1 FMVSS 126 sine with dwell [C 49 CFR 571.126 S5.2, S7.9]: car_sedan + the unit with REAR tyres lambda_muy x0.75
//       (so the car oversteers; ESC off must fail - negative control), 80 km/h, gear neutral, coasting; steer amplitude
//       A = min(6.5 x the steer channel giving 0.3 g lateral in a slow ramp at 80 km/h, the channel of 270 deg steering-wheel
//       angle at an assumed overall steering ratio 16 [AS]); 0.7 Hz sine, 500 ms dwell at the second peak; left-first and
//       right-first runs. Pass (ESC on, both directions): yaw rate 1.00 s after the end of steer (COS) <= 35 % of the first
//       peak yaw rate after the steer's sign change; 1.75 s after COS <= 20 %; lateral displacement of the CoG 1.07 s after
//       the beginning of steer (BOS) >= 1.83 m. ESC off: at least one direction fails the 35 % criterion
//   E2  ESC side: during E1 (ESC on) the largest ESC brake command after the second steer peak sits on a FRONT wheel; in an
//       understeer step (unmodified tyres, 25 m/s, steer channel 0.6 step, coasting) the largest ESC brake command sits on
//       the REAR wheel on the inside of the turn
//   E3  torque cut: the E2 understeer step at throttle 1 in 3rd: while ESC intervenes PowertrainDebug::throttle_eff <= 0.9
//       on at least one tick; ESC off: throttle_eff never below 0.99 in the same window
//   E4  sport mode intervenes later: in E1 (left-first) the first tick with a nonzero ESC brake comes no earlier in sport
//       than in on, and the total ESC brake (sum over ticks and wheels) is smaller in sport
//   D1  drive modes: a drive_modes part with abs_default / esc_default per mode; a mode change writes assist.abs and
//       assist.esc as pending channel writes (get_control reads the mode's values after that step); the driver's next write
//       wins; a car without the blocks ignores the keys
//   H1  1 vs 10 workers: equal state hash every tick through the A1 stop and the E1 sine (ESC on); zero own allocation per
//       tick (debug_alloc)
//   H2  car_sedan + the unit with assist.abs 0 and assist.esc 0: chassis pose and velocity bit-identical every tick to the
//       plain car_sedan through a 4 s brake + steer + throttle run (the switched-off controllers act on nothing)
//   Hash gate: every data/scenarios file at 1 and 10 workers unchanged (no shipped car carries the blocks).
//
// SABOTAGES (temporary edits in core/src/vehicle/vehicle_instance.cpp, each reverted; filled in after the runs):

#include <catch2/catch_test_macros.hpp>
