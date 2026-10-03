class_name GaugeLogic
extends RefCounted
# adapters/godot/demo/scripts/gauge_logic.gd — pure, engine-agnostic layout
# and state math for the driver HUD tachometer gauge (PLAN.md P4 "Driver
# HUD", CLAUDE.md/MEMORY.md's engine-neutral-logic rule: no Godot Control/
# _draw()/CanvasItem call anywhere in this file, so it ports to UE5 UMG/C++
# untouched). tach_gauge.gd is the Godot-specific renderer that calls these
# static functions and does its own conversion into whatever polar-angle/
# coordinate convention its drawing API expects.
#
# Angle convention used by every function here: "clock degrees" - 0 = 12
# o'clock (top), increasing CLOCKWISE, exactly like an hour hand. The gauge
# sweeps from GAUGE_START_CLOCK_DEG (~7 o'clock) clockwise through 12 to
# GAUGE_START_CLOCK_DEG + GAUGE_SWEEP_DEG (~3 o'clock) - matches the owner's
# hand sketch (PLAN.md P4 "Driver HUD" paragraph).

const GAUGE_START_CLOCK_DEG := 210.0 # 7 o'clock
const GAUGE_SWEEP_DEG := 240.0        # ... clockwise to 3 o'clock (210 + 240 = 450 = 90 mod 360)
const MAJOR_TICK_STEP_RPM := 1000.0
const MINOR_TICK_STEP_RPM := 500.0

# "Round max rpm up to the next 1000 above the limiter" (brief) - generic,
# not per-car: a fuel-cut car limiter (car_sedan.json: 6800 -> 7000) and a
# soft-cut tractor governor rpm (tractor.json: 2400 -> 3000) both come out
# right from the same formula.
static func max_rpm_for_limiter(limiter_rpm: float) -> float:
	if limiter_rpm <= 0.0:
		return MAJOR_TICK_STEP_RPM
	return floor(limiter_rpm / MAJOR_TICK_STEP_RPM) * MAJOR_TICK_STEP_RPM + MAJOR_TICK_STEP_RPM

# rpm values for major ticks (every MAJOR_TICK_STEP_RPM, 0..max_rpm inclusive).
static func major_tick_rpms(max_rpm: float) -> Array:
	var out := []
	var rpm := 0.0
	while rpm <= max_rpm + 0.01:
		out.append(rpm)
		rpm += MAJOR_TICK_STEP_RPM
	return out

# rpm values for minor-only ticks (excludes anything that is also a major tick).
static func minor_tick_rpms(max_rpm: float) -> Array:
	var out := []
	var rpm := MINOR_TICK_STEP_RPM
	while rpm <= max_rpm + 0.01:
		if fmod(rpm, MAJOR_TICK_STEP_RPM) > 1.0:
			out.append(rpm)
		rpm += MINOR_TICK_STEP_RPM
	return out

# Needle/tick position for a given rpm, in clock degrees (see file header).
static func clock_deg_for_rpm(rpm: float, max_rpm: float) -> float:
	var frac: float = clamp(rpm / max(max_rpm, 1.0), 0.0, 1.0)
	return GAUGE_START_CLOCK_DEG + frac * GAUGE_SWEEP_DEG

# Redline zone [limiter_rpm, max_rpm], as clock degrees, clamped so a
# limiter at or above max_rpm still yields a valid (zero-width) zone rather
# than an inverted one.
static func redline_start_clock_deg(limiter_rpm: float, max_rpm: float) -> float:
	return clock_deg_for_rpm(min(limiter_rpm, max_rpm), max_rpm)

static func redline_end_clock_deg(max_rpm: float) -> float:
	return clock_deg_for_rpm(max_rpm, max_rpm)

# Gear label: 0 = "N", positive = forward gear number, negative = "R" -
# matches ps::drivetrain::PowertrainSnapshot::gear's own convention
# (core/include/ps/drivetrain/powertrain_desc.h's kGearRequestNone comment:
# "0 = N, k > 0 forward, -k reverse Rk"). I3 never ships more than one
# reverse ratio as a selectable "gear" (both data/vehicles/*.json's
# gearboxes have several reverse_ratios internally, but the driver only ever
# selects "R" - PLAN.md's sketch shows a single R), so every negative gear
# collapses to the one "R" label.
static func gear_label(gear: int) -> String:
	if gear == 0:
		return "N"
	if gear < 0:
		return "R"
	return str(gear)

# One assist/system lamp's render state (PLAN.md P4 "Driver HUD": "a lamp
# shows three states: not fitted (hidden), fitted but off, and on/
# intervening (lit, flashing while it acts)").
enum LampState { HIDDEN, OFF, ON, INTERVENING }

static func lamp_state(fitted: bool, active: bool, intervening: bool) -> int:
	if not fitted:
		return LampState.HIDDEN
	if intervening:
		return LampState.INTERVENING
	if active:
		return LampState.ON
	return LampState.OFF

# Light exponential smoothing toward `target` at `rate_per_s`, framerate-
# independent - used for the needle only (brief: "smooth it lightly ... no
# smoothing on the gear").
static func smooth_towards(current: float, target: float, rate_per_s: float, delta: float) -> float:
	var t: float = clamp(rate_per_s * delta, 0.0, 1.0)
	return lerp(current, target, t)
