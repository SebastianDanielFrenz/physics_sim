extends CanvasLayer
# adapters/godot/demo/scripts/hud.gd — on-screen readout (PLAN.md P2: "an
# on-screen readout (tick rate, sim time, controls)"; PLAN.md M1 step 5:
# "speed (km/h), throttle/brake/steer bars, per-wheel load and slip if
# available, sim tick rate, render fps, adapter main-thread time per frame,
# resident terrain tiles, and the surface under each wheel if available").
#
# Reads this frame's adapter main-thread time via
# PsSimulation.consume_adapter_frame_time_us() - see frame_timing.h's
# comment for why that total is meaningful only once every other adapter
# node (PsBodyNode3D, PsInputMap, PsDebugDraw, free_cam.gd/chase_cam.gd) has
# already run its own _process this frame; this node is added last in
# main.gd's tree so Godot processes it last among siblings.

@export var simulation_path: NodePath
@export var input_map_path: NodePath
@export var vehicle_name: String = ""

var _simulation: Node
var _input_map: Node
var _label: Label

# For the tick-rate-stability measurement (PLAN.md P2 acceptance): tracks
# ticks_run() over a rolling one-second window of Godot process() calls, so
# the HUD (and the smoke-test log, via print()) shows the sim's own 240 Hz
# independent of whatever frame rate Godot itself is running at.
var _window_start_s: float = 0.0
var _window_start_ticks: int = 0
var _last_measured_hz: float = 0.0
var _printed_first_tick: bool = false

# Adapter main-thread time per frame (PLAN.md P2 front-end target: < 1 ms) -
# a rolling max/avg over the same 1 s window, since a single frame's number
# is noisy.
var _frame_time_sum_us: int = 0
var _frame_time_max_us: int = 0
var _frame_time_count: int = 0
var _last_avg_frame_time_us: float = 0.0
var _last_max_frame_time_us: int = 0

func _ready() -> void:
	_simulation = get_node_or_null(simulation_path)
	_input_map = get_node_or_null(input_map_path)
	_label = get_node("Readout")
	_window_start_s = Time.get_ticks_msec() / 1000.0

func _bar(value01: float, width: int = 20) -> String:
	var v: float = clamp(value01, 0.0, 1.0)
	var filled: int = int(round(v * width))
	return "[" + "#".repeat(filled) + "-".repeat(width - filled) + "]"

func _process(_delta: float) -> void:
	if _simulation == null:
		return

	var now_s := Time.get_ticks_msec() / 1000.0
	var ticks: int = _simulation.get_step_count()

	# A short --quit-after headless smoke-test run may finish well under a
	# second of wall clock, before the measurement window below ever
	# fires - print as soon as the sim thread has produced its first tick
	# so a smoke test has SOMETHING to grep for regardless of run length.
	if not _printed_first_tick and ticks > 0:
		_printed_first_tick = true
		print("ps_godot HUD: sim thread running, first observed tick count: %d (target rate %.1f Hz)" % [ticks, _simulation.get_tick_rate_hz()])

	# This frame's adapter main-thread cost - consumed (and reset) exactly
	# once per frame, from the LAST node to process this frame (see file
	# comment). Folded into the rolling window below.
	var frame_time_us: int = _simulation.consume_adapter_frame_time_us()
	_frame_time_sum_us += frame_time_us
	_frame_time_max_us = max(_frame_time_max_us, frame_time_us)
	_frame_time_count += 1

	if now_s - _window_start_s >= 1.0:
		_last_measured_hz = float(ticks - _window_start_ticks) / (now_s - _window_start_s)
		_window_start_s = now_s
		_window_start_ticks = ticks
		_last_avg_frame_time_us = float(_frame_time_sum_us) / max(_frame_time_count, 1)
		_last_max_frame_time_us = _frame_time_max_us
		_frame_time_sum_us = 0
		_frame_time_max_us = 0
		_frame_time_count = 0
		print("ps_godot HUD: measured sim tick rate over last window: %.2f Hz (target %.1f Hz), fps=%.1f, adapter_ms avg=%.3f max=%.3f, terrain tiles resident=%d starved=%d" % [
			_last_measured_hz, _simulation.get_tick_rate_hz(), Engine.get_frames_per_second(),
			_last_avg_frame_time_us / 1000.0, _last_max_frame_time_us / 1000.0,
			_simulation.terrain_resident_tile_count(), _simulation.terrain_starved_tile_count()])

	var lines := PackedStringArray()
	lines.append("physics_sim M1 demo - test drive")
	lines.append("sim tick rate: target %.1f Hz  measured %.2f Hz" % [_simulation.get_tick_rate_hz(), _last_measured_hz])
	lines.append("godot fps: %.1f    adapter main-thread: avg %.3f ms  max %.3f ms" % [
		Engine.get_frames_per_second(), _last_avg_frame_time_us / 1000.0, _last_max_frame_time_us / 1000.0])
	lines.append("sim time: %.2f s   ticks: %d   bodies: %d" % [_simulation.get_sim_time(), ticks, _simulation.get_body_count()])
	lines.append("terrain tiles: resident %d  starved %d" % [_simulation.terrain_resident_tile_count(), _simulation.terrain_starved_tile_count()])
	lines.append("")

	if vehicle_name != "" and _simulation.get_vehicle_names().has(vehicle_name):
		var chassis_body: String = _simulation.get_vehicle_chassis_body_name(vehicle_name)
		var speed_mps: float = _simulation.get_body_speed_mps(chassis_body)
		lines.append("speed: %6.1f km/h" % (speed_mps * 3.6))
		if _input_map != null:
			lines.append("steer     %+.2f %s" % [_input_map.get_steer(), _bar((_input_map.get_steer() + 1.0) * 0.5)])
			lines.append("throttle  %5.2f %s" % [_input_map.get_throttle(), _bar(_input_map.get_throttle())])
			lines.append("brake     %5.2f %s" % [_input_map.get_brake(), _bar(_input_map.get_brake())])
			lines.append("handbrake %5.2f  clutch %5.2f" % [_input_map.get_handbrake(), _input_map.get_clutch()])
		lines.append("")
		var wheel_count: int = _simulation.get_vehicle_wheel_count(vehicle_name)
		lines.append("wheel  load(N)  slip_ratio  slip_angle(deg)  surface")
		for i in range(wheel_count):
			var wname: String = _simulation.get_wheel_name(vehicle_name, i)
			var load_n: float = _simulation.get_wheel_load_n(vehicle_name, i)
			var slip_ratio: float = _simulation.get_wheel_slip_ratio(vehicle_name, i)
			var slip_angle_deg: float = rad_to_deg(_simulation.get_wheel_slip_angle(vehicle_name, i))
			var surface: String = _simulation.get_wheel_surface_name(vehicle_name, i)
			if surface == "":
				surface = "?"
			lines.append("%-5s  %7.0f     %+.3f          %+6.2f          %s" % [wname, load_n, slip_ratio, slip_angle_deg, surface])
	else:
		lines.append("(no vehicle spawned)")

	lines.append("")
	lines.append("controls: gamepad (steer/throttle/brake/handbrake) + WASD/space keyboard fallback")
	lines.append("V camera(free/chase)  G debug draw  R reset vehicle  Back(joy)=reset")
	lines.append("1 straight  2 skidpad  3 ramps (teleport)")
	_label.text = "\n".join(lines)
