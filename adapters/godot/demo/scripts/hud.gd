extends CanvasLayer
# adapters/godot/demo/scripts/hud.gd — on-screen readout (PLAN.md P2 demo
# requirement: "an on-screen readout (tick rate, sim time, controls)").

@export var simulation_path: NodePath
@export var input_map_path: NodePath

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

func _ready() -> void:
	_simulation = get_node_or_null(simulation_path)
	_input_map = get_node_or_null(input_map_path)
	_label = get_node("Readout")
	_window_start_s = Time.get_ticks_msec() / 1000.0

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

	if now_s - _window_start_s >= 1.0:
		_last_measured_hz = float(ticks - _window_start_ticks) / (now_s - _window_start_s)
		_window_start_s = now_s
		_window_start_ticks = ticks
		print("ps_godot HUD: measured sim tick rate over last window: %.2f Hz (target %.1f Hz), fps=%.1f" % [
			_last_measured_hz, _simulation.get_tick_rate_hz(), Engine.get_frames_per_second()])

	var lines := PackedStringArray()
	lines.append("physics_sim P2 demo")
	lines.append("sim tick rate (target): %.1f Hz" % _simulation.get_tick_rate_hz())
	lines.append("sim tick rate (measured): %.1f Hz" % _last_measured_hz)
	lines.append("godot fps: %.1f" % Engine.get_frames_per_second())
	lines.append("sim time: %.2f s   ticks: %d" % [_simulation.get_sim_time(), ticks])
	lines.append("bodies: %d" % _simulation.get_body_count())
	lines.append("")
	lines.append("controls (arrows: camera, shift: boost)")
	if _input_map != null:
		lines.append("steer=%.2f throttle=%.2f brake=%.2f handbrake=%.2f clutch=%.2f" % [
			_input_map.get_steer(), _input_map.get_throttle(), _input_map.get_brake(),
			_input_map.get_handbrake(), _input_map.get_clutch()])
	_label.text = "\n".join(lines)
