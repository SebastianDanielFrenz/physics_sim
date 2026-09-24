extends Control
# adapters/godot/demo/scripts/tach_gauge.gd — driver HUD tachometer gauge
# (PLAN.md P4 "Driver HUD" paragraph, owner's hand sketch): bottom-right
# round tachometer (rpm x1000 around the arc, redline zone, needle) with the
# gear in a circle at its centre, assist lamps around/below that circle, and
# a digital km/h readout beside it. Generic, not per car - every number is
# derived from the vehicle's own data via PsSimulation.get_vehicle_
# gauge_info()/get_vehicle_powertrain(), never a hardcoded car_sedan/tractor
# constant.
#
# All the non-rendering math (tick/label layout, needle angle, max-rpm
# rounding, lamp state, needle smoothing) lives in gauge_logic.gd as plain
# static functions with no Godot Control/_draw() dependency - this script
# only converts GaugeLogic's "clock degrees" (see that file's header
# comment) into Godot's own CanvasItem polar-angle convention and issues
# draw_arc/draw_line/draw_string/draw_circle calls. Keeping that split means
# gauge_logic.gd ports to UE5 UMG/C++ untouched (CLAUDE.md/MEMORY.md's
# engine-neutral-logic rule); only this file is Godot-specific.
#
# No textures anywhere (brief) - everything is primitive draws, scaled by
# viewport height (`scale` below) so the gauge reads the same size at 1080p
# and 4K.

@export var simulation_path: NodePath
@export var input_map_path: NodePath
@export var vehicle_name: String = ""

# Explicit preload, not a bare "GaugeLogic" reference: this demo builds its
# scene procedurally and loads both scripts via load() at runtime
# (main.gd's _setup_tach_gauge), which can run before Godot has rescanned
# res:// for global class_name declarations (global_script_class_cache.cfg)
# - a bare reference then fails to parse ("Identifier not declared"), found
# running smoke_test.ps1. Preloading the script resource directly sidesteps
# the global class cache entirely; static function/enum access below
# (GaugeLogic.foo(), GaugeLogic.LampState.X) works the same either way.
const GaugeLogic := preload("res://scripts/gauge_logic.gd")

var _simulation: Node
var _input_map: Node

# Cached once per vehicle (static data: idle/limiter rpm, gear count, fitted
# assists) - see PsSimulation::get_vehicle_gauge_info's own header comment
# for why this is separate from the per-tick PowertrainSnapshot below.
var _gauge_info: Dictionary = {}
var _last_pt: Dictionary = {}

# Needle smoothing state (brief: "smooth it lightly ... no smoothing on the
# gear" - so only this one value is filtered, via GaugeLogic.smooth_towards).
var _display_rpm: float = 0.0
const NEEDLE_SMOOTH_RATE_PER_S := 10.0

const COLOR_BG := Color(0.05, 0.05, 0.07, 0.55)
const COLOR_TICK_MAJOR := Color(0.92, 0.92, 0.95, 0.95)
const COLOR_TICK_MINOR := Color(0.6, 0.6, 0.66, 0.7)
const COLOR_REDLINE := Color(0.85, 0.15, 0.1, 0.9)
const COLOR_NEEDLE := Color(1.0, 1.0, 1.0, 0.95)
const COLOR_NEEDLE_REDLINE := Color(1.0, 0.3, 0.15, 1.0)
const COLOR_GEAR_BG := Color(0.08, 0.08, 0.1, 0.85)
const COLOR_GEAR_TEXT := Color(0.95, 0.95, 1.0, 1.0)
const COLOR_GEAR_STALLED := Color(0.6, 0.15, 0.15, 1.0)
const COLOR_GEAR_GRIND_FLASH := Color(1.0, 0.7, 0.0, 1.0)
const COLOR_LAMP_OFF := Color(0.4, 0.4, 0.44, 0.85)
const COLOR_LAMP_ON := Color(0.25, 0.9, 0.35, 1.0)
const COLOR_LAMP_INTERVENING := Color(1.0, 0.75, 0.0, 1.0)
const COLOR_SPEED_TEXT := Color(0.95, 0.95, 1.0, 1.0)

# Cosmetic slot layout (clock degrees, see gauge_logic.gd) for the five
# not-yet-implemented system-assist lamps around the gear circle, matching
# the owner's sketch roughly - ABS/TC/LC lower-left/left/bottom, ESP/HILL
# right/upper-right. All five are HIDDEN today (fitted_assists is always
# empty - see PsSimulation::get_vehicle_gauge_info's comment); the slots
# exist so a future increment adding real fitted flags is a data change
# only, not a layout change.
const SYSTEM_LAMP_NAMES := ["ABS", "TC", "ESP", "LC", "HILL"]
const SYSTEM_LAMP_SLOTS_CLOCK_DEG := [250.0, 322.0, 34.0, 106.0, 178.0]

func _ready() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_simulation = get_node_or_null(simulation_path)
	_input_map = get_node_or_null(input_map_path)

func _process(delta: float) -> void:
	if _simulation != null and vehicle_name != "" and _simulation.get_vehicle_names().has(vehicle_name):
		if _gauge_info.is_empty():
			_gauge_info = _simulation.get_vehicle_gauge_info(vehicle_name)
		_last_pt = _simulation.get_vehicle_powertrain(vehicle_name)
		var target_rpm: float = _last_pt.get("rpm", 0.0)
		_display_rpm = GaugeLogic.smooth_towards(_display_rpm, target_rpm, NEEDLE_SMOOTH_RATE_PER_S, delta)
	else:
		_last_pt = {}
	queue_redraw()

# --- Godot polar-angle conversion (see gauge_logic.gd's file header: its
# functions all return "clock degrees", 0 = 12 o'clock, clockwise) --------
func _clock_rad(clock_deg: float) -> float:
	return deg_to_rad(clock_deg - 90.0)

func _polar(center: Vector2, radius: float, clock_deg: float) -> Vector2:
	var rad: float = _clock_rad(clock_deg)
	return center + radius * Vector2(cos(rad), sin(rad))

func _draw_centered_text(font: Font, pos: Vector2, text: String, font_size: int, color: Color) -> void:
	var text_size: Vector2 = font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size)
	draw_string(font, pos - Vector2(text_size.x * 0.5, -text_size.y * 0.35), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, color)

func _draw_lamp(font: Font, pos: Vector2, label: String, state: int, dot_radius: float, font_size: int) -> void:
	var color: Color
	if state == GaugeLogic.LampState.ON:
		color = COLOR_LAMP_ON
	elif state == GaugeLogic.LampState.INTERVENING:
		var blink: bool = fmod(Time.get_ticks_msec() / 1000.0, 0.4) < 0.2
		color = COLOR_LAMP_INTERVENING if blink else COLOR_LAMP_OFF
	else:
		color = COLOR_LAMP_OFF
	draw_circle(pos, dot_radius, color)
	_draw_centered_text(font, pos + Vector2(0, dot_radius + font_size * 0.9), label, font_size, color)

# One table, lamp name -> the field that drives it (PLAN.md P4 "Driver HUD":
# "Keep one table mapping lamp name -> snapshot field, so adding a lamp
# later is one line"). These four are always fitted - every vehicle has
# auto-clutch/auto-blip/auto-shift (PowertrainSnapshot::assist_active_bits)
# and the demo always has the adapter's own gamepad-rumble haptics
# (PsInputMap.get_rumble_enabled) - unlike ABS/TC/ESP/LC/HILL above, whose
# fitted state is genuinely per-vehicle data (PsSimulation.
# get_vehicle_gauge_info's fitted_assists).
func _existing_assist_lamp_states() -> Array:
	var rumble_on: bool = _input_map != null and bool(_input_map.get_rumble_enabled())
	var table := [
		{"name": "CLUTCH", "active": bool(_last_pt.get("assist_auto_clutch", false))},
		{"name": "BLIP", "active": bool(_last_pt.get("assist_auto_blip", false))},
		{"name": "SHIFT", "active": bool(_last_pt.get("assist_auto_shift", false))},
		{"name": "RUMBLE", "active": rumble_on},
	]
	var out := []
	for row in table:
		out.append({"name": row["name"], "state": GaugeLogic.lamp_state(true, row["active"], false)})
	return out

func _draw() -> void:
	if _simulation == null or vehicle_name == "" or _gauge_info.is_empty() or _last_pt.is_empty():
		return

	var idle_rpm: float = _gauge_info.get("idle_rpm", 0.0)
	var limiter_rpm: float = _gauge_info.get("limiter_rpm", 0.0)
	var gear_count: int = int(_gauge_info.get("gear_count", 0))
	var max_rpm: float = GaugeLogic.max_rpm_for_limiter(limiter_rpm)

	var vp_size: Vector2 = get_viewport_rect().size
	var scale: float = max(vp_size.y / 1080.0, 0.4)
	var radius: float = 150.0 * scale
	var margin: float = 64.0 * scale
	# The gauge's own lowest drawn element is the existing-assist lamp row,
	# at radius*1.28 below centre plus its label text (see the row further
	# down) - reserve half a radius of extra vertical room beyond the dial
	# itself so that row never clips off the bottom of a short viewport
	# (found narrower than 1080p, e.g. this demo's default un-maximised
	# window).
	var center := Vector2(vp_size.x - margin - radius, vp_size.y - margin - radius * 1.5)

	var font: Font = get_theme_default_font()
	var font_size_tick: int = int(round(15.0 * scale))
	# Tractor-class gearboxes (data/gearboxes/tractor_12f4r.json: 12 forward
	# gears) need two digits in the gear circle where the car (6) needs one -
	# gear_count (from the gearbox, not hardcoded) picks the font size so
	# both fit without per-vehicle tuning.
	var gear_font_scale: float = 1.0 if gear_count < 10 else 0.75
	var font_size_gear: int = int(round(46.0 * scale * gear_font_scale))
	var font_size_lamp: int = int(round(13.0 * scale))
	var font_size_speed: int = int(round(52.0 * scale))
	var font_size_speed_unit: int = int(round(16.0 * scale))

	# --- dial background ---
	draw_circle(center, radius * 1.08, COLOR_BG)

	# --- redline zone (limiter_rpm .. max_rpm), under the ticks ---
	var redline_start_rad: float = _clock_rad(GaugeLogic.redline_start_clock_deg(limiter_rpm, max_rpm))
	var redline_end_rad: float = _clock_rad(GaugeLogic.redline_end_clock_deg(max_rpm))
	if redline_end_rad > redline_start_rad + 0.001:
		draw_arc(center, radius * 1.02, redline_start_rad, redline_end_rad, 32, COLOR_REDLINE, radius * 0.05, true)

	# --- major/minor ticks + rpm labels (x1000) ---
	for major_rpm in GaugeLogic.major_tick_rpms(max_rpm):
		var deg: float = GaugeLogic.clock_deg_for_rpm(major_rpm, max_rpm)
		draw_line(_polar(center, radius * 0.86, deg), _polar(center, radius, deg), COLOR_TICK_MAJOR, scale * 2.5, true)
		_draw_centered_text(font, _polar(center, radius * 0.70, deg), str(int(round(major_rpm / 1000.0))), font_size_tick, COLOR_TICK_MAJOR)
	for minor_rpm in GaugeLogic.minor_tick_rpms(max_rpm):
		var deg2: float = GaugeLogic.clock_deg_for_rpm(minor_rpm, max_rpm)
		draw_line(_polar(center, radius * 0.93, deg2), _polar(center, radius, deg2), COLOR_TICK_MINOR, scale * 1.5, true)

	# --- idle marker (small dot, from the engine's own idle_rpm - generic,
	# not hardcoded: car_sedan.json idles at 800, tractor.json at 850) ---
	if idle_rpm > 0.0:
		draw_circle(_polar(center, radius * 0.79, GaugeLogic.clock_deg_for_rpm(idle_rpm, max_rpm)), scale * 3.0, COLOR_TICK_MINOR)

	# --- needle (smoothed rpm - _display_rpm, not the raw pt.rpm) ---
	var needle_deg: float = GaugeLogic.clock_deg_for_rpm(_display_rpm, max_rpm)
	var in_redline: bool = limiter_rpm > 0.0 and _display_rpm >= limiter_rpm
	var needle_color: Color = COLOR_NEEDLE_REDLINE if in_redline else COLOR_NEEDLE
	draw_line(_polar(center, radius * 0.14, needle_deg + 180.0), _polar(center, radius * 0.88, needle_deg), needle_color, scale * 3.0, true)
	draw_circle(center, radius * 0.06, needle_color)

	# --- gear circle (no smoothing - brief: "No smoothing on the gear") ---
	var gear_radius: float = radius * 0.34
	var engine_state: String = String(_last_pt.get("engine_state", "off"))
	var grind_active: bool = bool(_last_pt.get("grind_active", false))
	var blink_on: bool = fmod(Time.get_ticks_msec() / 1000.0, 0.3) < 0.15
	var gear_text_color: Color = COLOR_GEAR_TEXT
	if grind_active and blink_on:
		gear_text_color = COLOR_GEAR_GRIND_FLASH
	elif engine_state == "stalled" or engine_state == "off":
		gear_text_color = COLOR_GEAR_STALLED
	draw_circle(center, gear_radius, COLOR_GEAR_BG)
	draw_arc(center, gear_radius, 0.0, TAU, 48, COLOR_TICK_MINOR, scale * 1.5, true)
	_draw_centered_text(font, center, GaugeLogic.gear_label(int(_last_pt.get("gear", 0))), font_size_gear, gear_text_color)

	# --- system assist lamps around the gear circle (ABS/TC/ESP/LC/HILL -
	# all hidden today, see this file's header comment) ---
	var fitted_assists: PackedStringArray = _gauge_info.get("fitted_assists", PackedStringArray())
	for i in range(SYSTEM_LAMP_NAMES.size()):
		var lamp_name: String = SYSTEM_LAMP_NAMES[i]
		var state: int = GaugeLogic.lamp_state(fitted_assists.has(lamp_name), false, false)
		if state == GaugeLogic.LampState.HIDDEN:
			continue
		_draw_lamp(font, _polar(center, gear_radius * 1.7, SYSTEM_LAMP_SLOTS_CLOCK_DEG[i]), lamp_name, state, 6.0 * scale, font_size_lamp)

	# --- existing driver-assist lamps, below the gauge ---
	var extra_lamps: Array = _existing_assist_lamp_states()
	var row_y: float = center.y + radius * 1.28
	var row_spacing: float = 62.0 * scale
	var row_start_x: float = center.x - row_spacing * (extra_lamps.size() - 1) * 0.5
	for i in range(extra_lamps.size()):
		var entry: Dictionary = extra_lamps[i]
		_draw_lamp(font, Vector2(row_start_x + i * row_spacing, row_y), entry["name"], entry["state"], 6.0 * scale, font_size_lamp)

	# --- digital speed readout, left of the gauge (stays on-screen at any
	# resolution, unlike drawing it further right past the gauge) ---
	var speed_kmh: float = float(_last_pt.get("speedo_kmh", 0.0))
	var speed_pos := Vector2(center.x - radius * 1.55, center.y)
	_draw_centered_text(font, speed_pos + Vector2(0, -font_size_speed * 0.35), "%d" % int(round(speed_kmh)), font_size_speed, COLOR_SPEED_TEXT)
	_draw_centered_text(font, speed_pos + Vector2(0, font_size_speed * 0.55), "km/h", font_size_speed_unit, COLOR_TICK_MINOR)

	# --- engine warning banner above the gauge (owner: "the engine has
	# stalled without telling me" - a dimmed gear digit alone was not enough) ---
	var warning: String = ""
	if engine_state == "stalled":
		warning = "ENGINE STALLED - hold Start / K to restart"
	elif engine_state == "off":
		warning = "IGNITION OFF - press D-pad Up / I"
	elif grind_active:
		warning = "GRIND!"
	var slow_blink_on: bool = fmod(Time.get_ticks_msec() / 1000.0, 1.0) < 0.7
	if warning != "" and (engine_state == "off" or (slow_blink_on if engine_state == "stalled" else blink_on)):
		var banner_pos := Vector2(center.x - radius * 0.5, center.y - radius * 1.35)
		_draw_centered_text(font, banner_pos, warning, int(round(24.0 * scale)), COLOR_NEEDLE_REDLINE)
