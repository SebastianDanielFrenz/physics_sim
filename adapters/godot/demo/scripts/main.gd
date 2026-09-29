extends Node3D
# adapters/godot/demo/scripts/main.gd — PLAN.md M1 demo project entry point,
# "first test drive".
#
# Builds the scene procedurally: a fresh, empty World (scenarios/
# m1_world.json - just WorldConfig, no bodies: this demo builds everything
# else itself, unlike the P1/P2 demo which played back a pre-built P1
# scenario file), the test ground's REAL collision + procedural terrain
# (PsSimulation.import_test_ground), the P3 passenger car spawned on it
# (PsSimulation.create_body_dynamic + spawn_vehicle, at data/tracks/
# test_ground/test_ground.json's own "spawn_straight_car" point - the paved
# asphalt straight, not the open dirt disc at "spawn_main" (M1 visual pass:
# spawning on dirt made the first drive feel sluggish) - the vehicle is
# also the terrain's streaming interest point, PLAN.md M1, updated
# automatically every tick inside PsSimulation's post_step), shown with the
# existing car_sedan.glb art (vehicle_visual.gd), a free-look camera and a
# chase camera (toggled with the "camera" control action), a debug-draw
# toggle, and the HUD (hud.gd). Xbox/keyboard input (PsInputMap) feeds the
# vehicle only through named control channels (PsSimulation.set_control) -
# PLAN.md P2/M1's own rule. Keys 1/2/3 teleport the vehicle to the straight/
# skidpad/ramps (ZONE_SPAWNS below) for quick access without a long drive.
#
# --m1-autopilot (a Godot user cmdline arg, after "--" - see
# adapters/godot/smoke_test.ps1): runs a scripted throttle/brake/steer
# sequence instead of waiting for a human, measures distance travelled
# (integral of speed - robust to origin rebasing/spawn point, unlike a
# start/end position diff) and top speed, checks for NaN and that the
# chassis stayed upright, prints one grep-able "ps_godot M1 smoke:" summary
# line, then quits - this IS the M1 headless smoke test's driving check.
#
# --m1-force-rebase-test (combined with --m1-autopilot): verification for
# the chase-cam origin-rebase bug (chase_cam.gd's header comment) - forces
# one origin rebase, through the chase camera's own rebase_focus call,
# partway through the autopilot run (the placeholder drive alone never
# covers the 500 m a real rebase needs), then watches for a frame-to-frame
# JUMP in camera-to-chassis distance (something a smooth exponential lag
# can never produce on its own) and prints a grep-able "ps_godot M1 camera
# check:" summary line with a pass/fail verdict - see
# _run_camera_rebase_check's own comment for exactly how the rebase is
# forced and why that specific signal was chosen.

const VEHICLE_NAME := "car"
const CHASSIS_BODY_NAME := "car_chassis"
# Generic internal handles (above) - not vehicle-specific, left as-is
# whichever vehicle is spawned.

# Per-vehicle chassis body + art + spawn table (tractor demo, 2026-09-28):
# keyed by the vehicle json's own basename (vehicle_relative_path.get_file().
# get_basename()), resolved once in _ready() into the _chassis_*/model_
# relative_path/spawn_id instance state below - NO physics logic here, just
# plain data the adapter needs to build the chassis rigid body and load the
# right art, exactly mirroring what World::create_vehicle/spawn_vehicle
# already read from data/vehicles/<name>.json for the rest of the vehicle.
# car_sedan's entry mirrors the pre-existing hardcoded consts exactly (byte
# for byte) so the sedan's own behaviour is unchanged. tractor's chassis
# mirrors data/scenarios/vehicle_tractor_i4.json's own "chassis" body
# (mass 6000, half_extents [1.2, 0.35, 0.9], z 1.28) the same way car_sedan's
# entry mirrors vehicle_step_steer.json's.
const VEHICLE_TABLE := {
	"car_sedan": {
		"chassis_mass": 1500.0,
		"chassis_half_extents": Vector3(2.0, 0.4, 0.15),
		"chassis_z_offset": 0.6,
		"model_relative_path": "data/models/car_sedan/car_sedan.glb",
		"spawn_id": "spawn_straight_car",
	},
	"tractor": {
		"chassis_mass": 6000.0,
		"chassis_half_extents": Vector3(1.2, 0.35, 0.9),
		"chassis_z_offset": 1.28,
		"model_relative_path": "data/models/tractor/tractor.glb",
		"spawn_id": "spawn_straight_tractor",
	},
}

# Resolved once in _ready() from VEHICLE_TABLE (defaults below match
# car_sedan so nothing looks uninitialized before _ready() runs); read by
# both _ready() (chassis spawn) and _reset_vehicle() (R key / zone teleport)
# since CHASSIS_Z_OFFSET used to be a plain const read from two places.
var _chassis_mass: float = 1500.0
var _chassis_half_extents: Vector3 = Vector3(2.0, 0.4, 0.15)
var _chassis_z_offset: float = 0.6
# Tractor demo (2026-09-28): only tractor.json's engine declares a governor
# (core's CLAUDE.md "Drivetrain": TorqueMapEngineDesc::has_governor), so only
# it declares the "hand_throttle" control channel (Powertrain::declare_
# control_channels). set_control() on any other vehicle logs a per-call
# "unknown channel, not pre-seeded" warning (core's adapter-seams contract) -
# harmless once, but hand_throttle is sent EVERY frame (a continuous ramp,
# not an edge-triggered toggle like the other tractor-only channels below),
# so gate it on the resolved vehicle key or car_sedan would spam this warning
# at up to 240 Hz. Resolved once in _ready() alongside the chassis fields.
var _vehicle_has_hand_throttle: bool = false

@export var vehicle_relative_path: String = "data/vehicles/car_sedan.json"
@export var track_relative_dir: String = "data/tracks/test_ground"
@export var model_relative_path: String = "data/models/car_sedan/car_sedan.glb"
@export var terrain_model_relative_path: String = "data/tracks/test_ground/test_ground.glb"
@export var spawn_id: String = "spawn_straight_car" # M1 visual pass: spawn on the paved asphalt straight (data/tracks/test_ground/test_ground.json), not spawn_main's open dirt disc - the dirt's low traction made the very first drive feel sluggish. The FL/FR/RL/RR wheel_surface_name asphalt<->dirt flicker once reported here was PHYS-021 (terrain/static-feature coincident-collision, see core's CLAUDE.md), fixed in core by 46275a8: a headless measurement under this demo's own load order, config and interest-point timing confirms FL/FR report "asphalt" with ZERO transitions over 1440 ticks, hitting the asphalt mesh body every tick from tick 0, terrain tiles resident by tick 5 (~21 ms). PHYS-021's own residual - a sub-quad-width sliver right at a feature's true edge, up to ~4 m at this resolution (CLAUDE.md) - can still show a brief flicker if a wheel sits exactly on that edge, so don't read a flicker here as this issue coming back; check whether the wheel is that close to the straight's edge first.
@export var terrain_stream_radius_m: float = 700.0
@export var terrain_max_resident_tiles: int = 512
@export var terrain_max_tile_fills_per_tick: int = 16
# Testing aid (owner request): load data/surfaces/surfaces_test_grip.json
# instead of the shared surfaces.json - the open procedural terrain's grass/
# dirt/field (everything under the plain grid) grips like dry asphalt, so
# handling can be judged anywhere. Kerb and low_mu keep their real values.
# Set false for the real surfaces.
@export var test_grip_everywhere: bool = true
const TEST_GRIP_SURFACES_RELATIVE_PATH := "data/surfaces/surfaces_test_grip.json"

# M1 visual pass: quick teleports so Sebastian can reach the skidpad/ramps
# without a 2 km drive from the straight - keys 1/2/3 (see _process below).
const ZONE_SPAWNS := {
	KEY_1: "spawn_straight_car",
	KEY_2: "spawn_skidpad_car",
	KEY_3: "spawn_ramp_05_foot",
}

# test_ground.json spawn "area" values that are a start-line spawn on a
# narrow paved feature, where _read_spawn's forward nudge (see its own
# comment) is needed and safe. Used by _read_spawn.
const NUDGE_AREAS := {
	"straight": true,
	"split_mu": true,
}

var simulation: Node
var hud: CanvasLayer
var debug_draw: Node
var input_map: Node
var free_camera: Camera3D
var chase_camera: Camera3D
var using_chase_camera: bool = true
var ground_grid: MeshInstance3D

# ps_audio A11: engine sound, only when the spawned vehicle's own engine has
# a live cycle model (VEHICLE_NAME's simulated engine with pipe blocks -
# see ps_simulation.h's has_vehicle_voice/adapters/godot/src/vehicle_voice.h
# for what qualifies). null on any vehicle without one (most, including the
# default car_sedan.json) - _process below no-ops in that case.
var _engine_audio_player: AudioStreamPlayer = null
var _engine_audio_underrun_last: int = 0

# Tractor demo (2026-09-28): true once --m1-spawn= is seen on the cmdline, so
# the VEHICLE_TABLE resolution in _ready() does not clobber an explicit
# owner-chosen spawn with the selected vehicle's own table default.
var _spawn_id_explicit: bool = false

# M1 visual pass bug fix: ground_grid used to be a fire-and-forget child with
# an identity transform, fixed forever at Godot (0,0,0) - the speed_grid
# shader's own header comment already assumed otherwise ("this ground plane
# is itself repositioned relative to the origin like everything else the
# adapter draws"), but nothing ever did that repositioning. As soon as the
# floating origin rebases (chase_cam.gd's rebase_focus, e.g. teleporting to
# a zone key or just driving past the 500 m threshold), every OTHER visual
# (terrain_visual.gd, PsBodyNode3D bodies, both cameras) stays glued to the
# car while this plane stayed exactly where it started - visually "flies
# off" to wherever the old origin now maps in the new, rebased frame, which
# can put it well above/behind the camera depending on the rebase direction.
# Fixed the same way terrain_visual.gd tracks the origin: re-read
# PsSimulation.get_world_origin_godot_position() every frame (_process
# below). A small negative Y offset keeps it from z-fighting with
# terrain_visual's feature meshes (also sitting at/near the ISO z=0 flat-zone
# height) at a grazing viewing angle - an offset was chosen over drawing it
# as a depth-tested-off overlay because the grid is meant to read as part of
# the world (a ground-plane reference), not a HUD-like overlay floating in
# front of everything; 2 cm is well under what's visible at this camera's
# distances but enough to resolve the depth-buffer tie everywhere.
const GRID_Y_OFFSET_M := -0.02

var _autopilot: bool = false
var _autopilot_phase_t: float = 0.0
var _autopilot_distance_m: float = 0.0
var _autopilot_max_speed_kmh: float = 0.0
var _autopilot_nan_seen: bool = false
var _autopilot_done: bool = false
var _autopilot_last_logged_second: int = -1
var _autopilot_diag_wheel_index: int = -1 # resolved lazily via get_wheel_driven; -1 = not yet resolved
# PHYS-033 P4 I3 (C8) section 12.4: the autopilot now drives the real
# engine/clutch/gearbox chain via the monotonic shift_up_count/
# shift_down_count channels (same path a human driver's PsInputMap/
# InputMapper uses - see _setup_input_map/_process below), not gear_request
# directly - exercises the adapter's new counter-based channel path end to
# end. These track the absolute counter VALUES sent so far (never reset,
# matching PLAN.md 6.5.1's "consumed as differences" contract).
var _autopilot_shift_up_count: float = 0.0
var _autopilot_shift_down_count: float = 0.0
var _autopilot_max_gear: int = 0
var _autopilot_stalled_seen: bool = false
var _autopilot_final_engine_state: String = ""

# --m1-haptics-cruise: a scripted steady-speed drive on the asphalt straight
# for MEASURING haptics_core.h's per-source debug readout (owner task,
# 2026-09-25: "measure first - find out WHICH source(s) fire while driving
# along the asphalt straight ... at steady speeds"), separate from
# --m1-autopilot's own short launch/brake/steer placeholder drive (that one
# never holds a steady speed long enough to be useful for this). Combine
# with --m1-spawn=spawn_long_straight_car (test_ground.json's 10 km
# top-speed straight, area "long_straight" - plenty of room to accelerate
# to and hold 150 km/h, unlike the ~1000 m "straight" area
# spawn_straight_car itself sits on) and PS_HAPTICS_LOG=1 (input_map.cpp) to
# get one stdout line/second of every source's raw intensity while this
# drives. A simple proportional throttle controller holds each of
# _HAPTICS_CRUISE_TARGETS_KMH in turn (naturally landing at part throttle in
# whichever gear auto-shift picks for that speed - task's "also under part
# throttle in 3rd-4th"), settles, holds for
# HAPTICS_CRUISE_HOLD_S, prints a phase marker, then moves to the next
# target; _finish_haptics_cruise() below quits the run once every target
# has been held (or timed out).
const HAPTICS_CRUISE_TARGETS_KMH: Array[float] = [50.0, 100.0, 150.0]
const HAPTICS_CRUISE_TOLERANCE_MPS := 0.5 # +-1.8 km/h counts as "settled"
const HAPTICS_CRUISE_HOLD_S := 4.0        # settled time required before logging + advancing
const HAPTICS_CRUISE_PHASE_TIMEOUT_S := 90.0 # safety: advance anyway if a phase never settles
# PHYS-033 P4 I3 section 12.4 / test_powertrain_gearbox: a dog-clutch gear
# engagement is refused while the input shaft is under load ("dog release
# refused while loaded then releasing once torque drops"). Requesting the
# launch shift_up_count=1 and letting the P controller's initial (large)
# error slam full throttle in the SAME tick races the engagement so it
# never completes - the car free-revs in Neutral forever (measured: gear
# stayed 0, rpm pinned ~6700-6800 near the limiter, for the whole run before
# this fix). _run_autopilot avoids this by holding throttle at 0 for its
# first 0.5s while the launch shift lands; _run_haptics_cruise does the same
# via this grace window (phase 0 only, throttle forced to 0 until gear > 0
# or the window elapses).
const HAPTICS_CRUISE_LAUNCH_GRACE_S := 1.5
# Measured (this session): setting the "assist.auto_shift" control channel
# does NOT make the car upshift - ps::drivetrain::ManualTcu's auto-shift
# up/down decision (core/src/drivetrain/manual_tcu.cpp step(), the
# `desc_.auto_shift.enabled && ...` branch) reads the STATIC per-vehicle
# AutoShiftDesc::enabled field from data/vehicles/car_sedan.json (currently
# "false"), never the runtime Sensors::assist_auto_shift the control channel
# maps to - that field is set (core/src/drivetrain/powertrain.cpp) but never
# read anywhere in manual_tcu.cpp. First haptics-cruise run confirmed this:
# with the channel set, the car launched into 1st but then held WOT at the
# 6800 rpm fuel-cut limiter for the full 90s phase timeout, never
# upshifting, so "100/150 km/h steady cruise" was never reached. This is a
# core/src/drivetrain gap outside adapters/godot/** scope, so instead of
# relying on the (currently no-op) assist, this script upshifts itself via
# the same shift_up_count channel a human driver/the autopilot use, at
# HAPTICS_CRUISE_UPSHIFT_RPM (car_sedan.json's own auto_shift.
# up_rpm_at_full_throttle=6300, comfortably under the 6800 rpm limiter).
const HAPTICS_CRUISE_UPSHIFT_RPM := 6300.0
const HAPTICS_CRUISE_SHIFT_COOLDOWN_S := 1.2 # >= one gearbox shift's worst-case dwell, avoids re-triggering mid-shift
var _haptics_cruise: bool = false
var _haptics_cruise_launched: bool = false
var _haptics_cruise_done: bool = false
var _haptics_cruise_phase: int = 0
var _haptics_cruise_phase_t: float = 0.0
var _haptics_cruise_hold_t: float = 0.0
var _haptics_cruise_shift_up_count: float = 1.0 # mirrors the literal 1.0 sent by the launch shift below
var _haptics_cruise_shift_cooldown_t: float = 0.0

# --m1-force-rebase-test: verification for the chase-cam origin-rebase bug
# (adapters/godot/demo/scripts/chase_cam.gd's own header comment) - see
# _run_camera_rebase_check's comment for why this forces a rebase (a
# teleport far enough to make the chase cam's own rebase_focus call cross
# OriginRebase's threshold) instead of waiting for the autopilot's
# placeholder drive to cover 500 m, and for why the PRIMARY pass/fail
# signal is a frame-to-frame DISCONTINUITY check, not a plain distance
# bound: the teleport itself legitimately makes the chase cam's own
# exponential lag take real, non-bug time to catch up (its target jumped
# ~618 m), so a bound on the raw distance can't tell that apart from the
# bug's own jump without very fragile timing assumptions. A single
# exponential lag converging on a fixed (or slowly-moving, like a driving
# car) target can only ever move the camera CLOSER to it, frame over
# frame, up to sub-metre per-tick noise - it can never make
# camera-to-chassis distance jump back UP by metres in one frame. The
# origin-rebase bug does exactly that (chase_cam.gd's header comment):
# `_smoothed_pos` stays in the OLD frame for one frame, then `position`
# (read from the same, now-stale `_smoothed_pos`) is off by up to the
# rebase's own delta - a discontinuity a smooth lag cannot produce.
var _force_rebase_test: bool = false
var _rebase_test_triggered: bool = false
var _rebase_test_trigger_t: float = 0.0
var _camera_check_max_dist_m: float = 0.0
var _camera_check_max_dist_after_settle_m: float = 0.0
var _camera_check_max_frame_jump_m: float = 0.0
var _camera_check_prev_dist_m: float = -1.0
var _camera_check_frame_count: int = 0
# Rendered-vs-current chassis desync: the chassis PsBodyNode3D's drawn
# position vs. a fresh get_body_transform read later in the same frame.
# The two differ by at most a tick or two of motion, EXCEPT when the
# floating origin moved in between (a node that placed itself before the
# rebase is then off by the whole rebase delta, ~500 m, for one frame -
# the "car bugs back and bounces back" glitch; fixed by the cameras'
# process_priority, see chase_cam.gd's _ready). The teleport frame itself
# is skipped (fresh ABSOLUTE position jumped).
var _camera_check_max_render_desync_m: float = 0.0
var _camera_check_prev_abs: Vector3 = Vector3.INF
# How long to give the chase cam's OWN, legitimate exponential lag
# (chase_cam.gd's position_lag=6.0 -> ~0.17 s time constant) to finish
# catching up to a teleport before this script starts holding the
# post-trigger distance to the tight "parked" bound - informational only,
# see the discontinuity check above for the actual pass/fail gate.
const CAMERA_CHECK_SETTLE_S := 2.5
# Skip this long right after the teleport before tracking frame-to-frame
# jumps: the teleport's OWN first frame legitimately jumps `desired_pos`
# (and therefore, after the lerp, `position` moves toward it) - that one
# expected step must not be counted as the bug's discontinuity.
const CAMERA_CHECK_JUMP_GRACE_S := 0.1

var _screenshot_path: String = ""
var _screenshot_t: float = 0.0
var _screenshot_taken: bool = false
var _screenshot_view: String = "drive"
var _screenshot_delay_s: float = 3.0

# rig.json's wheel_FL pivot (ISO-local, relative to the chassis origin) -
# only used to frame the wheels-close-up screenshot view below, not a sim
# value, so a hardcoded approximation is fine here.
const WHEEL_FL_ISO_OFFSET := Vector3(1.35, 0.78, 0.317)

var _zone_keys_prev: Dictionary = {}

func _repo_root() -> String:
	# demo/ is adapters/godot/demo; the repo root is three levels up.
	return ProjectSettings.globalize_path("res://").path_join("../../..")

func _ready() -> void:
	# Godot processes a parent's _process before its children's (pre-order
	# tree traversal) unless process_priority says otherwise. This node's
	# own _process reads PsInputMap's channels (a child, added below) and
	# must see THIS frame's value, not last frame's - a high priority
	# number forces it to run LAST among this frame's _process calls
	# (lower priority number = earlier; every child below keeps the
	# default priority 0).
	set_process_priority(1000)

	var user_args := OS.get_cmdline_user_args()
	_autopilot = "--m1-autopilot" in user_args
	# See the _force_rebase_test var declaration / _run_camera_rebase_check
	# for what this does; only meaningful combined with --m1-autopilot.
	_force_rebase_test = "--m1-force-rebase-test" in user_args
	# See _haptics_cruise's var declaration above - a separate scripted
	# drive from --m1-autopilot, mutually exclusive with it (the branch in
	# _process below only checks _autopilot first, so pass one or the
	# other, not both, on the command line).
	_haptics_cruise = "--m1-haptics-cruise" in user_args
	# --m1-screenshot=<absolute_path.png>: saves one PNG of the running
	# demo a few seconds in (viewport texture -> Image, no user interaction
	# needed - only works when Godot actually has a rendering context, not
	# under --headless; see this adapter's report).
	for a in user_args:
		if a.begins_with("--m1-screenshot="):
			_screenshot_path = a.substr(len("--m1-screenshot="))
		elif a.begins_with("--m1-screenshot-view="):
			# Framing for the screenshot above: "drive" (default) leaves the
			# normal chase camera running; "side"/"three_quarter"/"wheels"
			# snap the free camera to a canned shot of the parked vehicle,
			# for the M1 visual-pass report's reference screenshots.
			_screenshot_view = a.substr(len("--m1-screenshot-view="))
		elif a.begins_with("--m1-screenshot-delay="):
			# How many simulated seconds to wait before capturing (default
			# 3.0) - a "drive" shot further down the straight/at higher
			# speed needs longer than the default lets the car crawl for.
			_screenshot_delay_s = a.substr(len("--m1-screenshot-delay=")).to_float()
		elif a.begins_with("--m1-spawn="):
			# Overrides the exported spawn_id default (spawn_straight_car) for
			# report/debugging screenshots at another zone (e.g.
			# spawn_skidpad_car, spawn_main) without touching the scene's own
			# default - added for the M1 visual-pass bug report's "screenshot
			# at the skidpad" requirement.
			spawn_id = a.substr(len("--m1-spawn="))
			_spawn_id_explicit = true
		elif a.begins_with("--m1-vehicle="):
			# Picks data/vehicles/<name>.json instead of the exported default -
			# e.g. car_sedan (FWD reference), car_sedan_rwd (RWD reference),
			# tractor (MFWD - see VEHICLE_TABLE above for its chassis/model/
			# spawn resolution just below).
			vehicle_relative_path = "data/vehicles/%s.json" % a.substr(len("--m1-vehicle="))

	# Tractor demo (2026-09-28): resolve the chassis mass/half-extents/
	# z-offset, art model and default spawn for whichever vehicle
	# vehicle_relative_path now names, keyed by its own basename - e.g.
	# "data/vehicles/tractor.json" -> "tractor". An unknown key (a
	# --m1-vehicle= typo, or a vehicle with no VEHICLE_TABLE entry yet)
	# falls back to car_sedan's entry rather than failing outright, so a
	# typo produces a sedan-shaped chassis around whatever vehicle json
	# was actually requested instead of a crash - spawn_vehicle's own
	# error (checked below) is still what reports the real problem.
	var vehicle_key := vehicle_relative_path.get_file().get_basename()
	var vt: Dictionary = VEHICLE_TABLE.get(vehicle_key, VEHICLE_TABLE["car_sedan"])
	_chassis_mass = vt["chassis_mass"]
	_chassis_half_extents = vt["chassis_half_extents"]
	_chassis_z_offset = vt["chassis_z_offset"]
	model_relative_path = vt["model_relative_path"]
	if not _spawn_id_explicit:
		spawn_id = vt["spawn_id"]
	_vehicle_has_hand_throttle = (vehicle_key == "tractor")
	print("ps_godot main.gd: _ready start, autopilot=", _autopilot, " screenshot_path=", _screenshot_path, " screenshot_view=", _screenshot_view, " spawn_id=", spawn_id, " vehicle=", vehicle_relative_path, " vehicle_key=", vehicle_key)
	_setup_lighting()

	simulation = ClassDB.instantiate("PsSimulation")
	simulation.name = "PsSimulation"
	add_child(simulation)

	var repo_root := _repo_root()
	var world_scenario_path := ProjectSettings.globalize_path("res://scenarios/m1_world.json")
	print("ps_godot main.gd: loading empty world from: ", world_scenario_path)
	var ok: bool = simulation.load_scenario(world_scenario_path)
	print("ps_godot main.gd: load_scenario returned: ", ok, " err=", simulation.get_last_error())

	var track_dir := repo_root.path_join(track_relative_dir)
	var surfaces_override := repo_root.path_join(TEST_GRIP_SURFACES_RELATIVE_PATH) if test_grip_everywhere else ""
	var track_ok: bool = simulation.import_test_ground(track_dir, terrain_max_resident_tiles, terrain_max_tile_fills_per_tick, surfaces_override)
	print("ps_godot main.gd: import_test_ground returned: ", track_ok, " err=", simulation.get_last_error())

	var spawn := _read_spawn(track_dir, spawn_id)
	var spawn_pos: Vector3 = spawn["position"]
	var spawn_yaw_rad: float = spawn["yaw_rad"]
	print("ps_godot main.gd: spawn '", spawn_id, "' at ", spawn_pos, " yaw_rad=", spawn_yaw_rad)

	var chassis_pos := spawn_pos + Vector3(0, 0, _chassis_z_offset)
	var chassis_ok: bool = simulation.create_body_dynamic(CHASSIS_BODY_NAME, chassis_pos, spawn_yaw_rad, _chassis_mass, _chassis_half_extents)
	print("ps_godot main.gd: create_body_dynamic returned: ", chassis_ok, " err=", simulation.get_last_error())

	var vehicle_path := repo_root.path_join(vehicle_relative_path)
	var vehicle_ok: bool = simulation.spawn_vehicle(VEHICLE_NAME, vehicle_path, CHASSIS_BODY_NAME, terrain_stream_radius_m)
	print("ps_godot main.gd: spawn_vehicle returned: ", vehicle_ok, " err=", simulation.get_last_error())

	simulation.start()
	print("ps_godot main.gd: sim started, is_running=", simulation.is_running())

	_setup_terrain_visual(repo_root)
	_spawn_body_nodes(repo_root)
	_setup_cameras()
	_apply_screenshot_view(_screenshot_view)
	_setup_debug_draw()
	_setup_input_map()
	_setup_hud()
	_setup_tach_gauge()
	_setup_engine_audio()

func _setup_lighting() -> void:
	# M1 visual pass fix: the demo previously had a DirectionalLight3D but
	# a flat Environment.BG_COLOR background with no ambient term set - the
	# car_sedan.glb materials are proper PBR (baseColorFactor + embedded
	# textures + normals, verified by reading the .glb's own glTF JSON),
	# so the near-black renders (out/renders/m1_final2.png) were a lighting
	# problem, not a material one: with zero ambient, every surface not
	# directly facing the one directional light (most of the car body from
	# a chase-cam angle) had nothing to reflect. Fix: a real sky (gives a
	# physically-plausible ambient/reflection term for free, and the actual
	# background pixels are barely different from the old flat colour) plus
	# a brighter, warmer sun.
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-55, -30, 0)
	light.light_energy = 1.3
	light.light_color = Color(1.0, 0.98, 0.92)
	light.shadow_enabled = false
	add_child(light)

	var sky_material := ProceduralSkyMaterial.new()
	sky_material.sky_top_color = Color(0.35, 0.55, 0.85)
	sky_material.sky_horizon_color = Color(0.75, 0.82, 0.9)
	sky_material.ground_bottom_color = Color(0.3, 0.3, 0.3)
	sky_material.ground_horizon_color = Color(0.55, 0.53, 0.5)
	var sky := Sky.new()
	sky.sky_material = sky_material

	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_SKY
	e.sky = sky
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	e.ambient_light_energy = 1.0
	env.environment = e
	add_child(env)

	# A world-space grid shader (PLAN.md M1: "a world-space grid shader in
	# Godot as the speed reference") - a large plane under the car; the
	# REAL ground is the test-ground collision/terrain loaded into the sim
	# (see import_test_ground above), this is purely a visual distance
	# reference that tiles with world position, not a collider.
	var ground := MeshInstance3D.new()
	ground.name = "GroundGrid"
	var plane := PlaneMesh.new()
	plane.size = Vector2(4000, 4000)
	# 20 m quads instead of one 4 km quad pair: over a single huge triangle
	# the interpolated world position (and depth) loses precision near the
	# camera, and the grid lines visibly swam (owner test drive 2026-09-24).
	plane.subdivide_width = 199
	plane.subdivide_depth = 199
	ground.mesh = plane
	var mat := ShaderMaterial.new()
	mat.shader = load("res://shaders/speed_grid.gdshader")
	ground.material_override = mat
	ground.position = Vector3(0, GRID_Y_OFFSET_M, 0) # re-placed every frame in _process once `simulation` exists
	add_child(ground)
	ground_grid = ground

func _read_spawn(track_dir: String, id: String) -> Dictionary:
	# test_ground.json's own "spawns" array (PLAN.md M1's test ground) -
	# read directly as data (GDScript's built-in JSON parser), no C++
	# needed: this is a one-shot startup lookup, not a per-tick hot path.
	var result := {"position": Vector3.ZERO, "yaw_rad": 0.0}
	var path := track_dir.path_join("test_ground.json")
	if not FileAccess.file_exists(path):
		push_warning("main.gd: test_ground.json not found at " + path + " - defaulting spawn to world origin")
		return result
	var f := FileAccess.open(path, FileAccess.READ)
	var text := f.get_as_text()
	f.close()
	var data = JSON.parse_string(text)
	if data == null or not (data is Dictionary) or not data.has("spawns"):
		push_warning("main.gd: could not parse spawns from " + path)
		return result
	for spawn in data["spawns"]:
		if spawn.get("id", "") == id:
			var p = spawn["position"]
			# Raw sim-space (ISO 8855 / test-ground ENU - algebraically
			# identical per data/tracks/README.md) numbers, NOT Godot-space
			# - see PsSimulation::create_body_dynamic's doc comment.
			result["position"] = Vector3(p[0], p[1], p[2])
			result["yaw_rad"] = deg_to_rad(spawn.get("yaw_deg", 0.0))
			# WORKAROUND, not a fix: some of test_ground.json's own spawn
			# points sit at the very first metre of a LINEAR feature
			# (spawn_straight_car is exactly the straight's x0), so a car
			# placed there has its REAR axle behind the pavement and
			# starts with two wheels on dirt. The proper fix is in the
			# track data itself (trackgen re-anchoring these start-line
			# spawns past a wheelbase's worth of margin) - that lane's task
			# is currently UNOWNED (the session that was going to do it was
			# deleted), so this nudge is load-bearing until someone picks
			# it up.
			#
			# It must NOT apply to every spawn: most of test_ground.json's
			# spawns sit at an AREA's centre (spawn_main, spawn_grass,
			# spawn_dirt, spawn_side_slope), a short plateau
			# (spawn_ramp_*_plateau - a forward nudge can push the car off
			# the far end), or a point where "forward" isn't the axis that
			# needs clearance (spawn_skidpad_* is yaw 90 deg, so nudging
			# along heading just moves it off the intended centreline
			# point). Scoped by the spawn's own "area" field rather than
			# its "id" so it covers a spawn's car and tractor variants alike
			# without naming each one - "straight" and "split_mu" are the
			# areas confirmed (by measurement, commit 0716ecf) to be a
			# start-line spawn on a narrow paved feature where this
			# straddle actually happens.
			if NUDGE_AREAS.has(spawn.get("area", "")):
				var nudge_m := 4.0
				var yaw: float = result["yaw_rad"]
				result["position"] += Vector3(cos(yaw), sin(yaw), 0.0) * nudge_m
			return result
	push_warning("main.gd: spawn id '" + id + "' not found in " + path + " - defaulting to world origin")
	return result

func _setup_terrain_visual(repo_root: String) -> void:
	# M1 visual pass problem 2: without this, the test ground's real
	# surfaces (asphalt/grass/dirt/etc.) were never visible - see
	# terrain_visual.gd's header comment.
	var terrain_visual := Node3D.new()
	terrain_visual.name = "TerrainVisual"
	terrain_visual.set_script(load("res://scripts/terrain_visual.gd"))
	terrain_visual.set("simulation_path", simulation.get_path())
	terrain_visual.set("glb_absolute_path", repo_root.path_join(terrain_model_relative_path))
	add_child(terrain_visual)

func _spawn_body_nodes(repo_root: String) -> void:
	var sim_path := simulation.get_path()

	for body_name in simulation.get_body_names():
		var node = ClassDB.instantiate("PsBodyNode3D")
		node.name = "Body_" + String(body_name)
		node.simulation_path = sim_path
		node.body_name = body_name
		add_child(node)

		if String(body_name) == CHASSIS_BODY_NAME:
			# PLAN.md M1 step 3: show the car with the existing art.
			var visual := Node3D.new()
			visual.name = "VehicleVisual"
			visual.set_script(load("res://scripts/vehicle_visual.gd"))
			visual.set("simulation_path", sim_path)
			visual.set("vehicle_name", VEHICLE_NAME)
			visual.set("model_absolute_path", repo_root.path_join(model_relative_path))
			node.add_child(visual)
		else:
			var mesh := MeshInstance3D.new()
			var box := BoxMesh.new()
			box.size = Vector3(1.0, 1.0, 1.0)
			mesh.mesh = box
			node.add_child(mesh)

	for art_name in simulation.get_articulation_names():
		var link_count: int = simulation.get_articulation_link_count(art_name)
		for link_index in range(-1, link_count):
			var node = ClassDB.instantiate("PsBodyNode3D")
			node.name = "Articulation_%s_link_%d" % [String(art_name), link_index]
			node.simulation_path = sim_path
			node.articulation_name = art_name
			node.articulation_link_index = link_index
			var mesh := MeshInstance3D.new()
			var box := BoxMesh.new()
			box.size = Vector3(0.8, 0.2, 0.2)
			mesh.mesh = box
			node.add_child(mesh)
			add_child(node)

func _setup_cameras() -> void:
	# M1 visual pass: the free camera used to start at a fixed (8,4,8)
	# looking at the WORLD origin - fine for spawn_main (world origin) but
	# left the camera looking at empty terrain 500+ m from the car once the
	# default spawn moved to spawn_straight_car (see spawn_id above). Seed
	# it from the chassis's own Godot transform instead (already computed
	# by the one real ISO->Godot conversion, PsSimulation.get_body_transform)
	# so it always starts near the vehicle, wherever it spawns.
	var chassis_xform: Transform3D = simulation.get_body_transform(CHASSIS_BODY_NAME)
	var chassis_pos := chassis_xform.origin
	var chassis_forward := chassis_xform.basis.x # car's own forward - see chase_cam.gd's comment on why it's .x, not .z

	free_camera = Camera3D.new()
	free_camera.name = "FreeCamera"
	free_camera.transform.origin = chassis_pos - chassis_forward.normalized() * 8.0 + Vector3.UP * 4.0
	free_camera.set_script(load("res://scripts/free_cam.gd"))
	free_camera.set("simulation_path", simulation.get_path())
	add_child(free_camera) # look_at() below needs the node inside the tree first
	free_camera.look_at(chassis_pos + Vector3.UP, Vector3.UP)

	# PLAN.md M1 step 7: a chase camera that works at 50 km from the origin
	# (origin rebasing) and does not jitter - see chase_cam.gd.
	chase_camera = Camera3D.new()
	chase_camera.name = "ChaseCamera"
	chase_camera.set_script(load("res://scripts/chase_cam.gd"))
	chase_camera.set("simulation_path", simulation.get_path())
	chase_camera.set("chassis_body_name", CHASSIS_BODY_NAME)
	add_child(chase_camera)

	using_chase_camera = true
	chase_camera.current = true
	free_camera.current = false

func _apply_screenshot_view(view: String) -> void:
	if view == "" or view == "drive":
		return # default chase-camera framing, already set up by _setup_cameras
	var chassis_xform: Transform3D = simulation.get_body_transform(CHASSIS_BODY_NAME)
	var pos := chassis_xform.origin
	# basis.x/y/z = the chassis's own forward/left/up, expressed in Godot
	# space (frame_convert_core.h's iso_to_godot_basis - see chase_cam.gd's
	# comment for why .z, not .y, is "up").
	var fwd := chassis_xform.basis.x
	var left := chassis_xform.basis.y
	var up := chassis_xform.basis.z
	var cam_pos: Vector3
	var look_at_pos: Vector3
	match view:
		"side":
			cam_pos = pos + left * 6.0 + up * 1.3
			look_at_pos = pos + up * 0.6
		"three_quarter":
			cam_pos = pos - fwd * 4.5 + left * 4.5 + up * 2.3
			look_at_pos = pos + up * 0.5
		"wheels":
			var wheel_pos: Vector3 = pos + chassis_xform.basis * WHEEL_FL_ISO_OFFSET
			cam_pos = wheel_pos + fwd * 1.0 + left * 1.4 + up * 0.5
			look_at_pos = wheel_pos
		_:
			push_warning("main.gd: unknown --m1-screenshot-view '%s' - ignoring" % view)
			return
	free_camera.transform.origin = cam_pos
	free_camera.look_at(look_at_pos, up)
	free_camera.current = true
	chase_camera.current = false
	using_chase_camera = false

func _toggle_camera() -> void:
	using_chase_camera = not using_chase_camera
	chase_camera.current = using_chase_camera
	free_camera.current = not using_chase_camera

func _setup_debug_draw() -> void:
	debug_draw = ClassDB.instantiate("PsDebugDraw")
	debug_draw.name = "DebugDraw"
	debug_draw.simulation_path = simulation.get_path()
	add_child(debug_draw)

func _setup_input_map() -> void:
	input_map = ClassDB.instantiate("PsInputMap")
	input_map.name = "InputMap"
	add_child(input_map)
	# Gamepad rumble (haptics_core.h): lets PsInputMap reach the running
	# vehicle's own telemetry through PsSimulation's race-free getters -
	# same NodePath-export pattern hud.gd already uses for simulation_path.
	input_map.set("simulation_path", simulation.get_path())
	input_map.set("vehicle_name", VEHICLE_NAME)

func _setup_hud() -> void:
	hud = CanvasLayer.new()
	hud.name = "HUD"
	var label := Label.new()
	label.name = "Readout"
	label.position = Vector2(12, 12)
	hud.add_child(label)
	# Script + exported properties must be set BEFORE add_child(): attaching
	# a script to a node already inside the tree fires _ready() right away
	# (Godot 4 behaviour), so setting properties AFTER add_child() would
	# leave hud.gd's _ready() resolving empty paths (found the hard way in
	# the P2 demo - see hud.gd's own history).
	hud.set_script(load("res://scripts/hud.gd"))
	hud.set("simulation_path", simulation.get_path())
	hud.set("input_map_path", NodePath("../InputMap"))
	hud.set("vehicle_name", VEHICLE_NAME)
	add_child(hud)

func _setup_tach_gauge() -> void:
	# PLAN.md P4 "Driver HUD" (owner hand sketch, 2026-09-24): the round
	# tachometer/gear/assist-lamps/speed gauge - a Control drawn on the same
	# HUD CanvasLayer as hud.gd's text readout (_setup_hud above), on top of
	# it (added after, so it draws last / on top). Same NodePath-export
	# wiring pattern as hud.gd/PsInputMap.
	var gauge := Control.new()
	gauge.name = "TachGauge"
	gauge.set_script(load("res://scripts/tach_gauge.gd"))
	gauge.set("simulation_path", simulation.get_path())
	gauge.set("input_map_path", NodePath("../../InputMap")) # gauge's parent is the HUD CanvasLayer, whose own parent is Main (where InputMap lives) - one level deeper than hud.gd's own "../InputMap"
	gauge.set("vehicle_name", VEHICLE_NAME)
	hud.add_child(gauge)

func _setup_engine_audio() -> void:
	# ps_audio A11 ("engine sound in Godot, minimal"): only when the spawned
	# vehicle's own physics engine has a live cycle model does it have a
	# voice at all (has_vehicle_voice - see vehicle_voice.h's try_create for
	# exactly what qualifies: a "kind": "simulated" engine with a "cycle"
	# block and a fuel, e.g. car_sedan_pipes.json; car_sedan.json's
	# torque-map engine and car_sedan_sim.json's maps-only engine do not).
	# No voice -> this function is a no-op, not an error.
	if simulation == null or not simulation.has_vehicle_voice(VEHICLE_NAME):
		print("ps_godot main.gd: no engine voice for vehicle '", VEHICLE_NAME, "' (see PsSimulation's own log line above for why)")
		return

	var sample_rate: float = simulation.get_vehicle_voice_sample_rate_hz(VEHICLE_NAME)
	var channels: int = simulation.get_vehicle_voice_channel_count(VEHICLE_NAME)
	if sample_rate <= 0.0 or channels <= 0:
		return

	var stream := AudioStreamGenerator.new()
	stream.mix_rate = sample_rate
	# Buffer length: comfortably longer than one _process tick at a low
	# frame rate (say 20 fps = 50 ms) so a slow frame does not itself cause
	# an audible gap - the Runner thread keeps rendering independently of
	# Godot's frame rate (audio/include/ps_audio/runner.h), this buffer is
	# only the hand-off cushion between it and AudioStreamGenerator.
	stream.buffer_length = 0.2

	_engine_audio_player = AudioStreamPlayer.new()
	_engine_audio_player.name = "EngineAudio"
	_engine_audio_player.stream = stream
	_engine_audio_player.autoplay = true
	# channels > 1 (multiple radiating sources, ps_audio A8) is mixed down
	# to mono here for this minimal pass - AudioStreamGenerator's playback
	# is itself mono/stereo only, and the brief scope is "minimal", not a
	# full per-source spatial mix.
	add_child(_engine_audio_player)
	_engine_audio_player.play()

func _process(delta: float) -> void:
	if simulation != null and ground_grid != null:
		ground_grid.position = simulation.get_world_origin_godot_position() + Vector3(0, GRID_Y_OFFSET_M, 0)

	if input_map != null and simulation != null:
		if input_map.get_camera():
			_toggle_camera()
		if input_map.get_debug_toggle() and debug_draw != null:
			debug_draw.debug_enabled = not debug_draw.debug_enabled
		if input_map.get_reset_vehicle():
			_reset_vehicle()
		_handle_zone_keys()

		# PLAN.md P2/M1: input reaches the sim only as named control
		# channels, never a Godot type - see PsInputMap/input_map_core.h.
		if not _autopilot:
			simulation.set_control("steer", input_map.get_steer())
			simulation.set_control("throttle", input_map.get_throttle())
			simulation.set_control("brake", input_map.get_brake())
			simulation.set_control("handbrake", input_map.get_handbrake())
			simulation.set_control("clutch", input_map.get_clutch())
			# PHYS-033 P4 I3 (C8) section 12.2: shift_up_count/
			# shift_down_count are monotonic counters, forwarded as-is
			# every frame (the TCU consumes DIFFERENCES between ticks,
			# PLAN.md 6.5.1 - never write gear_request directly here).
			simulation.set_control("shift_up_count", input_map.get_shift_up_count())
			simulation.set_control("shift_down_count", input_map.get_shift_down_count())
			simulation.set_control("ignition", input_map.get_ignition())
			simulation.set_control("starter", input_map.get_starter())
			if input_map.get_toggle_auto_shift():
				simulation.set_control("assist.auto_shift", 1.0 - simulation.get_control("assist.auto_shift"))
			if input_map.get_toggle_auto_clutch():
				simulation.set_control("assist.auto_clutch", 1.0 - simulation.get_control("assist.auto_clutch"))
			if input_map.get_toggle_auto_blip():
				simulation.set_control("assist.auto_blip", 1.0 - simulation.get_control("assist.auto_blip"))
			# Tractor demo (2026-09-28): drivetrain component-channel toggles
			# (PHYS-033 P4 I4 C4/C9/C10/C11 - see core's CLAUDE.md
			# "Drivetrain" section) plus the hand-throttle ramp (C12's
			# governor). Harmless on car_sedan (a vehicle that never
			# declared these channels never reads them back - "Adapter
			# seams" in core's CLAUDE.md).
			if input_map.get_toggle_awd():
				simulation.set_control("awd", 1.0 - simulation.get_control("awd"))
			if input_map.get_toggle_diff_lock():
				simulation.set_control("diff_lock", 1.0 - simulation.get_control("diff_lock"))
			if input_map.get_toggle_pto():
				simulation.set_control("pto", 1.0 - simulation.get_control("pto"))
			if input_map.get_toggle_pto_speed():
				# pto_speed semantics (tractor.json/Pto component): >= 770
				# selects 1000 rpm mode, else 540 - flip between the two
				# literal mode values, not a plain 0/1 toggle.
				simulation.set_control("pto_speed", 540.0 if simulation.get_control("pto_speed") >= 770.0 else 1000.0)
			if input_map.get_toggle_park_lock():
				simulation.set_control("park_lock", 1.0 - simulation.get_control("park_lock"))
			# hand_throttle is a continuous per-frame ramp (unlike the
			# edge-triggered toggles above), so unconditionally sending it
			# to a vehicle that never declared the channel (car_sedan) would
			# spam the "unknown channel" warning every single frame instead
			# of just once - see _vehicle_has_hand_throttle's own comment.
			if _vehicle_has_hand_throttle:
				simulation.set_control("hand_throttle", input_map.get_hand_throttle())

	if _autopilot:
		_run_autopilot(delta)
		if _force_rebase_test:
			_run_camera_rebase_check(delta)
	elif _haptics_cruise:
		_run_haptics_cruise(delta)

	if _screenshot_path != "" and not _screenshot_taken:
		_screenshot_t += delta
		if not _autopilot and _screenshot_view == "drive":
			# PHYS-033 P4 I3 (C8) section 12.4: the drivetrain sits in
			# Neutral until a gear is requested - a single shift_up_count=1
			# (edge-triggered, so setting it repeatedly every frame is
			# harmless) plus throttle is enough for auto-clutch to pull away.
			simulation.set_control("shift_up_count", 1.0)
			simulation.set_control("throttle", 0.5) # so the "drive" screenshot shows the car actually driving, not sitting still
		if _screenshot_t > _screenshot_delay_s:
			_screenshot_taken = true
			var img: Image = get_viewport().get_texture().get_image()
			var err := img.save_png(_screenshot_path)
			print("ps_godot main.gd: screenshot saved to ", _screenshot_path, " err=", err)
			await get_tree().create_timer(0.2).timeout
			get_tree().quit()

	_process_engine_audio()

func _process_engine_audio() -> void:
	# ps_audio A11: drain read_vehicle_voice_audio() into the
	# AudioStreamGenerator's own ring, once per Godot frame - the Voice's
	# Runner thread renders independently (audio/include/ps_audio/
	# runner.h), this only moves already-rendered frames across. No-op
	# when _setup_engine_audio found no voice.
	if _engine_audio_player == null:
		return
	var playback: AudioStreamGeneratorPlayback = _engine_audio_player.get_stream_playback()
	if playback == null:
		return

	var to_fill: int = playback.get_frames_available()
	if to_fill <= 0:
		return
	var channels: int = simulation.get_vehicle_voice_channel_count(VEHICLE_NAME)
	var samples: PackedFloat32Array = simulation.read_vehicle_voice_audio(VEHICLE_NAME, to_fill)
	var frames_got: int = (samples.size() / channels) if channels > 0 else 0
	for i in frames_got:
		# Minimal pass (brief scope): mix every source/channel down to one
		# mono value, duplicated to both AudioStreamGeneratorPlayback
		# output channels (push_frame always takes a stereo Vector2) - a
		# real spatial mix of ps_audio A8's per-source channels is future
		# work, not part of A11.
		var mono: float = 0.0
		for c in channels:
			mono += samples[i * channels + c]
		mono /= float(channels)
		playback.push_frame(Vector2(mono, mono))

	# Two counters: ring empty when Godot asked (the voice fell behind) and
	# Godot's own playback skips (its buffer ran dry - an audible dropout).
	# Print only on a Godot-side skip (an audible dropout). ring_empty alone
	# also counts frames that simply came faster than one 256-frame block
	# (5.3 ms) - measured 451 in a headless smoke run with 0 skips.
	var underruns: int = playback.get_skips()
	if underruns != _engine_audio_underrun_last:
		print("ps_godot main.gd: engine audio ring_empty = ", simulation.get_vehicle_voice_underrun_count(VEHICLE_NAME), ", godot_skips = ", playback.get_skips())
		_engine_audio_underrun_last = underruns

func _reset_vehicle(target_spawn_id: String = spawn_id) -> void:
	var spawn := _read_spawn(_repo_root().path_join(track_relative_dir), target_spawn_id)
	var pos: Vector3 = spawn["position"] + Vector3(0, 0, _chassis_z_offset)
	simulation.reset_vehicle(VEHICLE_NAME, pos, spawn["yaw_rad"])

# M1 visual pass: 1/2/3 teleport straight to the straight/skidpad/ramps
# (ZONE_SPAWNS above) instead of driving several km - manual edge-detection
# since these are demo-only conveniences, not PsInputMap control channels.
func _handle_zone_keys() -> void:
	for key in ZONE_SPAWNS.keys():
		var pressed := Input.is_key_pressed(key)
		if pressed and not _zone_keys_prev.get(key, false):
			_reset_vehicle(ZONE_SPAWNS[key])
		_zone_keys_prev[key] = pressed

func _run_autopilot(delta: float) -> void:
	if _autopilot_done:
		return
	_autopilot_phase_t += delta
	# PHYS-033 P4 I3 (C8) section 12.4: the real engine/clutch/gearbox chain
	# sits in Neutral until something requests a gear (C6 "turns the smoke
	# autopilot red: the car sits in N"). Drive it the same way a human
	# would: the monotonic shift_up_count/shift_down_count channels (NOT
	# gear_request directly - PLAN.md 12.4's own schedule), so this run also
	# exercises the adapter's new counter-based channel path end to end.
	# Auto-clutch is on by default (data/vehicles/car_sedan.json), so a
	# requested gear change alone is enough to get moving.
	var pt: Dictionary = simulation.get_vehicle_powertrain(VEHICLE_NAME)
	var rpm: float = pt.get("rpm", 0.0)

	if _autopilot_phase_t >= 0.2 and _autopilot_shift_up_count < 1.0:
		_autopilot_shift_up_count = 1.0
		simulation.set_control("shift_up_count", _autopilot_shift_up_count)
	if _autopilot_shift_up_count >= 1.0 and _autopilot_shift_up_count < 2.0 \
			and (_autopilot_phase_t >= 2.5 or rpm > 5500.0):
		_autopilot_shift_up_count = 2.0
		simulation.set_control("shift_up_count", _autopilot_shift_up_count)
	if _autopilot_phase_t >= 6.0 and _autopilot_shift_down_count < 1.0:
		_autopilot_shift_down_count = 1.0
		simulation.set_control("shift_down_count", _autopilot_shift_down_count)

	# Scripted throttle/brake/steer sequence (PLAN.md 12.4's own schedule):
	# a short settle after the launch shift, throttle to build speed and
	# shift to 2nd, brake to a near-stop (auto-clutch anti-stall must keep
	# the engine running), downshift back to 1st, then throttle + a steer
	# step.
	if _autopilot_phase_t < 0.5:
		simulation.set_control("throttle", 0.0)
		simulation.set_control("brake", 0.0)
		simulation.set_control("steer", 0.0)
	elif _autopilot_phase_t < 4.0:
		simulation.set_control("throttle", 0.7)
		simulation.set_control("brake", 0.0)
		simulation.set_control("steer", 0.0)
	elif _autopilot_phase_t < 6.0:
		simulation.set_control("throttle", 0.0)
		simulation.set_control("brake", 1.0)
		simulation.set_control("steer", 0.0)
	elif _autopilot_phase_t < 10.0:
		simulation.set_control("throttle", 0.7)
		simulation.set_control("brake", 0.0)
		simulation.set_control("steer", 0.3)
	else:
		_finish_autopilot()
		return

	var gear: int = int(pt.get("gear", 0))
	_autopilot_max_gear = max(_autopilot_max_gear, gear)
	var engine_state: String = String(pt.get("engine_state", ""))
	_autopilot_final_engine_state = engine_state
	if engine_state == "stalled":
		_autopilot_stalled_seen = true


	var speed_mps: float = simulation.get_body_speed_mps(CHASSIS_BODY_NAME)
	if is_nan(speed_mps):
		_autopilot_nan_seen = true
	else:
		_autopilot_distance_m += speed_mps * delta
		_autopilot_max_speed_kmh = max(_autopilot_max_speed_kmh, speed_mps * 3.6)

	var chassis_xform: Transform3D = simulation.get_body_transform(CHASSIS_BODY_NAME)
	var p := chassis_xform.origin
	if is_nan(p.x) or is_nan(p.y) or is_nan(p.z):
		_autopilot_nan_seen = true

	# Diagnostic (cheap, autopilot-only): one driven-wheel telemetry line per
	# second, so a smoke-test log always shows whether the driven wheel is
	# actually touching the ground and producing longitudinal force. Engine-
	# neutral: the driven wheel index is resolved via get_wheel_driven, never
	# assumed to be FL/index 0 (car_sedan.json's drive layout - FWD, RWD, or
	# any future AWD reference variant - is not this adapter's business).
	if _autopilot_diag_wheel_index < 0:
		_autopilot_diag_wheel_index = 0
		var wheel_count: int = simulation.get_vehicle_wheel_count(VEHICLE_NAME)
		for w in range(wheel_count):
			if simulation.get_wheel_driven(VEHICLE_NAME, w):
				_autopilot_diag_wheel_index = w
				break
	var whole_second := int(_autopilot_phase_t)
	if whole_second != _autopilot_last_logged_second:
		_autopilot_last_logged_second = whole_second
		var up: Vector3 = chassis_xform.basis.z
		print("ps_godot M1 autopilot diag: t=%.1f speed_kmh=%.2f up_dot=%.3f gear=%d rpm=%.0f engine=%s driven_wheel[%d]{compression=%.4f omega=%.2f fz=%.1f fx=%.1f surface=%s}" % [
			_autopilot_phase_t, speed_mps * 3.6, up.dot(Vector3.UP), gear, rpm, engine_state,
			_autopilot_diag_wheel_index,
			simulation.get_wheel_compression(VEHICLE_NAME, _autopilot_diag_wheel_index),
			simulation.get_wheel_omega(VEHICLE_NAME, _autopilot_diag_wheel_index),
			simulation.get_wheel_load_n(VEHICLE_NAME, _autopilot_diag_wheel_index),
			simulation.get_wheel_fx(VEHICLE_NAME, _autopilot_diag_wheel_index),
			simulation.get_wheel_surface_name(VEHICLE_NAME, _autopilot_diag_wheel_index)])

func _run_haptics_cruise(delta: float) -> void:
	# See _haptics_cruise's own var-block comment above for the full
	# rationale. Launch (leave Neutral) exactly once via shift_up_count=1
	# (auto-clutch path, same as the ordinary autopilot). "assist.auto_shift"
	# is still set for completeness/forward-compat, but (see
	# HAPTICS_CRUISE_UPSHIFT_RPM's comment) it currently has no effect on
	# this vehicle, so this function also drives every later upshift itself.
	if _haptics_cruise_done:
		return
	if not _haptics_cruise_launched:
		_haptics_cruise_launched = true
		simulation.set_control("assist.auto_shift", 1.0)
		simulation.set_control("shift_up_count", 1.0)
		print("ps_godot M1 haptics cruise: launched, targets_kmh=", HAPTICS_CRUISE_TARGETS_KMH)

	_haptics_cruise_phase_t += delta
	if _haptics_cruise_phase >= HAPTICS_CRUISE_TARGETS_KMH.size():
		_finish_haptics_cruise()
		return

	var target_kmh: float = HAPTICS_CRUISE_TARGETS_KMH[_haptics_cruise_phase]
	var target_mps: float = target_kmh / 3.6
	var speed_mps: float = simulation.get_body_speed_mps(CHASSIS_BODY_NAME)
	var error_mps: float = target_mps - speed_mps

	var pt: Dictionary = simulation.get_vehicle_powertrain(VEHICLE_NAME)
	var gear: int = int(pt.get("gear", 0))
	var rpm: float = pt.get("rpm", 0.0)

	# Manual upshift emulation (see HAPTICS_CRUISE_UPSHIFT_RPM's comment for
	# why this script drives it instead of the assist). Cooldown-gated so a
	# shift already in flight (gearbox phase Selecting/Synchronising) isn't
	# re-triggered every frame while rpm is still above threshold.
	_haptics_cruise_shift_cooldown_t = max(0.0, _haptics_cruise_shift_cooldown_t - delta)
	if gear >= 1 and rpm > HAPTICS_CRUISE_UPSHIFT_RPM and _haptics_cruise_shift_cooldown_t <= 0.0:
		_haptics_cruise_shift_up_count += 1.0
		simulation.set_control("shift_up_count", _haptics_cruise_shift_up_count)
		_haptics_cruise_shift_cooldown_t = HAPTICS_CRUISE_SHIFT_COOLDOWN_S

	var throttle: float
	if _haptics_cruise_phase == 0 and gear == 0 and _haptics_cruise_phase_t < HAPTICS_CRUISE_LAUNCH_GRACE_S:
		# See HAPTICS_CRUISE_LAUNCH_GRACE_S's own comment: let the launch
		# shift land before applying any throttle.
		throttle = 0.0
	else:
		# Simple proportional controller with a small baseline (asphalt rolling
		# resistance/drag alone would otherwise let speed decay slightly under
		# pure P control right at the target) - not trying to be a precise
		# cruise control, just steady and settled enough to measure a real
		# "part throttle, steady speed" telemetry window. No braking: letting
		# off throttle is enough to bleed off the modest overshoot a plain P
		# controller produces at these accelerations.
		throttle = clamp(0.12 + error_mps * 0.10, 0.0, 1.0)
	simulation.set_control("throttle", throttle)
	simulation.set_control("brake", 0.0)
	simulation.set_control("steer", 0.0)

	if abs(error_mps) <= HAPTICS_CRUISE_TOLERANCE_MPS:
		_haptics_cruise_hold_t += delta
	else:
		_haptics_cruise_hold_t = 0.0

	var settled: bool = _haptics_cruise_hold_t >= HAPTICS_CRUISE_HOLD_S
	var timed_out: bool = _haptics_cruise_phase_t >= HAPTICS_CRUISE_PHASE_TIMEOUT_S
	if settled or timed_out:
		# Re-fetch: gear/rpm may have advanced (auto_shift upshifting) since
		# the throttle calc above.
		var pt_now: Dictionary = simulation.get_vehicle_powertrain(VEHICLE_NAME)
		var hd: Dictionary = input_map.get_haptics_debug() if input_map != null else {}
		var fl_surface: String = simulation.get_wheel_surface_name(VEHICLE_NAME, 0)
		print("ps_godot M1 haptics cruise: phase=%d target_kmh=%.0f speed_kmh=%.2f settled=%s gear=%d rpm=%.0f throttle=%.2f FL_surface=%s src3_load=%.4f src4_slip=%.4f src7_surf=%.4f strong_out=%.4f weak_out=%.4f" % [
			_haptics_cruise_phase, target_kmh, speed_mps * 3.6, settled, int(pt_now.get("gear", 0)), pt_now.get("rpm", 0.0), throttle, fl_surface,
			hd.get("source3_load_noise", 0.0), hd.get("source4_light_slip", 0.0), hd.get("source7_surface_texture", 0.0),
			hd.get("strong_out", 0.0), hd.get("weak_out", 0.0)])
		_haptics_cruise_phase += 1
		_haptics_cruise_phase_t = 0.0
		_haptics_cruise_hold_t = 0.0

func _finish_haptics_cruise() -> void:
	_haptics_cruise_done = true
	print("ps_godot M1 haptics cruise: done")
	await get_tree().create_timer(0.2).timeout
	get_tree().quit()

func _run_camera_rebase_check(_delta: float) -> void:
	# Verification for the chase-cam origin-rebase bug (owner report: "the
	# camera suddenly swings around as if the car had teleported" -
	# chase_cam.gd's header comment has the full diagnosis). The P3
	# placeholder drive covers only a few metres over the autopilot's ~10 s
	# scripted sequence (see _finish_autopilot's own distance numbers) -
	# nowhere near the 500 m OriginRebase threshold a real drive would
	# eventually cross.
	#
	# IMPORTANT: this must trigger the origin snap THROUGH chase_cam.gd's
	# OWN per-frame rebase_focus(position) call, not via some unrelated
	# external call - an earlier version of this check called
	# PsSimulation.rebase_focus directly with an arbitrary far-away point,
	# which does move the origin, but chase_cam.gd's fix only ever reacts
	# to a delta produced by ITS OWN call (matching the real bug: the
	# origin always moves because the ACTIVE chase camera's own tracked
	# position finally crossed the threshold, not because of some
	# unrelated third party). That external-call version produced the
	# EXACT SAME huge max distance whether or not the fix was applied -
	# the very first post-rebase frame is unavoidably wrong in both cases
	# (chase_cam hasn't run its own reactive code yet), which is not a
	# useful regression signal. So instead: teleport the CHASSIS itself,
	# with reset_vehicle (the same call keys 1/2/3 already use), from
	# spawn_straight_car to spawn_skidpad_car - test_ground.json's own
	# coordinates put those ~618 m apart, comfortably over the 500 m
	# threshold. desired_pos then jumps by that same ~618 m, the chase
	# cam's own exponential lag chases it (legitimate, expected - NOT the
	# bug), and SOMEWHERE during that chase _smoothed_pos/`position`
	# itself crosses 500 m from the pre-teleport origin, triggering
	# chase_cam's OWN rebase_focus call - the actual real-world trigger
	# path, exercising the actual fix.
	if chase_camera == null or simulation == null:
		return
	if not _rebase_test_triggered and _autopilot_phase_t > 1.0:
		_rebase_test_triggered = true
		_rebase_test_trigger_t = _autopilot_phase_t
		_reset_vehicle("spawn_skidpad_car")
		print("ps_godot M1 camera check: teleported chassis to spawn_skidpad_car at t=%.2f (forces the chase cam's own rebase_focus call to cross threshold)" % _autopilot_phase_t)

	_camera_check_frame_count += 1
	if _camera_check_frame_count <= 1:
		return # first observed frame - chase_cam.gd's own _initialized snap has no lag to measure yet

	var chassis_xform: Transform3D = simulation.get_body_transform(CHASSIS_BODY_NAME)
	var dist: float = chassis_xform.origin.distance_to(chase_camera.global_position)
	var chassis_node := get_node_or_null("Body_" + CHASSIS_BODY_NAME) as Node3D
	var fresh_abs: Vector3 = chassis_xform.origin - simulation.get_world_origin_godot_position()
	var teleported: bool = _camera_check_prev_abs != Vector3.INF and fresh_abs.distance_to(_camera_check_prev_abs) > 5.0
	_camera_check_prev_abs = fresh_abs
	if chassis_node != null and not teleported:
		_camera_check_max_render_desync_m = max(_camera_check_max_render_desync_m,
				chassis_node.global_position.distance_to(chassis_xform.origin))
	_camera_check_max_dist_m = max(_camera_check_max_dist_m, dist)

	# Informational only (see the discontinuity check below for the actual
	# gate): the legitimate lag needs real time to converge regardless of
	# the fix, so this is just "how far off is it once that's had time to
	# finish" - useful context in the printed summary, not proof either way
	# on its own (a ~600 m teleport's own catch-up and the bug's own decay
	# share close to the same ~0.17 s time constant, so a plain
	# after-a-delay bound cannot cleanly tell them apart - see file header).
	if _rebase_test_triggered and _autopilot_phase_t > _rebase_test_trigger_t + CAMERA_CHECK_SETTLE_S:
		_camera_check_max_dist_after_settle_m = max(_camera_check_max_dist_after_settle_m, dist)

	# PRIMARY signal: a frame-to-frame INCREASE in camera-to-chassis
	# distance. A single exponential lag converging on a fixed (or
	# slowly-moving) target only ever shrinks this distance, tick over
	# tick, up to sub-metre noise - it cannot produce a multi-metre jump.
	# The origin-rebase bug can and does (see the var declarations' own
	# comment). Skipped for CAMERA_CHECK_JUMP_GRACE_S right after the
	# teleport, which legitimately moves `desired_pos` in one step.
	if _rebase_test_triggered and _autopilot_phase_t > _rebase_test_trigger_t + CAMERA_CHECK_JUMP_GRACE_S:
		if _camera_check_prev_dist_m >= 0.0:
			var jump: float = dist - _camera_check_prev_dist_m
			_camera_check_max_frame_jump_m = max(_camera_check_max_frame_jump_m, jump)
		_camera_check_prev_dist_m = dist

func _finish_autopilot() -> void:
	_autopilot_done = true
	var chassis_xform: Transform3D = simulation.get_body_transform(CHASSIS_BODY_NAME)
	# chassis_xform.basis's column i is the SIM's own local axis i (ISO
	# 8855: x forward, y left, z UP) expressed in Godot space (frame_
	# convert_core.h's iso_to_godot_basis) - so the car's own "up" is
	# basis.z, not basis.y (see chase_cam.gd's fix comment for the same
	# convention, found via the same bug there).
	var up: Vector3 = chassis_xform.basis.z
	var upright: bool = up.dot(Vector3.UP) > 0.5

	# PHYS-033 P4 I3 (C8) section 12.4: new fields appended, old ones keep
	# their meaning so smoke_test.ps1's existing regex still matches the
	# common prefix - max_gear/stalled/engine_state are the new drivetrain
	# gates (the car must reach 2nd, never stall, and still be running at
	# the end of the scripted sequence).
	print("ps_godot M1 smoke: distance_m=%.2f max_speed_kmh=%.2f upright=%s nan=%s max_gear=%d stalled=%s engine_state=%s" % [
		_autopilot_distance_m, _autopilot_max_speed_kmh, upright, _autopilot_nan_seen,
		_autopilot_max_gear, _autopilot_stalled_seen, _autopilot_final_engine_state])

	if _force_rebase_test:
		# PRIMARY gate: max_frame_jump_m (see _run_camera_rebase_check's
		# comment) - a single-target exponential lag can only ever shrink
		# camera-to-chassis distance tick over tick (up to sub-metre
		# desired_pos drift from the chassis's own ongoing driving), so a
		# multi-metre one-frame INCREASE is specifically the origin-rebase
		# discontinuity, not ordinary catch-up lag. jump_bound_m is
		# generous versus expected per-tick drift and far below the
		# hundreds-of-metres jump the unfixed bug produces.
		#
		# The other two numbers are informational context only (both
		# legitimately large right after any big teleport, fixed or not -
		# see their own var declarations' comments).
		var jump_bound_m := 3.0
		var desync_bound_m := 3.0
		var camera_ok: bool = _rebase_test_triggered and _camera_check_max_frame_jump_m <= jump_bound_m and _camera_check_max_render_desync_m <= desync_bound_m
		print("ps_godot M1 camera check: rebase_forced=%s max_frame_jump_m=%.2f jump_bound_m=%.2f max_cam_chassis_dist_m=%.2f max_cam_chassis_dist_after_settle_m=%.2f pass=%s max_render_desync_m=%.2f desync_bound_m=%.2f" % [
			_rebase_test_triggered, _camera_check_max_frame_jump_m, jump_bound_m, _camera_check_max_dist_m, _camera_check_max_dist_after_settle_m, camera_ok, _camera_check_max_render_desync_m, desync_bound_m])

	await get_tree().create_timer(0.2).timeout
	get_tree().quit()
