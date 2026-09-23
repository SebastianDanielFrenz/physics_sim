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
# Mirrors data/scenarios/vehicle_step_steer.json's own "chassis" body specs
# (already validated by every P3 vehicle acceptance test, out/
# handover_p3.md) rather than inventing new numbers - sized for
# car_sedan.json's own wheel geometry (wheelbase 2.70 m, track 1.56 m,
# wheel attachment z = -0.343 relative to chassis origin).
const CHASSIS_MASS := 1500.0
const CHASSIS_HALF_EXTENTS := Vector3(2.0, 0.4, 0.15)
const CHASSIS_Z_OFFSET := 0.6

@export var vehicle_relative_path: String = "data/vehicles/car_sedan.json"
@export var track_relative_dir: String = "data/tracks/test_ground"
@export var model_relative_path: String = "data/models/car_sedan/car_sedan.glb"
@export var terrain_model_relative_path: String = "data/tracks/test_ground/test_ground.glb"
@export var spawn_id: String = "spawn_straight_car" # M1 visual pass: spawn on the paved asphalt straight (data/tracks/test_ground/test_ground.json), not spawn_main's open dirt disc - the dirt's low traction made the very first drive feel sluggish. The FL/FR/RL/RR wheel_surface_name asphalt<->dirt flicker once reported here was PHYS-021 (terrain/static-feature coincident-collision, see core's CLAUDE.md), fixed in core by 46275a8: a headless measurement under this demo's own load order, config and interest-point timing confirms FL/FR report "asphalt" with ZERO transitions over 1440 ticks, hitting the asphalt mesh body every tick from tick 0, terrain tiles resident by tick 5 (~21 ms). PHYS-021's own residual - a sub-quad-width sliver right at a feature's true edge, up to ~4 m at this resolution (CLAUDE.md) - can still show a brief flicker if a wheel sits exactly on that edge, so don't read a flicker here as this issue coming back; check whether the wheel is that close to the straight's edge first.
@export var terrain_stream_radius_m: float = 700.0
@export var terrain_max_resident_tiles: int = 512
@export var terrain_max_tile_fills_per_tick: int = 16

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
	print("ps_godot main.gd: _ready start, autopilot=", _autopilot, " screenshot_path=", _screenshot_path, " screenshot_view=", _screenshot_view, " spawn_id=", spawn_id)
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
	var track_ok: bool = simulation.import_test_ground(track_dir, terrain_max_resident_tiles, terrain_max_tile_fills_per_tick)
	print("ps_godot main.gd: import_test_ground returned: ", track_ok, " err=", simulation.get_last_error())

	var spawn := _read_spawn(track_dir, spawn_id)
	var spawn_pos: Vector3 = spawn["position"]
	var spawn_yaw_rad: float = spawn["yaw_rad"]
	print("ps_godot main.gd: spawn '", spawn_id, "' at ", spawn_pos, " yaw_rad=", spawn_yaw_rad)

	var chassis_pos := spawn_pos + Vector3(0, 0, CHASSIS_Z_OFFSET)
	var chassis_ok: bool = simulation.create_body_dynamic(CHASSIS_BODY_NAME, chassis_pos, spawn_yaw_rad, CHASSIS_MASS, CHASSIS_HALF_EXTENTS)
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

	if _autopilot:
		_run_autopilot(delta)
		if _force_rebase_test:
			_run_camera_rebase_check(delta)

	if _screenshot_path != "" and not _screenshot_taken:
		_screenshot_t += delta
		if not _autopilot and _screenshot_view == "drive":
			simulation.set_control("throttle", 0.5) # so the "drive" screenshot shows the car actually driving, not sitting still
		if _screenshot_t > _screenshot_delay_s:
			_screenshot_taken = true
			var img: Image = get_viewport().get_texture().get_image()
			var err := img.save_png(_screenshot_path)
			print("ps_godot main.gd: screenshot saved to ", _screenshot_path, " err=", err)
			await get_tree().create_timer(0.2).timeout
			get_tree().quit()

func _reset_vehicle(target_spawn_id: String = spawn_id) -> void:
	var spawn := _read_spawn(_repo_root().path_join(track_relative_dir), target_spawn_id)
	var pos: Vector3 = spawn["position"] + Vector3(0, 0, CHASSIS_Z_OFFSET)
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

	# Scripted control sequence (this task's smoke-test spec): throttle for
	# a few seconds, then brake, then a steer step.
	if _autopilot_phase_t < 4.0:
		simulation.set_control("throttle", 1.0)
		simulation.set_control("brake", 0.0)
		simulation.set_control("steer", 0.0)
	elif _autopilot_phase_t < 6.0:
		simulation.set_control("throttle", 0.0)
		simulation.set_control("brake", 1.0)
		simulation.set_control("steer", 0.0)
	elif _autopilot_phase_t < 10.0:
		simulation.set_control("throttle", 1.0)
		simulation.set_control("brake", 0.0)
		simulation.set_control("steer", 0.3)
	else:
		_finish_autopilot()
		return

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

	# Diagnostic (cheap, autopilot-only): one FL-wheel telemetry line per
	# second, so a smoke-test log always shows whether the driven wheel is
	# actually touching the ground and producing longitudinal force.
	var whole_second := int(_autopilot_phase_t)
	if whole_second != _autopilot_last_logged_second:
		_autopilot_last_logged_second = whole_second
		var up: Vector3 = chassis_xform.basis.z
		print("ps_godot M1 autopilot diag: t=%.1f speed_kmh=%.2f up_dot=%.3f FL{compression=%.4f omega=%.2f fz=%.1f fx=%.1f surface=%s}" % [
			_autopilot_phase_t, speed_mps * 3.6, up.dot(Vector3.UP),
			simulation.get_wheel_compression("car", 0), simulation.get_wheel_omega("car", 0),
			simulation.get_wheel_load_n("car", 0), simulation.get_wheel_fx("car", 0),
			simulation.get_wheel_surface_name("car", 0)])

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

	print("ps_godot M1 smoke: distance_m=%.2f max_speed_kmh=%.2f upright=%s nan=%s" % [
		_autopilot_distance_m, _autopilot_max_speed_kmh, upright, _autopilot_nan_seen])

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
		var camera_ok: bool = _rebase_test_triggered and _camera_check_max_frame_jump_m <= jump_bound_m
		print("ps_godot M1 camera check: rebase_forced=%s max_frame_jump_m=%.2f jump_bound_m=%.2f max_cam_chassis_dist_m=%.2f max_cam_chassis_dist_after_settle_m=%.2f pass=%s" % [
			_rebase_test_triggered, _camera_check_max_frame_jump_m, jump_bound_m, _camera_check_max_dist_m, _camera_check_max_dist_after_settle_m, camera_ok])

	await get_tree().create_timer(0.2).timeout
	get_tree().quit()
