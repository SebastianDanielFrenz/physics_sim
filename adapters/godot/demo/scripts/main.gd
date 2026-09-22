extends Node3D
# adapters/godot/demo/scripts/main.gd — PLAN.md P2 demo project entry point.
#
# Builds the scene procedurally (lighting, ground plane with the
# world-space speed grid, PsSimulation, free camera, HUD, debug draw, input
# map) rather than hand-authoring a large .tscn, so the demo has one
# obvious place to change which scenario/track it loads. Loads a P1
# scenario (any file under data/scenarios/) to satisfy PLAN.md P2's "P1
# scenarios play back in Godot" acceptance criterion, and spawns one
# PsBodyNode3D per body and one per articulation link generically (no
# hardcoded body/articulation names) via PsSimulation's
# get_body_names()/get_articulation_names()/get_articulation_link_count().

@export var scenario_relative_path: String = "data/scenarios/articulation_chain.json"
@export var load_test_ground: bool = true

var simulation: Node
var hud: CanvasLayer

func _repo_root() -> String:
	# demo/ is adapters/godot/demo; the repo root is three levels up.
	return ProjectSettings.globalize_path("res://").path_join("../../..")

func _ready() -> void:
	print("ps_godot main.gd: _ready start")
	_setup_lighting_and_ground()

	simulation = ClassDB.instantiate("PsSimulation")
	simulation.name = "PsSimulation"
	add_child(simulation)

	var scenario_path := _repo_root().path_join(scenario_relative_path)
	print("ps_godot main.gd: loading scenario from: ", scenario_path)
	var ok: bool = simulation.load_scenario(scenario_path)
	print("ps_godot main.gd: load_scenario returned: ", ok)
	if not ok:
		push_error("PsSimulation.load_scenario failed for: " + scenario_path)
	else:
		if load_test_ground:
			var track_dir := _repo_root().path_join("data/tracks/test_ground")
			simulation.import_test_ground(track_dir)
		simulation.start()
		print("ps_godot main.gd: sim started, is_running=", simulation.is_running())
		_spawn_body_nodes()

	_setup_camera()
	_setup_debug_draw()
	_setup_input_map()
	_setup_hud()

func _setup_lighting_and_ground() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-55, -30, 0)
	light.shadow_enabled = false
	add_child(light)

	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.55, 0.65, 0.8)
	env.environment = e
	add_child(env)

	var ground := MeshInstance3D.new()
	ground.name = "GroundGrid"
	var plane := PlaneMesh.new()
	plane.size = Vector2(2000, 2000)
	ground.mesh = plane
	var mat := ShaderMaterial.new()
	mat.shader = load("res://shaders/speed_grid.gdshader")
	ground.material_override = mat
	add_child(ground)

func _spawn_body_nodes() -> void:
	var sim_path := simulation.get_path()

	for body_name in simulation.get_body_names():
		var node = ClassDB.instantiate("PsBodyNode3D")
		node.name = "Body_" + String(body_name)
		node.simulation_path = sim_path
		node.body_name = body_name
		var mesh := MeshInstance3D.new()
		var sphere := SphereMesh.new()
		sphere.radius = 0.5
		sphere.height = 1.0
		mesh.mesh = sphere
		node.add_child(mesh)
		add_child(node)

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

func _setup_camera() -> void:
	var cam := Camera3D.new()
	cam.name = "FreeCamera"
	cam.transform.origin = Vector3(8, 4, 8)
	cam.current = true
	cam.set_script(load("res://scripts/free_cam.gd"))
	cam.set("simulation_path", simulation.get_path())
	add_child(cam) # look_at() below needs the node inside the tree first
	cam.look_at(Vector3.ZERO, Vector3.UP)

func _setup_debug_draw() -> void:
	var draw = ClassDB.instantiate("PsDebugDraw")
	draw.name = "DebugDraw"
	draw.simulation_path = simulation.get_path()
	add_child(draw)

func _setup_input_map() -> void:
	var input_map = ClassDB.instantiate("PsInputMap")
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
	# (Godot 4 behaviour), so setting simulation_path/input_map_path AFTER
	# add_child() left hud.gd's _ready() resolving an empty NodePath and the
	# HUD silently never finding PsSimulation - found by actually running
	# the headless smoke test, not assumed.
	hud.set_script(load("res://scripts/hud.gd"))
	hud.set("simulation_path", simulation.get_path())
	hud.set("input_map_path", NodePath("../InputMap"))
	add_child(hud)
