extends SceneTree
func _initialize() -> void:
	call_deferred("run_check")
func run_check() -> void:
	var scene := load("res://scenes/main.tscn") as PackedScene
	if scene == null:
		quit(1)
		return
	var demo := scene.instantiate()
	root.add_child(demo)
	await create_timer(0.5).timeout
	var sim: Node = demo.get_node("PsSimulation")
	var info: Dictionary = sim.get_vehicle_gauge_info("car")
	var initial: Dictionary = sim.get_vehicle_powertrain("car")
	var start_fuel: float = initial.get("fuel_mass_kg",0.0)
	# Stop the demo's human-input loop from overwriting scripted controls.
	demo.set_process(false)
	sim.set_control("shift_up_count",1.0)
	sim.set_control("throttle",0.55)
	var peak_flow := 0.0
	for i in range(120):
		await create_timer(0.1).timeout
		var tick: Dictionary = sim.get_vehicle_powertrain("car")
		peak_flow = maxf(peak_flow,tick.get("fuel_flow_g_s",0.0))
	var final: Dictionary = sim.get_vehicle_powertrain("car")
	var end_fuel: float = final.get("fuel_mass_kg",0.0)
	var flow: float = final.get("fuel_flow_g_s",0.0)
	var cluster: Node = demo.get_node("Body_car_chassis/VehicleVisual").find_child("LiveInstruments",true,false)
	var display: Node = cluster.get_child(0).get_child(0)
	var distance: float = display.get("distance")
	var density: float = info.get("fuel_density_kg_m3",0.0)
	var average: float = maxf(start_fuel-end_fuel,0.0)*1000.0/density*100000.0/distance
	var good: bool = info.get("fuel_capacity_kg",0.0)>0.0 and density>0.0 and start_fuel>end_fuel and peak_flow>0.0 and distance>100.0 and is_finite(average) and average>0.0
	print("DASHBOARD_CHECK fuel_start_kg=%.6f fuel_end_kg=%.6f flow_g_s=%.3f distance_m=%.2f average_l_100km=%.2f pass=%s" % [start_fuel,end_fuel,flow,distance,average,good])
	quit(0 if good else 1)
