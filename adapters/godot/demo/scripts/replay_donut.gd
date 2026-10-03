extends SceneTree
func _initialize() -> void:
	call_deferred("run")
func run() -> void:
	var source := FileAccess.open("S:/claude_code/physics_sim/out/telemetry/hyper_1790882812_393.csv",FileAccess.READ)
	var headers := source.get_csv_line()
	var rows: Array[PackedStringArray] = []
	while not source.eof_reached():
		var row := source.get_csv_line()
		if row.size() == headers.size(): rows.append(row)
	var demo := (load("res://scenes/main.tscn") as PackedScene).instantiate()
	root.add_child(demo)
	demo.set_process(false)
	var sim: Node = demo.get_node("PsSimulation")
	var started := Time.get_ticks_usec()
	var index := 0
	var shifted := false
	while index < rows.size()-1:
		await process_frame
		var t := (Time.get_ticks_usec()-started)/1000000.0
		while index < rows.size()-1 and float(rows[index+1][0])<=t: index+=1
		var row := rows[index]
		for control in ["throttle","brake","steer"]: sim.set_control(control,float(row[headers.find(control)]))
		if int(row[headers.find("gear")]) > 0 and not shifted:
			sim.set_control("shift_up_count",1.0); shifted=true
	print("REPLAY COMPLETE")
	quit()
