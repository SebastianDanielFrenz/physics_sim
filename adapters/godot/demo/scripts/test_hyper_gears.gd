extends SceneTree
func _initialize(): call_deferred("run")
func run():
    var main = load("res://scenes/main.tscn").instantiate()
    root.add_child(main)
    await process_frame
    main.set_process(false)
    var sim = main.simulation
    sim.set_control("brake", 1.0)
    await create_timer(0.5).timeout
    sim.set_control("shift_down_count", 1.0)
    await create_timer(1.5).timeout
    var pt = sim.get_vehicle_powertrain("car")
    print("HYPER_REVERSE gear=",pt.get("gear")," phase=",pt.get("shift_phase"))
    var reverse_ok = int(pt.get("gear",0)) == -1
    sim.set_control("gear_request", 0.0)
    await create_timer(0.5).timeout
    sim.set_control("park_lock", 1.0)
    await create_timer(0.2).timeout
    pt = sim.get_vehicle_powertrain("car")
    var park_ok = bool(pt.get("park_lock_engaged",false))
    print("HYPER_PARK engaged=",park_ok)
    sim.set_control("park_lock", 0.0)
    await create_timer(0.1).timeout
    var released = not bool(sim.get_vehicle_powertrain("car").get("park_lock_engaged",true))
    print("HYPER_PARK released=",released)
    quit(0 if reverse_ok and park_ok and released else 1)
