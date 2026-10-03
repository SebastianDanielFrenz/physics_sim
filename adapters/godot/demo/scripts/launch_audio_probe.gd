extends SceneTree
func _initialize() -> void:
	call_deferred("setup")
func setup() -> void:
	var lock_record := Node.new()
	lock_record.set_script(load("res://scripts/runtime_lock_record.gd"))
	root.add_child(lock_record)
	lock_record.begin("spatial audio probe")
	root.title="Spatial audio probe — live position and height"
	root.size=Vector2i(760,940)
	root.content_scale_size=Vector2i.ZERO
	root.content_scale_mode=Window.CONTENT_SCALE_MODE_DISABLED
	root.min_size=Vector2i(720,920)
	var probe := Control.new()
	probe.set_script(load("res://scripts/spatial_audio_probe.gd"))
	root.add_child(probe)
	if "--probe-screenshot" in OS.get_cmdline_user_args():
		call_deferred("capture")
func capture() -> void:
	await create_timer(1.0).timeout
	var image := root.get_texture().get_image()
	image.save_png("C:/Users/sebas/Documents/ChatGPT/physics_engine/audio_probe.png")
	quit()
