extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var node := Node.new()
	node.set_script(load("res://scripts/runtime_lock_record.gd"))
	root.add_child(node)
	node.begin("runtime lock verification")
	var path: String = node.path
	assert(FileAccess.file_exists(path))
	var first: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path))
	assert(first.pid==OS.get_process_id())
	assert(first.agent_model=="test-model")
	assert(first.expected_close_at_unix>first.started_at_unix)
	await create_timer(5.3).timeout
	var second: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path))
	assert(second.heartbeat_at_unix>first.heartbeat_at_unix)
	node.queue_free()
	await process_frame
	assert(not FileAccess.file_exists(path))
	print("RUNTIME_LOCK_CHECK creation, metadata, heartbeat and cleanup PASS")
	quit()
