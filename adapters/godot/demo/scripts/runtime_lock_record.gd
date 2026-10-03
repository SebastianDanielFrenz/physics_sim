extends Node
# Coordination metadata only: never authorisation to stop a process.
var path := ""
var record: Dictionary
var since_heartbeat := 0.0
func begin(label: String) -> void:
	process_mode=Node.PROCESS_MODE_ALWAYS
	var repo := ProjectSettings.globalize_path("res://").path_join("../../..").simplify_path()
	var folder := repo.path_join("out/runtime_locks")
	if DirAccess.make_dir_recursive_absolute(folder) != OK:
		push_warning("Cannot create runtime lock metadata directory")
		return
	var session := OS.get_environment("CODEX_THREAD_ID")
	var model := OS.get_environment("PS_AGENT_MODEL")
	if model.is_empty(): model=OS.get_environment("CODEX_MODEL")
	var now := Time.get_unix_time_from_system()
	var expected_seconds: Variant = null
	var expected_reason := "Interactive session; close time unknown"
	var vehicle := ""
	for argument in OS.get_cmdline_user_args():
		if argument.begins_with("--runtime-expected-seconds="):
			var value := argument.trim_prefix("--runtime-expected-seconds=")
			if value.is_valid_float() and float(value)>0:
				expected_seconds=float(value)
				expected_reason="Caller estimate; not a shutdown deadline"
		elif argument.begins_with("--runtime-agent-model="):
			model=argument.trim_prefix("--runtime-agent-model=")
		elif argument.begins_with("--m1-vehicle="):
			vehicle=argument.trim_prefix("--m1-vehicle=")
	var expected_unix: Variant = now+expected_seconds if expected_seconds != null else null
	record={"schema_version":1,"pid":OS.get_process_id(),"executable":OS.get_executable_path(),
		"session_label":label,"owner_kind":"agent" if not session.is_empty() else "user_or_unknown",
		"agent_session_id":session if not session.is_empty() else null,"agent_model":model if not model.is_empty() else null,
		"started_at_unix":now,"started_at_utc":Time.get_datetime_string_from_unix_time(int(now))+"Z",
		"expected_close_at_unix":expected_unix,"expected_close_reason":expected_reason,
		"locked_resource":repo.path_join("adapters/godot/demo/bin/libps_godot.dll"),"vehicle":vehicle,
		"note":"Check PID and heartbeat; metadata does not authorise terminating this process."}
	path=folder.path_join("godot_"+str(OS.get_process_id())+".json")
	_write()
	print("ps_godot runtime lock record: ",path)
func _write() -> void:
	if path.is_empty(): return
	record["heartbeat_at_unix"]=Time.get_unix_time_from_system()
	record["heartbeat_at_utc"]=Time.get_datetime_string_from_unix_time(int(record["heartbeat_at_unix"]))+"Z"
	var file := FileAccess.open(path+".tmp",FileAccess.WRITE)
	if file == null: return
	file.store_string(JSON.stringify(record,"\t"))
	file.close()
	var error := DirAccess.rename_absolute(path+".tmp",path)
	if error != OK: push_warning("Cannot update runtime lock record: " + str(error))
func _process(delta: float) -> void:
	since_heartbeat+=delta
	if since_heartbeat>=5:
		since_heartbeat=0; _write()
func _exit_tree() -> void:
	if not path.is_empty():
		DirAccess.remove_absolute(path)
		if FileAccess.file_exists(path+".tmp"): DirAccess.remove_absolute(path+".tmp")
