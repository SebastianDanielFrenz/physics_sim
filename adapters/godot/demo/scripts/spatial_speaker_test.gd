extends Node
# Real Windows dynamic-object routing test, with reflections disabled.
var spatial: PsSpatialAudio
var frame := 0
var announced := -1
const RATE := 48000
const SECONDS_PER_CHANNEL := 2
const LABELS := ["FRONT LEFT","CENTER","FRONT RIGHT","SURROUND LEFT","SURROUND RIGHT","REAR LEFT","REAR RIGHT","FRONT HEIGHT LEFT","FRONT HEIGHT RIGHT","REAR HEIGHT LEFT","REAR HEIGHT RIGHT"]
const POSITIONS := [Vector3(-2,0,-3),Vector3(0,0,-3),Vector3(2,0,-3),Vector3(-3,0,0),Vector3(3,0,0),Vector3(-2,0,3),Vector3(2,0,3),Vector3(-2,2,-3),Vector3(2,2,-3),Vector3(-2,2,3),Vector3(2,2,3)]
func _ready() -> void:
	spatial = PsSpatialAudio.new()
	spatial.set_reference_distance_m(8.0)
	if not spatial.open(RATE,LABELS.size()):
		push_error(spatial.get_status())
		get_tree().quit(1)
		return
	spatial.set_positions(PackedVector3Array(POSITIONS))
	print("SPEAKER TEST: ",spatial.get_status(),"; eleven positions, two seconds each, no LFE or reflection mix")
func _process(_delta: float) -> void:
	if spatial == null or not spatial.is_active(): return
	var needed := spatial.get_frames_needed()
	var samples := PackedFloat32Array()
	samples.resize(needed*LABELS.size())
	for f in range(needed):
		var channel := frame/(RATE*SECONDS_PER_CHANNEL)
		if channel >= LABELS.size():
			spatial.close()
			print("SPEAKER TEST complete")
			get_tree().quit()
			return
		if channel != announced:
			announced=channel
			print("SPEAKER TEST: ",LABELS[channel])
		var phase := frame % (RATE*SECONDS_PER_CHANNEL)
		# 10 ms edge fades, -20 dBFS peak; no sudden clicks.
		var envelope := minf(1.0,minf(phase/480.0,(RATE*SECONDS_PER_CHANNEL-phase)/480.0))
		samples[f*LABELS.size()+channel] = 0.1*envelope*sin(TAU*440.0*frame/RATE)
		frame+=1
	spatial.push_audio(samples)
func _exit_tree() -> void:
	if spatial != null: spatial.close()
