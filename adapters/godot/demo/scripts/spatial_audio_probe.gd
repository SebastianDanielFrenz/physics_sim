extends Control
# Interactive diagnostic for the exact Windows object renderer used by the game.
var spatial: PsSpatialAudio
var position_m := Vector3(0,0,-3)
var volume := 0.06
var playing := false
var direct_channel := ""
var frame := 0
var pad: Control
var readout: Label
var status: Label
var height_slider: HSlider
var play_button: Button
const RATE := 48000
const RANGE := 5.0

func _ready() -> void:
	set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var background := ColorRect.new()
	background.color=Color("10151e")
	background.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	background.mouse_filter=Control.MOUSE_FILTER_IGNORE
	add_child(background)
	var panel := VBoxContainer.new()
	panel.position=Vector2(28,20)
	panel.add_theme_constant_override("separation",12)
	add_child(panel)
	var title := Label.new()
	title.text="Spatial audio probe"
	title.add_theme_font_size_override("font_size",26)
	panel.add_child(title)
	var help := Label.new()
	help.text="Drag the dot: left/right and front/rear. The listener stays at the centre.\nAdjust height separately. Reflections are disabled."
	panel.add_child(help)
	status=Label.new()
	status.autowrap_mode=TextServer.AUTOWRAP_WORD_SMART
	status.custom_minimum_size=Vector2(650,48)
	panel.add_child(status)
	pad=Control.new()
	pad.custom_minimum_size=Vector2(600,400)
	pad.draw.connect(_draw_pad)
	pad.gui_input.connect(_pad_input)
	panel.add_child(pad)
	readout=Label.new()
	readout.add_theme_font_size_override("font_size",18)
	panel.add_child(readout)
	var height_label := Label.new()
	height_label.text="Height (metres): below −3 … above +5"
	panel.add_child(height_label)
	height_slider=HSlider.new()
	height_slider.min_value=-3; height_slider.max_value=5; height_slider.step=0.05
	height_slider.value=0
	height_slider.value_changed.connect(func(value: float): position_m.y=value; _update_position())
	panel.add_child(height_slider)
	var volume_label := Label.new()
	volume_label.text="Volume"
	panel.add_child(volume_label)
	var volume_slider := HSlider.new()
	volume_slider.min_value=0; volume_slider.max_value=0.25; volume_slider.step=0.005; volume_slider.value=volume
	volume_slider.value_changed.connect(func(value: float): volume=value)
	panel.add_child(volume_slider)
	var buttons := HBoxContainer.new()
	buttons.add_theme_constant_override("separation",12)
	panel.add_child(buttons)
	play_button=Button.new()
	play_button.text="Play tone"
	play_button.custom_minimum_size=Vector2(160,40)
	play_button.pressed.connect(_toggle_tone)
	buttons.add_child(play_button)
	var reset := Button.new()
	reset.text="Reset: front, ear level"
	reset.pressed.connect(func(): position_m=Vector3(0,0,-3); height_slider.value=0; _update_position())
	buttons.add_child(reset)
	var reconnect := Button.new()
	reconnect.text="Reconnect audio"
	reconnect.pressed.connect(_connect_audio)
	buttons.add_child(reconnect)
	var direct_buttons := HBoxContainer.new()
	direct_buttons.add_theme_constant_override("separation",12)
	panel.add_child(direct_buttons)
	for channel in ["Surround L","Surround Back L"]:
		var direct := Button.new()
		direct.text="Direct: " + channel
		direct.pressed.connect(_direct_test.bind(channel))
		direct_buttons.add_child(direct)
	var pcm_buttons := HBoxContainer.new()
	pcm_buttons.add_theme_constant_override("separation",12)
	panel.add_child(pcm_buttons)
	for channel in ["Surround L","Surround Back L"]:
		var pcm := Button.new()
		pcm.text="7.1 PCM: " + channel
		pcm.pressed.connect(_pcm_test.bind(channel))
		pcm_buttons.add_child(pcm)
	get_window().focus_exited.connect(_release_pcm)
	_connect_audio()
	_update_position()

func _connect_audio() -> void:
	direct_channel=""
	if spatial != null: spatial.close()
	spatial=PsSpatialAudio.new()
	# Fixed gain throughout the UI range makes direction comparisons fair.
	spatial.set_reference_distance_m(20)
	var opened := spatial.open(RATE,1)
	status.text=spatial.get_status()
	play_button.disabled=not opened
	_update_position()

func _direct_test(channel: String) -> void:
	if spatial != null: spatial.close()
	spatial=PsSpatialAudio.new()
	direct_channel=channel
	var opened := spatial.open_static_channel(channel)
	status.text=spatial.get_status()
	playing=opened
	play_button.disabled=not opened
	play_button.text="Mute" if opened else "Play tone"
	_update_position()

func _pcm_test(channel: String) -> void:
	if spatial != null: spatial.close()
	spatial=PsSpatialAudio.new()
	direct_channel="7.1 PCM: " + channel
	var opened := spatial.open_pcm_channel(channel)
	status.text=spatial.get_status()
	playing=opened
	play_button.disabled=not opened
	play_button.text="Mute" if opened else "Play tone"
	_update_position()

func _release_pcm() -> void:
	if direct_channel.begins_with("7.1 PCM") and spatial != null:
		spatial.close()
		playing=false
		play_button.text="Play tone"
		status.text="PCM released on focus loss. Click a PCM button to resume."

func _toggle_tone() -> void:
	if direct_channel.begins_with("7.1 PCM"):
		if playing:
			_release_pcm()
		else:
			_pcm_test(direct_channel.trim_prefix("7.1 PCM: "))
		return
	playing=not playing
	play_button.text="Mute" if playing else "Play tone"

func _update_position() -> void:
	if spatial != null: spatial.set_positions(PackedVector3Array([position_m]))
	if readout != null:
		readout.text=("DIRECT: " + direct_channel + " — position ignored") if not direct_channel.is_empty() else "X %+.2f m    front/rear Z %+.2f m    height %+.2f m" % [position_m.x,position_m.z,position_m.y]
	if pad != null: pad.queue_redraw()

func _draw_pad() -> void:
	var rect := Rect2(Vector2(40,30),pad.size-Vector2(80,60))
	var centre := rect.get_center()
	pad.draw_rect(rect,Color("182433"))
	for i in range(11):
		var x := rect.position.x+rect.size.x*i/10.0
		var y := rect.position.y+rect.size.y*i/10.0
		pad.draw_line(Vector2(x,rect.position.y),Vector2(x,rect.end.y),Color("2a3c50"))
		pad.draw_line(Vector2(rect.position.x,y),Vector2(rect.end.x,y),Color("2a3c50"))
	pad.draw_circle(centre,8,Color("eeeeee"))
	pad.draw_line(centre,centre+Vector2(0,-20),Color.WHITE,3)
	var dot := centre+Vector2(position_m.x/RANGE*rect.size.x/2,position_m.z/RANGE*rect.size.y/2)
	pad.draw_line(centre,dot,Color("40cfe8"),2)
	pad.draw_circle(dot,12,Color("40cfe8"))
	var font := ThemeDB.fallback_font
	pad.draw_string(font,Vector2(centre.x-25,22),"FRONT",HORIZONTAL_ALIGNMENT_LEFT,-1,16)
	pad.draw_string(font,Vector2(centre.x-22,pad.size.y-8),"REAR",HORIZONTAL_ALIGNMENT_LEFT,-1,16)
	pad.draw_string(font,Vector2(0,centre.y),"LEFT",HORIZONTAL_ALIGNMENT_LEFT,-1,13)
	pad.draw_string(font,Vector2(pad.size.x-38,centre.y),"RIGHT",HORIZONTAL_ALIGNMENT_LEFT,-1,13)

func _pad_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index==MOUSE_BUTTON_LEFT and event.pressed:
		_move_dot(event.position)
	elif event is InputEventMouseMotion and event.button_mask & MOUSE_BUTTON_MASK_LEFT:
		_move_dot(event.position)

func _move_dot(point: Vector2) -> void:
	if not direct_channel.is_empty(): _connect_audio()
	var rect := Rect2(Vector2(40,30),pad.size-Vector2(80,60))
	var relative := (point-rect.get_center())/(rect.size/2)
	position_m.x=clampf(relative.x,-1,1)*RANGE
	position_m.z=clampf(relative.y,-1,1)*RANGE
	_update_position()

func _process(_delta: float) -> void:
	if spatial == null: return
	if not spatial.is_active():
		status.text=spatial.get_status()
		play_button.disabled=true
		return
	var needed := spatial.get_frames_needed()
	var samples := PackedFloat32Array()
	samples.resize(needed)
	for i in range(needed):
		# Broadband harmonic tone is easier to localise than a pure sine.
		var t := float(frame)/RATE
		var waveform := (sin(TAU*220*t)+0.5*sin(TAU*440*t)+0.3*sin(TAU*880*t)+0.2*sin(TAU*1760*t))/2.0
		samples[i]=waveform*volume if playing else 0.0
		frame+=1
	spatial.push_audio(samples)

func _exit_tree() -> void:
	if spatial != null: spatial.close()
