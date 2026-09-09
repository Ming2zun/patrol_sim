extends Node3D

const NET_SCRIPT = preload("res://scripts/net_server.gd")
const WORLD_SCRIPT = preload("res://scripts/world_loader.gd")
const MAP_SCRIPT = preload("res://scripts/minimap.gd")
const ROS_BASIS := Basis(Vector3(0,-1,0),Vector3(0,0,1),Vector3(-1,0,0))
var world: AlpineWorld
var rover: CharacterBody3D
var visual: Node3D
var wheel_rig: RoverVisualRig
var lidar36: Node3D
var camera: Camera3D
var ground_details: Node3D
var low_quality := false
var quality_button: Button
var terrain_materials: Array[Dictionary] = []
var far_proxies: Array[MultiMeshInstance3D] = []
var quality_nodes: Array[Dictionary] = []
var frame_peak_ms := 0.0
var last_frame_peak_ms := 0.0
var quality_timer := 0.0
var link: SimLink
var map
var settings: Dictionary
var route: Array[Vector3] = []
var mode := "manual"
var emergency_stop := false
var paused := false
var yaw := 0.0
var speed := 0.0
var target_v := 0.0
var target_w := 0.0
var actual_w := 0.0
var steering := 0.0
var target_steering := 0.0
var max_steering := deg_to_rad(28.0)
var steering_rate := deg_to_rad(55.0)
var cruise_speed := 36.0/3.6
var max_speed := 10.0
var max_turn := 1.2
var command_timeout := 0.5
var last_command_ms := -100000
var last_command_seq := -1
var sim_time := 0.0
var state_accumulator := 0.0
var seq := 0
var epoch := 0
var route_index := 0
var acceleration_ros := Vector3.ZERO
var angular_ros := Vector3.ZERO
var previous_velocity := Vector3.ZERO
var previous_attitude := Quaternion.IDENTITY
var previous_angular_ros := Vector3.ZERO
var imu_ready_at := 0.35
var imu_seq := 0
var camera_distance := 7.4
var camera_yaw := 0.48
var camera_pitch := 0.23
var overview := false
var orbiting := false
var interface_canvas: CanvasLayer
var eye_button: Button
var hud_canvas: CanvasLayer
var hud_stats: Label
var hud_network: Label
var hud_mode: Label
var hud_notice: Label
var status_accumulator := 0.0
var notice := ""
var preview_style := ""
var screenshot_path := ""
var screenshot_at := 0.0
var exit_after := 0.0
var test_mode := false
var sensor_view: SubViewport
var sensor_camera: Camera3D
var image_accumulator := 0.0
var image_busy := false

func _ready() -> void:
	Engine.max_physics_steps_per_frame = 32
	settings = _load_settings()
	low_quality = settings.get("rendering", {}).get("quality", "high") == "low"
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--quality="):
			low_quality = arg.get_slice("=",1) == "low"
		elif arg == "--test-run":
			test_mode = true
		elif arg.begins_with("--test-port="):
			settings["network"]["port"] = int(arg.get_slice("=",1))
		elif arg.begins_with("--screenshot="):
			screenshot_path = arg.trim_prefix("--screenshot=")
			screenshot_at = 12.0
		elif arg.begins_with("--preview="):
			preview_style = arg.trim_prefix("--preview=")
		elif arg.begins_with("--exit-after="):
			exit_after = float(arg.trim_prefix("--exit-after="))
	max_speed = float(settings["vehicle"].get("max_speed_mps",10.0))
	cruise_speed = float(settings["vehicle"].get("cruise_speed_kmh",36.0))/3.6
	max_turn = float(settings["vehicle"].get("max_yaw_rate_rps",1.2))
	max_steering = deg_to_rad(float(settings["vehicle"].get("max_steering_deg",28.0)))
	steering_rate = deg_to_rad(float(settings["vehicle"].get("steering_rate_deg_s",55.0)))
	command_timeout = float(settings["network"].get("command_timeout_s",0.5))
	world = WORLD_SCRIPT.new()
	add_child(world)
	world.build()
	for p in world.manifest["route"]:
		route.append(Vector3(float(p[0]),float(p[1]),float(p[2])))
	_build_rover()
	lidar36 = preload("res://scripts/lidar36.gd").new()
	add_child(lidar36)
	lidar36.configure(settings["sensors"])
	ground_details = preload("res://scripts/ground_details.gd").new()
	add_child(ground_details)
	ground_details.setup(rover)
	_build_camera()
	_build_sensor_camera()
	_build_hud()
	_remember_quality_nodes(world)
	_remember_terrain_materials(world)
	_build_far_proxies()
	_apply_quality()
	link = NET_SCRIPT.new()
	add_child(link)
	link.packet_received.connect(_handle_packet)
	link.link_changed.connect(_on_link_changed)
	link.configure(settings["network"])
	reset_vehicle()
	if preview_style == "snow":
		var index := 0
		for i in range(route.size()-8):
			if route[i].y>109.0:
				index = i
				break
		rover.global_position = route[index]+Vector3.UP*0.4
		var direction := route[index+7]-route[index]
		yaw = atan2(-direction.x,-direction.z)
		rover.rotation.y = yaw
		wheel_rig.reset_history()
		camera_distance = 9.5
		camera_pitch = 0.28
	elif preview_style == "vehicle":
		camera_distance = 4.8
		camera_yaw = 0.95
		camera_pitch = 0.14
	if test_mode:
		set_mode("ros")
	print("SIM_READY " + JSON.stringify({"port":link.port,"instances":world.instance_count,"colliders":world.collider_count,"route_points":route.size(),"mode":mode}))

func _load_settings() -> Dictionary:
	var path := "res://config/sim.json"
	var external := OS.get_executable_path().get_base_dir().path_join("sim_config.json")
	if not OS.has_feature("editor") and FileAccess.file_exists(external):
		path = external
	var data = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not data is Dictionary:
		push_error("Invalid simulator configuration")
		get_tree().quit(2)
		return {}
	return data

func _build_rover() -> void:
	rover = CharacterBody3D.new()
	rover.name = "InspectionRover"
	rover.collision_layer = 2
	rover.collision_mask = 1
	rover.floor_snap_length = 0.6
	rover.floor_max_angle = deg_to_rad(38)
	rover.floor_stop_on_slope = true
	var collision := CollisionShape3D.new()
	var capsule := CapsuleShape3D.new()
	capsule.radius = 0.72
	capsule.height = 1.60
	collision.shape = capsule
	collision.position.y = 0.80
	rover.add_child(collision)
	add_child(rover)
	visual = Node3D.new()
	visual.name = "TerrainAlignedVisual"
	rover.add_child(visual)
	var packed: PackedScene = load("res://assets/vehicle/rover.glb")
	var model: Node3D = packed.instantiate()
	model.rotation.y = PI
	visual.add_child(model)
	_upgrade_rover(model)
	wheel_rig = preload("res://scripts/rover_visual.gd").new()
	visual.add_child(wheel_rig)
	wheel_rig.build(model)
	world.refine_vehicle(model)

func _build_camera() -> void:
	camera = Camera3D.new()
	camera.fov = 57
	camera.near = 0.10
	camera.far = 2200
	add_child(camera)
	camera.make_current()

func _build_hud() -> void:
	var canvas := CanvasLayer.new()
	hud_canvas = canvas
	add_child(canvas)
	var root := Control.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	canvas.add_child(root)
	var theme := Theme.new()
	if ResourceLoader.exists("res://assets/fonts/NotoSansSC.ttf"):
		var font := FontVariation.new()
		font.base_font = load("res://assets/fonts/NotoSansSC.ttf")
		font.variation_opentype = {"wght":450.0}
		theme.default_font = font
	theme.default_font_size = 17
	theme.set_color("font_color","Label",Color(0.93,0.95,0.97))
	theme.set_color("font_color","Button",Color(0.92,0.95,0.97))
	root.theme = theme
	var panel := PanelContainer.new()
	panel.position = Vector2(22,22)
	panel.custom_minimum_size = Vector2(360,0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.035,0.055,0.075,0.94)
	style.corner_radius_top_left = 12
	style.corner_radius_top_right = 12
	style.corner_radius_bottom_left = 12
	style.corner_radius_bottom_right = 12
	style.content_margin_left = 22
	style.content_margin_right = 22
	style.content_margin_top = 18
	style.content_margin_bottom = 18
	panel.add_theme_stylebox_override("panel",style)
	root.add_child(panel)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation",12)
	panel.add_child(box)
	var title := Label.new()
	title.text = "无人车巡检"
	title.add_theme_font_size_override("font_size",26)
	box.add_child(title)
	var subtitle := Label.new()
	subtitle.text = "ALPINE PATROL  /  SIMULATOR 09"
	subtitle.add_theme_font_size_override("font_size",13)
	subtitle.modulate = Color(0.44,0.79,0.83)
	box.add_child(subtitle)
	hud_mode = Label.new()
	box.add_child(hud_mode)
	for item in [["前轮转向  ·  WASD","manual"],["环线巡航  ·  2.14 km","route"],["ROS 2 远程控制","ros"]]:
		var button := Button.new()
		button.text = item[0]
		button.custom_minimum_size.y = 42
		button.pressed.connect(set_mode.bind(item[1]))
		box.add_child(button)
	var row := HBoxContainer.new()
	box.add_child(row)
	var stop := Button.new()
	stop.text = "急停 / 解除"
	stop.custom_minimum_size = Vector2(148,40)
	stop.pressed.connect(func(): emergency_stop = not emergency_stop; _clear_command())
	row.add_child(stop)
	var reset := Button.new()
	reset.text = "车辆复位"
	reset.custom_minimum_size = Vector2(148,40)
	reset.pressed.connect(reset_vehicle)
	row.add_child(reset)
	var camera_button := Button.new()
	camera_button.text = "切换总览 / 跟车  ·  Tab"
	camera_button.pressed.connect(func(): overview = not overview)
	box.add_child(camera_button)
	quality_button = Button.new()
	quality_button.pressed.connect(func(): low_quality = not low_quality; _apply_quality())
	box.add_child(quality_button)

	hud_network = Label.new()
	hud_network.add_theme_font_size_override("font_size",14)
	box.add_child(hud_network)
	hud_stats = Label.new()
	hud_stats.add_theme_font_size_override("font_size",15)
	box.add_child(hud_stats)
	hud_notice = Label.new()
	hud_notice.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	hud_notice.custom_minimum_size.x = 310
	hud_notice.add_theme_font_size_override("font_size",13)
	hud_notice.modulate = Color(0.94,0.73,0.37)
	box.add_child(hud_notice)
	map = MAP_SCRIPT.new()
	map.set_anchors_and_offsets_preset(Control.PRESET_TOP_RIGHT)
	map.offset_left = -302
	map.offset_right = -22
	map.offset_top = 22
	map.offset_bottom = 276
	map.custom_minimum_size = Vector2(280,254)
	map.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(map)
	map.configure(route)
	var help := Label.new()
	help.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_LEFT)
	help.position = Vector2(24,-58)
	help.text = "WASD / 方向键 驾驶   ·   右键拖动 环视   ·   滚轮 缩放\n空格 急停   R 复位   F2 控制模式   P 暂停   Tab 总览   H / 眼睛 隐藏界面"
	help.add_theme_font_size_override("font_size",15)
	help.add_theme_color_override("font_shadow_color",Color(0,0,0,0.95))
	help.add_theme_constant_override("shadow_offset_x",1)
	help.add_theme_constant_override("shadow_offset_y",2)
	root.add_child(help)
	_build_interface_toggle(theme)

func _build_interface_toggle(theme: Theme) -> void:
	interface_canvas = CanvasLayer.new()
	interface_canvas.name = "AlwaysAvailableInterfaceToggle"
	interface_canvas.layer = 20
	add_child(interface_canvas)
	var overlay := Control.new()
	overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	overlay.theme = theme
	interface_canvas.add_child(overlay)
	eye_button = Button.new()
	eye_button.name = "ToggleInterfaceVisibility"
	eye_button.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_RIGHT)
	eye_button.offset_left = -76
	eye_button.offset_right = -22
	eye_button.offset_top = -70
	eye_button.offset_bottom = -22
	eye_button.focus_mode = Control.FOCUS_NONE
	eye_button.icon = preload("res://assets/ui/eye_open.svg")
	eye_button.icon_alignment = HORIZONTAL_ALIGNMENT_CENTER
	eye_button.tooltip_text = "隐藏界面（H）：按钮、参数和地图"
	var normal := StyleBoxFlat.new()
	normal.bg_color = Color(.035,.055,.075,.92)
	normal.set_corner_radius_all(10)
	eye_button.add_theme_stylebox_override("normal",normal)
	var hover: StyleBoxFlat = normal.duplicate()
	hover.bg_color = Color(.09,.23,.26,.97)
	eye_button.add_theme_stylebox_override("hover",hover)
	eye_button.add_theme_stylebox_override("pressed",hover)
	eye_button.pressed.connect(_toggle_interface)
	overlay.add_child(eye_button)

func _toggle_interface() -> void:
	hud_canvas.visible = not hud_canvas.visible
	eye_button.icon = preload("res://assets/ui/eye_open.svg") if hud_canvas.visible else preload("res://assets/ui/eye_closed.svg")
	eye_button.tooltip_text = "隐藏界面（H）：按钮、参数和地图" if hud_canvas.visible else "显示界面（H）"

func set_mode(value: String) -> void:
	if value not in ["manual","route","ros"]:
		return
	mode = value
	_clear_command()
	route_index = _nearest_route_index()
	notice = "等待 ROS 2 指令；超时将自动停车。" if mode == "ros" else ""

func _clear_command() -> void:
	target_v = 0
	target_w = 0
	last_command_ms = -100000
	last_command_seq = -1

func reset_vehicle() -> void:
	if lidar36 != null:
		lidar36.reset_scan()
	_clear_command()
	previous_angular_ros = Vector3.ZERO
	imu_ready_at = sim_time+0.35
	speed = 0
	actual_w = 0
	steering = 0
	target_steering = 0
	yaw = 0
	rover.velocity = Vector3.ZERO
	rover.rotation = Vector3.ZERO
	visual.basis = Basis.IDENTITY
	if wheel_rig != null:
		wheel_rig.reset_history()
	var request := PhysicsRayQueryParameters3D.create(Vector3(0,15,0),Vector3(0,-30,0),1)
	var hit := get_world_3d().direct_space_state.intersect_ray(request)
	rover.global_position = hit["position"] + Vector3.UP*0.15 if not hit.is_empty() else Vector3(0,1,0)
	previous_velocity = Vector3.ZERO
	previous_attitude = Quaternion.IDENTITY
	route_index = _nearest_route_index()
	epoch += 1
	notice = "车辆已复位。仿真时间保持连续。"

func _nearest_route_index() -> int:
	var best := INF
	var found := 0
	for i in range(route.size()):
		var d := rover.global_position.distance_squared_to(route[i])
		if d < best:
			best = d
			found = i
	return found

func _physics_process(delta: float) -> void:
	if rover == null or link == null:
		return
	if not paused:
		sim_time += delta
		_update_control()
		var desired_v := 0.0 if emergency_stop else target_v
		var desired_steering := 0.0 if emergency_stop else target_steering
		speed = move_toward(speed,desired_v,4.0*delta if absf(desired_v)<absf(speed) else 1.8*delta)
		steering = move_toward(steering,desired_steering,steering_rate*delta)
		# Bicycle kinematics referenced to the rear axle; no zero-speed spin.
		actual_w = speed*tan(steering)/wheel_rig.wheelbase
		yaw = wrapf(yaw + actual_w*delta,-PI,PI)
		rover.rotation.y = yaw
		var forward := Vector3(-sin(yaw),0,-cos(yaw))
		var left := Vector3(-cos(yaw),0,sin(yaw))
		var planar_velocity := forward*speed + left*(actual_w*wheel_rig.rear_offset)
		rover.velocity.x = planar_velocity.x
		rover.velocity.z = planar_velocity.z
		if not rover.is_on_floor():
			rover.velocity.y -= 9.81*delta
		else:
			rover.velocity.y = -0.1
		rover.move_and_slide()
		if rover.global_position.y < -80:
			reset_vehicle()
		var normal := rover.get_floor_normal() if rover.is_on_floor() else Vector3.UP
		if normal.length() < 0.5:
			normal = Vector3.UP
		normal = wheel_rig.ground_normal(normal)
		var tangent := (forward - normal*forward.dot(normal)).normalized()
		var right := tangent.cross(normal).normalized()
		var attitude := Basis(right,normal,-tangent).orthonormalized()
		visual.global_basis = visual.global_basis.orthonormalized().slerp(attitude,minf(1,delta*8)).orthonormalized()
		wheel_rig.step(delta,rover.is_on_floor(),steering)
		world.update_vehicle_pose(visual.global_transform)
		var q := visual.global_basis.get_rotation_quaternion()
		var dq := (previous_attitude.inverse()*q).normalized()
		if dq.w < 0:
			dq = -dq
		var angle := dq.get_angle()
		angular_ros = _to_ros(dq.get_axis()*angle/delta) if angle > 0.000001 else Vector3.ZERO
		var acceleration := (rover.velocity - previous_velocity)/delta - Vector3(0,-9.81,0)
		acceleration_ros = _to_ros(visual.global_basis.inverse()*acceleration)
		var omega := ROS_BASIS.inverse()*angular_ros
		var angular_acc := ROS_BASIS.inverse()*(angular_ros-previous_angular_ros)/delta
		acceleration_ros += _to_ros(angular_acc.cross(lidar36.mount)+omega.cross(omega.cross(lidar36.mount)))
		previous_angular_ros = angular_ros
		if sim_time>=imu_ready_at:
			_publish_imu()
		previous_velocity = rover.velocity
		previous_attitude = q
	lidar36.step(delta,sim_time,epoch,visual.global_transform,paused,link)
	state_accumulator += delta
	if state_accumulator + 0.000001 >= 0.05:
		state_accumulator -= 0.05
		_publish_state()


func _update_control() -> void:
	if mode == "manual":
		var throttle := float(Input.is_physical_key_pressed(KEY_W) or Input.is_physical_key_pressed(KEY_UP)) - float(Input.is_physical_key_pressed(KEY_S) or Input.is_physical_key_pressed(KEY_DOWN))
		var turn := float(Input.is_physical_key_pressed(KEY_A) or Input.is_physical_key_pressed(KEY_LEFT)) - float(Input.is_physical_key_pressed(KEY_D) or Input.is_physical_key_pressed(KEY_RIGHT))
		target_v = throttle*max_speed
		target_steering = turn*max_steering
	elif mode == "route":
		var rear_position := rover.global_position+Vector3(sin(yaw),0,cos(yaw))*wheel_rig.rear_offset
		var rear2 := Vector2(rear_position.x,rear_position.z)
		var best := INF
		var nearest := route_index
		# Search only adjacent route segments: don't skip across hairpins.
		for offset in range(-4,21):
			var index := posmod(route_index+offset,route.size())
			var point := Vector2(route[index].x,route[index].z)
			var distance := rear2.distance_squared_to(point)
			if distance<best:
				best=distance
				nearest=index
		route_index=nearest
		var lookahead := clampf(2.0+absf(speed)*.35,2.0,5.5)
		var target_index := nearest
		var ahead := 0.0
		for offset in range(1,35):
			var index := (nearest+offset)%route.size()
			ahead += Vector2(route[index].x-route[target_index].x,route[index].z-route[target_index].z).length()
			target_index=index
			if ahead>=lookahead: break
		var d := route[target_index]-rear_position
		var error := wrapf(atan2(-d.x,-d.z)-yaw,-PI,PI)
		var curvature := 2.0*sin(error)/maxf(Vector2(d.x,d.z).length(),1.0)
		target_steering=atan(wheel_rig.wheelbase*curvature)
		target_v=minf(minf(max_speed,cruise_speed),sqrt(1.8/maxf(absf(curvature),.001)))
		# Brake before a sharp bend, using route curvature and available distance.
		ahead=0.0
		var previous_index := nearest
		for offset in range(0,81,4):
			var index := (nearest+offset)%route.size()
			ahead+=Vector2(route[index].x-route[previous_index].x,route[index].z-route[previous_index].z).length()
			previous_index=index
			var a: Vector3=route[posmod(index-4,route.size())]
			var b: Vector3=route[index]
			var c: Vector3=route[(index+4)%route.size()]
			var ab:=Vector2(b.x-a.x,b.z-a.z)
			var bc:=Vector2(c.x-b.x,c.z-b.z)
			var turn_curvature:=absf(ab.angle_to(bc))/maxf(.5*(ab.length()+bc.length()),.1)
			var bend_speed:=sqrt(1.7/maxf(turn_curvature,.001))
			var approach_speed:=sqrt(bend_speed*bend_speed+4.8*maxf(0,ahead-4.0))
			target_v=minf(target_v,approach_speed)
			if ahead>35.0: break
		target_v*=clampf(1.0-absf(error)/PI,.35,1.0)
	else:
		if not link.authenticated or Time.get_ticks_msec()-last_command_ms>int(command_timeout*1000):
			target_v = 0
			target_w = 0
		# Twist angular.z remains requested yaw rate; project infeasible commands
		# onto the steering limit. At rest it can turn the wheels, never the body.
		target_steering = atan(target_w*wheel_rig.wheelbase/target_v) if absf(target_v)>.05 else signf(target_w)*max_steering
	target_steering = clampf(target_steering,-max_steering,max_steering)
	if absf(speed)>2.0:
		var limit := atan(wheel_rig.wheelbase*minf(max_turn/absf(speed),3.5/(speed*speed)))
		target_steering = clampf(target_steering,-limit,limit)

func _publish_state() -> void:
	if not link.authenticated:
		return
	seq += 1
	var p := _to_ros(rover.global_position)
	var basis_ros := ROS_BASIS*visual.global_basis*ROS_BASIS.inverse()
	var q := basis_ros.get_rotation_quaternion().normalized()
	var velocity_ros := _to_ros(visual.global_basis.inverse()*rover.velocity)
	var packet := {"type":"state","version":1,"seq":seq,"epoch":epoch,"sim_time":sim_time,"mode":mode,"paused":paused,"estop":emergency_stop,"watchdog_stopped":mode=="ros" and Time.get_ticks_msec()-last_command_ms>int(command_timeout*1000),
		"pose":{"position":_array(p),"orientation":[q.x,q.y,q.z,q.w]},
		"twist":{"linear":_array(velocity_ros),"angular":_array(angular_ros)},
		"imu":{"orientation":[q.x,q.y,q.z,q.w],"angular_velocity":_array(angular_ros),"linear_acceleration":_array(acceleration_ros)},
		"visual_wheels":wheel_rig.diagnostics(),
		"steering":{"model":"ackermann","central_angle_rad":steering,"max_angle_rad":max_steering,"actual_yaw_rate":actual_w},
		"lidar3d":lidar36.diagnostics(),
		"performance":{"fps":Engine.get_frames_per_second(),"frame_peak_ms":last_frame_peak_ms,"process_ms":Performance.get_monitor(Performance.TIME_PROCESS)*1000.0,"physics_ms":Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS)*1000.0,"draw_calls":Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME),"quality":"low" if low_quality else "high"},
		"applied_command":{"linear_x":target_v,"angular_z":target_w}}
	link.send_packet(packet)

func _number(packet: Dictionary, key: String) -> float:
	var value = packet.get(key, null)
	if typeof(value) != TYPE_FLOAT and typeof(value) != TYPE_INT:
		return NAN
	return float(value)

func _handle_packet(packet: Dictionary) -> void:
	var kind := str(packet.get("type",""))
	if kind == "cmd_vel":
		var linear := _number(packet,"linear_x")
		var angular := _number(packet,"angular_z")
		var sequence := _number(packet,"seq")
		if not is_finite(linear) or not is_finite(angular) or not is_finite(sequence) or int(sequence)<=last_command_seq:
			return
		if mode != "ros" or emergency_stop or paused:
			return
		last_command_seq = int(sequence)
		target_v = clampf(linear,-max_speed,max_speed)
		target_w = clampf(angular,-max_turn,max_turn)
		last_command_ms = Time.get_ticks_msec()
	elif kind == "set_mode":
		set_mode(str(packet.get("mode","")))
		_ack(packet)
	elif kind == "reset":
		reset_vehicle()
		_ack(packet)
	elif kind == "estop":
		if packet.get("enabled") is bool:
			emergency_stop = packet["enabled"]
			_clear_command()
			_ack(packet)
	elif kind == "pause":
		if packet.get("enabled") is bool:
			paused = packet["enabled"]
			rover.velocity = Vector3.ZERO
			speed = 0
			actual_w = 0
			_clear_command()
			_ack(packet)
	elif kind == "ping":
		link.send_packet({"type":"pong","sim_time":sim_time})

func _ack(packet: Dictionary) -> void:
	link.send_packet({"type":"ack","request_id":packet.get("request_id",""),"operation":packet.get("type",""),"ok":true,"mode":mode,"paused":paused,"estop":emergency_stop})

func _on_link_changed(connected: bool) -> void:
	_clear_command()
	notice = "ROS 2 通信桥已连接。" if connected else "连接已断开；远程控制指令已清除。"

func _process(delta: float) -> void:
	frame_peak_ms = maxf(frame_peak_ms, delta*1000.0)
	quality_timer += delta
	if quality_timer >= 1.0:
		last_frame_peak_ms = frame_peak_ms
		frame_peak_ms = 0.0
		quality_timer = 0.0
	if not preview_style.is_empty():
		overview = false
		hud_canvas.visible = false
		interface_canvas.visible = false
		camera_distance = 2.7 if preview_style=="livery" else (4.8 if preview_style=="vehicle" else 9.5)
		camera_pitch = 0.0 if preview_style=="livery" else (0.14 if preview_style=="vehicle" else 0.28)
		camera_yaw = PI/2 if preview_style=="livery" else (0.95 if preview_style=="vehicle" else 0.48)
	if camera == null or rover == null:
		return
	if overview:
		camera.global_position = Vector3(220,420,95)
		camera.look_at(Vector3(-20,45,-170),Vector3.UP)
	else:
		var angle := yaw+camera_yaw
		var offset := Vector3(sin(angle)*cos(camera_pitch),sin(camera_pitch),cos(angle)*cos(camera_pitch))*camera_distance
		var target := rover.global_position+Vector3.UP*1.2
		var desired := target+offset
		camera.global_position = camera.global_position.lerp(desired,1-exp(-delta*7))
		if camera.global_position.distance_to(target)>0.2:
			camera.look_at(target,Vector3.UP)
	if sensor_camera != null:
		sensor_camera.global_position = rover.global_position + visual.global_basis*Vector3(0,1.5,-0.7)
		sensor_camera.global_basis = visual.global_basis
		image_accumulator += delta
		if image_accumulator > 0.2 and not image_busy and link != null and link.authenticated:
			image_accumulator = 0
			_capture_image()
	map.vehicle_position = Vector2(rover.global_position.x,rover.global_position.z)
	map.heading = yaw
	map.queue_redraw()
	status_accumulator += delta
	if status_accumulator > 0.2 and link != null:
		status_accumulator = 0
		var ros := _to_ros(rover.global_position)
		hud_mode.text = "● " + {"manual":"手动驾驶","route":"环线巡航","ros":"ROS 2 远程控制"}[mode] + ("  [急停]" if emergency_stop else "") + ("  [暂停]" if paused else "")
		hud_network.text = ("网络已连接" if link.authenticated else "等待通信桥") + "  ·  TCP " + str(link.port)
		hud_network.modulate = Color(0.4,0.9,0.72) if link.authenticated else Color(0.65,0.71,0.77)
		hud_stats.text = "速度  %.2f m/s · %.1f km/h\n航向  %.1f°\n位置  X %.1f   Y %.1f   Z %.1f m\n仿真  %.1f s    帧率  %d FPS\n雷达  36 线 × %d 列 / %.0f Hz\n惯导  200 Hz · 与雷达同步" % [speed,speed*3.6,rad_to_deg(yaw),ros.x,ros.y,ros.z,sim_time,Engine.get_frames_per_second(),lidar36.columns,lidar36.rate_hz]
		hud_notice.text = link.error_text if not link.error_text.is_empty() else notice
	if screenshot_at > 0 and sim_time > screenshot_at:
		screenshot_at = -1
		_save_screenshot.call_deferred()
	if exit_after > 0 and sim_time > exit_after:
		get_tree().quit()

func _save_screenshot() -> void:
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	var error := image.save_png(screenshot_path)
	print("SCREENSHOT " + str(error) + " " + screenshot_path + " camera=" + str([camera_distance,camera_pitch,camera_yaw]) + " FPS=" + str(Engine.get_frames_per_second()) + " lidar=" + JSON.stringify(lidar36.diagnostics()))

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			orbiting = event.pressed
		if event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_UP:
			camera_distance = maxf(4,camera_distance-1)
		if event.pressed and event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			camera_distance = minf(55,camera_distance+1)
	if event is InputEventMouseMotion and orbiting:
		camera_yaw -= event.relative.x*0.005
		camera_pitch = clampf(camera_pitch+event.relative.y*0.003,-0.1,1.25)
	if event is InputEventKey and event.pressed and not event.echo:
		match event.physical_keycode:
			KEY_H:
				_toggle_interface()
			KEY_SPACE:
				emergency_stop = not emergency_stop
				_clear_command()
			KEY_R:
				reset_vehicle()
			KEY_TAB:
				overview = not overview
			KEY_F2:
				set_mode("ros" if mode != "ros" else "manual")
			KEY_P:
				paused = not paused
				speed = 0
				actual_w = 0
				rover.velocity = Vector3.ZERO
				_clear_command()

func _to_ros(v: Vector3) -> Vector3:
	return Vector3(-v.z,-v.x,v.y)

func _array(v: Vector3) -> Array:
	return [v.x,v.y,v.z]


func _build_sensor_camera() -> void:
	if DisplayServer.get_name() == "headless" or not bool(settings["sensors"].get("camera_enabled",true)):
		return
	sensor_view = SubViewport.new()
	sensor_view.size = Vector2i(320,180)
	sensor_view.world_3d = get_world_3d()
	sensor_view.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(sensor_view)
	sensor_camera = Camera3D.new()
	sensor_camera.keep_aspect = Camera3D.KEEP_WIDTH
	sensor_camera.fov = 70
	sensor_camera.near = 0.1
	sensor_camera.far = 300
	sensor_view.add_child(sensor_camera)
	sensor_camera.make_current()

func _capture_image() -> void:
	image_busy = true
	var capture_stamp := sim_time
	sensor_view.render_target_update_mode = SubViewport.UPDATE_ONCE
	await RenderingServer.frame_post_draw
	if not is_instance_valid(sensor_view):
		return
	var frame := sensor_view.get_texture().get_image()
	if frame != null and not frame.is_empty() and link.authenticated:
		var jpeg := frame.save_jpg_to_buffer(0.7)
		var projection := sensor_camera.get_camera_projection()
		link.send_packet({"type":"image","version":1,"sim_time":capture_stamp,
			"width":320,"height":180,"format":"jpeg",
			"fx":projection.x.x*160,"fy":projection.y.y*90,"cx":160.0,"cy":90.0,
			"data":Marshalls.raw_to_base64(jpeg)})
	image_busy = false

func _upgrade_rover(model: Node3D) -> void:
	for node in world._mesh_nodes(model):
		var label := str(node.name).to_lower()
		if label.begins_with("roof shoulder | vent slat"):
			node.position.z -= 0.45
		if label.contains("unit identification") or label.begins_with("lidar "):
			node.visible = false
	var additions: PackedScene = load("res://assets/vehicle/livery_lidar36_v007.glb")
	model.add_child(additions.instantiate())
	for node in world._mesh_nodes(model):
		if str(node.name).begins_with("Livery "):
			node.visible = false
	_rover_label(model,Vector3(0,1.372,-.835),Vector3(-PI/2,0,0),.87)
	_rover_label(model,Vector3(.706,1.21,0),Vector3(0,PI/2,0),.78)
	_rover_label(model,Vector3(-.706,1.21,0),Vector3(0,-PI/2,0),.78)

func _rover_label(parent: Node3D, location: Vector3, angles: Vector3, width: float) -> void:
	var label := Label3D.new()
	label.name = "ChineseLivery"
	label.text = "喵了个水蓝蓝"
	var font := FontVariation.new()
	font.base_font = load("res://assets/fonts/NotoSansSC.ttf")
	font.variation_opentype = {"wght":650.0}
	font.variation_embolden = 0.7
	label.font = font
	label.font_size = 128
	label.pixel_size = width/font.get_string_size(label.text,HORIZONTAL_ALIGNMENT_LEFT,-1,128).x
	label.outline_size = 0
	label.modulate = Color(.035,.045,.055)
	label.shaded = false
	label.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	label.double_sided = false
	label.position = location
	label.rotation = angles
	parent.add_child(label)

func _publish_imu() -> void:
	if not link.authenticated or not link.wants_imu:
		return
	imu_seq += 1
	var basis_ros := ROS_BASIS*visual.global_basis*ROS_BASIS.inverse()
	var q := basis_ros.get_rotation_quaternion().normalized()
	link.send_packet({"type":"imu_sample","version":1,"seq":imu_seq,"epoch":epoch,"stamp":sim_time,"frame_id":"imu_link","position_ros":[-lidar36.mount.z,-lidar36.mount.x,lidar36.mount.y],"orientation":[q.x,q.y,q.z,q.w],"angular_velocity":_array(angular_ros),"linear_acceleration":_array(acceleration_ros)})


func _remember_quality_nodes(node: Node) -> void:
	if node is GeometryInstance3D:
		quality_nodes.append({"node":node,"shadow":node.cast_shadow,"begin":node.visibility_range_begin,"end":node.visibility_range_end,"margin":node.visibility_range_end_margin,"visible":node.visible})
	elif node is DirectionalLight3D:
		quality_nodes.append({"node":node,"shadow":node.shadow_enabled})
	for child in node.get_children():
		_remember_quality_nodes(child)

func _apply_quality() -> void:
	for record in terrain_materials:
		record["node"].set_surface_override_material(record["surface"],record["low"] if low_quality else record["original"])
	for proxy in far_proxies:
		proxy.visible = low_quality
	camera.far = 350.0 if low_quality else 2200.0
	get_viewport().scaling_3d_scale = 0.75 if low_quality else 1.0
	get_viewport().msaa_3d = Viewport.MSAA_DISABLED if low_quality else Viewport.MSAA_2X
	for record in quality_nodes:
		var node: Node = record["node"]
		if node is GeometryInstance3D:
			node.visible = record["visible"]
			node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF if low_quality else record["shadow"]
			node.visibility_range_begin = record["begin"]
			node.visibility_range_end = record["end"]
			node.visibility_range_end_margin = record["margin"]
			if low_quality and node is MultiMeshInstance3D:
				if record["begin"] > 0 or (record["end"] > 0 and record["end"] <= 25):
					node.visible = false
				if node.visibility_range_begin > 0 and node.visibility_range_begin <= 25:
					node.visibility_range_begin = 12.0
				if node.visibility_range_end > 0:
					node.visibility_range_end = 12.0 if node.visibility_range_end <= 25 else minf(180.0,node.visibility_range_end)
				node.visibility_range_end_margin = 0.0
		elif node is DirectionalLight3D:
			node.shadow_enabled = false if low_quality else record["shadow"]
	ground_details.visible = not low_quality
	ground_details.set_physics_process(not low_quality)
	quality_button.text = "画质：流畅  ·  点击切换精细" if low_quality else "画质：精细  ·  点击切换流畅"

func _build_far_proxies() -> void:
	var groups := {}
	for record in quality_nodes:
		var node: Node = record["node"]
		if not node is MultiMeshInstance3D or record["begin"] <= 0:
			continue
		var mm: MultiMesh = node.multimesh
		var key := mm.mesh.get_instance_id()
		if not groups.has(key):
			groups[key] = {"mesh":mm.mesh,"transforms":[]}
		for index in range(mm.instance_count):
			groups[key]["transforms"].append(world.global_transform.affine_inverse()*node.global_transform*mm.get_instance_transform(index))
	for group in groups.values():
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.mesh = group["mesh"]
		mm.instance_count = group["transforms"].size()
		for index in range(mm.instance_count):
			mm.set_instance_transform(index,group["transforms"][index])
		var proxy := MultiMeshInstance3D.new()
		proxy.multimesh = mm
		proxy.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		world.add_child(proxy)
		far_proxies.append(proxy)

func _remember_terrain_materials(node: Node) -> void:
	if node is MeshInstance3D and node.mesh != null:
		for surface in range(node.mesh.get_surface_count()):
			var material: Material = node.get_active_material(surface)
			if material is ShaderMaterial and material.shader.resource_path.ends_with("world_surface.gdshader"):
				var simple := ShaderMaterial.new()
				simple.shader = Shader.new()
				simple.shader.code = LOW_SURFACE_SHADER
				for parameter in ["road_mask","mask_bounds","road_overlay","soil_color","road_color","snow_color"]:
					simple.set_shader_parameter(parameter,material.get_shader_parameter(parameter))
				terrain_materials.append({"node":node,"surface":surface,"original":node.get_surface_override_material(surface),"low":simple})
	for child in node.get_children():
		_remember_terrain_materials(child)

const LOW_SURFACE_SHADER = """
shader_type spatial;
render_mode diffuse_lambert;
uniform sampler2D road_mask : filter_linear_mipmap, repeat_disable;
uniform vec4 mask_bounds;
uniform bool road_overlay = false;
uniform sampler2D soil_color : source_color, filter_linear_mipmap, repeat_enable;
uniform sampler2D road_color : source_color, filter_linear_mipmap, repeat_enable;
uniform sampler2D snow_color : source_color, filter_linear_mipmap, repeat_enable;
varying vec3 wp;
void vertex(){wp=(MODEL_MATRIX*vec4(VERTEX,1.0)).xyz;}
void fragment(){
 vec3 mask=texture(road_mask,(wp.xz-mask_bounds.xy)/mask_bounds.zw).rgb;
 float road=mask.r*(1.0-smoothstep(0.4,2.0,abs(wp.y-(mask.b*140.0-5.0))));
 if(road_overlay && road<0.65){discard;}
 vec3 soil=texture(soil_color,wp.xz*0.58).rgb*vec3(0.78,0.88,0.60);
 vec3 gravel=texture(road_color,wp.xz*0.63).rgb*0.91;
 vec3 snow=texture(snow_color,wp.xz*0.22).rgb*0.82;
 ALBEDO=mix(mix(soil,gravel,road),snow,smoothstep(78.0,110.0,wp.y));
 ROUGHNESS=0.96; SPECULAR=0.1;
}
"""
