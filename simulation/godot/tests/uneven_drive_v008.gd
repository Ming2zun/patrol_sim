extends SceneTree
var sim: Node3D
var checks := []
var worst_track := 0.0
var traveled := 0.0
var peak_speed := 0.0
var suspension_min := 1.0
var suspension_max := -1.0
var attitude_change := 0.0
var trajectory := []
var rough_deltas: Array[float] = []
var base_path := "D:/ruanjian111111111/blender/others/_1/"
func _initialize() -> void:
	_run.call_deferred()
func check(label: String, ok: bool, value = null) -> void:
	checks.append({"name":label,"passed":ok,"detail":value})
func meshes(n: Node) -> Array[MeshInstance3D]:
	var out: Array[MeshInstance3D] = []
	if n is MeshInstance3D: out.append(n)
	for c in n.get_children(): out.append_array(meshes(c))
	return out
func _run() -> void:
	sim = load("res://main.tscn").instantiate()
	root.add_child(sim)
	await create_timer(.5).timeout
	for node in sim.world.find_children("*","StaticBody3D",true,false):
		if node.has_meta("surface_kind"):
			var name_lower: String = str(node.get_parent().name).to_lower()
			if node.get_meta("surface_kind")=="terrain": node.collision_layer |= 32
			elif not (name_lower.contains("snow banks") or name_lower.contains("safety") or name_lower.contains("direction arrows")): node.collision_layer |= 16
	var reference := Node3D.new()
	root.add_child(reference)
	for path in ["roads","terrain_0","terrain_1","terrain_2"]:
		var n: Node3D = load("res://assets/world/"+path+".glb").instantiate()
		reference.add_child(n)
		n.visible = false
		for m in meshes(n):
			var body := StaticBody3D.new()
			body.collision_layer = 8
			var name_lower := str(m.name).to_lower()
			if path!="roads": body.collision_layer |= 128
			elif not (name_lower.contains("snow banks") or name_lower.contains("safety") or name_lower.contains("direction arrows")): body.collision_layer |= 64
			body.collision_mask = 0
			var shape := CollisionShape3D.new()
			shape.shape = m.mesh.create_trimesh_shape()
			m.add_child(body)
			body.add_child(shape)
	await physics_frame
	await physics_frame
	var info: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(base_path+"logs/v008_road_geometry.json"))
	var negative := 0
	for pit in info["pits_ros_world"]:
		var p := Vector3(pit[0],pit[2],pit[1])
		var space := sim.get_world_3d().direct_space_state
		var old: Dictionary = space.intersect_ray(PhysicsRayQueryParameters3D.create(p+Vector3.UP,p-Vector3.UP*2,8))
		var now: Dictionary = space.intersect_ray(PhysicsRayQueryParameters3D.create(p+Vector3.UP,p-Vector3.UP*2,1))
		if not old.is_empty() and not now.is_empty():
			var change: float = now["position"].y-old["position"].y
			rough_deltas.append(change)
			if change<-.025: negative+=1
	check("potholes change actual raycast collision heights",negative>75,{"depressed_pits":negative,"samples":rough_deltas.size(),"minimum":rough_deltas.min(),"maximum":rough_deltas.max()})
	var shoulder_samples := 0
	var worst_gap_change := 0.0
	var largest_gap := 0.0
	for index in range(20,sim.route.size()-20,10):
		var point: Vector3=sim.route[index]
		var tangent: Vector3=(sim.route[index+2]-sim.route[index-2]).normalized()
		var side:=Vector3(-tangent.z,0,tangent.x).normalized()
		for direction in [-1,1]:
			var probe:=point+side*float(direction)*2.4
			var hits := []
			for layer in [16,32,64,128]:
				hits.append(sim.get_world_3d().direct_space_state.intersect_ray(PhysicsRayQueryParameters3D.create(probe+Vector3.UP*.3,probe-Vector3.UP*.7,layer)))
			if hits.all(func(h): return not h.is_empty()):
				var new_gap: float=hits[0]["position"].y-hits[1]["position"].y
				var old_gap: float=hits[2]["position"].y-hits[3]["position"].y
				worst_gap_change=maxf(worst_gap_change,new_gap-old_gap)
				largest_gap=maxf(largest_gap,new_gap)
				shoulder_samples+=1
	check("roughness does not create a road-to-grass shoulder step",shoulder_samples>100 and worst_gap_change<.06 and largest_gap<.06,{"samples":shoulder_samples,"maximum_added_gap_m":worst_gap_change,"largest_road_terrain_clearance_m":largest_gap})
	reference.queue_free()
	if "--seam-only" in OS.get_cmdline_user_args():
		var success := true
		for c in checks: success=success and c["passed"]
		FileAccess.open(base_path+"logs/v008_shoulder_results.json",FileAccess.WRITE).store_string(JSON.stringify({"success":success,"checks":checks},"  "))
		print("SHOULDERS ",JSON.stringify(checks))
		quit(0 if success else 1)
		return
	sim.set_mode("route")
	var previous: Vector3 = sim.rover.global_position
	var previous_normal: Vector3 = sim.visual.global_basis.y
	var start_time: float = sim.sim_time
	while sim.sim_time-start_time<45.0:
		await create_timer(.1).timeout
		var p: Vector3 = sim.rover.global_position
		traveled += p.distance_to(previous)
		previous = p
		peak_speed = maxf(peak_speed,absf(sim.speed))
		var distance := INF
		for j in range(sim.route_index-70,sim.route_index+20):
			var a: Vector3 = sim.route[posmod(j,sim.route.size())]
			var b: Vector3 = sim.route[posmod(j+1,sim.route.size())]
			var av := Vector2(a.x,a.z)
			var bv := Vector2(b.x,b.z)
			var point := Vector2(p.x,p.z)
			var t := clampf((point-av).dot(bv-av)/maxf((bv-av).length_squared(),.00001),0,1)
			distance = minf(distance,point.distance_to(av.lerp(bv,t)))
		worst_track = maxf(worst_track,distance)
		trajectory.append({"t":sim.sim_time-start_time,"x":p.x,"z":p.z,"speed":sim.speed,"offset":distance,"steering":sim.steering,"index":sim.route_index})
		var rig: Dictionary = sim.wheel_rig.diagnostics()
		for offset in rig["suspension_m"]:
			suspension_min=minf(suspension_min,offset)
			suspension_max=maxf(suspension_max,offset)
		attitude_change=maxf(attitude_change,sim.visual.global_basis.y.angle_to(previous_normal))
		previous_normal=sim.visual.global_basis.y
	check("36 kmh route cruise accelerates beyond former 10 kmh limit",peak_speed>7.5,peak_speed*3.6)
	check("route drive advances over uneven terrain",traveled>140,traveled)
	check("Ackermann cruise follows road corridor",worst_track<2.05,worst_track)
	check("suspension moves through uneven road contacts",suspension_max-suspension_min>.06,[suspension_min,suspension_max])
	check("body attitude responds to uneven road",attitude_change>.001,attitude_change)
	# A parked steering close-up displays the actual rig without changing physics.
	sim.paused=true
	sim.steering=.35
	sim.wheel_rig.step(.05,true,.35)
	sim.hud_canvas.visible=false
	sim.camera_distance=4.4
	sim.camera_yaw=PI+.7
	sim.camera_pitch=.20
	await create_timer(1.0).timeout
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png(base_path+"renders/godot_ackermann_v008.png")
	var success := true
	for c in checks: success=success and c["passed"]
	var report := {"success":success,"checks":checks}
	FileAccess.open(base_path+"logs/v008_route_trajectory.json",FileAccess.WRITE).store_string(JSON.stringify(trajectory))
	FileAccess.open(base_path+"logs/v008_road_drive_results.json",FileAccess.WRITE).store_string(JSON.stringify(report,"  "))
	print("ROAD_DRIVE ",JSON.stringify(report))
	quit(0 if success else 1)
