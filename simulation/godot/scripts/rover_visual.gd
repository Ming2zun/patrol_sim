extends Node3D
class_name RoverVisualRig
var wheels: Array[Node3D] = []
var knuckles: Array[Node3D] = []
var centers: Array[Vector3] = []
var previous: Array[Vector3] = []
var angles: Array[float] = []
var steering_angles: Array[float] = []
var fronts: Array[bool] = []
var contact_error: Array[float] = []
var radius := 0.48
var wheelbase := 1.56
var track := 1.88
var rear_offset := 0.78
var initialized := false
var wheel_parts := 0
var links: Array[Dictionary] = []
var tyre_materials: Array[ShaderMaterial] = []

func build(model: Node3D) -> void:
	var parts: Array[MeshInstance3D] = []
	_collect(model,parts)
	for part in parts:
		if str(part.name).to_lower().contains("tire carcass"):
			var center := to_local(part.to_global(part.mesh.get_aabb().get_center()))
			var knuckle := Node3D.new()
			knuckle.name = "SteeringKnuckle_"+str(wheels.size())
			add_child(knuckle)
			knuckle.position = center
			var axle := Node3D.new()
			axle.name = "RollingAxle_"+str(wheels.size())
			knuckle.add_child(axle)
			knuckles.append(knuckle)
			wheels.append(axle)
			centers.append(center)
			previous.append(knuckle.global_position)
			angles.append(0.0)
			steering_angles.append(0.0)
			contact_error.append(0.0)
	var zmin := INF
	var zmax := -INF
	var xmin := INF
	var xmax := -INF
	for c in centers:
		zmin = minf(zmin,c.z)
		zmax = maxf(zmax,c.z)
		xmin = minf(xmin,c.x)
		xmax = maxf(xmax,c.x)
	wheelbase = zmax-zmin
	track = xmax-xmin
	rear_offset = zmax
	for c in centers:
		fronts.append(c.z < (zmin+zmax)*0.5)
	for part in parts:
		var label := str(part.name)
		if label.begins_with("Suspension |") or label.begins_with("Drivetrain | differential axle"):
			part.visible = false
		if not label.begins_with("Wheel"):
			continue
		var p := to_local(part.to_global(part.mesh.get_aabb().get_center()))
		var nearest := 0
		for i in range(centers.size()):
			if p.distance_squared_to(centers[i]) < p.distance_squared_to(centers[nearest]):
				nearest = i
		part.reparent(wheels[nearest],true)
		wheel_parts += 1
	for i in range(wheels.size()):
		var tyre_mat := ShaderMaterial.new()
		tyre_mat.shader = preload("res://shaders/tire_rotation_marks.gdshader")
		tyre_mat.set_shader_parameter("wheel_inverse",wheels[i].global_transform.affine_inverse())
		tyre_materials.append(tyre_mat)
		var tyre_parts: Array[MeshInstance3D] = []
		_collect(wheels[i],tyre_parts)
		for part in tyre_parts:
			for surface in range(part.mesh.get_surface_count()):
				var original := part.get_active_material(surface)
				if original != null and (original.resource_name.to_lower().contains("rubber") or original.resource_name.to_lower().contains("tread")):
					part.set_surface_override_material(surface,tyre_mat)
		# Articulated lower/upper arms, damper and steering tie rod.
		for spec in [[-0.14,-0.08,0.045],[0.14,-0.08,0.045],[0.0,0.10,0.033],[0.0,0.34,0.052]]:
			_add_link(i,Vector3(signf(centers[i].x)*0.43,centers[i].y+spec[1],centers[i].z+spec[0]),Vector3(-signf(centers[i].x)*0.13,0.0,0.0),spec[2],false)
		if fronts[i]:
			_add_link(i,Vector3(signf(centers[i].x)*0.37,centers[i].y+0.055,centers[i].z+0.16),Vector3(-signf(centers[i].x)*0.17,0.03,0.15),0.022,true)
	print("WHEEL_RIG Ackermann ",wheels.size()," wheels, ",wheel_parts," rolling parts, wheelbase=",wheelbase," track=",track)

func _add_link(index: int, anchor: Vector3, endpoint: Vector3, thickness: float, tie: bool) -> void:
	var node := MeshInstance3D.new()
	node.name = "SteeringTieRod" if tie else "ArticulatedSuspensionLink"
	var mesh := CylinderMesh.new()
	mesh.top_radius = thickness
	mesh.bottom_radius = thickness
	mesh.height = 1.0
	mesh.radial_segments = 10
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(.31,.34,.37) if tie else Color(.11,.13,.15)
	mat.metallic = .75
	mat.roughness = .35
	mesh.material = mat
	node.mesh = mesh
	add_child(node)
	links.append({"node":node,"index":index,"anchor":anchor,"endpoint":endpoint})

func _collect(node: Node, result: Array[MeshInstance3D]) -> void:
	if node is MeshInstance3D:
		result.append(node)
	for child in node.get_children():
		_collect(child,result)

func reset_history() -> void:
	initialized = false
	for i in range(wheels.size()):
		knuckles[i].position = centers[i]
		knuckles[i].rotation.y = 0
		steering_angles[i] = 0

func ground_normal(fallback: Vector3) -> Vector3:
	var front := Vector3.ZERO
	var rear := Vector3.ZERO
	var left := Vector3.ZERO
	var right := Vector3.ZERO
	for i in range(centers.size()):
		var p := to_global(centers[i])
		var q := PhysicsRayQueryParameters3D.create(p+Vector3.UP*.8,p-Vector3.UP*1.2,1)
		var hit := get_world_3d().direct_space_state.intersect_ray(q)
		if hit.is_empty():
			return fallback
		var contact: Vector3 = hit["position"]
		if fronts[i]: front += contact
		else: rear += contact
		if centers[i].x<0: left += contact
		else: right += contact
	var normal := (rear-front).cross(right-left).normalized()
	return normal if normal.y>.55 else fallback

func step(delta: float, grounded: bool, central_steer: float = 0.0) -> void:
	var curvature := tan(central_steer)/wheelbase
	for i in range(wheels.size()):
		var steer := atan2(wheelbase*curvature,1.0+centers[i].x*curvature) if fronts[i] else 0.0
		steering_angles[i] = steer
		knuckles[i].rotation.y = steer
		var p := to_global(centers[i])
		var forward := -knuckles[i].global_basis.z.normalized()
		if initialized and grounded:
			var distance := (p-previous[i]).dot(forward)
			if absf(distance)<1.0:
				var delta_angle := -distance/radius
				contact_error[i] = absf(distance+radius*sin(delta_angle))/delta
				angles[i] += delta_angle
		previous[i] = p
		var q := PhysicsRayQueryParameters3D.create(p+Vector3.UP*.7,p-Vector3.UP*1.0,1)
		var hit := get_world_3d().direct_space_state.intersect_ray(q)
		var offset := 0.0
		if not hit.is_empty():
			offset = clampf(to_local(hit["position"]+Vector3.UP*radius).y-centers[i].y,-.22,.22)
		knuckles[i].position.y = lerpf(knuckles[i].position.y,centers[i].y+offset,1-exp(-delta*22))
		wheels[i].rotation.x = wrapf(angles[i],-PI,PI)
		tyre_materials[i].set_shader_parameter("wheel_inverse",wheels[i].global_transform.affine_inverse())
	for record in links:
		var node: MeshInstance3D = record["node"]
		var a: Vector3 = record["anchor"]
		var b: Vector3 = to_local(knuckles[record["index"]].to_global(record["endpoint"]))
		var axis := b-a
		node.position = (a+b)*.5
		node.quaternion = Quaternion(Vector3.UP,axis.normalized())
		node.scale = Vector3(1,axis.length(),1)
	initialized = true

func diagnostics() -> Dictionary:
	var offsets: Array = []
	var positions: Array = []
	for i in range(wheels.size()):
		offsets.append(knuckles[i].position.y-centers[i].y)
		positions.append([centers[i].x,centers[i].y,centers[i].z])
	return {"wheel_count":wheels.size(),"rotating_parts":wheel_parts,"radius_m":radius,"angle_rad":angles,"suspension_m":offsets,"contact_error_mps":contact_error,"steering_rad":steering_angles,"front_wheels":fronts,"centers_godot":positions,"wheelbase_m":wheelbase,"track_m":track,"steering_model":"ackermann"}
