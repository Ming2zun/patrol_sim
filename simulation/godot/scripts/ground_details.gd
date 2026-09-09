extends Node3D
# Near-field physical grains and small grass tufts. Deterministic 12 m cells,
# generated one cell per physics tick and removed beyond the local neighborhood.
var target: Node3D
var cells := {}
var pending: Array[Vector2i] = []
var last_cell := Vector2i(999999,999999)
var pebble_mesh: SphereMesh
var tuft_mesh: ArrayMesh
var tuft_variants: Array[ArrayMesh] = []

func setup(vehicle: Node3D) -> void:
	target = vehicle
	pebble_mesh = SphereMesh.new()
	pebble_mesh.radius = 1.0
	pebble_mesh.height = 2.0
	pebble_mesh.radial_segments = 6
	pebble_mesh.rings = 2
	var stone := StandardMaterial3D.new()
	stone.albedo_color = Color(0.52,0.49,0.42)
	stone.vertex_color_use_as_albedo = true
	stone.roughness = 0.97
	pebble_mesh.material = stone
	tuft_mesh = _make_tuft(0)
	tuft_variants = [tuft_mesh,_make_tuft(1),_make_tuft(2)]

func _physics_process(_delta: float) -> void:
	if target == null or DisplayServer.get_name() == "headless":
		return
	var current := Vector2i(floori(target.global_position.x/12.0),floori(target.global_position.z/12.0))
	if current != last_cell:
		last_cell = current
		pending.clear()
		for key in cells.keys():
			if absi(key.x-current.x)>2 or absi(key.y-current.y)>2:
				cells[key].queue_free()
				cells.erase(key)
		for dx in range(-2,3):
			for dz in range(-2,3):
				var key := current+Vector2i(dx,dz)
				if not cells.has(key):
					pending.append(key)
		pending.sort_custom(func(a: Vector2i,b: Vector2i): return a.distance_squared_to(current)<b.distance_squared_to(current))
	if not pending.is_empty():
		_build_cell(pending.pop_front())

func _build_cell(key: Vector2i) -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = hash(str(key)+"alpine_grains_v3")
	var holder := Node3D.new()
	holder.position = Vector3(key.x*12.0,0,key.y*12.0)
	add_child(holder)
	cells[key] = holder
	var stones: Array[Transform3D] = []
	var tufts: Array[Transform3D] = []
	for i in range(360):
		var x := holder.position.x+rng.randf()*12.0
		var z := holder.position.z+rng.randf()*12.0
		var query := PhysicsRayQueryParameters3D.create(Vector3(x,target.global_position.y+35,z),Vector3(x,target.global_position.y-40,z),1)
		var hit := get_world_3d().direct_space_state.intersect_ray(query)
		if hit.is_empty() or hit["normal"].y<0.65:
			continue
		var collider: Object = hit["collider"]
		if not collider.has_meta("surface_kind"):
			continue
		var p: Vector3 = hit["position"]
		var on_road: bool = collider.get_meta("surface_kind") == "road"
		var normal: Vector3 = hit["normal"]
		var right := normal.cross(Vector3.FORWARD).normalized()
		var basis := Basis(right,normal,right.cross(normal)).orthonormalized()
		if on_road or i%5==0:
			if rng.randf() < (0.85 if p.y>95.0 else 0.45):
				continue
			var radius := rng.randf_range(0.012,0.045) if on_road else rng.randf_range(0.025,0.07)
			var scale := Vector3(radius,radius*rng.randf_range(0.35,0.7),radius*rng.randf_range(0.7,1.3))
			stones.append(Transform3D(basis.rotated(normal,rng.randf()*TAU).scaled_local(scale),p-holder.position+normal*scale.y*0.25))
		elif p.y<85 and rng.randf()<0.78:
			var scale := rng.randf_range(0.65,1.45)
			tufts.append(Transform3D(basis.rotated(normal,rng.randf()*TAU).scaled_local(Vector3.ONE*scale),p-holder.position))
	_batch(holder,pebble_mesh,stones,true,rng)
	_batch(holder,tuft_variants[posmod(key.x+key.y*7,3)],tufts,false,rng)

func _batch(holder: Node3D, mesh: Mesh, transforms: Array[Transform3D], stones: bool, rng: RandomNumberGenerator) -> void:
	if transforms.is_empty():
		return
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = stones
	mm.mesh = mesh
	mm.instance_count = transforms.size()
	for i in range(transforms.size()):
		mm.set_instance_transform(i,transforms[i])
		if stones:
			var shade := rng.randf_range(0.65,1.15)
			mm.set_instance_color(i,Color(shade,shade,shade,1))
	var node := MultiMeshInstance3D.new()
	node.multimesh = mm
	node.visibility_range_end = 38
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON if stones else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	holder.add_child(node)

func _make_tuft(variant: int) -> ArrayMesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for i in range(7):
		var angle := i*2.39996
		var side := Vector3(cos(angle),0,sin(angle))*(0.024 if variant==1 else 0.012)
		var bend := Vector3(sin(angle),0,cos(angle))*0.065
		var height := (0.23 if variant==2 else 0.13)+0.018*(i%4)
		var a := -side
		var b := side
		var c := Vector3.UP*height*0.55+bend*0.25-side*0.48
		var d := Vector3.UP*height*0.55+bend*0.25+side*0.48
		var e := Vector3.UP*height+bend
		for p in [a,b,c,b,d,c,c,d,e]:
			st.set_color(Color(0.20,0.24,0.075).lerp((Color(0.37,0.29,0.12) if variant==2 else Color(0.34,0.42,0.15)),p.y/height))
			st.add_vertex(p)
	st.generate_normals()
	var mesh := st.commit()
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.backlight_enabled = true
	mat.backlight = Color(0.18,0.22,0.08)
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat.roughness = 0.95
	mesh.surface_set_material(0,mat)
	return mesh
