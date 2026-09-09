extends Node3D
var instance_count := 0
var collider_count := 0
var species_counts := {}
var world: Node3D
var bank := {}
var lidar_obstacles: StaticBody3D

func build(source: Dictionary, owner_world: Node3D, obstacles: StaticBody3D) -> void:
	world = owner_world
	lidar_obstacles = StaticBody3D.new()
	lidar_obstacles.name = "LidarVegetationVolumes"
	lidar_obstacles.collision_layer = 4
	lidar_obstacles.collision_mask = 0
	add_child(lidar_obstacles)
	var groups := {}
	var rng := RandomNumberGenerator.new()
	rng.seed = 50509
	for group in source["instances"]:
		if group["kind"] != "tree":
			continue
		for values in group["transforms"]:
			var t: Transform3D = world._decode_transform(values)
			var draw := rng.randf()
			var kind := "pine"
			if t.origin.y>87:
				kind = "snow_pine" if draw<0.5 else "snow_fir"
			elif t.origin.y<52:
				kind = "broadleaf" if draw<0.24 else ("twin_broadleaf" if draw<0.47 else ("fir" if draw<0.67 else ("mountain_pine" if draw<0.85 else "pine")))
			else:
				kind = "fir" if draw<0.40 else ("mountain_pine" if draw<0.75 else "pine")
			if not groups.has(kind):
				groups[kind] = []
			t.basis = t.basis.scaled_local(Vector3(rng.randf_range(0.88,1.14),rng.randf_range(0.86,1.13),rng.randf_range(0.88,1.14)))
			groups[kind].append(t)
	var sites: Array = JSON.parse_string(FileAccess.get_file_as_string("res://assets/vegetation/sites.json"))
	for site in sites:
		var kind: String = site["kind"]
		if not groups.has(kind):
			groups[kind] = []
		groups[kind].append(world._decode_transform(site["transform"]))
	for kind in groups:
		var data := _load_species(kind)
		var transforms: Array = groups[kind]
		species_counts[kind] = transforms.size()
		instance_count += transforms.size()
		var cells := {}
		for t in transforms:
			var key := Vector2i(floori(t.origin.x/24.0),floori(t.origin.z/24.0))
			if not cells.has(key):
				cells[key] = []
			cells[key].append(t)
			_add_lidar_volume(kind,t,data)
		for key in cells:
			for record in data["records"]:
				_batch(record["mesh"],cells[key],record["transform"],0.0,22.0,not str(kind).contains("shrub"))
			_batch(data["far"],cells[key],Transform3D.IDENTITY,22.0,180.0 if str(kind).contains("shrub") else 520.0,false)
		if not str(kind).contains("shrub"):
			for t in transforms:
				var scale: Vector3 = t.basis.get_scale()
				var cylinder := CylinderShape3D.new()
				cylinder.radius = maxf(0.10,0.12*maxf(scale.x,scale.z))
				cylinder.height = 3.2*scale.y
				var shape := CollisionShape3D.new()
				shape.shape = cylinder
				shape.position = t.origin+Vector3.UP*cylinder.height*0.5
				obstacles.add_child(shape)
				collider_count += 1
	print("VEGETATION_BANK ",JSON.stringify(species_counts))

func _batch(mesh: Mesh, transforms: Array, local: Transform3D, begin: float, end: float, shadows: bool) -> void:
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = mesh
	mm.instance_count = transforms.size()
	for i in range(transforms.size()):
		mm.set_instance_transform(i,transforms[i]*local)
	var node := MultiMeshInstance3D.new()
	node.multimesh = mm
	node.visibility_range_begin = begin
	node.visibility_range_end = end
	node.lod_bias = 0.6
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON if shadows else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(node)

func _load_species(kind: String) -> Dictionary:
	if bank.has(kind):
		return bank[kind]
	var names := {"pine":"dense_pine","fir":"fir_tree_01","broadleaf":"tree_small_02","snow_pine":"dense_pine","snow_fir":"fir_tree_01","snow_shrub":"shrub_03","shrub_01":"shrub_01","shrub_03":"shrub_03","shrub_02":"shrub_02","shrub_04":"shrub_04","shrub_fern":"fern_02","mountain_pine":"pine_tree_01","twin_broadleaf":"twin_broadleaf"}
	var asset: String = names[kind]
	var packed: PackedScene = load("res://assets/vegetation/"+asset+".glb")
	var prototype := packed.instantiate()
	var records: Array = []
	world._collect_records(prototype,Transform3D.IDENTITY,records)
	for record in records:
		var mesh: Mesh = record["mesh"].duplicate()
		record["mesh"] = mesh
		for i in range(mesh.get_surface_count()):
			var original = mesh.surface_get_material(i)
			if not original is StandardMaterial3D:
				continue
			if kind.begins_with("snow"):
				var mat := ShaderMaterial.new()
				mat.shader = load("res://shaders/snow_foliage.gdshader")
				mat.set_shader_parameter("tint",original.albedo_color)
				mat.set_shader_parameter("textured",original.albedo_texture!=null)
				if original.albedo_texture!=null:
					mat.set_shader_parameter("leaf_tex",original.albedo_texture)
				mesh.surface_set_material(i,mat)
			else:
				var mat: StandardMaterial3D = original.duplicate()
				mat.roughness = maxf(mat.roughness,0.85)
				mat.metallic_specular = 0.2
				mat.cull_mode = BaseMaterial3D.CULL_DISABLED
				if mat.transparency!=BaseMaterial3D.TRANSPARENCY_DISABLED:
					mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
				mesh.surface_set_material(i,mat)
	prototype.free()
	var image_name := asset
	if kind=="snow_fir":
		image_name = "fir_snow"
	elif kind=="snow_shrub":
		image_name = "snow_shrub"
	elif kind=="snow_pine":
		image_name = "dense_pine_snow"
	var info: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://assets/vegetation/"+image_name+".json"))
	var quad := QuadMesh.new()
	quad.size = Vector2(info["width"],info["height"])
	quad.center_offset = Vector3(info["center"][0],info["center"][1],info["center"][2])
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = load("res://assets/vegetation/"+image_name+".png")
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
	mat.alpha_scissor_threshold = 0.27
	mat.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y
	mat.billboard_keep_scale = true
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	quad.material = mat
	var data := {"records":records,"far":quad,"height":info["tree_height"],"width":info["width"]}
	bank[kind] = data
	return data

func _add_lidar_volume(kind: String, t: Transform3D, data: Dictionary) -> void:
	# Lightweight foliage envelopes affect lidar only; they do not block driving.
	var shape := CollisionShape3D.new()
	var scale: Vector3 = t.basis.get_scale()
	var radius_scale := maxf(scale.x,scale.z)
	if kind.contains("shrub"):
		var sphere := SphereShape3D.new()
		sphere.radius = clampf(float(data["width"])*0.32,0.22,0.65)*radius_scale
		shape.shape = sphere
		shape.position = t.origin+Vector3.UP*float(data["height"])*scale.y*0.45
	elif kind.contains("broadleaf"):
		var sphere := SphereShape3D.new()
		sphere.radius = 1.15*radius_scale
		shape.shape = sphere
		shape.position = t.origin+Vector3.UP*3.25*scale.y
	else:
		var capsule := CapsuleShape3D.new()
		capsule.radius = 0.60*radius_scale
		capsule.height = maxf(capsule.radius*2,2.75*scale.y)
		shape.shape = capsule
		shape.position = t.origin+Vector3.UP*2.9*scale.y
	lidar_obstacles.add_child(shape)
