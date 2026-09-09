extends Node3D
class_name AlpineWorld

var manifest: Dictionary
var collider_count := 0
var instance_count := 0
var gravel_material: StandardMaterial3D
var far_tree_mesh: Mesh
var paint_material: ShaderMaterial
var vegetation: Node3D

func build() -> void:
	manifest = JSON.parse_string(FileAccess.get_file_as_string("res://assets/manifest.json"))
	for path in manifest["terrain"]:
		_add_scene(str(path), true)
	for path in manifest["roads"]:
		_add_scene(str(path), true)
	for path in manifest["grass"]:
		if str(path).ends_with("grass_0.glb") or str(path).ends_with("grass_1.glb"):
			continue
		var grass := _add_scene(str(path), false)
		for mesh in _mesh_nodes(grass):
			mesh.visibility_range_end = 150.0
			mesh.visibility_range_end_margin = 25.0
			mesh.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			for i in range(mesh.mesh.get_surface_count()):
				var mat = mesh.get_active_material(i)
				if mat is BaseMaterial3D:
					mat.cull_mode = BaseMaterial3D.CULL_DISABLED
					mat.albedo_color = Color(0.23,0.27,0.10)
					mat.roughness = 0.95
	var obstacles := StaticBody3D.new()
	obstacles.name = "SimplifiedObstacleCollisions"
	obstacles.collision_layer = 1
	obstacles.collision_mask = 2
	add_child(obstacles)
	vegetation = preload("res://scripts/vegetation_bank.gd").new()
	add_child(vegetation)
	vegetation.build(manifest,self,obstacles)
	instance_count += vegetation.instance_count
	collider_count += vegetation.collider_count
	var fallback_scene: PackedScene = load("res://assets/prototypes/tree_lod.glb")
	var fallback_node := fallback_scene.instantiate()
	var fallback_records: Array = []
	_collect_records(fallback_node,Transform3D.IDENTITY,fallback_records)
	if not fallback_records.is_empty():
		far_tree_mesh = fallback_records[0]["mesh"]
		_refine_prototype(far_tree_mesh,"tree")
		far_tree_mesh = _tree_impostor()
	fallback_node.free()
	for group in manifest["instances"]:
		if group["kind"] == "tree":
			continue
		var packed: PackedScene = load("res://assets/" + str(group["file"]))
		var prototype := packed.instantiate()
		var records: Array = []
		_collect_records(prototype, Transform3D.IDENTITY, records)
		var transforms: Array[Transform3D] = []
		for values in group["transforms"]:
			transforms.append(_decode_transform(values))
		for record in records:
			_refine_prototype(record["mesh"],str(group["kind"]))
			if group["kind"] == "tree" and record["mesh"].get_faces().size() < 6000:
				record["mesh"] = far_tree_mesh
			var cells := {}
			for transform in transforms:
				var p := transform.origin
				var key := Vector2i(floori(p.x / 24.0), floori(p.z / 24.0))
				if not cells.has(key):
					cells[key] = []
				cells[key].append(transform * record["transform"])
			for key in cells:
				var multimesh := MultiMesh.new()
				multimesh.transform_format = MultiMesh.TRANSFORM_3D
				multimesh.mesh = record["mesh"]
				multimesh.instance_count = cells[key].size()
				for i in range(cells[key].size()):
					multimesh.set_instance_transform(i, cells[key][i])
				var batch := MultiMeshInstance3D.new()
				batch.multimesh = multimesh
				batch.name = "Instanced_" + str(group["kind"])
				var high_tree: bool = group["kind"] == "tree" and record["mesh"].get_faces().size()>30000
				batch.visibility_range_end = 25 if high_tree else (350 if group["kind"]=="rock" else 420)
				batch.visibility_range_end_margin = 10
				batch.lod_bias = 0.3
				if group["kind"] == "tree" and not high_tree:
					batch.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
				add_child(batch)
				if high_tree and far_tree_mesh != null:
					var far_mm := MultiMesh.new()
					far_mm.transform_format = MultiMesh.TRANSFORM_3D
					far_mm.mesh = far_tree_mesh
					far_mm.instance_count = cells[key].size()
					for i in range(cells[key].size()):
						far_mm.set_instance_transform(i,cells[key][i])
					var far_batch := MultiMeshInstance3D.new()
					far_batch.multimesh = far_mm
					far_batch.visibility_range_begin = 25
					far_batch.visibility_range_end = 420
					far_batch.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
					add_child(far_batch)
		if not records.is_empty():
			var aabb: AABB = records[0]["mesh"].get_aabb()
			for transform in transforms:
				var scale := transform.basis.get_scale()
				var collision := CollisionShape3D.new()
				if group["kind"] == "tree":
					var cylinder := CylinderShape3D.new()
					cylinder.radius = maxf(0.10, 0.12 * maxf(scale.x, scale.z))
					cylinder.height = maxf(0.5, aabb.size.y * scale.y * 0.7)
					collision.shape = cylinder
					collision.position = transform.origin + Vector3.UP * cylinder.height * 0.5
				else:
					var radius := maxf(aabb.size.x * scale.x, aabb.size.z * scale.z) * 0.37
					if radius < 0.45:
						collision.free()
						continue
					var sphere := SphereShape3D.new()
					sphere.radius = radius
					collision.shape = sphere
					collision.position = transform * aabb.get_center()
				obstacles.add_child(collision)
				collider_count += 1
		instance_count += transforms.size()
		prototype.free()
	_setup_light()

func _add_scene(path: String, collision: bool) -> Node3D:
	var packed: PackedScene = load("res://assets/" + path)
	var node: Node3D = packed.instantiate()
	add_child(node)
	for mesh in _mesh_nodes(node):
		for i in range(mesh.mesh.get_surface_count()):
			var mat = mesh.get_active_material(i)
			if mat is StandardMaterial3D:
				var label: String = mat.resource_name.to_lower()
				if label.contains("terrain pbr"):
					mesh.set_surface_override_material(i,_terrain_material(mat))
				elif label.contains("gravel") or label.contains("grit"):
					mesh.set_surface_override_material(i,_road_material())
					mesh.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
				elif label.contains("snow") or label.contains("shoulder"):
					mesh.set_surface_override_material(i,_pbr("snow_02",0.35,Color(0.88,0.9,0.92)))

	if collision:
		for mesh in _mesh_nodes(node):
			if mesh.mesh == null:
				continue
			var body := StaticBody3D.new()
			body.set_meta("surface_kind","road" if path.contains("roads") else "terrain")
			body.collision_layer = 1
			body.collision_mask = 2
			var shape := CollisionShape3D.new()
			shape.shape = mesh.mesh.create_trimesh_shape()
			mesh.add_child(body)
			body.add_child(shape)
			collider_count += 1
	return node

func _mesh_nodes(node: Node) -> Array[MeshInstance3D]:
	var result: Array[MeshInstance3D] = []
	if node is MeshInstance3D:
		result.append(node)
	for child in node.get_children():
		result.append_array(_mesh_nodes(child))
	return result

func _collect_records(node: Node, parent: Transform3D, result: Array) -> void:
	var transform := parent
	if node is Node3D:
		transform = parent * node.transform
	if node is MeshInstance3D and node.mesh != null:
		result.append({"mesh":node.mesh, "transform":transform})
	for child in node.get_children():
		_collect_records(child, transform, result)

func _decode_transform(v: Array) -> Transform3D:
	return Transform3D(Basis(Vector3(v[0],v[1],v[2]),Vector3(v[3],v[4],v[5]),Vector3(v[6],v[7],v[8])), Vector3(v[9],v[10],v[11]))

var pbr_cache := {}

func _pbr(asset: String, scale: float, tint: Color) -> StandardMaterial3D:
	var key := asset+str(scale)+str(tint)
	if pbr_cache.has(key):
		return pbr_cache[key]
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = load("res://assets/pbr/"+asset+"_color.jpg")
	mat.albedo_color = tint
	mat.normal_enabled = true
	mat.normal_texture = load("res://assets/pbr/"+asset+"_normal.jpg")
	mat.normal_scale = 0.8
	mat.roughness_texture = load("res://assets/pbr/"+asset+"_roughness.jpg")
	mat.roughness_texture_channel = BaseMaterial3D.TEXTURE_CHANNEL_RED
	mat.roughness = 0.96
	mat.metallic_specular = 0.28
	mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	mat.uv1_triplanar = true
	mat.uv1_world_triplanar = true
	mat.uv1_scale = Vector3.ONE*scale
	pbr_cache[key] = mat
	return mat

func _terrain_material(_original: StandardMaterial3D) -> ShaderMaterial:
	return _world_surface()

func _refine_prototype(mesh: Mesh, kind: String) -> void:
	for i in range(mesh.get_surface_count()):
		var original = mesh.surface_get_material(i)
		if not original is StandardMaterial3D:
			continue
		if kind == "rock":
			mesh.surface_set_material(i,_pbr("rock_boulder_dry",0.22,Color(0.49,0.51,0.50)))
		else:
			var mat: StandardMaterial3D = original.duplicate()
			mat.roughness = 0.95
			mat.metallic_specular = 0.18
			if not mat.resource_name.to_lower().contains("bark"):
				mat.albedo_color = mat.albedo_color.lerp(Color(0.13,0.18,0.08),0.3)
			mesh.surface_set_material(i,mat)

func refine_vehicle(model: Node3D) -> void:
	# Wheels have been reparented, so include the entire aligned visual.
	for node in _mesh_nodes(model.get_parent()):
		for i in range(node.mesh.get_surface_count()):
			var original = node.get_active_material(i)
			if not original is StandardMaterial3D:
				continue
			var mat: StandardMaterial3D = original.duplicate()
			var label: String = mat.resource_name.to_lower()
			if label.contains("white"):
				if paint_material == null:
					paint_material = ShaderMaterial.new()
					paint_material.shader = load("res://shaders/vehicle_paint.gdshader")
					paint_material.set_shader_parameter("dust_tex",load("res://assets/pbr/gravel_stones_color.jpg"))
				node.set_surface_override_material(i,paint_material)
				continue
			elif label.contains("rubber") or label.contains("tread"):
				mat.albedo_color = Color(0.042,0.038,0.032)
				mat.roughness = 0.91
				mat.normal_enabled = true
				mat.normal_texture = load("res://assets/pbr/rocky_trail_02_normal.jpg")
				mat.normal_scale = 0.12
				mat.uv1_triplanar = true
				mat.uv1_scale = Vector3.ONE*9
			elif label.contains("aluminum"):
				mat.metallic = 0.82
				mat.roughness = 0.24
			elif label.contains("optical") or label.contains("sapphire"):
				mat.albedo_color = Color(0.025,0.045,0.055)
				mat.metallic = 0.65
				mat.roughness = 0.08
				mat.clearcoat_enabled = true
				mat.clearcoat = 0.8
			node.set_surface_override_material(i,mat)

func _setup_light() -> void:
	var world_environment := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var material := PanoramaSkyMaterial.new()
	material.panorama = load("res://assets/pbr/alpine_sky.hdr")
	material.energy_multiplier = 1.0
	sky.sky_material = material
	environment.sky = sky
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	environment.ambient_light_energy = 0.85
	environment.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	environment.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	environment.tonemap_exposure = 1.05
	environment.fog_enabled = true
	environment.fog_light_color = Color(0.67,0.72,0.74)
	environment.fog_light_energy = 0.65
	environment.fog_density = 0.00055
	world_environment.environment = environment
	add_child(world_environment)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-32,-118,0)
	sun.light_color = Color(1.0,0.96,0.89)
	sun.light_energy = 1.18
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 100
	sun.shadow_bias = 0.035
	sun.shadow_normal_bias = 0.65
	add_child(sun)


func _road_material() -> ShaderMaterial:
	if pbr_cache.has("feathered_road_overlay"):
		return pbr_cache["feathered_road_overlay"]
	var mat: ShaderMaterial = _world_surface().duplicate()
	mat.set_shader_parameter("road_overlay",true)
	pbr_cache["feathered_road_overlay"] = mat
	return mat

func _tree_impostor() -> Mesh:
	var mesh := QuadMesh.new()
	mesh.size = Vector2(2.484,4.968)
	mesh.center_offset = Vector3(0.031,2.3,-0.017)
	var mat := StandardMaterial3D.new()
	mat.albedo_texture = load("res://assets/prototypes/tree_impostor.png")
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
	mat.alpha_scissor_threshold = 0.38
	mat.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y
	mat.billboard_keep_scale = true
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mesh.material = mat
	return mesh

func _world_surface() -> ShaderMaterial:
	if pbr_cache.has("unified_world_surface"):
		return pbr_cache["unified_world_surface"]
	var mat := ShaderMaterial.new()
	mat.shader = load("res://shaders/world_surface.gdshader")
	var info: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://assets/pbr/road_ground_blend.json"))
	mat.set_shader_parameter("mask_bounds",Vector4(info["x"],info["z"],info["width"],info["depth"]))
	mat.set_shader_parameter("road_mask",load("res://assets/pbr/road_ground_blend.png"))
	for pair in [["soil","forest_ground_04"],["road","rocky_trail_02"],["rock","rock_boulder_dry"],["snow","snow_02"]]:
		for suffix in ["color","normal"]:
			mat.set_shader_parameter(pair[0]+"_"+suffix,load("res://assets/pbr/"+pair[1]+"_"+suffix+".jpg"))
	mat.set_shader_parameter("grass_color",load("res://assets/pbr/aerial_grass_rock_color.jpg"))
	pbr_cache["unified_world_surface"] = mat
	return mat

func update_vehicle_pose(pose: Transform3D) -> void:
	if paint_material != null:
		paint_material.set_shader_parameter("vehicle_inverse",pose.affine_inverse())
