extends SceneTree
func _initialize() -> void:
	for file in ["rover.glb","livery_lidar36_v006.glb"]:
		var scene: PackedScene = load("res://assets/vehicle/"+file)
		var node := scene.instantiate()
		inspect(node)
		node.free()
	quit()
func inspect(node: Node) -> void:
	var label := str(node.name).to_lower()
	if label.contains("lidar") or label.contains("livery") or label.contains("identification"):
		print(node.name," type=",node.get_class())
	for child in node.get_children():
		inspect(child)
