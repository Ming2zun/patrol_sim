extends SceneTree
func _initialize() -> void:
	run.call_deferred()
func run() -> void:
	var sim: Node3D=load("res://main.tscn").instantiate()
	root.add_child(sim)
	await create_timer(3.0).timeout
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png("D:/ruanjian111111111/blender/others/_1/renders/godot_junction_v008.png")
	quit()
