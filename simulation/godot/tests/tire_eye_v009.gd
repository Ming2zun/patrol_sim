extends SceneTree
var sim: Node3D
var checks := []
const BASE := "D:/ruanjian111111111/blender/others/_1/"
func _initialize() -> void:
	run.call_deferred()
func check(label: String, ok: bool) -> void:
	checks.append({"name":label,"passed":ok})
func click_eye() -> void:
	var point: Vector2=sim.eye_button.get_global_rect().get_center()
	var move:=InputEventMouseMotion.new()
	move.position=point
	root.push_input(move,true)
	Input.flush_buffered_events()
	await process_frame
	for down in [true,false]:
		var e:=InputEventMouseButton.new()
		e.position=point
		e.button_index=MOUSE_BUTTON_LEFT
		e.pressed=down
		root.push_input(e,true)
		Input.flush_buffered_events()
		await create_timer(.1).timeout
func capture(name: String) -> void:
	await RenderingServer.frame_post_draw
	root.get_texture().get_image().save_png(BASE+"renders/"+name+".png")
func run() -> void:
	sim=load("res://main.tscn").instantiate()
	root.add_child(sim)
	await create_timer(2.0).timeout
	check("eye is visible in full interface",sim.eye_button.is_visible_in_tree() and sim.hud_canvas.visible)
	await capture("godot_ui_visible_v009")
	await click_eye()
	check("actual eye click hides HUD canvas",not sim.hud_canvas.visible)
	check("map is inside hidden HUD",sim.map.get_canvas_layer_node()==sim.hud_canvas)
	check("eye remains available to restore interface",sim.eye_button.is_visible_in_tree() and sim.interface_canvas.visible)
	await capture("godot_ui_hidden_v009")
	await click_eye()
	check("second actual eye click restores HUD",sim.hud_canvas.visible)
	var e:=InputEventKey.new()
	e.physical_keycode=KEY_H
	e.pressed=true
	root.push_input(e,true)
	Input.flush_buffered_events()
	await create_timer(.1).timeout
	e=InputEventKey.new();e.physical_keycode=KEY_H;e.pressed=false;root.push_input(e,true)
	check("H shortcut shares eye visibility state",not sim.hud_canvas.visible and sim.eye_button.tooltip_text.begins_with("显示"))
	sim.camera_distance=4.2
	sim.camera_yaw=PI/2+.15
	sim.camera_pitch=.12
	await create_timer(.7).timeout
	await capture("godot_tire_marks_still_v009")
	var before: float=sim.wheel_rig.angles[0]
	sim.set_mode("route")
	await create_timer(1.8).timeout
	check("marked tyres roll with actual vehicle movement",abs(sim.wheel_rig.angles[0]-before)>1.0)
	var aligned:=true
	for i in range(4):
		var point:=Vector3(.20,.37,0)
		var inverse: Transform3D=sim.wheel_rig.tyre_materials[i].get_shader_parameter("wheel_inverse")
		aligned=aligned and (inverse*sim.wheel_rig.wheels[i].to_global(point)).distance_to(point)<.0001
	check("all four painted patterns stay fixed to rolling wheel coordinates",aligned)
	await capture("godot_tire_marks_rolling_v009")
	var success:=true
	for c in checks:success=success and c["passed"]
	FileAccess.open(BASE+"logs/v009_visual_ui_checks.json",FileAccess.WRITE).store_string(JSON.stringify({"success":success,"checks":checks},"  "))
	print("VISUAL_UI_CHECKS ",JSON.stringify(checks))
	quit(0 if success else 1)
