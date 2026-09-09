extends SceneTree
var failures := 0
var checks := []
var lidar: Node3D

func _initialize() -> void:
	_run.call_deferred()

func check(label: String, passed: bool) -> void:
	checks.append({"name":label,"passed":passed})
	if not passed:
		failures += 1
		push_error(label)

func box(parent: Node3D, position: Vector3, size: Vector3, layer: int) -> void:
	var body := StaticBody3D.new()
	body.collision_layer = layer
	body.position = position
	var shape := CollisionShape3D.new()
	var cube := BoxShape3D.new()
	cube.size = size
	shape.shape = cube
	body.add_child(shape)
	parent.add_child(body)

func point(ring: int, column: int) -> Vector3:
	var offset := (ring*180+column)*24
	return Vector3(lidar.data.decode_float(offset),lidar.data.decode_float(offset+4),lidar.data.decode_float(offset+8))

func _run() -> void:
	var world := Node3D.new()
	root.add_child(world)
	box(world,Vector3(0,-2.05,0),Vector3(100,.1,100),1)
	box(world,Vector3(0,0,-10),Vector3(4,8,1),1)
	box(world,Vector3(0,0,-2),Vector3(2,2,1),2)
	box(world,Vector3(5,0,0),Vector3(1,2,1),4)
	var link := preload("res://scripts/net_server.gd").new()
	world.add_child(link)
	lidar = preload("res://scripts/lidar36.gd").new()
	world.add_child(lidar)
	lidar.configure({"lidar_hz":10,"lidar_azimuth_samples":180,"lidar_position_ros":[0,0,0]})
	await physics_frame
	await physics_frame
	for i in range(20):
		lidar.step(1.0/200,i/200.0,1,Transform3D.IDENTITY,false,link)
	check("organized binary includes all 36 rings",lidar.data.size()==36*180*24 and lidar.completed_scans==1)
	var front := point(23,90)
	check("ROS +X hits front wall at 9.5 m and excludes vehicle layer",absf(front.x-9.5)<.01 and absf(front.y)<.01)
	var ground := point(0,90)
	check("downward channels hit ground at ROS Z=-2 m",absf(ground.z+2)<.01 and ground.x>4.0 and ground.x<4.5)
	var side := point(23,45)
	check("lidar sees vegetation-only collision layer",absf(side.y+4.5)<.01 and absf(side.x)<.01)
	var ring_ok := true
	for index in range(36*180):
		if lidar.data.decode_u16(index*24+16)!=index/180:
			ring_ok = false
	check("ring field matches organized row",ring_ok)
	var expected := deg_to_rad(-25.0+40.0*23/35)
	check("vertical angles are evenly spaced across 36 lines",absf(atan2(front.z,front.x)-expected)<.0001)
	lidar.reset_scan()
	var rotated := Transform3D(Basis(Vector3.UP,PI/2),Vector3.ZERO)
	for i in range(20):
		lidar.step(1.0/200,1.0+i/200.0,2,rotated,false,link)
	var turned := point(23,45)
	check("body rotation maps world wall to local ROS -Y",absf(turned.x)<.01 and absf(turned.y+9.5)<.01)
	check("reset publishes new epoch",lidar.last_packet["epoch"]==2)
	var count: int = lidar.completed_scans
	for i in range(12):
		lidar.step(1.0/200,1.1,2,rotated,true,link)
	check("pause does not generate new scans",lidar.completed_scans==count)
	var report := {"success":failures==0,"checks":checks}
	var file := FileAccess.open("D:/ruanjian111111111/blender/others/_1/logs/lidar36_geometry_results.json",FileAccess.WRITE)
	file.store_string(JSON.stringify(report,"  "))
	print("LIDAR_GEOMETRY_TEST ",JSON.stringify(report))
	quit(0 if failures==0 else 1)
