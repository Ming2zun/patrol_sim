extends Node3D
# Ideal 36-ring snapshot; ray queries are amortized over physics frames.
const RINGS := 36
var columns := 180
var rate_hz := 5.0
var minimum := 0.25
var maximum := 60.0
var vertical_min := -25.0
var vertical_max := 15.0
var mount := Vector3(0,2.55,-.335)
var directions := PackedVector3Array()
var data := PackedByteArray()
var cursor := 0
var active := false
var elapsed := 1.0
var stamp := 0.0
var scan_epoch := 0
var sequence := 0
var snapshot := Transform3D.IDENTITY
var last_packet := {}
var valid_returns := 0
var completed_scans := 0
var scan_cpu_ms := 0.0
var last_scan_cpu_ms := 0.0

func configure(settings: Dictionary) -> void:
	columns = clampi(int(settings.get("lidar_azimuth_samples",180)),90,720)
	rate_hz = clampf(float(settings.get("lidar_hz",5)),1.0,10.0)
	minimum = maxf(0.05,float(settings.get("lidar_range_min_m",0.25)))
	maximum = clampf(float(settings.get("lidar_range_m",60)),minimum+1.0,120.0)
	vertical_min = clampf(float(settings.get("lidar_vertical_min_deg",-25)),-60,0)
	vertical_max = clampf(float(settings.get("lidar_vertical_max_deg",15)),1,60)
	var offset: Array = settings.get("lidar_position_ros",[.335,0.0,2.55])
	mount = Vector3(-float(offset[1]),float(offset[2]),-float(offset[0]))
	for ring in range(RINGS):
		var elevation := deg_to_rad(lerpf(vertical_min,vertical_max,float(ring)/(RINGS-1)))
		for column in range(columns):
			var azimuth := -PI+TAU*column/columns
			directions.append(Vector3(-sin(azimuth)*cos(elevation),sin(elevation),-cos(azimuth)*cos(elevation)))

func reset_scan() -> void:
	active = false
	cursor = 0
	elapsed = 1.0
	last_packet.clear()

func step(delta: float, sim_time: float, epoch: int, pose: Transform3D, paused: bool, link: SimLink) -> void:
	if paused:
		return
	elapsed += delta
	if not active and elapsed+0.000001 >= 1.0/rate_hz:
		elapsed = 0.0
		active = true
		cursor = 0
		stamp = sim_time
		scan_epoch = epoch
		snapshot = pose*Transform3D(Basis.IDENTITY,mount)
		data.resize(RINGS*columns*24)
		valid_returns = 0
		scan_cpu_ms = 0.0
	if not active:
		return
	if scan_epoch != epoch:
		reset_scan()
		return
	var started := Time.get_ticks_usec()
	var space := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.new()
	query.collision_mask = 5 # Ground/structures plus lidar-only vegetation volumes.
	query.hit_back_faces = true
	var budget := ceili(float(RINGS*columns)/(float(Engine.physics_ticks_per_second)/rate_hz))
	var end := mini(cursor+budget,directions.size())
	for index in range(cursor,end):
		var direction := snapshot.basis*directions[index]
		query.from = snapshot.origin+direction*minimum
		query.to = snapshot.origin+direction*maximum
		var hit := space.intersect_ray(query)
		var local := Vector3(NAN,NAN,NAN)
		var intensity := 0.0
		if not hit.is_empty():
			var distance := snapshot.origin.distance_to(hit["position"])
			local = directions[index]*distance
			intensity = clampf(absf(direction.dot(hit["normal"]))*exp(-distance/100.0),0.0,1.0)
			valid_returns += 1

		var offset := index*24
		data.encode_float(offset,-local.z)
		data.encode_float(offset+4,-local.x)
		data.encode_float(offset+8,local.y)
		data.encode_float(offset+12,intensity)
		data.encode_u16(offset+16,index/columns)
		data.encode_u16(offset+18,0)
		data.encode_float(offset+20,0.0)
	cursor = end
	scan_cpu_ms += float(Time.get_ticks_usec()-started)/1000.0
	if cursor == directions.size():
		active = false
		sequence += 1
		completed_scans += 1
		last_scan_cpu_ms = scan_cpu_ms
		last_packet = {"type":"pointcloud","version":1,"seq":sequence,"epoch":scan_epoch,"stamp":stamp,"frame_id":"lidar36_link","position_ros":[-mount.z,-mount.x,mount.y],"height":RINGS,"width":columns,"point_step":24,"row_step":columns*24,"is_bigendian":false,"is_dense":false,"encoding":"xyz_irt_f32_u16_le","range_min":minimum,"range_max":maximum,"vertical_min_deg":vertical_min,"vertical_max_deg":vertical_max,"scan_time":1.0/rate_hz,"snapshot":true,"valid_returns":valid_returns}
		if link.authenticated and link.wants_pointcloud:
			var packet := last_packet.duplicate()
			packet["data"] = Marshalls.raw_to_base64(data)
			link.send_packet(packet)

func diagnostics() -> Dictionary:
	return {"rings":RINGS,"columns":columns,"hz":rate_hz,"range_max":maximum,"position_ros":[-mount.z,-mount.x,mount.y],"valid_returns":last_packet.get("valid_returns",0),"completed_scans":completed_scans,"cpu_ms_per_scan":last_scan_cpu_ms}
