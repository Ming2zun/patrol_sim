extends Node
class_name SimLink

signal packet_received(packet: Dictionary)
signal link_changed(connected: bool)

var server := TCPServer.new()
var peer: StreamPeerTCP
var rx := PackedByteArray()
var tx := PackedByteArray()
var authenticated := false
var wants_pointcloud := false
var wants_imu := false
const MAX_TX := 2097152
var token := ""
var port := 9090
var bind_address := "0.0.0.0"
var accepted_at := 0
var error_text := ""
const MAX_BUFFER := 262144
const MAX_LINE := 65536

func configure(settings: Dictionary) -> void:
	port = int(settings.get("port", 9090))
	bind_address = str(settings.get("bind_address", "0.0.0.0"))
	token = str(settings.get("token", ""))
	var error := server.listen(port, bind_address)
	if error != OK:
		error_text = "端口监听失败：%s" % error_string(error)
		push_error(error_text)

func disconnect_peer() -> void:
	if peer != null:
		peer.disconnect_from_host()
	peer = null
	rx.clear()
	tx.clear()
	var was_authenticated := authenticated
	authenticated = false
	wants_pointcloud = false
	wants_imu = false
	if was_authenticated:
		link_changed.emit(false)

func send_packet(packet: Dictionary) -> void:
	if peer == null or not authenticated:
		return
	var data := (JSON.stringify(packet) + "\n").to_utf8_buffer()
	if packet.get("type")=="pointcloud" and tx.size()>262144:
		return
	if tx.size() + data.size() > MAX_TX:
		disconnect_peer()
		return
	tx.append_array(data)

func _process(_delta: float) -> void:
	while server.is_connection_available():
		var candidate := server.take_connection()
		if peer != null:
			candidate.disconnect_from_host()
		else:
			peer = candidate
			peer.set_no_delay(true)
			accepted_at = Time.get_ticks_msec()
	if peer == null:
		return
	peer.poll()
	if peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		disconnect_peer()
		return
	if not authenticated and Time.get_ticks_msec() - accepted_at > 5000:
		disconnect_peer()
		return
	var available := peer.get_available_bytes()
	if available > 0:
		if rx.size() + available > MAX_BUFFER:
			disconnect_peer()
			return
		var result := peer.get_data(available)
		if result[0] != OK:
			disconnect_peer()
			return
		rx.append_array(result[1])
	var handled := 0
	while handled < 32:
		var end := rx.find(10)
		if end < 0:
			break
		if end > MAX_LINE:
			disconnect_peer()
			return
		var line := rx.slice(0, end).get_string_from_utf8()
		rx = rx.slice(end + 1)
		var parsed = JSON.parse_string(line)
		if not parsed is Dictionary:
			disconnect_peer()
			return
		if not authenticated:
			if parsed.get("type") != "hello" or int(parsed.get("version", 0)) != 1 or str(parsed.get("token", "")) != token or token.is_empty():
				disconnect_peer()
				return
			authenticated = true
			var capabilities = parsed.get("capabilities",[])
			wants_pointcloud = capabilities is Array and "pointcloud" in capabilities
			wants_imu = capabilities is Array and "imu_sample" in capabilities
			send_packet({"type":"hello", "version":1, "server":"alpine-patrol", "units":"SI", "coordinates":"ROS REP-103", "capabilities":["cmd_vel","odom","imu","clock","camera_jpeg","pointcloud","imu_sample","reset","pause","estop","mode"]})
			link_changed.emit(true)
		else:
			packet_received.emit(parsed)
		handled += 1
	if rx.size() > MAX_LINE:
		disconnect_peer()
		return
	if not tx.is_empty() and peer != null:
		var result := peer.put_partial_data(tx)
		if result[0] != OK:
			disconnect_peer()
		elif int(result[1]) > 0:
			tx = tx.slice(int(result[1]))

func _exit_tree() -> void:
	disconnect_peer()
	server.stop()
