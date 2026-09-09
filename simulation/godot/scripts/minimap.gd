extends Control
class_name PatrolMinimap
var cached_line := PackedVector2Array()
var cached_size := Vector2(-1,-1)
var route: PackedVector2Array
var vehicle_position := Vector2.ZERO
var heading := 0.0
var minimum := Vector2(-250,-400)
var extent := Vector2(500,460)

func configure(points: Array[Vector3]) -> void:
	var maximum := Vector2(-INF,-INF)
	minimum = Vector2(INF,INF)
	for p in points:
		var q := Vector2(p.x,p.z)
		minimum = minimum.min(q)
		maximum = maximum.max(q)
		route.append(q)
	minimum -= Vector2(18,18)
	extent = maximum - minimum + Vector2(18,18)
	cached_size = Vector2(-1,-1)
	queue_redraw()

func mapped(p: Vector2) -> Vector2:
	var scale := minf((size.x-28)/extent.x,(size.y-28)/extent.y)
	return (p-minimum-extent*0.5)*scale + size*0.5

func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO,size),Color(0.035,0.064,0.09,0.92))
	if route.size()<2:
		return
	if cached_size != size:
		cached_size = size
		cached_line.clear()
		for p in route:
			var point := mapped(p)
			if cached_line.is_empty() or point.distance_squared_to(cached_line[-1]) >= 0.5625:
				cached_line.append(point)
		if cached_line[-1] != mapped(route[-1]):
			cached_line.append(mapped(route[-1]))
	draw_polyline(cached_line,Color(0.28,0.72,0.78),2.0,true)
	var center := mapped(vehicle_position)
	draw_circle(center,5,Color(1.0,0.75,0.32))
	var forward := Vector2(-sin(heading),-cos(heading))
	draw_line(center,center+forward*15,Color(1,0.85,0.48),3,true)
