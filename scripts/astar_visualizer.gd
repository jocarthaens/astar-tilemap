extends CharacterBody2D

@export var Agent_Radius := 8.0
@export var speed = 256.0: set = set_speed, get = get_speed
@export var show_visited_heatmap: bool = false;

var pathfinder: AStarTerrain = AStarTerrain.new()

var start_point: Vector2
var target_point: Vector2

signal pathfinding_finished(debug_data: Dictionary, shortest_path: Array, visited_tiles: Array)
var path := []
var visit_path := []
var debug_data := {}

func set_speed(value: float):
	speed = value
func get_speed() -> float:
	return speed

@onready var tilemap: TileMapLayer = get_node("../TileMapper")

func _ready() -> void:
	pathfinder.calculation_finished.connect(on_calculation_finished);

func _physics_process(_delta):
	movement()
	move_and_slide()
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		path_find()
		queue_redraw()
	elif Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		queue_redraw()

func movement():
	velocity = Input.get_vector("ui_left","ui_right","ui_up","ui_down")
	velocity = velocity * speed


func path_find():
	var start = self.get_global_position()
	start_point = start
	var target: Vector2 = get_global_mouse_position()
	target_point = target
	#print("Start: ",start)
	#print("Target: ",target)
	pathfinder.pathfind(self, Agent_Radius, self.get_collision_mask(), start, target, tilemap)
	path = pathfinder.final_path
	visit_path = pathfinder.visited_path

func on_calculation_finished():
	path = pathfinder.final_path
	visit_path = pathfinder.visited_path
	debug_data = pathfinder.debug_data;
	pathfinding_finished.emit(debug_data, path, visit_path)




func _draw():
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		var visit_color := Color8(255,255,0,32)
		var agent_dimensions: Vector2 = Vector2(Agent_Radius,Agent_Radius);
		
		for point: Vector2 in visit_path:
			var rect := Rect2(point - start_point - agent_dimensions, agent_dimensions * 2)
			if show_visited_heatmap == true:
				draw_rect(rect, visit_color)
			draw_rect(rect, Color.BLACK, false, 1.0)
		for i in path.size():
			var prev = i-1 if i-1 > 0 else 0
			draw_circle(path[i] - start_point, 4, Color.BLUE)
			draw_line(path[prev] - start_point, path[i] - start_point, Color.BLACK)
		draw_circle(target_point - start_point, 4, Color.GREEN)
	


func _discretize_position(node_position: Vector2) -> Vector2:
	var vector_components := [node_position.x, node_position.y]
	var length := Agent_Radius * 2.0
	
	for i in vector_components.size():
		var value := vector_components[i]/length as float
		vector_components[i] = (floori(value) * Agent_Radius * 2) + Agent_Radius
	
	return Vector2(vector_components[0], vector_components[1])
