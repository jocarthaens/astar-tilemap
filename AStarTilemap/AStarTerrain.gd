extends Node
class_name AStarTerrain

var _requester: CollisionObject2D
var _radius: float
var _tilemap: TileMapLayer
var _obstacle_layers: int

var int_factor: int = 1#100_000
var max_iterations: int = 1000
var max_g_score: float = 16_000#_000_000
#var _max_path_count: int = 1_000#_000

var _open_list: Array = [] # An array of partially-sorted list of PathNodes 
var _node_data: Dictionary = {} # Stores discrete position as key and its associated PathNode as value

var chosen_heuristic_mode: int = 0
var use_diagonal := true

signal calculation_finished;

var visited_path := []
var final_path := []
var debug_data: Dictionary = {}

func _reset_debug_data():
	debug_data["is_successful"] = false;
	debug_data["travel_cost"] = 0.0;
	debug_data["execution_time_(usec)"] = 0;
	debug_data["open_list_time_(usec)"] = 0;
	debug_data["neighbor_check_time_(usec)"] = 0;
	debug_data["path_distance"] = 0.0;
	debug_data["path_node_count"] = 0;
	debug_data["post_smooth_path_distance"] = 0.0;
	debug_data["post_smooth_path_node_count"] = 0;




## Call this method to determine and generate the shortest path from start position to target position
func pathfind(agent: CollisionObject2D, agent_radius: float, obstacle_collision_layers: int,
	start_position: Vector2, target_position: Vector2, tile_map: TileMapLayer):
	
	visited_path.clear()
	final_path.clear()
	_reset_debug_data();
	
	_requester = agent
	_radius = agent_radius
	_obstacle_layers = obstacle_collision_layers
	
	_tilemap = tile_map
	
	chosen_heuristic_mode = 0
	use_diagonal = true
	
	#var start_time := Time.get_ticks_usec()
	var path = _find_path(start_position, target_position)
	#var end_time := Time.get_ticks_usec()
	#print(end_time - start_time, " usecs\n")
	calculation_finished.emit();
	
	return path

## Overridable method for providing terrain cost on a specified position
func get_terrain_cost(coordinate: Vector2):
	var terrain_cost: float = int_factor as float
	var tile_size = _tilemap.tile_set.tile_size
	var tile_data = _tilemap.get_cell_tile_data(Vector2i(coordinate.x/tile_size.x, coordinate.y/tile_size.y)) if tile_size else null
	if tile_data != null:
		var terrain_data = tile_data.get_terrain()
		match terrain_data:
			0: # GRASS
				terrain_cost *= 1
			1: # MUD
				terrain_cost *= 3
			2: # ICE
				terrain_cost *= 0.5
			_: # OTHER (INCLUDES ROCK)
				terrain_cost = INF
	elif tile_data == null:
		terrain_cost = INF
	return terrain_cost









# Main method that calculates the shortest path from source to destination
func _find_path(start_position: Vector2, target_position: Vector2):
	var discrete_start := _discretize_position(start_position)
	var start_id := str(discrete_start)
	var discrete_target := _discretize_position(target_position)
	
	_add_to_open_list(discrete_start)
	_node_data[start_id].g_score = 0.0
	_node_data[start_id].f_score = _heuristic(discrete_start, discrete_target, chosen_heuristic_mode)
	
	var open_time_total: float = 0.0
	var neigh_time_total: float = 0.0
	var n: int = 0
	var find_path_start: int= Time.get_ticks_usec()
	var find_path_end: int
	while not _open_list.is_empty() and n < max_iterations:
		var open_start = Time.get_ticks_usec()
		var current = _pop_node_position()
		var open_end = Time.get_ticks_usec()
		open_time_total += open_end - open_start
		var curr_id := str(current)
		var curr_data = _node_data[curr_id]
		
		if current == discrete_target or curr_data.g_score >= max_g_score:
			visited_path.append(current)
			
			find_path_end = Time.get_ticks_usec()
			
			debug_data["is_successful"] = true;
			debug_data["travel_cost"] = curr_data.g_score;
			debug_data["execution_time_(usec)"] = find_path_end - find_path_start;
			debug_data["open_list_time_(usec)"] = open_time_total;
			debug_data["neighbor_check_time_(usec)"] = neigh_time_total;
			
			return _reconstruct_path(current)
		
		# MAIN ISSUE STARTS HERE: (SLUGGISH GENERATION TIME DUE TO MORE EXPANDED NODES)
		var neigh_start = Time.get_ticks_usec()
		var current_neighbors = _find_neighbors(current, use_diagonal)
		for neighbor in current_neighbors:
			var neigh_id = str(neighbor)
			var has_data := _node_data.has(neigh_id)
			var new_g_score: float = curr_data.g_score + _cost(current, neighbor)
			var old_g_score: float = _node_data[neigh_id].g_score if has_data else INF
			var in_open := _in_open_list(neighbor)
			if not has_data or new_g_score < old_g_score:
				if not has_data or in_open == false:
					_add_to_open_list(neighbor)
				var neigh_data = _node_data[neigh_id]
				var h_value := _heuristic(neighbor, discrete_target, chosen_heuristic_mode)
				neigh_data.precursor = current
				neigh_data.g_score = new_g_score
				neigh_data.f_score = new_g_score + h_value

			visited_path.append(neighbor)
		visited_path.append(current)
		n+=1
		var neigh_end = Time.get_ticks_usec()
		neigh_time_total += neigh_end - neigh_start
		# MAIN ISSUE ENDS HERE: (SLUGGISH GENERATION TIME DUE TO MORE EXPANDED NODES)
	
	find_path_end = Time.get_ticks_usec();
	
	debug_data["is_successful"] = false;
	debug_data["travel_cost"] = -1;
	debug_data["execution_time_(usec)"] = find_path_end - find_path_start;
	debug_data["open_list_time_(usec)"] = open_time_total;
	debug_data["neighbor_check_time_(usec)"] = neigh_time_total;
	
	return _reconstruct_path(discrete_start)

# Reconstructing the newly-found shortest path by reversing the node path, 
# and then applying post-smoothing to the said path
func _reconstruct_path(node) -> Array[Vector2]:
	var path: Array[Vector2] = []
	var node_id = str(node)
	var distance := 0.0
	
	while node != null:
		path.push_front(node)
		node = _node_data[node_id].precursor
		if node != null:
			distance += (_node_data[node_id].position - _node_data[node_id].precursor).length()
		node_id = str(node)
	
	debug_data["path_distance"] = distance;
	debug_data["path_node_count"] = path.size();
	
	_clear_data()
	
	return _path_smoothing(path) #path





# Functions that aims to smoothen the shortest path by removing unnecessary nodes (Post-Smoothing)


# Function that smoothens the newly-found shortest path 
func _path_smoothing(path: Array[Vector2]) -> Array[Vector2]:
	var smooth_path: Array[Vector2] = []
	var sight := path[0]
	var distance := 0.0
	smooth_path.append(sight)
	
	for i in range(1, path.size()):
		if not (_line_of_sight(sight, path[i]) and _trace_terrain(sight, path[i])) and path[i-1] != path[0]:
			distance += (sight - path[i-1]).length()
			smooth_path.append(path[i-1])
			sight = path[i-1]
		if i == path.size() - 1:
			distance += (sight - path[i]).length()
			smooth_path.append(path[i])
	
	final_path.append_array(smooth_path)
	
	debug_data["post_smooth_path_distance"] = distance;
	debug_data["post_smooth_path_node_count"] = smooth_path.size();
	
	return smooth_path.duplicate()

# Helper function that checks for the line of sight between 2 tile positions.
# Checks for obstructing obstacles in between 2 positions.
func _line_of_sight(a: Vector2, b: Vector2) -> bool:
	var space_state: PhysicsDirectSpaceState2D = _requester.get_world_2d().get_direct_space_state()
	var query_shape := PhysicsShapeQueryParameters2D.new()
	var exclude_lists := []
	exclude_lists.append(_requester)
	
	query_shape.set_collision_mask(_obstacle_layers)
	query_shape.set_exclude(exclude_lists)
	
	var shape_RID := PhysicsServer2D.rectangle_shape_create()
	var margin := 0.0#0.025
	var half_length := (_radius * 2 - margin) / 2.0
	var half_width :=  (_radius * 2 - margin) / 2.0
	var dimensions := Vector2(half_length,half_width)
	PhysicsServer2D.shape_set_data(shape_RID, dimensions)
	var shape_transform := Transform2D(0, a)
	query_shape.set_shape_rid(shape_RID)
	query_shape.set_transform(shape_transform)
	query_shape.set_motion(b - a)
	
	space_state.cast_motion(query_shape)
	var collision_points: Array[Vector2] = space_state.collide_shape(query_shape,1)
	
	PhysicsServer2D.free_rid(shape_RID)
	
	return collision_points.is_empty()

# Helper function that checks if 2 tile positions and other tiles in between are in the same
# terrain. Utilizes the modified version of Bresenham's line algorithm taken from link below:
# https://dedu.fr/projects/bresenham/
func _trace_terrain(a: Vector2, b: Vector2) -> bool:
	var is_same_terrain: bool = false
	var tile_size = _tilemap.tile_set.tile_size
	a = Vector2(int(a.x / tile_size.x), int(a.y / tile_size.y)) if tile_size else Vector2.ZERO
	b = Vector2(int(b.x / tile_size.x), int(b.y / tile_size.y)) if tile_size else Vector2.ZERO
	var a_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(a.x), int(a.y))) if tile_size else null
	var a_id = float(a_tile_data.get_terrain()) if a_tile_data != null else INF
	
	if a_tile_data != null:
		var x_i: int
		var y_i: int
		var ddx: int
		var ddy: int
		var error: int
		var error_prev: int
		var x := int(a.x)
		var y := int(a.y)
		var dx := int((b.x - a.x))
		var dy := int((b.y - a.y))
		
		if dx < 0:
			x_i = -1
			dx = -dx
		else:
			x_i = 1
		
		if dy < 0:
			y_i = -1
			dy = -dy
		else:
			y_i = 1
		
		ddx = 2 * dx
		ddy = 2 * dy
		
		var c := Vector2(x, y)
		var c_id = INF
		var d := Vector2(x, y)
		var d_id = INF
		var e := Vector2(x, y)
		var e_id = INF
		
		if ddx >= ddy:
			error = dx
			error_prev = error
			
			for i in range(0,dx):
				x += x_i
				error += ddy
				if error > ddx:
					y += y_i
					error -= ddx
					if (error + error_prev < ddx):
						c = Vector2(x, y - y_i)
						var c_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(c.x), int(c.y)))
						c_id = float(c_tile_data.get_terrain()) if c_tile_data != null else INF
					elif(error + error_prev > ddx):
						d = Vector2(x - x_i, y)
						var d_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(d.x), int(d.y)))
						d_id = float(d_tile_data.get_terrain())if d_tile_data != null else INF
					else:
						c = Vector2(x, y - y_i)
						var c_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(c.x), int(c.y)))
						c_id = float(c_tile_data.get_terrain()) if c_tile_data != null else INF
						d = Vector2(x - x_i, y)
						var d_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(d.x), int(d.y)))
						d_id = float(d_tile_data.get_terrain())if d_tile_data != null else INF
				e = Vector2(x, y)
				var e_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(e.x), int(e.y)))
				e_id = float(e_tile_data.get_terrain()) if e_tile_data != null else INF
				error_prev = error
				
				if a_id != INF and e_id != INF:
					is_same_terrain = a_id == e_id and (a_id == c_id or a_id == d_id) if (c_id != INF or d_id != INF) else a_id == e_id
				if is_same_terrain == false:
					break
		
		else:
			error = dy
			error_prev = error
			
			for i in range(0,dy):
				y += y_i
				error += ddx
				if error > ddy:
					x += x_i
					error -= ddy
					if (error + error_prev < ddy):
						c = Vector2(x - x_i, y)
						var c_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(c.x), int(c.y)))
						c_id = float(c_tile_data.get_terrain()) if c_tile_data != null else INF
					elif(error + error_prev > ddy):
						d = Vector2(x, y - y_i)
						var d_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(d.x), int(d.y)))
						d_id = float(d_tile_data.get_terrain())if d_tile_data != null else INF
					else:
						c = Vector2(x - x_i, y)
						var c_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(c.x), int(c.y)))
						c_id = float(c_tile_data.get_terrain()) if c_tile_data != null else INF
						d = Vector2(x, y - y_i)
						var d_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(d.x), int(d.y)))
						d_id = float(d_tile_data.get_terrain())if d_tile_data != null else INF
				e = Vector2(x, y)
				var e_tile_data = _tilemap.get_cell_tile_data(Vector2i(int(e.x), int(e.y)))
				e_id = float(e_tile_data.get_terrain()) if e_tile_data != null else INF
				error_prev = error
				
				if a_id != INF and e_id != INF:
					is_same_terrain = a_id == e_id and (a_id == c_id or a_id == d_id) if (c_id != INF or d_id != INF) else a_id == e_id
				if is_same_terrain == false:
					break
		
	return is_same_terrain










# Functions that aid in finding the shortest path from start position to end position

# Helper function that discretizes inputted position using the radius
func _discretize_position(node_position: Vector2) -> Vector2:
	var vector_components := [node_position.x, node_position.y]
	var length := _radius * 2.0
	
	for i in vector_components.size():
		var value := vector_components[i]/length as float
		vector_components[i] = (floori(value) * _radius * 2) + _radius
	
	return Vector2(vector_components[0], vector_components[1])

# Function that calculates the nearest neighbors from a discretized position
func _find_neighbors(location: Vector2, with_diagonals: bool) -> Array[Vector2]:
	var size := _radius * 2
	var neighbor_cells: Array[Vector2] = []
	var neighbor_array: Array = [
		# Horizontal
		[location + Vector2(size, 0), # Right
		location + Vector2(-size, 0), # Left
		], 
		# Vertical
		[location + Vector2(0, size), # Down
		location + Vector2(0, -size), # Up
		],
		# Diagonal
		[location + Vector2(size, size), # RightDown
		location + Vector2(size, -size), # RightUp
		location + Vector2(-size, size), # LeftDown
		location + Vector2(-size, -size), # LeftUp
		]
	]
	
	neighbor_cells.append_array(neighbor_array[0])
	neighbor_cells.append_array(neighbor_array[1])
	if with_diagonals == true:
		neighbor_cells.append_array(neighbor_array[2])
	neighbor_array.clear()
	
	#randomize()
	#neighbor_cells.shuffle()
	
	var viable_neighbors: Array[Vector2] = []
	for cell in neighbor_cells:
		var tile_data = _tilemap.get_cell_tile_data(Vector2i(cell/16))
		if tile_data != null:
			var is_walkable: bool
			var cell_adjust: float = ceil((cell - location).length() / size) * size
			var x_offset: float  = (cell.x - location.x)
			var y_offset: float = (cell.y - location.y)
			is_walkable = _obstacle_query(_requester, cell_adjust, cell_adjust, x_offset, y_offset, _obstacle_layers, location)
			if is_walkable and not _node_data[str(location)].precursor == cell:
				viable_neighbors.append(cell)
	return viable_neighbors.duplicate()

# Queries for possible obstacles within the proximity of specified tile position
func _obstacle_query(agent: CollisionObject2D, agent_width: float, agent_length: float, nudge_x: float, nudge_y: float, collision_mask: int, origin: Vector2) -> bool:
	var space_state: PhysicsDirectSpaceState2D = agent.get_world_2d().get_direct_space_state()
	var query_shape := PhysicsShapeQueryParameters2D.new()
	var exclude_lists := []
	exclude_lists.append(agent)
	
	query_shape.set_collision_mask(collision_mask)
	query_shape.set_exclude(exclude_lists)
	
	var shape_RID := PhysicsServer2D.rectangle_shape_create()
	var margin := 0.0#0.025
	var half_length := (agent_length - margin) / 2.0
	var half_width :=  (agent_width - margin) / 2.0
	var dimensions := Vector2(half_length,half_width)
	PhysicsServer2D.shape_set_data(shape_RID, dimensions)
	var shape_transform := Transform2D(0, origin + Vector2(nudge_x, nudge_y))
	query_shape.set_shape_rid(shape_RID)
	query_shape.set_transform(shape_transform)
	
	var collision_points: Array[Vector2] = space_state.collide_shape(query_shape,1)
	
	PhysicsServer2D.free_rid(shape_RID)
	
	return collision_points.is_empty()

# Method that provides the overall cost of travelling from position a to position b. TURNING COST TO ZERO LEADS TO GREEDY SEARCH
func _cost(a: Vector2, b: Vector2) -> float:
	var _terrain_value := get_terrain_cost(b) as float
	var traversal_cost := (a - b).length() * _terrain_value#
	return traversal_cost

# Method for obtaining heuristic cost from position a to position b. TURNING HEURISTIC TO ZERO LEADS TO DJIKSTRA ALGORITHM
func _heuristic(a: Vector2, b: Vector2, heuristic_mode: int) -> float:
	var h_value: float
	var _terrain_value := get_terrain_cost(a) as float
	var dx: float = abs(a.x - b.x)
	var dy: float = abs(a.y - b.y)
	
	match heuristic_mode:
		0: # EUCLIDEAN HEURISTIC
			h_value = (a - b).length() 
		1: # MANHATTAN HEURISTIC
			h_value = dx + dy
		2: # OCTILE HEURISTIC
			var f: float = (sqrt(2) - 1)
			h_value = dx + (dy * f) if dy < dx else dy + (dx * f)
		3: # CHEBYSHEV HEURISTIC
			h_value =  max(dx, dy)
		_: # EUCLIDEAN HEURISTIC BY DEFAULT
			h_value = (a - b).length() 
	
	return h_value #* _terrain_value











# Functions that add, remove, and sort all PathNode instances in both the open list and node data

# Adds the position
func _add_to_open_list(position: Vector2) -> void:
	var pos_id := str(position)
	if not _node_data.has(pos_id):
		var node = PathNode.new(position)
		_node_data[pos_id] = node
	_node_data[pos_id].is_open = true
	_open_list.append(_node_data[pos_id])


func _pop_node_position() -> Vector2:
	#randomize()
	#heap.shuffle()
	#var pop_start = Time.get_ticks_usec()
	_build_heap_astar(_open_list) # HOW TO SPEED UP? USE ANOTHER SCRIPTING LANGUAGE INSTEAD
	#print(Time.get_ticks_usec() - pop_start)
	#var pop_start = Time.get_ticks_usec()
	var node_id = _open_list[0].id
	_node_data[node_id].is_open = false
	#print(heap[0].f_score)
	var swap = _open_list[0]
	_open_list[0] = _open_list[_open_list.size() - 1]
	_open_list[_open_list.size() - 1] = swap
	
	var node = _open_list.pop_back()
	#print(Time.get_ticks_usec() - pop_start)
	return node.position

# checks if position is in the open list
func _in_open_list(position: Vector2) -> bool:
	var pos_id := str(position)
	if _node_data.has(pos_id):
		return _node_data[pos_id].is_open
	return false

# Creates a partially-sorted heap data structure from an array using a heapify-down algorithm, 
# ensuring the topmost pathnode of the heap has the lowest f_score 
# (and highest g_score, if 2 nodes have the same f-score)
func _build_heap_astar(heap: Array):
	var size := heap.size()
	
	for i in range(int(heap.size()/2.0), -1, -1): # Sift-Down
		var parent := i
		var left := 2 * parent + 1
		var right := left + 1
		var swap_index: int
		var swap_data
		
		while left > 0 and left < size:
			swap_index = left
			if right < size and ((heap[right].f_score < heap[left].f_score)
					or (heap[right].f_score == heap[left].f_score and heap[right].g_score > heap[left].g_score)):
				swap_index = right
			if (heap[parent].f_score < heap[swap_index].f_score) or (heap[parent].f_score == heap[swap_index].f_score
					and heap[parent].g_score >= heap[swap_index].g_score ):
				break
			
			swap_data = heap[parent]
			heap[parent] = heap[swap_index]
			heap[swap_index] = swap_data
			
			parent = swap_index
			left = 2 * parent + 1
			right = left + 1

# clears up stored data in open list and node data
func _clear_data() -> void:
	_open_list.clear()
	_node_data.clear()



# Inner class node that stores AStar node related data for a specific discrete position.
class PathNode:
	var position
	var id: String
	var precursor = null
	var g_score: float = INF
	var f_score: float = INF
	#var path_count: int = 0
	#var visit_count: int = 0
	var is_open := false
		
	func _init(_position: Vector2):
		position = _position
		id = str(_position)
