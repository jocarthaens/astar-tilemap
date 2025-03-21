extends Label


func _on_a_star_cursor_pathfinding_finished(debug_data: Dictionary, shortest_path: Array, visited_tiles: Array) -> void:
	var data: Dictionary = debug_data;
	var string_data: String = "";
	for key: String in data:
		string_data = string_data.insert(string_data.length(), str(str(key) + " = " + str(data[key]) + "\n") )
	#print(string_data)
	text = string_data;
