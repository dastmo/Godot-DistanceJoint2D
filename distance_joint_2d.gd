extends Node2D
class_name DistanceJoint2D


@export var disable_collision: bool = true
@export var pivot: NodePath = NodePath("")
@export var links: Array[RigidBody2D]
@export var uniform_distance: bool = false
@export var max_distance: float = 0.0


var _pivot: Node2D
var _joint_valid: bool = true
var _link_distances: Array[float] = []

func _ready() -> void:
	init_joint()


func init_joint() -> void:
	if pivot != NodePath(""):
		_pivot = get_node(pivot)
	_joint_valid = _validate_joint()
	
	if not _joint_valid:
		return
	
	_set_initial_distances()
	
	if disable_collision:
		_add_collision_exceptions()


func _physics_process(delta: float) -> void:
	_monitor_node_change()
	_apply_constraint(delta)


func _validate_joint() -> bool:
	if links.size() == 0:
		push_error("No Rigidbody2D added to the links array. Joint is invalid.")
		return false
	
	if links.size() < 2 and not is_instance_valid(_pivot):
		push_error("Add at least two links or one link and one pivot. Joint is invalid.")
		return false
	
	if _pivot is RigidBody2D:
		if _pivot.freeze == false:
			_pivot.freeze = true
			push_warning("Rigidbody2D static pivot frozen by joint.")
	
	return true


func _add_collision_exceptions() -> void:
	if _pivot is PhysicsBody2D:
		for link in links:
			_pivot.add_collision_exception_with(link)
	
	for link in links:
		for other_link in links:
			if other_link != link:
				link.add_collision_exception_with(other_link)


func _set_initial_distances() -> void:
	_link_distances.clear()
	
	if uniform_distance:
		_set_uniform_distances()
	else:
		_set_distances()


func _set_uniform_distances() -> void:
	if is_instance_valid(_pivot):
		var _links_distance: float = max_distance / links.size()
		for i in links.size():
			_link_distances.append(_links_distance)
	else:
		var _links_distance: float = max_distance / (links.size() - 1)
		for i in links.size():
			if i == 0:
				_link_distances.append(0.0)
			else:
				_link_distances.append(_links_distance)


func _set_distances() -> void:
	for i in links.size():
		if i == 0:
			if is_instance_valid(_pivot):
				_link_distances.append(links[i].global_position.distance_to(_pivot.global_position))
			else:
				_link_distances.append(0.0)
		else:
			_link_distances.append(links[i].global_position.distance_to(links[i - 1].global_position))
	
	for distance in _link_distances:
		max_distance += distance


func _apply_constraint(delta: float) -> void:
	if not _joint_valid:
		return
	
	_apply_constraint_on_velocity(delta)


func _apply_constraint_with_anchor_on_velocity(delta) -> void:
	for i in links.size():
		var _next_pos: Vector2 = links[i].global_position + (links[i].linear_velocity * delta)
		var _next_distance = 0.0
		if i == 0:
			_next_distance = _next_pos.distance_to(_pivot.global_position)
		else:
			_next_distance = _next_pos.distance_to(links[i-1].global_position)
		
		if _next_distance <= _link_distances[i]:
			continue
		
		var _v: Vector2 = Vector2.ZERO
		if i == 0:
			_v = _next_pos.direction_to(_pivot.global_position) * (_next_distance - _link_distances[i])
		else:
			_v = _next_pos.direction_to(links[i-1].global_position) * (_next_distance - _link_distances[i])
		
		links[i].linear_velocity += _v / delta


func _apply_constraint_on_velocity(delta: float) -> void:
	for i in links.size():
		if i == 0 and is_instance_valid(_pivot):
			var _next_pos: Vector2 = links[i].global_position + (links[i].linear_velocity * delta)
			var _next_distance = 0.0
			_next_distance = _next_pos.distance_to(_pivot.global_position)
			
			if _next_distance <= _link_distances[i]:
				continue
			
			var _v: Vector2 = Vector2.ZERO
			_v = _next_pos.direction_to(_pivot.global_position) * (_next_distance - _link_distances[i])
			
			links[i].linear_velocity += _v / delta
		else:
			var _next_pos = links[i].global_position + links[i].linear_velocity * delta
			var _next_distance_previous = 0.0
			var _next_distance_next = 0.0
			
			if i == 0:
				_next_distance_next = _next_pos.distance_to(links[i + 1].global_position)
			elif i == links.size() - 1:
				_next_distance_previous = _next_pos.distance_to(links[i - 1].global_position)
			else:
				_next_distance_next = _next_pos.distance_to(links[i + 1].global_position)
				_next_distance_previous = _next_pos.distance_to(links[i - 1].global_position)
				
			
			if i < links.size() - 1:
				if _next_distance_next > _link_distances[i + 1]:
					var _distance = _link_distances[i + 1] - _next_distance_next
					links[i].linear_velocity -= (
						_next_pos.direction_to(links[i+1].global_position) *
						_distance /
						delta
					)
			if i > 0:
				if _next_distance_previous > _link_distances[i]:
					var _distance = _link_distances[i] - _next_distance_previous
					links[i].linear_velocity -= (
						_next_pos.direction_to(links[i-1].global_position) *
						_distance /
						delta
					)


func _apply_constraint_without_anchor_on_velocity(delta: float) -> void:
	for i in links.size():
		var _next_pos = links[i].global_position + links[i].linear_velocity * delta
		var _next_distance_previous = 0.0
		var _next_distance_next = 0.0
		
		if i == 0:
			_next_distance_next = _next_pos.distance_to(links[i + 1].global_position)
		elif i == links.size() - 1:
			_next_distance_previous = _next_pos.distance_to(links[i - 1].global_position)
		else:
			_next_distance_next = _next_pos.distance_to(links[i + 1].global_position)
			_next_distance_previous = _next_pos.distance_to(links[i - 1].global_position)
			
		
		if i < links.size() - 1:
			if _next_distance_next > _link_distances[i + 1]:
				var _distance = _link_distances[i + 1] - _next_distance_next
				links[i].linear_velocity -= (
					_next_pos.direction_to(links[i+1].global_position) *
					_distance /
					delta
				)
		if i > 0:
			if _next_distance_previous > _link_distances[i]:
				var _distance = _link_distances[i] - _next_distance_previous
				links[i].linear_velocity -= (
					_next_pos.direction_to(links[i-1].global_position) *
					_distance /
					delta
				)


func _get_drag_force(body: RigidBody2D) -> float:
	if body.linear_damp_mode == RigidBody2D.DAMP_MODE_COMBINE:
		return (
			1.0 -
			(body.linear_damp + ProjectSettings.get_setting("physics/2d/default_linear_damp")) /
			ProjectSettings.get_setting("physics/common/physics_ticks_per_second") *
			body.mass
		)
	else:
		return (
			1.0 -
			body.linear_damp / 
			ProjectSettings.get_setting("physics/common/physics_ticks_per_second") *
			body.mass
		)


func _monitor_node_change() -> void:
	var nodes_changed: bool = false
	
	#TODO: Implement node change monitoring.
	
	if nodes_changed:
		_joint_valid = _validate_joint()
		if _joint_valid:
			_set_initial_distances()
