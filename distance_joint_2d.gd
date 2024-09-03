extends Node2D
class_name DistanceJoint2D


@export var disable_collision: bool = true
@export var pivot: NodePath = NodePath("")
@export var links: Array[RigidBody2D]
@export var auto_distance: bool = false
@export var total_distance: float = 0.0
@export var joint_type: JointType = JointType.CHAIN


var _pivot: Node2D
var _joint_valid: bool = true
var _link_distances: Array[float] = []


# Variables used to monitor for changes
var _previous_disable_collision: bool
var _previous_pivot: NodePath
var _previous_links: Array[RigidBody2D]
var _previous_auto_distance: bool
var _previous_total_distance: float
var _previous_joint_type: JointType


enum JointType {CHAIN, ORBIT}


func _ready() -> void:
	_recalculate_joint()


func _recalculate_joint() -> void:
	_previous_disable_collision = disable_collision
	_previous_pivot = pivot
	_previous_links = links.duplicate()
	_previous_auto_distance = auto_distance
	_previous_joint_type = joint_type
	
	if pivot != NodePath(""):
		_pivot = get_node(pivot)
	else:
		_pivot = null
	
	_joint_valid = _validate_joint()
	if not _joint_valid:
		return
	
	_set_initial_distances()
	
	if disable_collision:
		_add_collision_exceptions()
	else:
		_remove_collision_exceptions()
	
	for link in links:
		link.can_sleep = false


func _physics_process(delta: float) -> void:
	if _check_for_changes():
		_recalculate_joint()
	
	if joint_type == JointType.CHAIN:
		_apply_chain_constraint(delta)
	elif joint_type == JointType.ORBIT:
		_apply_orbit_constraint(delta)


func _validate_joint() -> bool:
	if links.size() == 0:
		push_warning("No Rigidbody2D added to the links array. Joint is invalid.")
		return false
	
	if links.size() < 2 and not is_instance_valid(_pivot):
		push_warning("Add at least two links or one link and one pivot. Joint is invalid.")
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


func _remove_collision_exceptions() -> void:
	if _pivot is PhysicsBody2D:
		for link in links:
			_pivot.remove_collision_exception_with(link)
	
	for link in links:
		for other_link in links:
			if other_link != link:
				link.remove_collision_exception_with(other_link)


func _set_initial_distances() -> void:
	_link_distances.clear()
	
	if joint_type == JointType.CHAIN:
		if not auto_distance:
			_set_uniform_distances()
		else:
			_set_distances()
	elif joint_type == JointType.ORBIT:
		_set_orbit_distances()
	
	_previous_total_distance = total_distance


func _set_uniform_distances() -> void:
	if is_instance_valid(_pivot):
		var _links_distance: float = total_distance / links.size()
		for i in links.size():
			_link_distances.append(_links_distance)
	else:
		var _links_distance: float = total_distance / (links.size() - 1)
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
		total_distance += distance


func _set_orbit_distances() -> void:
	if auto_distance:
		var center: Vector2 = _get_orbit_center()
		
		for i in links.size():
			_link_distances.append(links[i].global_position.distance_to(center))
	else:
		for i in links.size():
			_link_distances.append(total_distance)


func _apply_chain_constraint(delta: float) -> void:
	if not _joint_valid:
		return
	
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


func _apply_orbit_constraint(delta: float) -> void:
	var center: Vector2 = _get_orbit_center()
	
	for i in links.size():
		var _next_pos: Vector2 = links[i].global_position + (links[i].linear_velocity * delta)
		var _next_distance = 0.0
		_next_distance = _next_pos.distance_to(center)
		
		if _next_distance <= _link_distances[i]:
			continue
		
		var _v: Vector2 = Vector2.ZERO
		_v = _next_pos.direction_to(center) * (_next_distance - _link_distances[i])
		
		links[i].linear_velocity += _v / delta


func _get_orbit_center() -> Vector2:
	var center: Vector2 = Vector2.ZERO
	
	if is_instance_valid(_pivot):
		center = _pivot.global_position
	else:
		for link in links:
			center += link.global_position
		center = center / links.size()
	
	return center


func _check_for_changes() -> bool:
	if disable_collision != _previous_disable_collision:
		return true
	
	if pivot != _previous_pivot:
		return true
	
	if links.hash() != _previous_links.hash():
		return true
	
	if auto_distance != _previous_auto_distance:
		return true
	
	if total_distance != _previous_total_distance:
		return true
	
	if joint_type != _previous_joint_type:
		return true
	
	return false
