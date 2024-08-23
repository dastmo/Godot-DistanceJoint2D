extends Node2D
class_name DistanceJoint2D


@export var disable_collision: bool = true
@export var node_a: NodePath = NodePath("")
@export var node_b: NodePath = NodePath("")
@export var auto_start_distance: bool = true
@export var max_distance: float = 0.0


var _node_a: Node2D
var _node_b: Node2D
var _joint_valid: bool = true
var _node_a_previous: NodePath = NodePath("")
var _node_b_previous: NodePath = NodePath("")
var _current_distance: float = 0.0
var _previous_v_a: Vector2 = Vector2.ZERO
var _previous_v_b: Vector2 = Vector2.ZERO

func _ready() -> void:
	if node_a != NodePath(""):
		_node_a = get_node(node_a)
	if node_b != NodePath(""):
		_node_b = get_node(node_b)
	_node_a_previous = node_a
	_node_b_previous = node_b
	_joint_valid = _validate_joint()
	
	if _joint_valid:
		_set_initial_max_distance()


func _physics_process(delta: float) -> void:
	_monitor_node_change()
	_apply_constraint(delta)


func _validate_joint() -> bool:
	if not is_instance_valid(_node_a) or not is_instance_valid(_node_b):
		push_error("Not all nodes are valid. Joint is invalid.")
		return false
	
	if not _node_a is RigidBody2D and not _node_b is RigidBody2D:
		push_error("No Rigidbody2D attached to the joint. Joint is invalid.")
		return false
	
	return true


func _set_initial_max_distance() -> void:
	if auto_start_distance:
		max_distance = _node_a.global_position.distance_to(_node_b.global_position)
		_current_distance = max_distance


func _apply_constraint(delta: float) -> void:
	if not _joint_valid:
		return
	
	_current_distance = _node_a.global_position.distance_to(_node_b.global_position)
	
	if not _node_a is RigidBody2D or not _node_b is RigidBody2D:
		_apply_constraint_with_anchor_on_velocity(delta)
	else:
		_apply_constraint_without_anchor_on_velocity(delta)


func _apply_constraint_with_anchor_on_velocity(delta) -> void:
	var anchor: Node2D
	var pendulum: RigidBody2D
	if _node_a is RigidBody2D:
		anchor = _node_b
		pendulum = _node_a
	else:
		anchor = _node_a
		pendulum = _node_b
	
	var _next_pos: Vector2 = pendulum.global_position + (pendulum.linear_velocity * delta)
	var _next_distance = _next_pos.distance_to(anchor.global_position)
	
	if _next_distance < max_distance:
		return
	
	var _v = _next_pos.direction_to(anchor.global_position) * (_next_distance - max_distance)
	
	pendulum.linear_velocity += _v * ProjectSettings.get_setting("physics/common/physics_ticks_per_second")


func _apply_constraint_without_anchor_on_velocity(delta: float) -> void:
	if _current_distance <= max_distance:
		return
	
	var _next_pos_a: Vector2 = _node_a.global_position + (_node_a.linear_velocity * delta)
	var _next_pos_b: Vector2 = _node_b.global_position + (_node_b.linear_velocity * delta)
	var _next_distance = _next_pos_a.distance_to(_next_pos_b)
	
	if _next_distance <= max_distance:
		return
	
	var _v_a = (
		_next_pos_a.direction_to(_next_pos_b) *
		(_next_distance - max_distance) *
		(_node_b.mass / _node_a.mass)
	)
	var _v_b = (
		_next_pos_b.direction_to(_next_pos_a) *
		(_next_distance - max_distance) *
		(_node_a.mass / _node_b.mass)
	)
	
	_node_a.linear_velocity += _v_a
	_node_b.linear_velocity += _v_b


# This method does not modify velocity directly.
# However, it is a lot more jittery.
func _apply_constraint_with_anchor(delta: float) -> void:
	var anchor: PhysicsBody2D
	var pendulum: RigidBody2D
	if _node_a is RigidBody2D:
		anchor = _node_b
		pendulum = _node_a
	else:
		anchor = _node_a
		pendulum = _node_b
	
	var _next_pos = pendulum.global_position + (pendulum.linear_velocity * delta)
	var _next_distance = _next_pos.distance_to(anchor.global_position)
	
	if _next_distance <= max_distance:
		return
	
	var vm = 2 * (_current_distance - max_distance) / delta
	var a = vm / (delta / 2)
	var Fd = get_drag_force(pendulum)
	var Fnet = pendulum.mass * a + Fd
	pendulum.apply_central_force(pendulum.global_position.direction_to(anchor.global_position) * Fnet * delta)


func get_drag_force(body: RigidBody2D) -> float:
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
	
	if _node_a_previous != node_a:
		_node_a = get_node(node_a)
		_node_a_previous = node_a
		nodes_changed = true
	
	if _node_b_previous != node_b:
		_node_b = get_node(node_b)
		_node_b_previous = node_b
		nodes_changed = true
	
	if nodes_changed:
		_joint_valid = _validate_joint()
		if _joint_valid and auto_start_distance:
			_set_initial_max_distance()
