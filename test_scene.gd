extends Node2D


@onready var rope_joint_2d: DistanceJoint2D = $RopeJoint2D
@onready var rigid_body_2d: RigidBody2D = $RigidBody2D
@onready var rigid_body_2d_2: RigidBody2D = $RigidBody2D2


func _ready() -> void:
	rigid_body_2d.apply_central_force(Vector2.LEFT * 10000)
	rigid_body_2d_2.apply_central_force(Vector2.RIGHT * 10000)


func _process(delta: float) -> void:
	if Input.is_key_pressed(KEY_UP):
		rope_joint_2d.max_distance -= 50.0 * delta
	
	if Input.is_key_pressed(KEY_DOWN):
		rope_joint_2d.max_distance += 50.0 * delta
