extends CharacterBody2D


@export var joint: DistanceJoint2D


var input: Vector2 = Vector2.ZERO
var movement_speed: float = 100.0


func _process(delta: float) -> void:
	input = Vector2.ZERO
	if Input.is_key_pressed(KEY_D):
		input += Vector2.RIGHT
	if Input.is_key_pressed(KEY_A):
		input += Vector2.LEFT
	if Input.is_key_pressed(KEY_S):
		input += Vector2.DOWN
	if Input.is_key_pressed(KEY_W):
		input += Vector2.UP
	if Input.is_key_pressed(KEY_UP):
		joint.total_distance -= (50.0 * delta)
	if Input.is_key_pressed(KEY_DOWN):
		joint.total_distance += (50.0 * delta)
	if Input.is_key_pressed(KEY_SPACE) and joint.links.size() > 0:
		joint.links.remove_at(0)
	if Input.is_key_pressed(KEY_SHIFT):
		joint.pivot = NodePath("")


func _physics_process(_delta: float) -> void:
	velocity = input * movement_speed
	move_and_slide()
