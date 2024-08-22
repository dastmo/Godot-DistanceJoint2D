# Godot Distance Joint
A simple distance joint for Godot that is not a spring.

## Known Issues
* When chaining multiple distance joints, they will overstretch more and more, with each joint added.
* When connecting Rigidbody2D's together, make sure they are of the same mass, or weird behaviour will ensue.
* The joint only works with PhysicsBody2D's. Probably not necessary, will investigate.
