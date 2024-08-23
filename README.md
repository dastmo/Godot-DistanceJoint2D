# Godot Distance Joint
A simple 2D distance joint for Godot that is not a spring.

## Known Issues
* When chaining multiple distance joints, they will overstretch more and more, with each joint added.
* Generally, chains of joints are not recommended. Better approach is currently being developed.
* Drastic change in the max_distance of the joint makes the Rigidbody2Ds attached to it shoot off in the direction of the other body attached. When making changes at runtime, make them gradual.
* When connecting Rigidbody2D's together, make sure they are of the same mass, or weird behaviour will ensue.
