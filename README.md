# Godot Distance Joint
A simple 2D distance joint for Godot that is not a spring.

This script does not allow the creation of double/triple/etc. pendulums and that is by design. Rigidbody2Ds will behave as if linked by a string that does have some mass.

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/L4L2128QIJ)

## Quickstart
1. Copy the distance_joint_2d.gd file to your project.
2. Instantiate a DistanceJoint2D node in the scene where you want to use it.
3. Assign nodes to the DistanceJoint2D in one of two ways:
   * Assign a pivot (any Node2D) and add as many Rigidbody2D nodes to the links array as you wish.
   * Assign at least two Rigidbody2D nodes to the links array, without a pivot.

## Properties

|Property|Type|Description|Default Value|
|--------|----|-----------|-------------|
|disable_collision|bool|Disables collision between all nodes in the system (pivot and links)|true|
|pivot|NodePath|Assigns a static pivot to the system, creating an anchor for the chain of links to hang from. If the pivot is a Rigidbody2D, it will be frozen on joint initialization.|""|
|links|Array[Rigidbody2D]|The array that holds the Rigidbody2D nodes that need to be in the system. They will be linked to each other in the order they are added to the array.|N/A|
|auto_distance|bool|If true, the maximum distance between the links will be calculated from their position at the time of joint initialization. total_distance will be the sum of all the distances between links. This allows for the distances between links to not be uniform. If false, the distance between links will be uniform and based on the total_distance of the joint.|false|
|total_distance|float|The total allowed distance between the pivot (or first link if no pivot is set) and the last link in the system. Changing it at runtime will have no effect if auto_distance is true.|

## Known Issues
* When having multiple links and a pivot assigned, the second link in the chain will jitter a lot.

## Planned features
* Mode that allows the links to orbit a single pivot, instead of being part of a chain.
* Mode that keeps the distance constant (think a metal bar, not string).
* Mode that allows the creation of chaotic pendulums.
