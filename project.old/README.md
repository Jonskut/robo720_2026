# Final Project: Compliant Head-Patting Arm (ex1–ex7 combined)

A conveyor carries "heads" (boxes topped with an ArUco marker). The FR3 arm
visual servoes to each detected head, waits above it, and pats it with
compliant impedance control. A big box obstacle on the conveyor is dodged via
potential-field avoidance (pose from Gazebo TF, no detection).

Combines:
- ex1–4: motion control via KDL kinematics (`franka_kdl::Solver`)
- ex5:   visual servoing to the ArUco marker (wrist camera)
- ex6:   potential-field obstacle avoidance
- ex7:   impedance (compliance) control for patting

All behaviours run in a single merged effort controller (`project/ProjectController`),
because only one effort controller can own the joint command interfaces at a time.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select project --symlink-install --executor sequential
source install/setup.bash
```

## Launch

```bash
ros2 launch project robot.launch.py
```

This starts Gazebo with `project_world.sdf`, the FR3 arm (wrist camera + FT sensor),
the merged `project_controller`, RViz, the ArUco pose estimator, the static obstacle
publisher, and the fake wrench publisher.

## Operation

- The arm waits above the detected ArUco head (visual servoing), pointing down.
- Lowering the goal or letting contact occur produces a compliant pat (impedance control).
- When the arm links approach the big box obstacle, the potential-field repulsion
  pushes the arm away (obstacle dodge).
- RViz shows the obstacle as a red cube on `/markers` and the goal as `goal_pose` TF.

## Parameters / runtime tuning

Set via `ros2 param set /project_controller <param> <value>`:

- `use_force_measurement` (bool): use real FT sensor feedback (default `true`)
- `use_fake_wrench` (bool): use the fake wrench publisher (default `false`)
- `wait_offset` (double[3]): end-effector offset above the head in base frame
- `apf_threshold`, `apf_k`: obstacle avoidance threshold and gain

The controller subscribes to `/goal_pose` (`geometry_msgs/PoseStamped`) for manual
pose overrides and `/marker_pose` (`std_msgs/Float64MultiArray`) from the ArUco
detector for visual servoing.

## Clean build

```bash
colcon build --packages-select project --symlink-install --executor sequential
```
