# Final Project: Compliant Head-Patting Arm (ex1–ex7 combined)

A slow conveyor carries "heads" (boxes topped with an ArUco marker). The FR3 arm
visual servoes to each detected head, waits above it, and pats it with
compliant impedance control. A big box obstacle on the conveyor is dodged via
potential-field avoidance (pose from Gazebo TF, no detection).

Combines:
- ex1–4: motion control via KDL kinematics (`franka_kdl::Solver`)
- ex5:   visual servoing to the ArUco marker (fixed side-mounted camera)
- ex6:   potential-field obstacle avoidance
- ex7:   impedance (compliance) control for patting

All behaviours run in a single merged effort controller (`project/ProjectController`),
because only one effort controller can own the joint command interfaces at a time.

## Build

```bash
cd ~/ros2_ws
colcon build --base-paths src/robo720_2026/project \
  --packages-select project --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch project robot.launch.py
```

This starts Gazebo with `project_world.sdf`, the FR3 arm (FT sensor),
the merged `project_controller`, RViz, the ArUco pose estimator, the static obstacle
publisher, and the fake wrench publisher.

## Operation

- The conveyor belt top is at world z=1.0, level with the robot's base table;
  heads and the large obstacle sit on that raised surface.
- The fixed side camera is outside the arm's normal workspace and views the
  conveyor obliquely to reduce arm occlusion; when one tag is blocked, the
  second visible head tag can still provide the target.
- The arm holds its current pose until the fixed side camera detects an
  ArUco tag. The detector's 0.1026 m marker width is calibrated to the printed
  code on the 0.15 m white-backed marker plane.
- Conveyor motion uses Gazebo simulation time so the heads do not move ahead of
  the arm when rendering slows the simulation.
- It approaches a hover offset, lowers to the contact offset for a short,
  impedance-controlled pat, then retreats to hover.
- If marker data expires during the sequence, it aborts contact and retreats
  using the last base-frame marker position. It does not jump to a default
  pose on marker loss.
- When arm links approach the big box obstacle, potential-field repulsion is
  added to the same controller's torque command.
- RViz shows the obstacle as a red cube on `/markers` and the goal as `goal_pose` TF.

## Parameters / runtime tuning

Set via `ros2 param set /project_controller <param> <value>`:

- `use_force_measurement` (bool): use real FT sensor feedback (default `true`)
- `use_fake_wrench` (bool): use the fake wrench publisher (default `false`)
- `wait_offset` (double[3]): hover offset from marker center in base frame
- `pat_offset` (double[3]): contact offset from marker center in base frame
- `marker_timeout` (seconds): maximum age of a valid marker observation
- `approach_tolerance` (meters): distance to switch from hover to contact
- `pat_duration`, `repeat_delay`, `retreat_timeout` (seconds): cycle timing
- `apf_threshold`, `apf_k`: obstacle avoidance threshold and gain

The controller subscribes to `/goal_pose` (`geometry_msgs/PoseStamped`) for manual
pose overrides while waiting, and `/marker_pose` (`std_msgs/Float64MultiArray`)
from the side-camera ArUco detector for visual servoing. The camera pose is
configured in `fr3_link0` as `camera_pose`.

## Clean build

```bash
colcon build --base-paths src/robo720_2026/project \
  --packages-select project --symlink-install
```
