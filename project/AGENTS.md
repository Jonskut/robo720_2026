# Final Project Plan

## Goal

Build a simulated FR3 conveyor task that combines motion control (ex1–4),
ArUco visual servoing (ex5), obstacle avoidance (ex6), and compliant patting
(ex7). Heads with ArUco tags move along a conveyor. The arm waits above a
visible head, approaches it, makes a gentle compliant contact, then retreats.
Large obstacles may be read directly from Gazebo poses; obstacle perception is
out of scope.

## Integration constraints

- Keep the implementation in this ROS 2 `project` package.
- Use one effort controller for all arm torques; do not activate competing
  controllers that claim the same effort command interfaces.
- Reuse `franka_kdl` for kinematics/dynamics, `ex5` for ArUco detection,
  `ex6` for obstacle geometry and potential-field avoidance, and `ex7` for
  impedance/compliance.
- Keep perception and simulation interfaces separate from the real-time
  controller update loop using the existing ROS topics and realtime buffers.
- Treat `project.old` as a reference copy; do not remove or edit it during
  implementation.

## Small-step implementation and verification

1. **[x] Restore the active package baseline.** Reuse the existing
   `project.old` package files in `project/`. Since `project.old` is also
   discovered as a package named `project`, build only the active package with
   `colcon build --base-paths src/robo720_2026/project --packages-select project --symlink-install`.
2. **[x] Verify simulation assets and launch wiring.** Check the world, head/tag,
   conveyor, obstacle, robot description, bridges, and node/topic names.
   The detector uses a fixed side-mounted camera outside the arm's normal
   workspace; transform its observations using `camera_pose` in `fr3_link0`.
   Keep the raised conveyor within its field of view and verify the optical
   extrinsics with live detections.
3. **Make the task sequence explicit.** Implement or refine a small controller
   state machine for waiting above a detected tag, approaching, compliant
   patting, and retreating. Loss of the tag must not produce an unsafe target.
   Verify state transitions and marker-loss behavior.
4. **Validate obstacle avoidance.** Reuse ex6 obstacle descriptions and Gazebo
   pose updates; ensure APF repulsion is combined with the task torque without
   disabling impedance. Verify the controller build and obstacle topic/pose
   contract.
5. **Validate the end-to-end demo.** Build, launch Gazebo, confirm marker
   detection and target updates, confirm head motion and obstacle updates, and
   observe a compliant pat and obstacle dodge. Record any environment-limited
   simulation checks.

## Scope decisions

- The conveyor and obstacle are simulation-only, and obstacle positions come
  from Gazebo rather than a perception pipeline.
- Prefer a simple repeatable head motion and one large obstacle over adding
  extra sensors or planning infrastructure.
- Do not duplicate exercise implementations unless integration requires a
  small adapter or safety fix.

## Progress

- Restored the active package from `project.old`; the targeted package build
  passes.
- Implemented the hover/pat/retreat controller state machine with marker-age
  handling and initialized its hold target from measured FK.
- Fixed conveyor updates to use Gazebo's batched pose service and keep each tag
  paired with its head. Pair movement is verified from Gazebo pose messages.
- Corrected the head visual/collision alignment and placed each marker above
  the rendered head surface.
- Raised the conveyor belt to the robot base box's top (world z=1.0), with the
  heads and obstacle moved to the same level. Relocated the fixed camera to the
  side of the arm's workspace to reduce occlusion.
- The side camera sees tags on the raised conveyor and OpenCV detects them.
  Its optical pose and the 0.1026 m printed-marker size reproduce the known
  Gazebo marker location within 5 mm. The robot partially occludes one tag in
  the sampled frame, while the other remains visible.
- Corrected obstacle repulsion to use linear surface clearance, with a
  recovery direction for points inside the box. The targeted package build
  passes; live obstacle dodging is still unverified.
- Moved the conveyor closer to the arm and slowed it, then synchronized its
  timer to Gazebo simulation time so rendering load cannot make heads outrun
  the controller.
- Bounded Gazebo runs confirm nonempty marker detections and controller goal
  TFs. The hover-to-contact transition and obstacle dodge remain unverified;
  simulation rendering was too slow to reach contact during the runs.
