# ROS And Robotics Guidelines

## ROS Interfaces

- Preserve topic/service/action names, message types, frame IDs, parameter names, and launch arguments unless the task requires an interface change.
- When adding parameters, provide safe defaults and document expected units.
- Use ROS logging levels intentionally: `ROS_DEBUG` for high-rate internals, `ROS_INFO` for lifecycle milestones, `ROS_WARN/ERROR` for operator-visible problems.
- Avoid blocking callbacks when a timer, action server, worker thread, or state machine is more appropriate.

## Motion Planning And Control

- Treat frame transforms, unit conventions, joint limits, velocity/acceleration limits, and collision settings as first-class correctness constraints.
- For MoveIt changes, inspect planning group names, end-effector links, planning scene updates, constraints, and execution result handling.
- For trajectory changes, verify timestamps, interpolation assumptions, controller compatibility, and abort/stop behavior.
- For force or compliant control, preserve saturation, filtering, contact thresholds, watchdogs, and emergency stop paths.

## Mobile Robot And Manipulator Integration

- Make coordination explicit: base state, arm state, task state, perception readiness, and recovery behavior should be inspectable.
- Be careful with mixed coordinate frames between map/odom/base/tool/camera/world.
- Avoid hidden sleeps as synchronization when ROS events, actions, services, TF availability, or state machines can express readiness.

## Perception And Calibration

- Keep camera intrinsics, extrinsics, depth units, image encodings, point cloud frames, and timestamp alignment visible.
- For OpenCV/PCL/deep-learning changes, verify input encoding, coordinate conventions, filtering thresholds, model paths, and runtime device assumptions.
- For hand-eye or sensor calibration, never silently change transforms or calibration files without noting validation requirements.

## Simulation And Visualization

- Keep Gazebo, RViz, URDF/xacro, mesh, and controller configs consistent.
- When changing robot descriptions, consider collision geometry, inertial values, joint limits, transmissions, plugins, and TF tree effects.

## Documentation

- Write technical notes in concrete terms: command, package, node, topic, frame, parameter, observed behavior, and verified result.
- Separate measured results from hypotheses.
