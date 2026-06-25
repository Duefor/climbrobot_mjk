# Project Memory

This file is the evolving workspace map. Keep it factual and concise. Mark uncertain notes as `inferred` until verified by code review, runtime checks, or the user.

## Workspace Snapshot

- The workspace appears to be a ROS1/catkin workspace with `build/`, `devel/`, `.catkin_tools/`, and packages under `src/`.
- Current visible package areas include:
  - `src/auto_tast_pmj/cs66_ctl_manipulation`: task-level manipulator control and automation code.
  - `src/auto_tast_pmj/cs66_arm_config1` and `src/cs66_moveit_config`: CS66 arm configuration and MoveIt config.
  - `src/auto_tast_pmj/cs66_arm_description` and `src/robot_simulation`: robot descriptions, URDF/xacro, meshes, Gazebo/world assets.
  - `src/car_ctl`: mobile base control and action/client code.
  - `src/realsense_set`, `src/realsense-ros`, and `src/auto_tast_pmj/mv3d_rgbd_ros`: camera and RGB-D related packages.
  - `src/camera_yolo`: vision/deep-learning related package.
  - `src/easy_handeye`: hand-eye calibration package.
  - `src/touch_set`: tactile/force-related messages or nodes.
  - `src/robot_sdk_wrapper`, `src/robot_set`, `src/controller_set`, `src/moveit`, `src/ROS-TCP-Endpoint`: SDK, robot setup, control, planning, and external integration areas.

## Architecture Notes

- Add verified package responsibilities here as they are learned.
- Record main launch entry points and dependencies between manipulator, base, perception, and planning nodes.
- Record important ROS topics, services, actions, parameters, TF frames, and coordinate conventions.

## Hardware And Safety Assumptions

- Add real robot model, controller, end-effector, camera, force/tactile sensor, and mobile base details when verified.
- Before changing motion or force-control behavior, identify limits, coordinate frames, stop conditions, and failure modes.

## Open Questions

- Which packages are actively maintained versus vendored third-party dependencies?
- What is the canonical command sequence for building and launching the full robot system?
- Which code paths are used on real hardware versus simulation?
