# Development Workflow

Use this file for repeatable project workflows. Prefer verified commands from the repo or user over generic ROS habits.

## Context Gathering

- Use `rg --files`, `find src -maxdepth ...`, `package.xml`, `CMakeLists.txt`, launch files, config files, and message/action definitions to locate package boundaries.
- Trace ROS interactions through publishers, subscribers, services, action servers/clients, parameters, TF frames, and launch remaps.
- For C++ changes, inspect headers, callbacks, object lifetime, threading/spinners, and blocking robot calls.
- For Python changes, inspect node initialization, parameter defaults, exception behavior, and executable permissions.

## Build And Validation

- Prefer targeted builds for changed packages when possible.
- Use catkin/catkin-tools commands already established by this workspace when discovered.
- After C++ changes, check compile errors in the touched package and direct dependents when practical.
- After launch/config changes, validate XML/YAML syntax and parameter names.
- After runtime behavior changes, identify a concrete validation path: unit test, dry-run node, simulation, log/bag replay, or hardware-safe manual check.

## Debugging

- Start from the symptom, then trace data flow across topics/services/actions/TF.
- Capture exact error logs, timestamps, node names, topic names, frame IDs, parameter values, and launch command.
- Separate build-time, launch-time, ROS graph, TF, planning, controller, perception, and hardware IO failures.
- For timing issues, inspect callback rates, queue sizes, sleep loops, blocking calls, timers, and system clock assumptions.

## Git And Change Hygiene

- Check existing changes before broad edits.
- Do not revert user changes unless explicitly asked.
- Keep patches scoped to the requested behavior.
- Avoid formatting churn in unrelated code.
- Explain untested areas and residual risk in the final response.

## Docker And System Work

- Treat Docker, device permissions, udev, GPU/CUDA, RealSense, and ROS environment setup as host-sensitive.
- Verify commands and versions before changing system-level assumptions.
- Document stable setup knowledge here once confirmed.
