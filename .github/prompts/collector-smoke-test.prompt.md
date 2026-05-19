---
name: "Collector Smoke Test"
description: "Run a ROS 2 smoke test for this package by building ros2_robot_data_collector, running its tests, and optionally probing collector launch startup. Use when: collector smoke test, collector sanity check, build and launch collector, ROS 2 collector health check."
argument-hint: "Optional scope or overrides, e.g. 'build+test only', 'launch with namespace /robot2', or 'output /tmp/check.h5'"
agent: "agent"
model: "GPT-5 (copilot)"
---
Run a smoke test for this repository's collector package.

Use these files as the source of truth:
- [Workspace rules](../../AGENTS.md)
- [ROS 2 workspace command instruction](../instructions/ros2-workspace-cwd.instructions.md)
- [Launch entrypoint](../../launch/collector.launch.py)
- [Default parameters](../../config/collector.yaml)

Interpret any user-supplied arguments as optional overrides for:
- test scope: `build-only`, `build+test`, or `full`
- `namespace_prefix`
- `output_path`
- `params_file`

Default behavior when no arguments are provided:
1. Change to `/home/yg/Workspace/cloi_ws`.
2. Source `/opt/ros/$ROS_DISTRO/setup.bash`.
3. Build `ros2_robot_data_collector` with `colcon build --packages-select ros2_robot_data_collector`.
4. Source `install/setup.bash`.
5. Run `colcon test --packages-select ros2_robot_data_collector`.
6. Run `colcon test-result --verbose`.
7. For a startup-only launch probe, start `ros2 launch ros2_robot_data_collector collector.launch.py namespace_prefix:=/robot1 output_path:=/tmp/collector-smoke-test.h5`, confirm the node starts without immediate errors, then stop it cleanly.

Important constraints:
- Do not assume [src/sample_data_publisher.cpp](../../src/sample_data_publisher.cpp) is runnable or available as a built target.
- Treat this as a smoke test, not a full data-capture integration test, unless the user explicitly asks for end-to-end publishing.
- If launch produces no HDF5 file because no samples arrive, do not count that alone as a failure.
- Keep the temporary output path used for the launch probe unless the user explicitly asks for cleanup.
- Stop at the first real failure and explain the root cause instead of plowing through later steps.

Report the result in this shape:
1. `Verdict`: pass, fail, or partial, plus the failing stage if any.
2. `Commands run`: the exact commands that were executed.
3. `Key observations`: the most relevant output, warnings, or failure cause.
4. `Artifacts`: any test-result summary or output path used.
5. `Next step`: the single most useful follow-up if something failed.