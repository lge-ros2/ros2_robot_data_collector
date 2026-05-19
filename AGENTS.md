# ros2_robot_data_collector

## Workspace rule

- Edit source files in this repository.
- Run ROS 2 workspace commands from `/home/yg/Workspace/cloi_ws`.
- This package is exposed to that workspace through the symlink at `src/simulator/ros2_robot_data_collector`.
- If a terminal is opened in this repository root, `cd /home/yg/Workspace/cloi_ws` before `colcon build`, `colcon test`, or `ros2 launch`.

## Build, test, run

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros2_robot_data_collector
source install/setup.bash
colcon test --packages-select ros2_robot_data_collector
colcon test-result --verbose
ros2 launch ros2_robot_data_collector collector.launch.py namespace_prefix:=/robot1 output_path:=/tmp/robot1.h5
```

## Package shape

- Main node: [src/collector_node.cpp](src/collector_node.cpp)
- Writer implementation: [src/hdf5_writer.cpp](src/hdf5_writer.cpp)
- Config and validation: [include/ros2_robot_data_collector/collector_config.hpp](include/ros2_robot_data_collector/collector_config.hpp)
- Topic name handling: [include/ros2_robot_data_collector/topic_resolver.hpp](include/ros2_robot_data_collector/topic_resolver.hpp)
- Sync window logic: [include/ros2_robot_data_collector/sync_policy.hpp](include/ros2_robot_data_collector/sync_policy.hpp)
- Launch entrypoint: [launch/collector.launch.py](launch/collector.launch.py)
- Default parameters: [config/collector.yaml](config/collector.yaml)
- Current tests: [test/test_sync_policy.cpp](test/test_sync_policy.cpp), [test/test_topic_resolution.cpp](test/test_topic_resolution.cpp)

## Project-specific behavior

- `collector_node` synchronizes image and joint-state samples around each action message.
- Action timestamps currently come from receive time in the node, not from a stamped action message.
- The HDF5 schema is fixed by the first accepted frame; later shape or metadata mismatches are rejected by the writer.
- Queue sizes are bounded. Old samples can be dropped under load.

## Editing guardrails

- Keep C++ changes compatible with C++17 and existing warning settings from [CMakeLists.txt](CMakeLists.txt).
- When adding executables or tests, update [CMakeLists.txt](CMakeLists.txt) and keep install/test targets consistent.
- Do not assume [src/sample_data_publisher.cpp](src/sample_data_publisher.cpp) is runnable today; it exists in source but is not added as a CMake target.
