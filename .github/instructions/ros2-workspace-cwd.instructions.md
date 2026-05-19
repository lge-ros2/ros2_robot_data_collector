---
description: "Use when running colcon build, colcon test, colcon test-result, ros2 launch, ros2 run, or other ROS 2 workspace commands for this package. Ensures commands run from /home/yg/Workspace/cloi_ws because this repository is symlinked into that workspace."
name: "ROS 2 Workspace CWD"
---
# ROS 2 Workspace CWD

- Run ROS 2 workspace commands from `/home/yg/Workspace/cloi_ws`, not from this repository root.
- This repository is mounted into that workspace through the symlink at `src/simulator/ros2_robot_data_collector`.
- Before `colcon build`, `colcon test`, `colcon test-result`, `ros2 launch`, or `ros2 run`, change directory to `/home/yg/Workspace/cloi_ws`.
- If the terminal is currently in this repository, `cd /home/yg/Workspace/cloi_ws` first.
- Typical sequence:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros2_robot_data_collector
source install/setup.bash
colcon test --packages-select ros2_robot_data_collector
colcon test-result --verbose
ros2 launch ros2_robot_data_collector collector.launch.py namespace_prefix:=/robot1 output_path:=/tmp/robot1.h5
```
