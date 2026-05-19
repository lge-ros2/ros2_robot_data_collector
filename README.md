# ros2_robot_data_collector

Namespace-aware ROS 2 data collector that synchronizes observations and actions and writes them to HDF5.

This package is usable today for one image stream, one joint-state stream, and one numeric action stream. It is not yet a full CLOiSim dataset recorder for `joint_command`, LiDAR, odometry, IMU, or explicit episode markers.

## Current Scope

Supported today:

- one `sensor_msgs/msg/Image` topic
- one `sensor_msgs/msg/JointState` topic
- one `std_msgs/msg/Float64MultiArray` action topic
- HDF5 output with timestamps, images, joint positions/velocities/efforts, and actions

Not supported today:

- direct `control_msgs/msg/JointJog` capture from `joint_command`
- direct `geometry_msgs/msg/Twist` capture from `cmd_vel`
- `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`
- `nav_msgs/msg/Odometry`
- `sensor_msgs/msg/Imu`
- multiple image streams at the same time, such as RGB + depth
- explicit episode start/end marker topic or service

## Workspace Rule

Edit source files in this repository, but run ROS 2 workspace commands from:

```bash
/home/yg/Workspace/cloi_ws
```

This package is exposed there through:

```bash
src/simulator/ros2_robot_data_collector
```

Do not run `colcon build`, `colcon test`, `ros2 launch`, or `ros2 run` from this repository root unless you first `cd /home/yg/Workspace/cloi_ws`.

## Dependencies

- ROS 2 with `rclcpp`, `sensor_msgs`, and `std_msgs`
- HDF5 development libraries (`libhdf5-dev`)
- CMake 3.16+
- C++17 compiler

## Build

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros2_robot_data_collector
source install/setup.bash
```

## CLOiSim Topic Audit

The current collector was compared against `cloisim_ros` topic publishers and subscribers. For a robot whose model namespace is `CLOiD`, the practical mapping is:

| Signal | Typical CLOiSim topic under `CLOiD` | Message type | Status in this collector | Notes |
| --- | --- | --- | --- | --- |
| Joint state | `/CLOiD/joint_states` | `sensor_msgs/msg/JointState` | Supported now | Published by CLOiSim joint control. This is the most important state topic and already fits the current HDF5 schema. |
| Joint action | `/CLOiD/joint_command` | `control_msgs/msg/JointJog` | Not supported yet | This is the natural action source for articulated robots, but the collector only records `Float64MultiArray` actions today. |
| Mobile action | `/CLOiD/cmd_vel` | `geometry_msgs/msg/Twist` | Not supported yet | Relevant for mobile robots using micom. Not recorded by the current collector. |
| RGB image | `/CLOiD/<camera_part>/camera/image_raw` | `sensor_msgs/msg/Image` | Supported with configuration | `<camera_part>` depends on the SDF part name. You must inspect the actual topic and configure it explicitly. |
| Depth image | `/CLOiD/<depth_part>/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Supported as an alternate single image stream | You can record depth instead of RGB, but not RGB and depth simultaneously. |
| LiDAR | `/CLOiD/scan` | `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2` | Not supported yet | Requires new subscribers and a new HDF5 schema. |
| Odometry | `/CLOiD/odom` | `nav_msgs/msg/Odometry` | Not supported yet | Useful state for mobile training, but not recorded today. |
| IMU | `/CLOiD/imu/data_raw` or `/CLOiD/<realsense_part>/imu` | `sensor_msgs/msg/Imu` | Not supported yet | Exact topic depends on the sensor plugin. |
| Range / sonar / IR | `/CLOiD/<range_part>/range` | `sensor_msgs/msg/Range` | Not supported yet | Requires new subscribers and dataset layout. |

The biggest mismatch is action capture. CLOiSim already uses `joint_command` or `cmd_vel`, but this collector expects a numeric `Float64MultiArray` on `action_topic`.

## HDF5 Output Today

The current writer creates:

- `/timestamps`
- `/observations/images/data`
- `/observations/joint_state/positions`
- `/observations/joint_state/velocities`
- `/observations/joint_state/efforts`
- `/actions/data`
- metadata under `/meta`

The first accepted frame locks the schema. Later frames with a different image shape, encoding, joint layout, or action vector size are rejected.

## Inspect Actual CLOiSim Topics First

Do not hardcode camera topics before checking what CLOiSim actually publishes for `CLOiD`.

If CLOiSim bringup is not running yet, one common way to start it is:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch cloisim_ros_bringup bringup_launch.py target_model:=CLOiD
```

In another terminal, inspect the runtime topics:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

export ROBOT_NS=/CLOiD

ros2 topic list | grep "^${ROBOT_NS}/" | sort
ros2 topic info ${ROBOT_NS}/joint_states
ros2 topic list | grep "^${ROBOT_NS}/.*/camera/image_raw$"
ros2 topic list | grep "^${ROBOT_NS}/.*/depth/image_rect_raw$"
ros2 topic info ${ROBOT_NS}/scan
ros2 topic info ${ROBOT_NS}/odom
```

Useful spot checks:

```bash
ros2 topic echo --once ${ROBOT_NS}/joint_states
ros2 topic echo --once ${ROBOT_NS}/scan
```

Pick the exact camera topic after inspection. For example, if CLOiSim publishes `/CLOiD/front_camera/camera/image_raw`, use that exact path instead of guessing from defaults.

## Run The Collector Against CLOiSim Today

### Recommended Path: Use `ros2 run` With Absolute Topic Names

This is the safest runtime path because the current launch file only exposes `params_file`, `namespace_prefix`, and `output_path`. It does not expose `image_topic`, `joint_state_topic`, or `action_topic` as launch arguments.

Example with a `CLOiD` robot and one discovered camera topic:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

export ROBOT_NS=/CLOiD
export CAMERA_TOPIC=/CLOiD/front_camera/camera/image_raw

ros2 run ros2_robot_data_collector collector_node --ros-args \
  -p namespace_prefix:="" \
  -p image_topic:="$CAMERA_TOPIC" \
  -p joint_state_topic:="$ROBOT_NS/joint_states" \
  -p action_topic:="$ROBOT_NS/actions" \
  -p output_path:=/tmp/CLOiD_episode_0001.h5 \
  -p sync_slop_ms:=100 \
  -p batch_size:=32 \
  -p flush_interval_ms:=1000
```

Why `namespace_prefix` is empty here:

- absolute topic names already include `/CLOiD/...`
- camera topics often include a part name, so absolute paths are clearer than mixing a namespace prefix with a guessed relative path

### Optional Path: Use `ros2 launch` With A Custom Params File

If you prefer `ros2 launch`, create a custom params YAML that overrides the exact topics. The built-in launch file does not expose those topic names directly.

Example YAML content:

```yaml
collector_node:
  ros__parameters:
    namespace_prefix: ""
    image_topic: "/CLOiD/front_camera/camera/image_raw"
    joint_state_topic: "/CLOiD/joint_states"
    action_topic: "/CLOiD/actions"
    output_path: "/tmp/CLOiD_episode_0001.h5"
    sync_slop_ms: 100
```

Then launch:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch ros2_robot_data_collector collector.launch.py \
  params_file:=/path/to/collector_cloid.yaml \
  output_path:=/tmp/CLOiD_episode_0001.h5
```

## Action Semantics For Training

### What You Probably Want

For a CLOiSim articulated robot, the natural action is usually:

- `/CLOiD/joint_command` with `control_msgs/msg/JointJog`

For a mobile robot, the natural action is often:

- `/CLOiD/cmd_vel` with `geometry_msgs/msg/Twist`

### What The Collector Actually Records Today

The collector only records:

- `std_msgs/msg/Float64MultiArray` on `action_topic`

That means the collector cannot directly capture your real CLOiSim control command yet. If your training dataset must store actions now, you need a numeric mirror topic such as `/CLOiD/actions` whose vector layout you define and keep consistent.

Examples:

- joint robot convention: `[joint_1_displacement, joint_1_velocity, joint_2_displacement, joint_2_velocity, ...]`
- mobile robot convention: `[linear_x, linear_y, angular_z]`

### Practical Joint Example Today

Terminal 1, collector:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

export ROBOT_NS=/CLOiD
export CAMERA_TOPIC=/CLOiD/front_camera/camera/image_raw

ros2 run ros2_robot_data_collector collector_node --ros-args \
  -p namespace_prefix:="" \
  -p image_topic:="$CAMERA_TOPIC" \
  -p joint_state_topic:="$ROBOT_NS/joint_states" \
  -p action_topic:="$ROBOT_NS/actions" \
  -p output_path:=/tmp/CLOiD_episode_0001.h5
```

Terminal 2, publish the numeric training action that will be stored in HDF5:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 topic pub -1 /CLOiD/actions std_msgs/msg/Float64MultiArray \
"{data: [-1.5708, 2.0]}"
```

Terminal 3, send the real CLOiSim joint command:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 topic pub -1 /CLOiD/joint_command control_msgs/msg/JointJog \
"{joint_names: ['link_Display_JOINT'], displacements: [-1.5708], velocities: [2.0]}"
```

This is only a workaround. The stored action vector and the real `JointJog` command can drift if you do not keep the conventions and timing aligned.

### Timing Caveat

The current collector timestamps actions with collector receive time, not with a message header. CLOiSim sensor streams use simulator timestamps. Because of that mismatch:

- publish the numeric `/CLOiD/actions` message as close as possible to the real control command
- keep `sync_slop_ms` realistic for your simulator timing
- do not assume strict replay-perfect alignment from the current implementation

## Episode Semantics Today

### Current Behavior

There is no explicit episode marker topic or service in the current collector.

Today, the practical episode boundary is:

- one collector process per episode
- one output file per episode
- end the episode by stopping the collector process

Example workflow:

1. Start collector with `output_path:=/data/CLOiD_episode_0001.h5`.
2. Run the task and publish matching numeric actions on `/CLOiD/actions`.
3. When the episode ends, stop the collector with `Ctrl-C`.
4. Start the next episode with a new output path such as `/data/CLOiD_episode_0002.h5`.

What `Ctrl-C` means here:

- it stops the node
- the writer thread flushes pending data
- the HDF5 file is finalized and closed

### What Is Not Available Yet

These commands do not exist today:

- no `/end_episode` service
- no `/episode_marker` topic
- no automatic file rotation per episode
- no success or failure label channel

If you need explicit episode boundaries, that requires code changes.

## Should LiDAR Become A Dataset?

Probably yes, but not with the current implementation.

If your policy depends on navigation or obstacle avoidance, LiDAR is a reasonable training observation. The problem is not whether LiDAR is useful. The problem is schema design.

Current constraints:

- the collector only knows how to store one image stream, one joint-state stream, and one numeric action stream
- CLOiSim LiDAR can publish either `LaserScan` or `PointCloud2`
- `LaserScan` is easier to store than `PointCloud2`, but still needs a new subscriber and a new HDF5 group
- `PointCloud2` is larger and more complex, and needs a more careful storage layout

Practical recommendation:

1. First add direct `joint_command` action capture.
2. Then add `LaserScan` support if LiDAR is important for the task.
3. Add `PointCloud2`, odometry, IMU, and explicit episode markers only after the basic action path is correct.

## Parameters

Default values are defined in [config/collector.yaml](config/collector.yaml).

| Parameter | Default | Description |
| --- | --- | --- |
| `namespace_prefix` | `""` | Prefix applied only when a topic name is relative. Absolute topic names bypass this. |
| `image_topic` | `camera/front/image_raw` | Image topic to subscribe to. This default is usually not the exact CLOiSim camera topic. |
| `joint_state_topic` | `joint_states` | Joint-state topic to subscribe to. |
| `action_topic` | `actions` | Numeric action topic to subscribe to. |
| `output_path` | `/tmp/ros2_robot_data.h5` | Destination HDF5 file path. |
| `topic_queue_depth` | `64` | Bounded queue size for image and joint-state samples. |
| `writer_queue_depth` | `512` | Bounded queue size for frames waiting to be written. |
| `batch_size` | `32` | Number of frames written per batch. |
| `sync_slop_ms` | `50` | Maximum allowed timestamp span for synchronized samples. |
| `flush_interval_ms` | `1000` | Maximum time between writer flushes. |
| `approximate_sync` | `true` | If `false`, image and joint-state timestamps must match exactly. |
| `enable_compression` | `false` | Enables HDF5 deflate compression for image data. |

## Testing

### Unit Tests

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros2_robot_data_collector
source install/setup.bash
colcon test --packages-select ros2_robot_data_collector
colcon test-result --verbose
```

Current automated coverage is limited to helper logic:

- [test/test_sync_policy.cpp](test/test_sync_policy.cpp)
- [test/test_topic_resolution.cpp](test/test_topic_resolution.cpp)

### Manual Smoke Test

This validates the collector path itself, not CLOiSim integration.

Terminal 1:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run ros2_robot_data_collector collector_node --ros-args \
  -p output_path:=/tmp/collector_smoke.h5 \
  -p sync_slop_ms:=5000 \
  -p batch_size:=1 \
  -p flush_interval_ms:=100
```

Terminal 2:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

stamp_sec=$(date +%s)
stamp_nsec=$(date +%N)

ros2 topic pub --once /camera/front/image_raw sensor_msgs/msg/Image \
"{header: {stamp: {sec: $stamp_sec, nanosec: $stamp_nsec}, frame_id: camera_front}, height: 1, width: 1, encoding: mono8, is_bigendian: 0, step: 1, data: [42]}"

ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: $stamp_sec, nanosec: $stamp_nsec}}, name: [joint_1, joint_2], position: [0.1, 0.2], velocity: [0.0, 0.0], effort: [0.0, 0.0]}"

ros2 topic pub --once /actions std_msgs/msg/Float64MultiArray \
"{data: [0.5, 0.6]}"

ls -l /tmp/collector_smoke.h5
```

Expected result:

- the node logs `frames=1` and `sync_miss=0`
- `/tmp/collector_smoke.h5` exists

If you want to inspect the HDF5 structure with `h5ls`, install:

```bash
sudo apt install hdf5-tools
```

## Known Limitations

- action timestamps currently use collector receive time, not a timestamp carried by the action message
- queue sizes are bounded, so old samples can be dropped under load
- only one image stream is supported at a time
- the first accepted frame fixes the output schema
- there is no direct support for `joint_command`, `cmd_vel`, LiDAR, odometry, IMU, or explicit episode markers
- [src/sample_data_publisher.cpp](src/sample_data_publisher.cpp) exists but is not added as a CMake target, so it cannot be run with `ros2 run` yet
- there is no automated end-to-end integration test for CLOiSim

## Code Layout

- [src/collector_node.cpp](src/collector_node.cpp): ROS 2 node and synchronization logic
- [src/hdf5_writer.cpp](src/hdf5_writer.cpp): HDF5 writer thread and dataset management
- [include/ros2_robot_data_collector/collector_config.hpp](include/ros2_robot_data_collector/collector_config.hpp): parameter loading and validation
- [include/ros2_robot_data_collector/topic_resolver.hpp](include/ros2_robot_data_collector/topic_resolver.hpp): namespace-aware topic resolution
- [include/ros2_robot_data_collector/sync_policy.hpp](include/ros2_robot_data_collector/sync_policy.hpp): timestamp window checks
