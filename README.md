# ros2_robot_data_collector

Namespace-aware ROS 2 data collector that synchronizes observations and actions and writes them to HDF5.

Current implementation scope:

- one `sensor_msgs/msg/Image` stream
- one joint-centric `sensor_msgs/msg/JointState` stream
- zero or one `sensor_msgs/msg/LaserScan` stream via `lidar_topic`
- exactly one action source chosen from:
  - `std_msgs/msg/Float64MultiArray` on `action_topic`
  - `control_msgs/msg/JointJog` on `joint_action_topic`
- optional mobile-base action path:
  - `geometry_msgs/msg/Twist` on `cmd_vel_action_topic`
- explicit episode control through services
  - `/collector_node/start_episode`
  - `/collector_node/end_episode`

Still not supported:

- `sensor_msgs/msg/PointCloud2`
- `nav_msgs/msg/Odometry`
- `sensor_msgs/msg/Imu`
- multiple image streams at the same time, such as RGB + depth
- automatic file rotation per episode

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

- ROS 2 with `rclcpp`, `control_msgs`, `sensor_msgs`, `std_msgs`, and `std_srvs`
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

The current collector was compared against `cloisim_ros` publishers and subscribers. For a robot whose model namespace is `CLOiD`, the practical mapping is:

| Signal | Typical CLOiSim topic under `CLOiD` | Message type | Status in this collector | Notes |
| --- | --- | --- | --- | --- |
| Joint state | `/CLOiD/joint_states` | `sensor_msgs/msg/JointState` | Supported now | This is the most important state topic and fits the current HDF5 schema directly. |
| Joint action | `/CLOiD/joint_command` | `control_msgs/msg/JointJog` | Supported now | Configure `joint_action_topic` and leave `action_topic` empty. The HDF5 action vector is stored as `[displacements..., velocities...]`. |
| Mobile action | `/CLOiD/cmd_vel` | `geometry_msgs/msg/Twist` | Supported as an optional path | Keep this secondary unless mobile-base action capture actually matters. The collector still depends on `/joint_states`, and the dataset schema remains joint-centric. |
| RGB image | `/CLOiD/<camera_part>/camera/image_raw` | `sensor_msgs/msg/Image` | Supported with configuration | `<camera_part>` depends on the SDF part name. Inspect the actual topic first. |
| Depth image | `/CLOiD/<depth_part>/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Supported as an alternate single image stream | You can record depth instead of RGB, but not RGB and depth simultaneously. |
| LiDAR | `/CLOiD/scan` | `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2` | `LaserScan` supported now, `PointCloud2` not supported | Configure `lidar_topic` to store `/observations/lidar/ranges` and `/observations/lidar/intensities`. The first accepted frame locks beam count and scan metadata. |
| Odometry | `/CLOiD/odom` | `nav_msgs/msg/Odometry` | Not supported yet | Useful state for mobile training, but not recorded today. |
| IMU | `/CLOiD/imu/data_raw` or `/CLOiD/<realsense_part>/imu` | `sensor_msgs/msg/Imu` | Not supported yet | Exact topic depends on the sensor plugin. |
| Range / sonar / IR | `/CLOiD/<range_part>/range` | `sensor_msgs/msg/Range` | Not supported yet | Requires new subscribers and dataset layout. |

For CLOiSim training data, `joint_states` should stay the primary focus. The main remaining gaps are `PointCloud2`, odometry, IMU, and the fact that the collector still requires a `joint_state` stream even when the optional action source is `cmd_vel`.

## HDF5 Output Today

The writer currently creates:

- `/timestamps`
- `/episodes/index`
- `/observations/images/data`
- `/observations/joint_state/positions`
- `/observations/joint_state/velocities`
- `/observations/joint_state/efforts`
- `/observations/lidar/ranges` when `lidar_topic` is configured
- `/observations/lidar/intensities` when `lidar_topic` is configured
- `/actions/data`
- metadata under `/meta`
- action metadata under `/actions`

Action metadata includes:

- `layout`
- `labels_csv`

Action layouts currently used are:

- `float64_multi_array`
- `joint_jog_displacements_then_velocities`
- `twist_linear_then_angular`

If `lidar_topic` is enabled, the first accepted frame also locks LaserScan beam count and scan metadata. Later frames with a different image shape, joint layout, lidar layout, or action layout are rejected.

## Joint State Guardrails

Joint-state quality is easy to corrupt silently if joint names drift while the numeric arrays keep the same length. The collector now rejects that case instead of writing mislabeled columns.

- the first accepted frame must provide distinct non-empty joint names
- later frames must keep the exact same joint names in the exact same order
- if names disappear, duplicate, or reorder, the frame is rejected instead of shifting data under the wrong column labels

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
ros2 topic info ${ROBOT_NS}/joint_command
ros2 topic list | grep "^${ROBOT_NS}/.*/camera/image_raw$"
ros2 topic list | grep "^${ROBOT_NS}/.*/depth/image_rect_raw$"
ros2 topic info ${ROBOT_NS}/scan
ros2 topic info ${ROBOT_NS}/odom
```

Useful spot checks:

```bash
ros2 topic echo --once ${ROBOT_NS}/joint_states
ros2 topic echo --once ${ROBOT_NS}/joint_command
```

Pick the exact camera topic after inspection. For example, if CLOiSim publishes `/CLOiD/front_camera/camera/image_raw`, use that exact path instead of guessing from defaults.

## How To Run It For CLOiD

### Recommended Joint-First Path

For `joint_command`, the cleanest runtime path is a params YAML because `action_topic` must be empty and `joint_action_topic` must be set.

Create a config file:

```yaml
collector_node:
  ros__parameters:
    namespace_prefix: ""
    image_topic: "/CLOiD/front_camera/camera/image_raw"
    joint_state_topic: "/CLOiD/joint_states"
    lidar_topic: ""
    action_topic: ""
    joint_action_topic: "/CLOiD/joint_command"
    cmd_vel_action_topic: ""
    output_path: "/tmp/CLOiD_episode_data.h5"
    sync_slop_ms: 100
    batch_size: 32
    flush_interval_ms: 1000
```

Run the collector:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run ros2_robot_data_collector collector_node --ros-args --params-file /path/to/collector_cloid_joint.yaml
```

This should stay the default workflow unless you have a concrete reason to model base velocity commands directly.

### Alternative Path For Numeric Action Topics

If you already have a numeric action topic such as `/CLOiD/actions`, keep `action_topic` set and leave `joint_action_topic` empty.

Exactly one action source must be configured. If multiple action topics are set, or all of them are empty, the node rejects the configuration on startup.

### Optional Mobile-Base Path

Keep this as a secondary path. It is useful only if the robot still publishes a compatible `joint_state` topic, and `cmd_vel` support does not remove that requirement.

```yaml
collector_node:
  ros__parameters:
    namespace_prefix: ""
    image_topic: "/CLOiD/front_camera/camera/image_raw"
    joint_state_topic: "/CLOiD/joint_states"
    lidar_topic: "/CLOiD/scan"
    action_topic: ""
    joint_action_topic: ""
    cmd_vel_action_topic: "/CLOiD/cmd_vel"
    output_path: "/tmp/CLOiD_cmd_vel_scan.h5"
    sync_slop_ms: 100
    batch_size: 32
    flush_interval_ms: 1000
```

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run ros2_robot_data_collector collector_node --ros-args --params-file /path/to/collector_cloid_cmd_vel_scan.yaml
```

## What Gets Stored As Action

### `action_topic`

If you use `std_msgs/msg/Float64MultiArray`, the raw numeric vector is stored in `/actions/data`.

### `joint_action_topic`

If you use `control_msgs/msg/JointJog`, the collector flattens each message into:

```text
[joint_1_displacement, joint_2_displacement, ..., joint_1_velocity, joint_2_velocity, ...]
```

Missing displacement or velocity values are padded with `NaN` to keep a fixed action width.

The corresponding labels are written into the HDF5 `/actions` metadata as `labels_csv`.

Example labels:

```text
joint_1/displacement,joint_2/displacement,joint_1/velocity,joint_2/velocity
```

### Optional `cmd_vel_action_topic`

If you use `geometry_msgs/msg/Twist`, the collector stores:

```text
[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
```

The corresponding labels are written into the HDF5 `/actions` metadata as:

```text
linear/x,linear/y,linear/z,angular/x,angular/y,angular/z
```

## Episode Semantics

The collector starts in `episode 1` and accepts frames immediately.

Explicit episode control is now available through two services:

- `/collector_node/start_episode`
- `/collector_node/end_episode`

Behavior:

- `end_episode` stops accepting new action-triggered frames
- `start_episode` begins the next episode index
- each accepted frame stores its episode number in `/episodes/index`
- the HDF5 file stays open; episodes are segmented inside one file

### Example Episode Flow

Terminal 1, start the collector:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run ros2_robot_data_collector collector_node --ros-args --params-file /path/to/collector_cloid_joint.yaml
```

Terminal 2, send robot actions during episode 1:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic pub -1 /CLOiD/joint_command control_msgs/msg/JointJog \
"{joint_names: ['link_Display_JOINT'], displacements: [-1.5708], velocities: [2.0]}"
```

When the behavior for that episode is finished:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 service call /collector_node/end_episode std_srvs/srv/Trigger '{}'
```

Start the next episode in the same file:

```bash
cd /home/yg/Workspace/cloi_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 service call /collector_node/start_episode std_srvs/srv/Trigger '{}'
```

If you prefer one file per episode instead of one file with `/episodes/index`, restart the collector with a different `output_path` for each run.

### What `end_episode` Does Not Do

It does not:

- close the file
- rotate to a new file automatically
- emit success or failure labels
- reset queues

It only stops accepting new action-triggered frames until `start_episode` is called again.

## Timing Caveat

Action timestamps still use collector receive time, not a timestamp from the action message header. CLOiSim sensor streams use simulator timestamps. Because of that mismatch:

- publish actions as close as possible to the actual control moment
- keep `sync_slop_ms` realistic for your simulator timing
- do not assume replay-perfect temporal alignment from the current implementation

## How LaserScan Is Stored

When `lidar_topic` is configured, the collector stores:

- `/observations/lidar/ranges`
- `/observations/lidar/intensities`

LaserScan metadata is written as attributes under `/observations/lidar`:

- `beam_count`
- `angle_min`
- `angle_max`
- `angle_increment`
- `time_increment`
- `scan_time`
- `range_min`
- `range_max`

If the publisher omits some intensity values, the collector pads them with `NaN` so the dataset width stays fixed.

`sensor_msgs/msg/PointCloud2` is still unsupported.

## Parameters

Default values are defined in [config/collector.yaml](config/collector.yaml).

| Parameter | Default | Description |
| --- | --- | --- |
| `namespace_prefix` | `""` | Prefix applied only when a topic name is relative. Absolute topic names bypass this. |
| `image_topic` | `camera/front/image_raw` | Image topic to subscribe to. This default is usually not the exact CLOiSim camera topic. |
| `joint_state_topic` | `joint_states` | Joint-state topic to subscribe to. |
| `lidar_topic` | `""` | Optional `sensor_msgs/msg/LaserScan` topic. Leave empty to disable LiDAR capture. |
| `action_topic` | `actions` | Numeric action topic. Leave this empty if you use `joint_action_topic` or `cmd_vel_action_topic`. |
| `joint_action_topic` | `""` | `control_msgs/msg/JointJog` action topic. Leave this empty if you use `action_topic` or `cmd_vel_action_topic`. |
| `cmd_vel_action_topic` | `""` | Optional `geometry_msgs/msg/Twist` action topic for mobile-base workflows. Leave this empty if you use `action_topic` or `joint_action_topic`. |
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

Current automated coverage includes:

- [test/test_hdf5_writer.cpp](test/test_hdf5_writer.cpp)
- [test/test_sync_policy.cpp](test/test_sync_policy.cpp)
- [test/test_topic_resolution.cpp](test/test_topic_resolution.cpp)
- [test/test_action_sample.cpp](test/test_action_sample.cpp)

### Manual Smoke Tests Already Verified

The following were verified during development:

- numeric action capture still writes HDF5 output
- `joint_action_topic` accepts `control_msgs/msg/JointJog`
- joint-state schema initialization now rejects unnamed or duplicated joint names
- later frames are rejected if joint names reorder under the same numeric width
- `lidar_topic` accepts `sensor_msgs/msg/LaserScan`
- `/collector_node/end_episode` stops new frames from being accepted
- `/episodes/index` is written alongside accepted frames
- the optional `cmd_vel_action_topic` path accepts `geometry_msgs/msg/Twist`
- a runtime smoke test produced one synchronized frame with image + joint state + LaserScan + `cmd_vel`

If you want to inspect the HDF5 structure with `h5ls`, install:

```bash
sudo apt install hdf5-tools
```

## Known Limitations

- action timestamps still use collector receive time
- queue sizes are bounded, so old samples can be dropped under load
- only one image stream is supported at a time
- the first accepted frame fixes the output schema
- `joint_state_topic` is still mandatory even when `cmd_vel_action_topic` is used
- `sensor_msgs/msg/PointCloud2`, odometry, IMU, and range sensors are still unsupported
- there is no automatic file rotation per episode
- [src/sample_data_publisher.cpp](src/sample_data_publisher.cpp) exists but is not added as a CMake target, so it cannot be run with `ros2 run` yet

## Code Layout

- [src/collector_node.cpp](src/collector_node.cpp): ROS 2 node, action subscriptions, and episode services
- [src/hdf5_writer.cpp](src/hdf5_writer.cpp): HDF5 writer thread and dataset management
- [include/ros2_robot_data_collector/collector_config.hpp](include/ros2_robot_data_collector/collector_config.hpp): parameter loading and validation
- [include/ros2_robot_data_collector/sample.hpp](include/ros2_robot_data_collector/sample.hpp): message-to-sample conversion for numeric actions, `JointJog`, `Twist`, and `LaserScan`
- [include/ros2_robot_data_collector/topic_resolver.hpp](include/ros2_robot_data_collector/topic_resolver.hpp): namespace-aware topic resolution
- [include/ros2_robot_data_collector/sync_policy.hpp](include/ros2_robot_data_collector/sync_policy.hpp): timestamp window checks
