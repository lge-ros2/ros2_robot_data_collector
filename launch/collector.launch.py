from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    namespace_prefix = LaunchConfiguration("namespace_prefix")
    output_path = LaunchConfiguration("output_path")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("ros2_robot_data_collector"), "config", "collector.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Collector parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "namespace_prefix",
                default_value="",
                description="Optional namespace prefix applied to relative topics.",
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="/tmp/ros2_robot_data.h5",
                description="Target HDF5 path for synchronized data collection.",
            ),
            Node(
                package="ros2_robot_data_collector",
                executable="collector_node",
                name="collector_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "namespace_prefix": namespace_prefix,
                        "output_path": output_path,
                    },
                ],
            ),
        ]
    )