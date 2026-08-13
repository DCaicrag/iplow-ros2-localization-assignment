"""Launch the complete iPlow bag localization and visualization stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Create the localization stack launch description."""

    bag_path = LaunchConfiguration("bag_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    package_share = FindPackageShare("robot_bringup")

    urdf_file = PathJoinSubstitution(
        [package_share, "urdf", "robot.urdf.xacro"]
    )

    rviz_file = PathJoinSubstitution(
        [package_share, "rviz", "config.rviz"]
    )

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str,
    )

    declare_bag_path = DeclareLaunchArgument(
        "bag_path",
        description="Path to the ROS2 bag directory.",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated time from the bag clock.",
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    localization_node = Node(
        package="robot_bringup",
        executable="localization_node.py",
        name="localization_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_file,
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_bag_path,
            declare_use_sim_time,
            bag_play,
            robot_state_publisher,
            joint_state_publisher,
            localization_node,
            map_to_odom,
            rviz,
        ]
    )