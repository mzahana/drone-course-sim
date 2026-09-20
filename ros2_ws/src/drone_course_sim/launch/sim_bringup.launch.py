"""Bring the simulated drone into ROS.

Assumes PX4 SITL is already running (`course sim`). This adds the ROS side:
the autopilot link, the camera, and the TF tree.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MODEL = "x500_course_gimbal_0"
CAM = f"/world/default/model/{MODEL}/link/camera_link/sensor/camera/image"


def generate_launch_description():
    pkg = FindPackageShare("drone_course_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # ParameterValue(..., value_type=str) is required: without it launch tries
    # to parse the xacro output as YAML and the whole launch file aborts.
    urdf = ParameterValue(
        Command(["xacro ", PathJoinSubstitution(
            [pkg, "urdf", "x500_course_gimbal.urdf.xacro"])]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Always true against SITL. With this false, TF lookups "
                        "by image timestamp silently return the wrong transform."),
        DeclareLaunchArgument("fcu_url", default_value="udp://:14540@127.0.0.1:14557"),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("mavros"), "launch", "px4.launch"])
            ]),
            launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        ),

        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_bridge", output="screen",
            parameters=[{
                "config_file": PathJoinSubstitution([pkg, "config", "bridge.yaml"]),
                "use_sim_time": use_sim_time,
            }],
        ),

        # Images use the dedicated image bridge: it avoids a full
        # serialise/deserialise round trip per frame.
        Node(
            package="ros_gz_image", executable="image_bridge",
            name="camera_bridge", output="screen",
            arguments=[CAM],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[(CAM, "/camera/image_raw")],
        ),

        # base_link -> ... -> camera_optical_frame, from measured joint angles.
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": urdf, "use_sim_time": use_sim_time}],
        ),
    ])
