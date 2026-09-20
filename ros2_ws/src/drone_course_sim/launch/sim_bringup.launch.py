"""Bring the simulated drone into ROS.

Assumes PX4 SITL is already running (`course sim`). This launch adds
everything on the ROS side: the autopilot link, the camera, and the TF tree.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAM = "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image"


def generate_launch_description():
    pkg = FindPackageShare("drone_course_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Always true against SITL. With this false, TF lookups "
                        "by image timestamp silently return the wrong transform."),
        DeclareLaunchArgument(
            "fcu_url", default_value="udp://:14540@127.0.0.1:14557"),

        # Autopilot link.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("mavros"), "launch", "px4.launch"])
            ]),
            launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        ),

        # Clock, camera_info and ground truth.
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_bridge", output="screen",
            parameters=[{
                "config_file": PathJoinSubstitution([pkg, "config", "bridge.yaml"]),
                "use_sim_time": use_sim_time,
            }],
        ),

        # Images go through the dedicated image bridge rather than the
        # parameter bridge: it avoids a full serialise/deserialise per frame.
        Node(
            package="ros_gz_image", executable="image_bridge",
            name="camera_bridge", output="screen",
            arguments=[CAM],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[(CAM, "/camera/image_raw")],
        ),

        # base_link -> camera_link -> camera_optical_frame, from gimbal feedback.
        Node(
            package="drone_course_sim", executable="gimbal_tf_node.py",
            name="gimbal_tf", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
