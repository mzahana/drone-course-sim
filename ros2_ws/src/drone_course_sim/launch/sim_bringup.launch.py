"""Bring the simulated drone into ROS.

Assumes PX4 SITL is already running (`course sim`). This adds the ROS side:
the autopilot link, the camera, and the TF tree.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MODEL = "x500_course_gimbal_0"
CAM = f"/world/course_world/model/{MODEL}/link/camera_link/sensor/camera/image"


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
        DeclareLaunchArgument(
            "detector", default_value="true",
            description="Run YOLO11n on the camera stream."),

        # MAVROS via node.launch rather than px4.launch: px4.launch hardcodes its
        # config path, and we need local-position TF on and sim time enabled.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("mavros"), "launch", "node.launch"])
            ]),
            launch_arguments={
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": "",
                "tgt_system": "1",
                "tgt_component": "1",
                "pluginlists_yaml": PathJoinSubstitution(
                    [pkg, "config", "mavros_pluginlists.yaml"]),
                "config_yaml": PathJoinSubstitution(
                    [pkg, "config", "mavros_px4.yaml"]),
            }.items(),
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

        # YOLO11n on the camera stream. Course-provided: students consume
        # /detections, they do not write the detector.
        Node(
            package="drone_course_sim", executable="detector_node.py",
            name="detector", output="screen",
            condition=IfCondition(LaunchConfiguration("detector")),
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # map -> base_link from the autopilot's position estimate.
        Node(
            package="drone_course_sim", executable="vehicle_tf_node.py",
            name="vehicle_tf", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Topic interface to the gimbal. Mirrors how the A8 mini is driven on
        # the real aircraft: from the onboard computer, not via the autopilot.
        Node(
            package="drone_course_sim", executable="gimbal_interface_node.py",
            name="gimbal_interface", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # base_link -> ... -> camera_optical_frame, from measured joint angles.
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": urdf, "use_sim_time": use_sim_time}],
        ),
    ])
