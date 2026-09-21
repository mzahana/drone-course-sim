from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    ff = LaunchConfiguration("feedforward")
    return LaunchDescription([
        DeclareLaunchArgument(
            "feedforward", default_value="true",
            description="Set false to see the steady-state lag v_T/Kp for yourself."),
        Node(package="lab5_follow", executable="target_tracker.py",
             name="target_tracker", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="lab5_follow", executable="follow_guidance.py",
             name="follow_guidance", output="screen",
             parameters=[{"use_sim_time": True,
                          "feedforward": ParameterValue(ff, value_type=bool)}]),
    ])
