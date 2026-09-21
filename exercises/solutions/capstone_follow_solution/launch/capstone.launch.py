"""The whole follower, assembled. Assumes `course sim` and `course bringup`
are already running.

    ros2 launch capstone_follow_solution capstone.launch.py score:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

SIM = {"use_sim_time": True}


def generate_launch_description():
    ff = LaunchConfiguration("feedforward")
    return LaunchDescription([
        DeclareLaunchArgument("feedforward", default_value="true"),
        DeclareLaunchArgument("score", default_value="false"),
        DeclareLaunchArgument("duration", default_value="120.0"),
        DeclareLaunchArgument("tier", default_value="1"),

        Node(package="lab4_perception_solution", executable="target_locator.py",
             name="target_locator", output="screen", parameters=[SIM]),
        Node(package="lab4_perception_solution", executable="gimbal_pointer.py",
             name="gimbal_pointer", output="screen", parameters=[SIM]),
        Node(package="lab5_follow_solution", executable="target_tracker.py",
             name="target_tracker", output="screen", parameters=[SIM]),
        Node(package="lab5_follow_solution", executable="follow_guidance.py",
             name="follow_guidance", output="screen",
             parameters=[SIM, {"start_enabled": False,
                               "feedforward": ParameterValue(ff, value_type=bool)}]),
        Node(package="capstone_follow_solution", executable="mission_manager.py",
             name="mission_manager", output="screen", parameters=[SIM]),

        Node(package="drone_course_sim", executable="scoring_node.py",
             name="scoring", output="screen",
             condition=IfCondition(LaunchConfiguration("score")),
             parameters=[SIM, {
                 "duration": ParameterValue(LaunchConfiguration("duration"),
                                            value_type=float),
                 "tier": ParameterValue(LaunchConfiguration("tier"), value_type=int)}]),
    ])
