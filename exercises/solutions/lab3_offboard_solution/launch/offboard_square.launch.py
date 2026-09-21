from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="lab3_offboard_solution", executable="offboard_square.py",
             name="offboard_square", output="screen",
             parameters=[{"use_sim_time": True, "altitude": 5.0, "side": 10.0}]),
    ])
