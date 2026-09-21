from launch import LaunchDescription
from launch_ros.actions import Node

SIM = {"use_sim_time": True}


def generate_launch_description():
    return LaunchDescription([
        Node(package="lab4_perception_solution", executable="target_locator.py",
             name="target_locator", output="screen", parameters=[SIM]),
        Node(package="lab4_perception_solution", executable="gimbal_pointer.py",
             name="gimbal_pointer", output="screen", parameters=[SIM]),
    ])
