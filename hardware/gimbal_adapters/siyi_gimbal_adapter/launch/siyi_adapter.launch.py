"""Start the SIYI gimbal adapter with its parameter file.

    ros2 launch siyi_gimbal_adapter siyi_adapter.launch.py

The siyi_ros2 driver is started separately (ros2 launch siyi_ros2 siyi.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("siyi_gimbal_adapter"),
                          "config", "siyi_adapter.yaml")
    return LaunchDescription([
        Node(package="siyi_gimbal_adapter", executable="siyi_gimbal_adapter",
             name="siyi_gimbal_adapter", output="screen",
             parameters=[params, {"use_sim_time": False}]),
    ])
