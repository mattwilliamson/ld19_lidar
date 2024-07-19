"""Publish URDF for LD19 LIDAR"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_name = "ld19.urdf"
    urdf = os.path.join(get_package_share_directory("ld19_lidar"), "urdf", urdf_file_name)
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="ldlidar_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
                arguments=[urdf],
            )
        ]
    )
