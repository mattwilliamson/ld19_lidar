import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('ld19_lidar')

    # RVIZ2 settings
    rviz2_config = os.path.join(share_dir, 'config', 'ldlidar.rviz')

    # RVIZ2node
    rviz2_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=[["-d"], [rviz2_config]])
    ld.add_action(rviz2_node)

    # Launch node
    ldlidar_launch = IncludeLaunchDescription(launch_description_source=PythonLaunchDescriptionSource([share_dir, '/launch/node.launch.py']))
    ld.add_action(ldlidar_launch)

    # Robot State Publisher
    rsp_launch = IncludeLaunchDescription(launch_description_source=PythonLaunchDescriptionSource([share_dir, '/launch/rsp.launch.py']))
    ld.add_action(rsp_launch)

    return ld