import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    node_name = LaunchConfiguration('node_name')

    # Launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the node'
    )

    # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('ld19_lidar'),
        'config',
        'ldlidar.rviz'
    )

    # RVIZ2node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    # Launch node
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ld19_lidar'),
            '/launch/lidar.launch'
        ]),
    )

    rsp_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ld19_lidar'),
            '/launch/rsp.launch'
        ]),
    )

    ld = LaunchDescription()

    ld.add_action(rviz2_node)
    ld.add_action(ldlidar_launch)
    ld.add_action(rsp_launch)

    return ld