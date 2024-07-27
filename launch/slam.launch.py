import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('ld19_lidar')

    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(share_dir, "params", "slam_toolbox.yaml")

    # SLAM Toolbox node in async mode
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace="",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config_path],  # Parameters
        #   remappings=[
        #       ('/scan', '/ldlidar_node/scan')
        #   ]
    )
    ld.add_action(slam_toolbox_node)


    # Fake odom publisher
    fake_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "laser"],
    )
    ld.add_action(fake_odom)


    # RVIZ2
    enable_rviz = LaunchConfiguration('rviz', default=True)
    rviz_arg = DeclareLaunchArgument('rviz', default_value=enable_rviz, description='When True create a RVIZ window.')
    rviz2_config = os.path.join(share_dir, "config", "ldlidar_slam.rviz")
    rviz2_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=[["-d"], [rviz2_config]], condition=IfCondition(enable_rviz))
    ld.add_action(rviz_arg)

    if LaunchConfigurationEquals('rviz', 'True') and rviz2_node is not None:
        ld.add_action(rviz2_node)


    # LIDAR Node
    ldlidar_launch = IncludeLaunchDescription(launch_description_source=PythonLaunchDescriptionSource([share_dir, "/launch/node.launch.py"]))
    ld.add_action(ldlidar_launch)

    # Robot State Publisher
    rsp_launch = IncludeLaunchDescription(launch_description_source=PythonLaunchDescriptionSource([share_dir, "/launch/rsp.launch.py"]))
    ld.add_action(rsp_launch)

    return ld
