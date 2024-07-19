import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():

    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(get_package_share_directory("ld19_lidar"), "params", "slam_toolbox.yaml")

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

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [get_package_share_directory("ld19_lidar"), "/launch/lidar.launch.py"]
        ),
    )

    # Fake odom publisher
    fake_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "laser"],
    )

    # RVIZ2 settings
    rviz2_config = os.path.join(get_package_share_directory("ldlidar_node"), "config", "ldlidar_slam.rviz")

    # RVIZ2node
    rviz2_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=[["-d"], [rviz2_config]]
    )

    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [get_package_share_directory("ld19_lidar"), "/launch/lidar.launch.py"]
        ),
    )

    rsp_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [get_package_share_directory("ld19_lidar"), "/launch/rsp.launch.py"]
        ),
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    ld.add_action(rsp_launch)
    ld.add_action(slam_toolbox_node)
    ld.add_action(fake_odom)
    ld.add_action(ldlidar_launch)
    ld.add_action(rviz2_node)

    return ld
