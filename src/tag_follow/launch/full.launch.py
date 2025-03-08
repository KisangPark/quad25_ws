"""
ROS2 launch file to
1) activate robot
2) execute two nodes

++ basic launch & executable files to create
"""


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    package_name = "tag_follow"

    #workspace package
    pkg_path = os.path.join(get_package_share_directory(package_name))

    #tag_location node
    tag_location = Node(
    package=package_name,
    executable='tag_location',
    name='tag_location',
    )

    #follower node
    follower = Node(
    package=package_name,
    executable='follower',
    name='follower',
    )

    #fake_robot node
    fake_robot = Node(
    package=package_name,
    executable='fake_robot',
    name='fake_robot',
    )

    return LaunchDescription(
        [
            tag_location,
            follower,
            fake_robot,
        ]
    )

