import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "SLAM"

    slammer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "slam.launch.py")]
        ),
    )

    # launch path follower, path generator nodes
    pf = Node(
        package="SLAM",
        executable="path_follower",
        output="screen",
    )

    pg = Node(
        package="SLAM",
        executable="path_generator",
        output="screen",
    )

    # Launch them all!
    return LaunchDescription(
        [
            slammer,
            pf,
            pg,
        ]
    )