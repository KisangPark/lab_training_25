"""
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
"""
"""
import os

os.system("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py")
os.system("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True")

"""

import os
import numpy as np
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # turtlebot gazebo launch
    #1. get path
    turtlebot = os.path.join('src','turtlebot3_simulations', 'turtlebot3_gazebo', 'launch', 'turtlebot3_world.launch.py')
    #turtlebot = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')

    #2. cartographer launch
    cartographer = os.path.join('src','turtlebot3', 'turtlebot3_cartographer', 'launch', 'cartographer.launch.py')
    #cartographer = os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartorapher),
            launch_arguments={'use_sim_time': 'True'}.items()
        )
    ])



"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': 'True'}.items()
        )
    ])

"""

