"""
turtlebot & ROS2 navigation launcher

-> other nodes can be added
"""

#launch file essential libraries
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

#others
#none

def generate_launch_description():
    #workspace package
    pkg_path = os.path.join(get_package_share_directory('week3'))
    package_name = "week3"

    #turtlebot
    turtle_simul_path = "/home/kisangpark/lab_training/src/turtlebot3_simulations/turtlebot3_gazebo"
    turtle_navi_path = "/home/kisangpark/lab_training/src/turtlebot3/turtlebot3_navigation2"

    #ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    turtlebot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch", "turtlebot3_world.launch.py")]
        ),#or directly input file directory
        #src/turtlebot3_simulations/turtlebot3_gazebo/launch
        #get_package_share_directory("turtlebot3_gazebo")
    )

    #ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("turtlebot3_navigation2"), "launch", "navigation2.launch.py")]
        ),#get_package_share_directory("turtlebot3_navigation2")
        launch_arguments={"use_sim_time": "true", "map": "$HOME/map.yaml"}.items(),
    )

    #get_frame node
    # get_frame = Node(
    # package='arm',
    # executable='get_frame',
    # name='get_frame',
    # )

    return LaunchDescription(
        [
            turtlebot_gazebo,
            nav2_launch,
        ]
    )
