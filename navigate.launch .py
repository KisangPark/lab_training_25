"""
process?

turtlebot launch
navigation
set parameter & service call
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    #basic configs -> use sim time, map location
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    mapfile = LaunchConfiguration('mapfile', default=[EnvironmentVariable('HOME'), '/map.yaml'])

    #launch turtlebot -> description source, python
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),'/turtlebot3_world.launch.py'])
    )

    #machangaji... launch navigation
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch'),'/navigation2.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map}.items()
    )



    #paramters -> executeprocess, command
    #parameters -> make 100000 points (particles) on map
    particle_increase = ExecuteProcess(cmd=['ros2', 'param', 'set', '/amcl', 'max_particles', '100000'],shell=True)
    #service call for global localization and 
    global_mode = ExecuteProcess(cmd=['ros2', 'service', 'call', '/reinitialize_global_localization', 'std_srvs/srv/Empty', '{}'],shell=True)

    update_loc = ExecuteProcess(cmd=['ros2', 'service', 'call', '/request_nomotion_update', 'std_srvs/srv/Empty', '{}'],shell=True)


    #after all this, set goal point and navigate
    #topic -> move_base_simple/goal
    #ros2 topic pub --once /name geometry_msgs/msg/PoseStamped ' contents '
    update_loc = ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '/move_base_simple/goal', '', '{}'],shell=True) #?

    #ld
    return LaunchDescription([
        turtlebot3_world_launch,
        navigation2_launch,
        particle_increase,
        global_mode,
        update_loc,
    ])
