import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "SLAM"
    pkg_path = os.path.join(get_package_share_directory('SLAM'))
    world_path = os.path.join(pkg_path, 'example.world')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "xacro_load.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    pkg_gazebo_ros=os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    # start gazebo
    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros])
        #launch_arguments = {'world': world_path}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    #slam
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")]
        ),
        launch_arguments={"use_sim_time": "true", "params_file": "src/SLAM/mapper_params_online_async.yaml"}.items(),
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo_start,
            spawn_entity,
            slam,
        ]
    )