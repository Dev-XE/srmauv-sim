import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('srmauv_description')

    # Full sim bringup: Gazebo + NIOT arena + AUV (with sensors) + thruster
    # controller (so you can still teleop). No autopilot / mission run.
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'niot_mission.launch.py')))

    # Depth sensor (derives depth from odometry).
    depth = Node(package='srmauv_description', executable='depth_sensor.py',
                 namespace='srmauv', output='screen')

    # RViz with all sensor feeds (cameras, Ping360 sonar, robot model, TF).
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', os.path.join(pkg, 'config', 'srmauv_sensors.rviz')],
                output='screen')

    return LaunchDescription([mission, depth, rviz])
