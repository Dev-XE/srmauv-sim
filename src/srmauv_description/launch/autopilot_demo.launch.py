import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('srmauv_description')

    # Reuse the full mission bringup (Gazebo + arena + AUV + thruster controller).
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'niot_mission.launch.py')))

    # Start the autopilot a few seconds later, once odom is flowing.
    autopilot = TimerAction(period=8.0, actions=[
        Node(package='srmauv_description', executable='autopilot.py',
             namespace='srmauv', output='screen'),
    ])

    return LaunchDescription([mission, autopilot])
