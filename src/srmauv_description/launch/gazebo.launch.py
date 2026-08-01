import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('srmauv_description')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg, 'urdf', 'srmauv.urdf.xacro')

    # Expand the xacro -> URDF string at launch time. Wrap in ParameterValue
    # with value_type=str so Humble doesn't try to parse the XML as YAML.
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # So Gazebo can resolve the package:// mesh URI for the visual.
    # 'pkg' is .../install/srmauv_description/share/srmauv_description, so its
    # parent is the prefix that makes package://srmauv_description/... resolve.
    model_path = os.path.dirname(pkg)
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.environ.get('GAZEBO_MODEL_PATH', '') + os.pathsep + model_path,
    )

    world = os.path.join(pkg, 'worlds', 'srmauv.world')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'verbose': 'true', 'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'srmauv',
                   '-z', '0.5'],
        output='screen',
    )

    return LaunchDescription([
        set_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn,
    ])
