from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    declared_args = [
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='1'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('config', default_value='base'),
    ]

    # Paths to other packages
    jackal_description_dir = get_package_share_directory('jackal_description')
    jackal_control_dir = get_package_share_directory('jackal_control')

    # Include description.launch.py
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jackal_description_dir, 'launch', 'description.launch.py')
        ),
        launch_arguments={'config': LaunchConfiguration('config')}.items()
    )

    # Include control.launch.py
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jackal_control_dir, 'launch', 'control.launch.py')
        )
    )

    # Spawn the robot in Gazebo
    spawn_jackal = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'jackal',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    return LaunchDescription(declared_args + [
        description_launch,
        control_launch,
        spawn_jackal
    ])
