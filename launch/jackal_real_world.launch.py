from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('x', default_value='-10'),
        DeclareLaunchArgument('y', default_value='-4'),
        DeclareLaunchArgument('z', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),
    ]

    pkg_share = FindPackageShare('vision_based_navigation_ttt')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'jackal_gazebo.urdf.xacro'])

    robot_description = ParameterValue(
        Command(['xacro', xacro_path]),
        value_type=str
    )

    # Publish TF using robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen'
    )

    # Launch ros2_control_node as a process
    ros2_control_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'ros2_control_node',
            '--ros-args',
            '-p', f'robot_description:={robot_description}'
        ],
        output='screen'
    )           

    spawner_jsb = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster'],
        output='screen'
    )

    spawner_diffdrive = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'jackal_velocity_controller'],
        output='screen'
    )

    # Event: start jsb only after ros2_control_node finishes loading
    load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=ros2_control_process,
            on_exit=[spawner_jsb],
        )
    )

    # Event: start diffdrive controller only after jsb is up
    load_diffdrive = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_diffdrive],
        )
    )

    return LaunchDescription(declared_args + [
        robot_state_publisher,
        ros2_control_process,
        load_jsb,
        load_diffdrive,
    ])
