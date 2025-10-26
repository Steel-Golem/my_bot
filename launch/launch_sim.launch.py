import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    # Launch robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # World argument
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value="empty.world",
        description='World to load'
    )

    # Run Gazebo Classic with the world
    gazebo = Node(
        package='gazebo_ros',
        executable='gazebo',
        arguments=['-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        output='screen'
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Load robot description
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # ROS2 Control node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_config},
            os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
        ],
        output='screen'
    )

    # Spawn controllers with timers to ensure controller_manager is ready
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        controller_manager_node,
        TimerAction(period=2.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=2.5, actions=[load_diff_drive_controller]),
    ])