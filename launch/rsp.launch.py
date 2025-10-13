from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
import xacro


def generate_launch_description():
    # Launch argument to toggle simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Locate and process the robot description (URDF/Xacro)
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Parameters for the robot_state_publisher
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    # Include Ignition Gazebo (ros_gz_sim) instead of classic gazebo_ros
    gz_pkg = get_package_share_directory('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': os.path.join(pkg_path, 'worlds', 'empty.world')
        }.items(),
    )

    # Return the final launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        node_robot_state_publisher,
        gz_launch
    ])
