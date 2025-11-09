import os
import xacro

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value="empty.sdf",
        description='World to load'
        )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                    arguments=[
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                    ],
                    output='screen')


    xacro_file = os.path.join(
                    get_package_share_directory('my_bot'),
                    'description',
                    'robot.urdf.xacro'
                    )
    robot_description_config = xacro.process_file(xacro_file).toxml()

    
 #   controller_manager_node = Node(
#        package='controller_manager', 
 #       executable='ros2_control_node',
  #      parameters=[
   #                 {'robot_description': robot_description_config},
    #                os.path.join(get_package_share_directory('my_bot'), 'config', 'my_controllers.yaml')
#        ],
  #      output='screen'
 #   )
    
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '15'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager-timeout', '15'],
        output='screen'
    )

    joy_drive = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/jso', 'deadzone': 0.05, 'autorepeat_rate': 20.0}]
    )

    joy_teleop = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy_node',
            parameters=['config/ps4_teleop.yaml']
    )

    # Launch them all!
    return LaunchDescription([
    rsp,
    world_arg,
    spawn_entity,
    bridge,
    #controller_manager_node,
    load_joint_state_broadcaster,
    load_diff_drive_controller,
    joy_drive,
    joy_teleop,
    ])

