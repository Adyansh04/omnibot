from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.actions import RegisterEventHandler

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ## LAUNCH ARGUMENTS

    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard',
        default_value='false',
        description='Whether to use keyboard teleop'
    )

    for_simulation_arg = DeclareLaunchArgument(
            'for_simulation',
            default_value='false',
            description='Whether to use for simulation'
        )
    
    use_keyboard = LaunchConfiguration('use_keyboard')
    for_simulation = LaunchConfiguration('for_simulation')

    

    ## PARAMETER FILES

    joy_params = os.path.join(get_package_share_directory('omnibot_bringup'), 'config', 'joystick.yaml')


    ## NODES

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params],
        )
    
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        remappings=[
            ('/cmd_vel','/cmd_vel_fast'),
        ],
        parameters=[joy_params],
    )

    controller_gazebo_node = Node(
        package='omnibot_scripts',
        executable='controller',
        name='controller',
        remappings=[
            ('/cmd_vel','/omnidirectional_controller/cmd_vel_unstamped'),
        ],
        condition=IfCondition(for_simulation)
    )   

    controller_robot_node = Node(
        package='omnibot_scripts',
        executable='controller',
        name='controller',
        condition=UnlessCondition(for_simulation)
    )

    teleop_keyboard_cmd = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', '--ros-args', '-r', '/cmd_vel:=/omnidirectional_controller/cmd_vel_unstamped'],
        condition=IfCondition(use_keyboard)
    )

    event_handler_gazebo = RegisterEventHandler(OnProcessStart(target_action=teleop_joy_node, on_start=[controller_gazebo_node]))
    event_handler_robot = RegisterEventHandler(OnProcessStart(target_action=teleop_joy_node, on_start=[controller_robot_node]))
    

    # tekeop_keyboard_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_keyboard_node',
    #     condition=IfCondition(use_keyboard),
    #     remappings=[
    #         ('/cmd_vel','/omnidirectional_controller/cmd_vel_unstamped'),
    #     ],
    # )


    return LaunchDescription([
        use_keyboard_arg,
        for_simulation_arg,
        joy_node,
        teleop_joy_node,
        teleop_keyboard_cmd,
        event_handler_gazebo,
        event_handler_robot
    ])