
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


    ## PARAMETER FILES

    ekf_params = os.path.join(get_package_share_directory('omnibot_bringup'), 'config', 'ekf.yaml')


    ## NODES

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        )
    
    complementary_filter = Node(
        package= 'imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node'
        )

    return LaunchDescription([
        complementary_filter,
        ekf_node
        
    ])
