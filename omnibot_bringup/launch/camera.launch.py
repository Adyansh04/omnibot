from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[
                # {'AeEnable': 'value'},
                # {'AnalogueGain': 'value'},
                # {'Brightness': 'value'},
                # {'Contrast': 'value'},
                # {'ExposureTime': 'value'},
                # {'Saturation': 'value'},
                {'camera': 0},
                # {'format': 'value'},
                {'height': 480},
                # {'role': 'value'},
                # {'use_sim_time': 'value'},
                {'width': 640},
            ]
        )
    ])