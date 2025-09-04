from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robostack',
            executable='video_streamer_node',
            name='video_streamer_node',
            output='screen'
        ),
        Node(
            package='robostack',
            executable='shape_detection_node',
            name='shape_detection_node',
            output='screen'
        )
    ])
