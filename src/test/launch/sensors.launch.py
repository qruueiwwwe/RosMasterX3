from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test',
            executable='laser_processor',
            name='laser_processor',
            output='screen'
        ),
        Node(
            package='test',
            executable='depth_processor',
            name='depth_processor',
            output='screen'
        )
    ]) 