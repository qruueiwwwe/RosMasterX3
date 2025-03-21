from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test',
            executable='move_test',
            name='move_test',
            output='screen'
        )
    ]) 