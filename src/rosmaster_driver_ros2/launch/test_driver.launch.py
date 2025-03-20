from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmaster_driver_ros2',
            executable='test_driver',
            name='test_driver',
            output='screen'
        )
    ]) 