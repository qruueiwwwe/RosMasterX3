from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明配置参数
    config_file = LaunchConfiguration('config_file')
    
    # 创建节点
    driver_node = Node(
        package='driver_node_ros2',
        executable='driver_node',  # 改回原来的名称
        name='driver_node',
        output='screen',
        parameters=[config_file]
    )
    
    # 声明启动参数
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/opt/ros/foxy/share/driver_node_ros2/config/deviceshifu_config.yaml',
        description='Path to the configuration file'
    )
    
    return LaunchDescription([
        config_arg,
        driver_node
    ])