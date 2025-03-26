from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    package_share_directory = get_package_share_directory('driver_node_ros2')
    
    # 声明配置参数
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_directory, 'config', 'deviceshifu_config.yaml'),
        description='DeviceShifu配置文件路径'
    )

    # 创建节点
    driver_node = Node(
        package='driver_node_ros2',
        executable='driver_node',
        name='deviceshifu_driver',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        driver_node
    ]) 