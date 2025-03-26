from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明配置参数
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/deviceshifu_config.yaml',
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