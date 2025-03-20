from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    log_file_arg = DeclareLaunchArgument(
        'log_file',
        default_value='rosmaster_operations.log',
        description='日志文件路径'
    )

    # 创建节点
    driver_node = Node(
        package='rosmaster_driver',
        executable='rosmaster_driver',
        name='rosmaster_driver',
        output='screen',
        parameters=[{
            'log_file': LaunchConfiguration('log_file')
        }]
    )

    # 创建启动描述
    return LaunchDescription([
        log_file_arg,
        driver_node
    ]) 