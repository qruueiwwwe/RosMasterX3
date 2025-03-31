#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock
from rclpy.time import Time

class DeviceShifuDriver(Node):
    def __init__(self):
        super().__init__('deviceshifu_driver')
        
        # 加载DeviceShifu配置
        self.load_config()
        
        # 初始化参数
        self.init_parameters()
        
        # 创建发布者和订阅者
        self.init_publishers_and_subscribers()
        
        # 命令状态跟踪
        self.last_command_time = {
            'humanoid': self.get_clock().now(),
            'dog': self.get_clock().now(),
            'jetson': self.get_clock().now()
        }
        self.current_command = {
            'humanoid': None,
            'dog': None,
            'jetson': None
        }
        
        # 创建安全监控定时器
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # 创建连接状态检查定时器
        self.connection_timer = self.create_timer(1.0, self.check_connections)
        
        self.get_logger().info('DeviceShifu驱动已初始化')

    def load_config(self):
        """加载DeviceShifu配置文件"""
        try:
            package_share_directory = get_package_share_directory('driver_node_ros2')
            config_path = os.path.join(package_share_directory, 'config', 'deviceshifu_config.yaml')
            
            with open(config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f'成功加载配置文件: {config_path}')
        except Exception as e:
            self.get_logger().error(f'加载配置文件失败: {str(e)}')
            raise

    def init_parameters(self):
        """初始化参数"""
        # 声明机器人参数
        self.declare_parameter('humanoid.linear.x', 0.5)
        self.declare_parameter('humanoid.linear.y', 0.0)
        self.declare_parameter('humanoid.linear.z', 0.0)
        self.declare_parameter('humanoid.angular.x', 0.0)
        self.declare_parameter('humanoid.angular.y', 0.0)
        self.declare_parameter('humanoid.angular.z', 0.2)
        
        self.declare_parameter('dog.linear.x', 0.5)
        self.declare_parameter('dog.linear.y', 0.0)
        self.declare_parameter('dog.linear.z', 0.0)
        self.declare_parameter('dog.angular.x', 0.0)
        self.declare_parameter('dog.angular.y', 0.0)
        self.declare_parameter('dog.angular.z', 0.2)
        
        self.declare_parameter('jetson.linear.x', 0.5)
        self.declare_parameter('jetson.linear.y', 0.0)
        self.declare_parameter('jetson.linear.z', 0.0)
        self.declare_parameter('jetson.angular.x', 0.0)
        self.declare_parameter('jetson.angular.y', 0.0)
        self.declare_parameter('jetson.angular.z', 0.2)
        
        # 安全参数
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('command_timeout', 0.5)
        
        # 获取参数值
        self.humanoid_speed = {
            'linear': {
                'x': self.get_parameter('humanoid.linear.x').value,
                'y': self.get_parameter('humanoid.linear.y').value,
                'z': self.get_parameter('humanoid.linear.z').value
            },
            'angular': {
                'x': self.get_parameter('humanoid.angular.x').value,
                'y': self.get_parameter('humanoid.angular.y').value,
                'z': self.get_parameter('humanoid.angular.z').value
            }
        }
        self.dog_speed = {
            'linear': {
                'x': self.get_parameter('dog.linear.x').value,
                'y': self.get_parameter('dog.linear.y').value,
                'z': self.get_parameter('dog.linear.z').value
            },
            'angular': {
                'x': self.get_parameter('dog.angular.x').value,
                'y': self.get_parameter('dog.angular.y').value,
                'z': self.get_parameter('dog.angular.z').value
            }
        }
        self.jetson_speed = {
            'linear': {
                'x': self.get_parameter('jetson.linear.x').value,
                'y': self.get_parameter('jetson.linear.y').value,
                'z': self.get_parameter('jetson.linear.z').value
            },
            'angular': {
                'x': self.get_parameter('jetson.angular.x').value,
                'y': self.get_parameter('jetson.angular.y').value,
                'z': self.get_parameter('jetson.angular.z').value
            }
        }
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.command_timeout = self.get_parameter('command_timeout').value

    def init_publishers_and_subscribers(self):
        """初始化发布者和订阅者"""
        # 创建发布者
        self.humanoid_pub = self.create_publisher(Twist, 'humanoid/cmd_vel', 10)
        self.dog_pub = self.create_publisher(Twist, 'dog/cmd_vel', 10)
        self.jetson_pub = self.create_publisher(Twist, 'jetson/cmd_vel', 10)
        
        # 创建订阅者
        self.humanoid_sub = self.create_subscription(
            Int32,
            'humanoid/remote_command',
            lambda msg: self.remote_command_callback(msg, 'humanoid'),
            10
        )
        self.dog_sub = self.create_subscription(
            Int32,
            'dog/remote_command',
            lambda msg: self.remote_command_callback(msg, 'dog'),
            10
        )
        self.jetson_sub = self.create_subscription(
            Int32,
            'jetson/remote_command',
            lambda msg: self.remote_command_callback(msg, 'jetson'),
            10
        )

    def check_connections(self):
        """检查发布者和订阅者的连接状态"""
        # 检查发布者连接
        humanoid_connections = self.humanoid_pub.get_subscription_count()
        dog_connections = self.dog_pub.get_subscription_count()
        jetson_connections = self.jetson_pub.get_subscription_count()
        
        # 检查订阅者连接
        humanoid_subscribers = self.humanoid_sub.get_publisher_count()
        dog_subscribers = self.dog_sub.get_publisher_count()
        jetson_subscribers = self.jetson_sub.get_publisher_count()
        
        # 打印连接状态
        self.get_logger().info(f'连接状态:')
        self.get_logger().info(f'人形机器人 - 发布者连接数: {humanoid_connections}, 订阅者连接数: {humanoid_subscribers}')
        self.get_logger().info(f'机器狗 - 发布者连接数: {dog_connections}, 订阅者连接数: {dog_subscribers}')
        self.get_logger().info(f'Jetson - 发布者连接数: {jetson_connections}, 订阅者连接数: {jetson_subscribers}')

    def validate_command(self, msg, robot_type):
        """验证命令的有效性"""
        # 检查命令值是否有效
        valid_commands = [0, 1, 2, 3, 4]
        if msg.data not in valid_commands:
            self.get_logger().warn(f'{robot_type}收到无效命令值: {msg.data}')
            return False
            
        # 检查命令频率
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_command_time[robot_type]).nanoseconds / 1e9
        if time_diff < 0.1:  # 最小命令间隔0.1秒
            self.get_logger().warn(f'{robot_type}命令频率过高，忽略命令')
            return False
            
        return True

    def safety_check(self):
        """安全监控检查"""
        current_time = self.get_clock().now()
        
        for robot_type in ['humanoid', 'dog', 'jetson']:
            # 检查命令超时
            time_diff = (current_time - self.last_command_time[robot_type]).nanoseconds / 1e9
            if time_diff > self.command_timeout and self.current_command[robot_type] is not None:
                self.get_logger().warn(f'{robot_type}命令超时，执行安全停止')
                self.execute_safety_stop(robot_type)
                self.current_command[robot_type] = None

    def execute_safety_stop(self, robot_type):
        """执行安全停止"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        publisher = getattr(self, f'{robot_type}_pub')
        publisher.publish(cmd)
        self.get_logger().info(f'{robot_type}执行安全停止')

    def create_twist_command(self, robot_type, command_type):
        """创建Twist命令"""
        cmd = Twist()
        speed_config = getattr(self, f'{robot_type}_speed')
        
        if command_type == 0:    # 停止
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0
        elif command_type == 1:  # 前进
            cmd.linear.x = min(speed_config['linear']['x'], self.max_linear_speed)
            cmd.linear.y = speed_config['linear']['y']
            cmd.linear.z = speed_config['linear']['z']
            cmd.angular.x = speed_config['angular']['x']
            cmd.angular.y = speed_config['angular']['y']
            cmd.angular.z = speed_config['angular']['z']
        elif command_type == 2:  # 后退
            cmd.linear.x = max(-speed_config['linear']['x'], -self.max_linear_speed)
            cmd.linear.y = speed_config['linear']['y']
            cmd.linear.z = speed_config['linear']['z']
            cmd.angular.x = speed_config['angular']['x']
            cmd.angular.y = speed_config['angular']['y']
            cmd.angular.z = speed_config['angular']['z']
        elif command_type == 3:  # 左转
            cmd.linear.x = speed_config['linear']['x']
            cmd.linear.y = speed_config['linear']['y']
            cmd.linear.z = speed_config['linear']['z']
            cmd.angular.x = speed_config['angular']['x']
            cmd.angular.y = speed_config['angular']['y']
            cmd.angular.z = min(speed_config['angular']['z'], self.max_angular_speed)
        elif command_type == 4:  # 右转
            cmd.linear.x = speed_config['linear']['x']
            cmd.linear.y = speed_config['linear']['y']
            cmd.linear.z = speed_config['linear']['z']
            cmd.angular.x = speed_config['angular']['x']
            cmd.angular.y = speed_config['angular']['y']
            cmd.angular.z = max(-speed_config['angular']['z'], -self.max_angular_speed)
            
        return cmd

    def remote_command_callback(self, msg, robot_type):
        """处理远程控制命令"""
        # 验证命令
        if not self.validate_command(msg, robot_type):
            return
            
        # 检查发布者连接
        publisher = getattr(self, f'{robot_type}_pub')
        if publisher.get_subscription_count() == 0:
            self.get_logger().warn(f'{robot_type}没有订阅者，命令将被忽略')
            return
            
        # 更新命令状态
        self.last_command_time[robot_type] = self.get_clock().now()
        self.current_command[robot_type] = msg.data
        
        # 创建并发送Twist命令
        cmd = self.create_twist_command(robot_type, msg.data)
        publisher.publish(cmd)
        
        # 打印详细的运动参数
        self.get_logger().info(f'{robot_type}接收到移动控制消息: 线速度 (x: %.2f, y: %.2f, z: %.2f), 角速度 (x: %.2f, y: %.2f, z: %.2f)',
                             cmd.linear.x, cmd.linear.y, cmd.linear.z,
                             cmd.angular.x, cmd.angular.y, cmd.angular.z)

    def destroy_node(self):
        """清理资源"""
        # 确保所有机器人都停止
        for robot_type in ['humanoid', 'dog', 'jetson']:
            self.execute_safety_stop(robot_type)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = DeviceShifuDriver()
        rclpy.spin(driver)
    except Exception as e:
        print(f'发生错误: {str(e)}')
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
