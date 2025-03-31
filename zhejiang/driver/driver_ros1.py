#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class DeviceShifuDriver:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # 初始化参数
        self.humanoid_speed = {
            'linear': {
                'x': rospy.get_param('~humanoid/linear/x', 0.5),
                'y': rospy.get_param('~humanoid/linear/y', 0.0),
                'z': rospy.get_param('~humanoid/linear/z', 0.0)
            },
            'angular': {
                'x': rospy.get_param('~humanoid/angular/x', 0.0),
                'y': rospy.get_param('~humanoid/angular/y', 0.0),
                'z': rospy.get_param('~humanoid/angular/z', 0.2)
            }
        }
        self.dog_speed = {
            'linear': {
                'x': rospy.get_param('~dog/linear/x', 0.5),
                'y': rospy.get_param('~dog/linear/y', 0.0),
                'z': rospy.get_param('~dog/linear/z', 0.0)
            },
            'angular': {
                'x': rospy.get_param('~dog/angular/x', 0.0),
                'y': rospy.get_param('~dog/angular/y', 0.0),
                'z': rospy.get_param('~dog/angular/z', 0.2)
            }
        }
        self.jetson_speed = {
            'linear': {
                'x': rospy.get_param('~jetson/linear/x', 0.5),
                'y': rospy.get_param('~jetson/linear/y', 0.0),
                'z': rospy.get_param('~jetson/linear/z', 0.0)
            },
            'angular': {
                'x': rospy.get_param('~jetson/angular/x', 0.0),
                'y': rospy.get_param('~jetson/angular/y', 0.0),
                'z': rospy.get_param('~jetson/angular/z', 0.2)
            }
        }
        
        # 安全参数
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5)
        self.command_timeout = rospy.get_param('~command_timeout', 0.5)
        
        # 创建发布者
        self.humanoid_pub = rospy.Publisher('humanoid/cmd_vel', Twist, queue_size=10)
        self.dog_pub = rospy.Publisher('dog/cmd_vel', Twist, queue_size=10)
        self.jetson_pub = rospy.Publisher('jetson/cmd_vel', Twist, queue_size=10)
        
        # 等待发布者连接
        rospy.sleep(0.5)  # 等待发布者初始化
        
        # 创建订阅者
        self.humanoid_sub = rospy.Subscriber('humanoid/remote_command', Int32, 
                                           lambda msg: self.command_callback(msg, 'humanoid'))
        self.dog_sub = rospy.Subscriber('dog/remote_command', Int32, 
                                      lambda msg: self.command_callback(msg, 'dog'))
        self.jetson_sub = rospy.Subscriber('jetson/remote_command', Int32, 
                                         lambda msg: self.command_callback(msg, 'jetson'))
        
        # 命令状态跟踪
        self.last_command_time = {
            'humanoid': rospy.Time.now(),
            'dog': rospy.Time.now(),
            'jetson': rospy.Time.now()
        }
        self.current_command = {
            'humanoid': None,
            'dog': None,
            'jetson': None
        }
        
        # 创建安全监控定时器
        self.safety_timer = rospy.Timer(rospy.Duration(0.1), self.safety_check)
        
        # 创建连接状态检查定时器
        self.connection_timer = rospy.Timer(rospy.Duration(1.0), self.check_connections)
        
        rospy.loginfo('DeviceShifu驱动已初始化')

    def check_connections(self, event):
        """检查发布者和订阅者的连接状态"""
        # 检查发布者连接
        humanoid_connections = self.humanoid_pub.get_num_connections()
        dog_connections = self.dog_pub.get_num_connections()
        jetson_connections = self.jetson_pub.get_num_connections()
        
        # 检查订阅者连接
        humanoid_subscribers = self.humanoid_sub.get_num_connections()
        dog_subscribers = self.dog_sub.get_num_connections()
        jetson_subscribers = self.jetson_sub.get_num_connections()
        
        # 打印连接状态
        rospy.loginfo(f'连接状态:')
        rospy.loginfo(f'人形机器人 - 发布者连接数: {humanoid_connections}, 订阅者连接数: {humanoid_subscribers}')
        rospy.loginfo(f'机器狗 - 发布者连接数: {dog_connections}, 订阅者连接数: {dog_subscribers}')
        rospy.loginfo(f'Jetson - 发布者连接数: {jetson_connections}, 订阅者连接数: {jetson_subscribers}')

    def wait_for_connections(self, timeout=5.0):
        """等待发布者建立连接"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if (self.humanoid_pub.get_num_connections() > 0 and
                self.dog_pub.get_num_connections() > 0 and
                self.jetson_pub.get_num_connections() > 0):
                rospy.loginfo('所有发布者已建立连接')
                return True
            rospy.sleep(0.1)
        rospy.logwarn('等待发布者连接超时')
        return False

    def validate_command(self, msg, robot_type):
        """验证命令的有效性"""
        # 检查命令值是否有效
        valid_commands = [0, 1, 2, 3, 4]
        if msg.data not in valid_commands:
            rospy.logwarn(f'{robot_type}收到无效命令值: {msg.data}')
            return False
            
        # 检查命令频率
        current_time = rospy.Time.now()
        time_diff = (current_time - self.last_command_time[robot_type]).to_sec()
        if time_diff < 0.1:  # 最小命令间隔0.1秒
            rospy.logwarn(f'{robot_type}命令频率过高，忽略命令')
            return False
            
        return True

    def safety_check(self, event):
        """安全监控检查"""
        current_time = rospy.Time.now()
        
        for robot_type in ['humanoid', 'dog', 'jetson']:
            # 检查命令超时
            time_diff = (current_time - self.last_command_time[robot_type]).to_sec()
            if time_diff > self.command_timeout and self.current_command[robot_type] is not None:
                rospy.logwarn(f'{robot_type}命令超时，执行安全停止')
                self.execute_safety_stop(robot_type)
                self.current_command[robot_type] = None

    def execute_safety_stop(self, robot_type):
        """执行安全停止"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        publisher = getattr(self, f'{robot_type}_pub')
        publisher.publish(cmd)
        rospy.loginfo(f'{robot_type}执行安全停止')

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

    def command_callback(self, msg, robot_type):
        """处理控制命令并转发"""
        # 验证命令
        if not self.validate_command(msg, robot_type):
            return
            
        # 检查发布者连接
        publisher = getattr(self, f'{robot_type}_pub')
        if publisher.get_num_connections() == 0:
            rospy.logwarn(f'{robot_type}没有订阅者，命令将被忽略')
            return
            
        # 更新命令状态
        self.last_command_time[robot_type] = rospy.Time.now()
        self.current_command[robot_type] = msg.data
        
        # 创建并发送Twist命令
        cmd = self.create_twist_command(robot_type, msg.data)
        publisher.publish(cmd)
        
        # 打印详细的运动参数
        rospy.loginfo(f'{robot_type}接收到移动控制消息: 线速度 (x: %.2f, y: %.2f, z: %.2f), 角速度 (x: %.2f, y: %.2f, z: %.2f)',
                     cmd.linear.x, cmd.linear.y, cmd.linear.z,
                     cmd.angular.x, cmd.angular.y, cmd.angular.z)

    def run(self):
        """运行节点"""
        try:
            # 等待发布者连接
            if not self.wait_for_connections():
                rospy.logerr('无法建立必要的连接，节点将退出')
                return
                
            rospy.spin()
        except Exception as e:
            rospy.logerr(f'发生错误: {str(e)}')
        finally:
            # 确保所有机器人都停止
            for robot_type in ['humanoid', 'dog', 'jetson']:
                self.execute_safety_stop(robot_type)

def main():
    try:
        driver = DeviceShifuDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 