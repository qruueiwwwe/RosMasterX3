#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import time

class MoveTest(Node):
    def __init__(self):
        super().__init__('move_test')
        # 创建速度发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建里程计订阅器
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 初始化位置和方向
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        
        self.get_logger().info('移动测试节点已启动')
        
    def odom_callback(self, msg):
        """里程计回调函数，更新机器人位置信息"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        # 从四元数中提取yaw角度（这里简化处理）
        self.orientation_z = msg.pose.pose.orientation.z
        
    def move_forward(self, distance, speed=0.2):
        """向前移动指定距离"""
        twist = Twist()
        twist.linear.x = speed
        
        start_x = self.position_x
        start_y = self.position_y
        
        while rclpy.ok():
            current_distance = ((self.position_x - start_x) ** 2 + 
                              (self.position_y - start_y) ** 2) ** 0.5
            if current_distance >= distance:
                break
                
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        # 停止移动
        self.stop()
        
    def rotate(self, angle, speed=0.5):
        """旋转指定角度（弧度）"""
        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed
        
        start_orientation = self.orientation_z
        target_orientation = start_orientation + angle
        
        while rclpy.ok():
            if abs(self.orientation_z - target_orientation) < 0.1:
                break
                
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        # 停止旋转
        self.stop()
        
    def stop(self):
        """停止机器人"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = MoveTest()
    
    try:
        # 等待一会儿确保里程计信息已经收到
        time.sleep(1)
        
        # 测试移动
        node.get_logger().info('开始前进1米')
        node.move_forward(1.0)
        node.get_logger().info('前进完成')
        
        time.sleep(1)
        
        # 测试旋转
        node.get_logger().info('开始旋转180度')
        node.rotate(pi)
        node.get_logger().info('旋转完成')
        
        # 保持节点运行
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 