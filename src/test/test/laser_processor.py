#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from math import cos, sin, pi
import numpy as np

class LaserProcessor(Node):
    def __init__(self):
        super().__init__('laser_processor')
        
        # 创建激光扫描订阅器
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
            
        # 创建障碍物标记发布器（用于RViz可视化）
        self.marker_pub = self.create_publisher(
            Marker,
            '/obstacle_points',
            10)
            
        # 初始化参数
        self.min_distance = 0.3  # 最小检测距离（米）
        self.max_distance = 5.0  # 最大检测距离（米）
        self.obstacle_threshold = 0.5  # 障碍物检测阈值（米）
        
        self.get_logger().info('激光处理节点已启动')
        
    def laser_callback(self, msg):
        """处理激光扫描数据"""
        # 创建障碍物点标记
        marker = Marker()
        marker.header = msg.header
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        
        obstacles = []
        
        # 处理每个激光点
        for i, distance in enumerate(msg.ranges):
            # 跳过无效的测量值
            if not msg.range_min <= distance <= msg.range_max:
                continue
                
            # 跳过超出设定范围的点
            if distance < self.min_distance or distance > self.max_distance:
                continue
                
            # 如果距离小于阈值，认为是障碍物
            if distance < self.obstacle_threshold:
                angle = msg.angle_min + i * msg.angle_increment
                
                # 计算障碍物点的坐标
                x = distance * cos(angle)
                y = distance * sin(angle)
                
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                
                marker.points.append(point)
                obstacles.append((x, y))
        
        # 发布障碍物标记
        self.marker_pub.publish(marker)
        
        # 如果检测到障碍物，输出警告
        if obstacles:
            self.get_logger().warn(f'检测到 {len(obstacles)} 个障碍物点')
            
    def get_closest_obstacle(self):
        """获取最近的障碍物距离"""
        if not hasattr(self, 'latest_scan'):
            return None
            
        valid_ranges = [r for r in self.latest_scan.ranges 
                       if self.min_distance <= r <= self.max_distance]
        
        if valid_ranges:
            return min(valid_ranges)
        return None

def main(args=None):
    rclpy.init(args=args)
    node = LaserProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 