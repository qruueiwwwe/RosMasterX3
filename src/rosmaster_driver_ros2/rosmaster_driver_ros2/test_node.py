#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, LaserScan, Image, BatteryState
from std_msgs.msg import Bool, String, Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import logging

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # 初始化变量
        self.cv_bridge = CvBridge()
        self.running = True
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            logging.warning("无法打开摄像头，将使用模拟图像")
        
        # 创建发布者
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.image_processed_pub = self.create_publisher(Image, '/camera/image_processed', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_data', 10)
        self.robot_state_pub = self.create_publisher(Twist, '/robot_state', 10)
        
        # 创建定时器
        self.create_timer(0.1, self.publish_imu_data)  # 10Hz
        self.create_timer(0.1, self.publish_lidar_data)  # 10Hz
        self.create_timer(0.1, self.publish_camera_image)  # 10Hz
        self.create_timer(0.1, self.publish_battery_state)  # 10Hz
        self.create_timer(0.05, self.publish_encoder_data)  # 20Hz
        self.create_timer(0.1, self.publish_robot_state)  # 10Hz
        
        self.get_logger().info('测试节点已启动')

    def publish_imu_data(self):
        """发布IMU数据"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # 模拟IMU数据
        imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.z = np.random.normal(9.81, 0.1)
        imu_msg.angular_velocity.x = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.y = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.z = np.random.normal(0, 0.1)
        
        self.imu_pub.publish(imu_msg)

    def publish_lidar_data(self):
        """发布激光雷达数据"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_link"
        scan_msg.angle_min = -np.pi / 2
        scan_msg.angle_max = np.pi / 2
        scan_msg.angle_increment = np.pi / 180
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [np.random.uniform(0.1, 10.0) for _ in range(180)]
        
        self.scan_pub.publish(scan_msg)

    def publish_camera_image(self):
        """发布摄像头图像"""
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(ros_image)
                
                # 图像预处理
                processed_image = cv2.GaussianBlur(frame, (5, 5), 0)
                ros_processed_image = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
                ros_processed_image.header.stamp = self.get_clock().now().to_msg()
                self.image_processed_pub.publish(ros_processed_image)
        else:
            # 生成模拟图像
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.circle(frame, (320, 240), 100, (0, 255, 0), -1)
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(ros_image)

    def publish_battery_state(self):
        """发布电池状态"""
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = np.random.normal(12.0, 0.1)
        battery_msg.percentage = max(0, min(100, np.random.normal(80, 5)))
        
        self.battery_pub.publish(battery_msg)

    def publish_encoder_data(self):
        """发布编码器数据"""
        encoder_msg = Int32MultiArray()
        encoder_msg.data = [int(np.random.normal(100, 10)) for _ in range(4)]
        self.encoder_pub.publish(encoder_msg)

    def publish_robot_state(self):
        """发布机器人状态"""
        state_msg = Twist()
        state_msg.linear = Vector3(
            x=np.random.normal(0, 0.1),
            y=np.random.normal(0, 0.1),
            z=0.0
        )
        state_msg.angular = Vector3(
            x=0.0,
            y=0.0,
            z=np.random.normal(0, 0.1)
        )
        self.robot_state_pub.publish(state_msg)

    def __del__(self):
        """清理资源"""
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 