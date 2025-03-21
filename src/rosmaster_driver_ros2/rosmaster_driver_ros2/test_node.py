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
import sys

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class TestNode(Node):
    def __init__(self):
        try:
            super().__init__('test_node')
            logging.info("正在初始化测试节点...")
            
            # 初始化变量
            self.cv_bridge = CvBridge()
            self.running = True
            
            # 初始化摄像头
            try:
                self.cap = cv2.VideoCapture(0)
                if not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(1)
                if not self.cap.isOpened():
                    logging.warning("无法打开摄像头，将使用模拟图像")
            except Exception as e:
                logging.error(f"摄像头初始化失败: {str(e)}")
                self.cap = None
            
            # 创建发布者
            try:
                self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
                self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
                self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
                self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
                self.image_processed_pub = self.create_publisher(Image, '/camera/image_processed', 10)
                self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_data', 10)
                self.robot_state_pub = self.create_publisher(Twist, '/robot_state', 10)
                logging.info("所有发布者创建成功")
            except Exception as e:
                logging.error(f"创建发布者失败: {str(e)}")
                raise
            
            # 创建定时器
            try:
                self.create_timer(0.1, self.publish_imu_data)  # 10Hz
                self.create_timer(0.1, self.publish_lidar_data)  # 10Hz
                self.create_timer(0.1, self.publish_camera_image)  # 10Hz
                self.create_timer(0.1, self.publish_battery_state)  # 10Hz
                self.create_timer(0.05, self.publish_encoder_data)  # 20Hz
                self.create_timer(0.1, self.publish_robot_state)  # 10Hz
                logging.info("所有定时器创建成功")
            except Exception as e:
                logging.error(f"创建定时器失败: {str(e)}")
                raise
            
            logging.info('测试节点初始化完成')
            
        except Exception as e:
            logging.error(f"测试节点初始化失败: {str(e)}")
            raise

    def publish_imu_data(self):
        """发布IMU数据"""
        try:
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
        except Exception as e:
            logging.error(f"发布IMU数据失败: {str(e)}")

    def publish_lidar_data(self):
        """发布激光雷达数据"""
        try:
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
        except Exception as e:
            logging.error(f"发布激光雷达数据失败: {str(e)}")

    def publish_camera_image(self):
        """发布摄像头图像"""
        try:
            if self.cap and self.cap.isOpened():
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
        except Exception as e:
            logging.error(f"发布摄像头图像失败: {str(e)}")

    def publish_battery_state(self):
        """发布电池状态"""
        try:
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = np.random.normal(12.0, 0.1)
            battery_msg.percentage = max(0, min(100, np.random.normal(80, 5)))
            
            self.battery_pub.publish(battery_msg)
        except Exception as e:
            logging.error(f"发布电池状态失败: {str(e)}")

    def publish_encoder_data(self):
        """发布编码器数据"""
        try:
            encoder_msg = Int32MultiArray()
            encoder_msg.data = [int(np.random.normal(100, 10)) for _ in range(4)]
            self.encoder_pub.publish(encoder_msg)
        except Exception as e:
            logging.error(f"发布编码器数据失败: {str(e)}")

    def publish_robot_state(self):
        """发布机器人状态"""
        try:
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
        except Exception as e:
            logging.error(f"发布机器人状态失败: {str(e)}")

    def __del__(self):
        """清理资源"""
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        except Exception as e:
            logging.error(f"清理资源失败: {str(e)}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = TestNode()
        logging.info("开始运行测试节点...")
        rclpy.spin(node)
    except Exception as e:
        logging.error(f"测试节点运行失败: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        logging.info("测试节点已关闭")

if __name__ == '__main__':
    main() 