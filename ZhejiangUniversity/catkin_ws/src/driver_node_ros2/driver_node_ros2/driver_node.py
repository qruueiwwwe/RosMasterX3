#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from typing import Optional, Tuple
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class DeviceShifuDriver(Node):
    def __init__(self):
        super().__init__('deviceshifu_driver')
        
        # 加载DeviceShifu配置
        self.load_config()
        
        # 初始化参数
        self.init_parameters()
        
        # 创建发布者和订阅者
        self.init_publishers_and_subscribers()
        
        # 初始化设备
        self.init_device()
        
        self.get_logger().info('DeviceShifu驱动已初始化')

    def load_config(self):
        """加载DeviceShifu配置文件"""
        try:
            # 获取包的共享目录
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
        # 声明参数
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('max_retry_count', 3)
        self.declare_parameter('retry_delay', 0.1)
        self.declare_parameter('min_fps', 10)
        self.declare_parameter('max_fps', 30)
        
        # 获取参数值
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.max_retry_count = self.get_parameter('max_retry_count').value
        self.retry_delay = self.get_parameter('retry_delay').value
        self.min_fps = self.get_parameter('min_fps').value
        self.max_fps = self.get_parameter('max_fps').value

    def init_publishers_and_subscribers(self):
        """初始化发布者和订阅者"""
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)
        
        # 创建订阅者
        self.command_sub = self.create_subscription(
            Int32,
            'remote_command',
            self.remote_command_callback,
            10
        )

    def init_device(self):
        """初始化设备"""
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.init_camera()
        
        # 创建定时器用于发布图像
        self.create_timer(1.0/self.camera_fps, self.publish_image)
        
        # 性能监控变量
        self.last_publish_time = time.time()
        self.frame_count = 0
        self.fps_update_interval = 1.0

    def init_camera(self) -> bool:
        """初始化摄像头，添加重试机制"""
        retry_count = 0
        while retry_count < self.max_retry_count:
            try:
                self.cap = cv2.VideoCapture(0)
                if self.cap.isOpened():
                    self.get_logger().info('成功连接摄像头')
                    self.has_camera = True
                    return True
                else:
                    raise Exception("无法打开摄像头")
            except Exception as e:
                retry_count += 1
                self.get_logger().warn(f'摄像头初始化失败 (尝试 {retry_count}/{self.max_retry_count}): {str(e)}')
                if retry_count < self.max_retry_count:
                    time.sleep(self.retry_delay)
        
        self.get_logger().warn('摄像头初始化失败，将使用模拟图像')
        self.has_camera = False
        self.cap = None
        return False

    def capture_image(self) -> Optional[np.ndarray]:
        """捕获图像，添加重试机制"""
        if not self.has_camera:
            return self.generate_simulated_image()
            
        retry_count = 0
        while retry_count < self.max_retry_count:
            ret, frame = self.cap.read()
            if ret:
                return frame
            retry_count += 1
            self.get_logger().warn(f'图像捕获失败 (尝试 {retry_count}/{self.max_retry_count})')
            if retry_count < self.max_retry_count:
                time.sleep(self.retry_delay)
        
        return None

    def generate_simulated_image(self) -> np.ndarray:
        """生成模拟图像"""
        image = np.zeros((480, 720, 3), dtype=np.uint8)
        image[:] = [187, 197, 57]  # BGR格式的 #39c5bb
        return image

    def publish_image(self):
        """发布图像数据，添加重试机制和动态帧率控制"""
        # 捕获图像
        frame = self.capture_image()
        if frame is None:
            self.get_logger().error('无法获取图像')
            return

        # 尝试发布图像
        retry_count = 0
        while retry_count < self.max_retry_count:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.image_pub.publish(msg)
                break
            except Exception as e:
                retry_count += 1
                self.get_logger().error(f'图像发布失败 (尝试 {retry_count}/{self.max_retry_count}): {str(e)}')
                if retry_count < self.max_retry_count:
                    time.sleep(self.retry_delay)

        # 更新性能监控
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_publish_time >= self.fps_update_interval:
            current_fps = self.frame_count / (current_time - self.last_publish_time)
            self.frame_count = 0
            self.last_publish_time = current_time
            
            # 动态调整帧率
            if current_fps < self.min_fps:
                self.camera_fps = max(self.min_fps, self.camera_fps - 1)
                self.get_logger().info(f'降低帧率到: {self.camera_fps}')
            elif current_fps > self.max_fps:
                self.camera_fps = min(self.max_fps, self.camera_fps + 1)
                self.get_logger().info(f'提高帧率到: {self.camera_fps}')

    def remote_command_callback(self, msg):
        """处理远程控制命令"""
        cmd = Twist()
        
        if msg.data == 0:    # 停止
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('执行停止命令')
        elif msg.data == 1:  # 前进
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('执行前进命令')
        elif msg.data == 2:  # 后退
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('执行后退命令')
        elif msg.data == 3:  # 左转
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().info('执行左转命令')
        elif msg.data == 4:  # 右转
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.get_logger().info('执行右转命令')
        else:
            self.get_logger().warn(f'未知命令: {msg.data}')
            return
            
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        """清理资源"""
        if self.has_camera and self.cap is not None:
            self.cap.release()
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
