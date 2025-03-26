#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
from driver_node_ros2.driver_node import DeviceShifuDriver
import time

class TestDeviceShifuDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = DeviceShifuDriver()
        cls.test_duration = 5.0  # 测试持续时间（秒）

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.received_messages = []
        self.start_time = time.time()

    def test_camera_initialization(self):
        """测试摄像头初始化"""
        self.assertIsNotNone(self.node.cap)
        self.assertIsInstance(self.node.has_camera, bool)

    def test_image_publishing(self):
        """测试图像发布功能"""
        def image_callback(msg):
            self.received_messages.append(msg)

        # 创建订阅者
        subscription = self.node.create_subscription(
            Image,
            'camera/image',
            image_callback,
            10
        )

        # 等待接收消息
        while time.time() - self.start_time < self.test_duration:
            rclpy.spin_once(self.node)
            if len(self.received_messages) > 0:
                break

        self.assertGreater(len(self.received_messages), 0, "未收到图像消息")
        subscription.destroy()

    def test_command_handling(self):
        """测试命令处理功能"""
        def cmd_vel_callback(msg):
            self.received_messages.append(msg)

        # 创建订阅者
        subscription = self.node.create_subscription(
            Twist,
            'cmd_vel',
            cmd_vel_callback,
            10
        )

        # 发送测试命令
        test_command = Int32()
        test_command.data = 1  # 前进命令
        self.node.command_callback(test_command)

        # 等待接收消息
        while time.time() - self.start_time < self.test_duration:
            rclpy.spin_once(self.node)
            if len(self.received_messages) > 0:
                break

        self.assertGreater(len(self.received_messages), 0, "未收到速度命令消息")
        subscription.destroy()

    def test_retry_mechanism(self):
        """测试重试机制"""
        # 模拟摄像头失败
        if self.node.has_camera:
            self.node.cap.release()
            self.node.has_camera = False

        # 尝试捕获图像
        frame = self.node.capture_image()
        self.assertIsNotNone(frame, "重试机制未能生成模拟图像")

    def test_dynamic_fps(self):
        """测试动态帧率调整"""
        initial_fps = self.node.camera_fps
        self.node.frame_count = 100  # 模拟高帧率
        self.node.last_publish_time = time.time() - 1.0  # 模拟1秒时间间隔
        
        # 触发帧率调整
        self.node.publish_image()
        
        # 验证帧率是否在合理范围内
        self.assertGreaterEqual(self.node.camera_fps, self.node.min_fps)
        self.assertLessEqual(self.node.camera_fps, self.node.max_fps)

if __name__ == '__main__':
    unittest.main() 