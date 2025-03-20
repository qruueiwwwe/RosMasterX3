#!/usr/bin/env python3
import numpy as np
import time
import threading
import cv2

class MockDevices:
    def __init__(self):
        # 初始化模拟数据
        self.encoder_data = [0, 0, 0, 0]  # 四个轮子的编码器数据
        self.robot_state = {
            'linear_velocity_x': 0.0,
            'linear_velocity_y': 0.0,
            'angular_velocity_z': 0.0,
            'battery_voltage': 12.0,
            'battery_percentage': 100.0
        }
        self.imu_data = {
            'acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        self.lidar_data = [10.0] * 360  # 360度激光雷达数据
        self.camera_frame = None
        self.battery_state = {'voltage': 12.0, 'percentage': 100.0}
        
        # 控制线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_data)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(1)  # 尝试第二个摄像头
        if not self.cap.isOpened():
            self.get_logger().warn("无法打开摄像头，将使用模拟图像")

    def _update_data(self):
        """更新模拟数据的线程"""
        while self.running:
            # 更新编码器数据
            self.encoder_data = [
                int(np.random.normal(100, 10)) for _ in range(4)
            ]
            
            # 更新IMU数据
            self.imu_data = {
                'acceleration': {
                    'x': np.random.normal(0, 0.1),
                    'y': np.random.normal(0, 0.1),
                    'z': np.random.normal(9.81, 0.1)
                },
                'angular_velocity': {
                    'x': np.random.normal(0, 0.1),
                    'y': np.random.normal(0, 0.1),
                    'z': np.random.normal(0, 0.1)
                }
            }
            
            # 更新激光雷达数据
            self.lidar_data = [
                np.random.uniform(0.1, 10.0) for _ in range(360)
            ]
            
            # 更新摄像头图像
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    self.camera_frame = frame
            else:
                # 生成模拟图像
                self.camera_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.circle(self.camera_frame, (320, 240), 100, (0, 255, 0), -1)
            
            # 更新电池状态
            self.battery_state = {
                'voltage': np.random.normal(12.0, 0.1),
                'percentage': max(0, min(100, np.random.normal(80, 5)))
            }
            
            time.sleep(0.1)  # 10Hz更新频率

    def get_encoder_data(self):
        """获取编码器数据"""
        return self.encoder_data

    def get_robot_state(self):
        """获取机器人状态"""
        return self.robot_state

    def get_imu_data(self):
        """获取IMU数据"""
        return self.imu_data

    def get_lidar_data(self):
        """获取激光雷达数据"""
        return self.lidar_data

    def get_camera_image(self):
        """获取摄像头图像"""
        return self.camera_frame

    def get_battery_state(self):
        """获取电池状态"""
        return self.battery_state

    def set_car_motion(self, v_x, v_y, v_z):
        """设置机器人运动"""
        self.robot_state['linear_velocity_x'] = v_x
        self.robot_state['linear_velocity_y'] = v_y
        self.robot_state['angular_velocity_z'] = v_z
        return True

    def stop(self):
        """停止模拟设备"""
        self.running = False
        if self.cap.isOpened():
            self.cap.release()
        self.update_thread.join() 