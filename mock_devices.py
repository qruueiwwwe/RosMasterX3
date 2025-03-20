#!/usr/bin/env python3
import numpy as np
import cv2
import threading
import time

class MockDevices:
    def __init__(self):
        # 初始化模拟数据
        self.encoder_data = [0, 0, 0, 0]
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
        self.lidar_data = [10.0] * 360
        self.camera_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.battery_state = {
            'voltage': 12.0,
            'percentage': 100.0
        }
        
        # 控制线程
        self._running = True
        self._thread = threading.Thread(target=self._update_data)
        self._thread.daemon = True
        self._thread.start()

    def _update_data(self):
        """更新模拟数据"""
        while self._running:
            # 更新编码器数据
            self.encoder_data = [np.random.randint(-1000, 1000) for _ in range(4)]
            
            # 更新IMU数据
            self.imu_data['acceleration']['x'] = np.random.uniform(-2.0, 2.0)
            self.imu_data['acceleration']['y'] = np.random.uniform(-2.0, 2.0)
            self.imu_data['acceleration']['z'] = 9.81 + np.random.uniform(-0.5, 0.5)
            self.imu_data['angular_velocity']['x'] = np.random.uniform(-0.5, 0.5)
            self.imu_data['angular_velocity']['y'] = np.random.uniform(-0.5, 0.5)
            self.imu_data['angular_velocity']['z'] = np.random.uniform(-1.0, 1.0)
            
            # 更新激光雷达数据
            self.lidar_data = [np.random.uniform(0.1, 10.0) for _ in range(360)]
            
            # 更新摄像头图像
            self.camera_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            # 添加一些模拟的视觉特征
            for _ in range(5):
                x = np.random.randint(0, 639)
                y = np.random.randint(0, 479)
                radius = np.random.randint(10, 50)
                color = (np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255))
                cv2.circle(self.camera_frame, (x, y), radius, color, -1)
            
            # 更新电池状态
            self.battery_state['voltage'] = max(9.0, self.battery_state['voltage'] - np.random.uniform(0.01, 0.05))
            self.battery_state['percentage'] = max(0.0, self.battery_state['percentage'] - np.random.uniform(0.1, 0.5))
            
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

    def set_pid_param(self, kp, ki, kd, forever=False):
        """设置PID参数"""
        print(f"设置PID参数: kp={kp}, ki={ki}, kd={kd}, forever={forever}")
        return True

    def stop(self):
        """停止模拟设备"""
        self._running = False
        if self._thread:
            self._thread.join() 