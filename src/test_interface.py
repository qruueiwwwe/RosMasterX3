#!/usr/bin/env python3
import sys
import os

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from mock_devices import MockDevices

class TestInterface:
    def __init__(self):
        self.mock_devices = MockDevices()
        self.is_test_mode = True

    def get_encoder_data(self):
        """获取编码器数据"""
        return self.mock_devices.get_encoder_data()

    def get_robot_state(self):
        """获取机器人状态"""
        return self.mock_devices.get_robot_state()

    def get_imu_data(self):
        """获取IMU数据"""
        return self.mock_devices.get_imu_data()

    def get_lidar_data(self):
        """获取激光雷达数据"""
        return self.mock_devices.get_lidar_data()

    def get_camera_image(self):
        """获取摄像头图像"""
        return self.mock_devices.get_camera_image()

    def get_battery_state(self):
        """获取电池状态"""
        return self.mock_devices.get_battery_state()

    def set_car_motion(self, v_x, v_y, v_z):
        """设置机器人运动"""
        return self.mock_devices.set_car_motion(v_x, v_y, v_z)

    def set_pid_param(self, kp, ki, kd, forever=False):
        """设置PID参数"""
        return self.mock_devices.set_pid_param(kp, ki, kd, forever)

    def stop(self):
        """停止测试接口"""
        self.mock_devices.stop() 