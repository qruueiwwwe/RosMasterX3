#!/usr/bin/env python3
import numpy as np
import cv2
import time
from threading import Thread
import random
from datetime import datetime

class MockIMU:
    def __init__(self):
        self.running = False
        self.thread = None
        self.data = {
            'acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        
    def start(self):
        self.running = True
        self.thread = Thread(target=self._update_data)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            
    def _update_data(self):
        while self.running:
            # 模拟加速度数据
            self.data['acceleration']['x'] = random.uniform(-2.0, 2.0)
            self.data['acceleration']['y'] = random.uniform(-2.0, 2.0)
            self.data['acceleration']['z'] = 9.81 + random.uniform(-0.5, 0.5)
            
            # 模拟角速度数据
            self.data['angular_velocity']['x'] = random.uniform(-0.5, 0.5)
            self.data['angular_velocity']['y'] = random.uniform(-0.5, 0.5)
            self.data['angular_velocity']['z'] = random.uniform(-1.0, 1.0)
            
            time.sleep(0.1)  # 10Hz更新频率
            
    def get_data(self):
        return self.data

class MockLidar:
    def __init__(self):
        self.running = False
        self.thread = None
        self.angle_min = -3.14159
        self.angle_max = 3.14159
        self.range_min = 0.1
        self.range_max = 10.0
        self.scan_data = []
        
    def start(self):
        self.running = True
        self.thread = Thread(target=self._update_data)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            
    def _update_data(self):
        while self.running:
            # 生成360个点的激光扫描数据
            self.scan_data = []
            for _ in range(360):
                # 模拟障碍物
                if random.random() < 0.1:  # 10%的概率生成障碍物
                    distance = random.uniform(0.5, 2.0)
                else:
                    distance = random.uniform(2.0, 10.0)
                self.scan_data.append(distance)
            
            time.sleep(0.1)  # 10Hz更新频率
            
    def get_scan(self):
        return self.scan_data

class MockCamera:
    def __init__(self):
        self.running = False
        self.thread = None
        self.frame = None
        
    def start(self):
        self.running = True
        self.thread = Thread(target=self._update_data)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            
    def _update_data(self):
        while self.running:
            # 创建一个640x480的随机图像
            self.frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            
            # 添加一些模拟的视觉特征
            # 随机绘制一些圆形
            for _ in range(5):
                x = random.randint(0, 639)
                y = random.randint(0, 479)
                radius = random.randint(10, 50)
                color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                cv2.circle(self.frame, (x, y), radius, color, -1)
            
            time.sleep(0.1)  # 10Hz更新频率
            
    def get_frame(self):
        return self.frame

class MockBattery:
    def __init__(self):
        self.running = False
        self.thread = None
        self.state = {
            'voltage': 12.0,
            'percentage': 100.0
        }
        
    def start(self):
        self.running = True
        self.thread = Thread(target=self._update_data)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            
    def _update_data(self):
        while self.running:
            # 模拟电池电量缓慢下降
            self.state['voltage'] = max(9.0, self.state['voltage'] - random.uniform(0.01, 0.05))
            self.state['percentage'] = max(0.0, self.state['percentage'] - random.uniform(0.1, 0.5))
            
            time.sleep(1.0)  # 1Hz更新频率
            
    def get_state(self):
        return self.state

if __name__ == "__main__":
    # 测试代码
    imu = MockIMU()
    lidar = MockLidar()
    camera = MockCamera()
    battery = MockBattery()

    # 启动所有设备
    imu.start()
    lidar.start()
    camera.start()
    battery.start()

    try:
        while True:
            # 打印IMU数据
            imu_data = imu.get_data()
            print("IMU数据:", imu_data)

            # 打印激光雷达数据（只显示前5个点）
            lidar_data = lidar.get_scan()[:5]
            print("激光雷达数据（前5个点）:", lidar_data)

            # 显示摄像头图像
            frame = camera.get_frame()
            if frame is not None:
                cv2.imshow('Mock Camera', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # 打印电池状态
            battery_state = battery.get_state()
            print("电池状态:", battery_state)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("测试结束")
    finally:
        # 停止所有设备
        imu.stop()
        lidar.stop()
        camera.stop()
        battery.stop()
        cv2.destroyAllWindows() 