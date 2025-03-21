#!/usr/bin/env python3
import numpy as np
import time
import threading
import cv2
import logging

class MockDevices:
    def __init__(self):
        self.running = False
        self.thread = None
        self.cap = None
        self._init_camera()
        
    def _init_camera(self):
        """Initialize camera or create mock camera"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(1)
            if not self.cap.isOpened():
                raise Exception("No camera available")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            logging.info("Camera initialized successfully")
        except Exception as e:
            logging.warning(f"无法打开摄像头，将使用模拟图像: {str(e)}")
            self.cap = None

    def start(self):
        """Start the mock device thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._update_data)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        """Stop the mock device thread"""
        self.running = False
        if self.thread:
            self.thread.join()
        if self.cap:
            self.cap.release()

    def _update_data(self):
        """Update mock device data"""
        while self.running:
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        self._process_frame(frame)
                time.sleep(0.1)  # 10Hz update rate
            except Exception as e:
                logging.error(f"Error updating mock device data: {str(e)}")
                time.sleep(0.1)

    def _process_frame(self, frame):
        """Process camera frame"""
        # Add any frame processing here
        pass

    def get_encoder_data(self):
        """Get encoder data"""
        return [int(np.random.normal(100, 10)) for _ in range(4)]

    def get_robot_state(self):
        """Get robot state"""
        return {
            'linear_velocity_x': np.random.normal(0, 0.1),
            'linear_velocity_y': np.random.normal(0, 0.1),
            'angular_velocity_z': np.random.normal(0, 0.1),
            'battery_voltage': np.random.normal(12.0, 0.1),
            'battery_percentage': max(0, min(100, np.random.normal(80, 5)))
        }

    def set_car_motion(self, v_x, v_y, v_z):
        """Set car motion"""
        return True

class MockIMU(MockDevices):
    def __init__(self):
        super().__init__()
        self.acceleration = {'x': 0.0, 'y': 0.0, 'z': 9.81}
        self.angular_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def _update_data(self):
        while self.running:
            try:
                # Generate random IMU data
                self.acceleration['x'] = np.random.normal(0, 0.1)
                self.acceleration['y'] = np.random.normal(0, 0.1)
                self.acceleration['z'] = 9.81 + np.random.normal(0, 0.1)
                self.angular_velocity['x'] = np.random.normal(0, 0.1)
                self.angular_velocity['y'] = np.random.normal(0, 0.1)
                self.angular_velocity['z'] = np.random.normal(0, 0.1)
                time.sleep(0.1)
            except Exception as e:
                logging.error(f"Error updating IMU data: {str(e)}")
                time.sleep(0.1)

    def get_data(self):
        return {
            'acceleration': self.acceleration,
            'angular_velocity': self.angular_velocity
        }

class MockLidar(MockDevices):
    def __init__(self):
        super().__init__()
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi / 180
        self.range_min = 0.1
        self.range_max = 10.0
        self.ranges = [5.0] * 360

    def _update_data(self):
        while self.running:
            try:
                # Generate random LiDAR data
                self.ranges = [np.random.uniform(0.1, 10.0) for _ in range(360)]
                time.sleep(0.1)
            except Exception as e:
                logging.error(f"Error updating LiDAR data: {str(e)}")
                time.sleep(0.1)

    def get_scan(self):
        return self.ranges

class MockCamera(MockDevices):
    def __init__(self):
        super().__init__()
        self.frame = None
        self._generate_mock_frame()

    def _generate_mock_frame(self):
        """Generate a mock camera frame"""
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(self.frame, (320, 240), 100, (0, 255, 0), -1)

    def _update_data(self):
        while self.running:
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        self.frame = frame
                else:
                    self._generate_mock_frame()
                time.sleep(0.1)
            except Exception as e:
                logging.error(f"Error updating camera data: {str(e)}")
                time.sleep(0.1)

    def get_frame(self):
        return self.frame

class MockBattery(MockDevices):
    def __init__(self):
        super().__init__()
        self.voltage = 12.0
        self.percentage = 100.0

    def _update_data(self):
        while self.running:
            try:
                # Simulate battery discharge
                self.voltage = np.random.uniform(11.0, 12.6)
                self.percentage = np.random.uniform(0, 100)
                time.sleep(0.1)
            except Exception as e:
                logging.error(f"Error updating battery data: {str(e)}")
                time.sleep(0.1)

    def get_state(self):
        return {
            'voltage': self.voltage,
            'percentage': self.percentage
        }

    def get_imu_data(self):
        """获取IMU数据"""
        return self.get_data()

    def get_lidar_data(self):
        """获取激光雷达数据"""
        return self.get_scan()

    def get_camera_image(self):
        """获取摄像头图像"""
        return self.get_frame()

    def get_battery_state(self):
        """获取电池状态"""
        return self.get_state()

    def _process_frame(self, frame):
        """Process camera frame"""
        # Add any frame processing here
        pass 