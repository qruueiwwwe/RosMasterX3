#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan, Image, BatteryState
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from datetime import datetime
from rosmaster_interfaces.msg import Encoder, RobotState
from rosmaster_interfaces.srv import SetPIDParam

class MockRosmaster:
    def __init__(self):
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
        self.lidar_data = [10.0] * 360  # 360个点的激光扫描数据
        self.camera_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.battery_state = {
            'voltage': 12.0,
            'percentage': 100.0
        }
        self._running = True
        self._thread = threading.Thread(target=self._update_data)
        self._thread.daemon = True
        self._thread.start()

    def _update_data(self):
        while self._running:
            # 更新编码器数据
            self.encoder_data = [np.random.randint(-1000, 1000) for _ in range(4)]
            
            # 更新机器人状态
            self.robot_state['linear_velocity_x'] = np.random.uniform(-1.0, 1.0)
            self.robot_state['linear_velocity_y'] = np.random.uniform(-1.0, 1.0)
            self.robot_state['angular_velocity_z'] = np.random.uniform(-5.0, 5.0)
            
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
        return self.encoder_data

    def get_robot_state(self):
        return self.robot_state

    def get_imu_data(self):
        return self.imu_data

    def get_lidar_data(self):
        return self.lidar_data

    def get_camera_image(self):
        return self.camera_frame

    def get_battery_state(self):
        return self.battery_state

    def set_car_motion(self, v_x, v_y, v_z):
        self.robot_state['linear_velocity_x'] = v_x
        self.robot_state['linear_velocity_y'] = v_y
        self.robot_state['angular_velocity_z'] = v_z

    def set_pid_param(self, kp, ki, kd, forever=False):
        print(f"设置PID参数: kp={kp}, ki={ki}, kd={kd}, forever={forever}")
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()

class TestDriver(Node):
    def __init__(self):
        super().__init__('test_driver')
        
        # 初始化模拟设备
        self.rm = MockRosmaster()
        
        # 初始化变量
        self.cv_bridge = CvBridge()
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.lock = threading.Lock()

        # 发布者
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.image_processed_pub = self.create_publisher(Image, '/camera/image_processed', 10)
        self.led_pub = self.create_publisher(String, '/led_control', 10)
        self.buzzer_pub = self.create_publisher(Bool, '/buzzer_cmd', 10)
        self.encoder_pub = self.create_publisher(Encoder, '/encoder_data', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/robot_state', 10)

        # 订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 服务
        self.pid_param_server = self.create_service(
            SetPIDParam,
            '/set_pid_param',
            self.set_pid_param_callback
        )

        # 创建定时器用于定期发布传感器数据
        self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        self.create_timer(0.05, self.publish_encoder_data)  # 20Hz
        self.create_timer(0.1, self.publish_robot_state)  # 10Hz

        self.get_logger().info('测试驱动节点已启动')

    def cmd_vel_callback(self, msg):
        """处理底盘运动控制指令"""
        try:
            with self.lock:
                v_x = msg.linear.x
                v_y = msg.linear.y
                v_z = msg.angular.z
                
                v_x = max(min(v_x, 1.0), -1.0)
                v_y = max(min(v_y, 1.0), -1.0)
                v_z = max(min(v_z, 5.0), -5.0)
                
                self.rm.set_car_motion(v_x, v_y, v_z)
                self.current_speed = v_x
                self.current_angular = v_z
                self.get_logger().info(f'收到cmd_vel: v_x={v_x}, v_y={v_y}, v_z={v_z}')
        except Exception as e:
            self.get_logger().error(f'cmd_vel_callback错误: {str(e)}')

    def publish_encoder_data(self):
        """发布编码器数据"""
        try:
            encoder_data = self.rm.get_encoder_data()
            
            encoder_msg = Encoder()
            encoder_msg.header.stamp = self.get_clock().now().to_msg()
            encoder_msg.header.frame_id = "base_link"
            
            encoder_msg.left_front = encoder_data[0]
            encoder_msg.right_front = encoder_data[1]
            encoder_msg.left_back = encoder_data[2]
            encoder_msg.right_back = encoder_data[3]
            
            self.encoder_pub.publish(encoder_msg)
        except Exception as e:
            self.get_logger().error(f'发布编码器数据错误: {str(e)}')

    def publish_robot_state(self):
        """发布机器人状态"""
        try:
            robot_state = self.rm.get_robot_state()
            
            state_msg = RobotState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = "base_link"
            
            state_msg.linear_velocity.x = robot_state['linear_velocity_x']
            state_msg.linear_velocity.y = robot_state['linear_velocity_y']
            state_msg.angular_velocity.z = robot_state['angular_velocity_z']
            state_msg.battery_voltage = robot_state['battery_voltage']
            state_msg.battery_percentage = robot_state['battery_percentage']
            
            self.robot_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f'发布机器人状态错误: {str(e)}')

    def set_pid_param_callback(self, request, response):
        """处理PID参数设置请求"""
        try:
            self.rm.set_pid_param(
                kp=request.kp,
                ki=request.ki,
                kd=request.kd,
                forever=request.forever
            )
            response.success = True
            response.message = "PID参数设置成功"
            self.get_logger().info(f'PID参数已更新: kp={request.kp}, ki={request.ki}, kd={request.kd}')
        except Exception as e:
            response.success = False
            response.message = f"PID参数设置失败: {str(e)}"
            self.get_logger().error(f'PID参数设置错误: {str(e)}')
        return response

    def publish_sensor_data(self):
        """发布所有传感器数据"""
        try:
            # 发布IMU数据
            imu_data = self.rm.get_imu_data()
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.x = imu_data['acceleration']['x']
            imu_msg.linear_acceleration.y = imu_data['acceleration']['y']
            imu_msg.linear_acceleration.z = imu_data['acceleration']['z']
            imu_msg.angular_velocity.x = imu_data['angular_velocity']['x']
            imu_msg.angular_velocity.y = imu_data['angular_velocity']['y']
            imu_msg.angular_velocity.z = imu_data['angular_velocity']['z']
            self.imu_pub.publish(imu_msg)

            # 发布激光雷达数据
            scan_data = self.rm.get_lidar_data()
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_link"
            scan_msg.angle_min = -3.14159
            scan_msg.angle_max = 3.14159
            scan_msg.angle_increment = 3.14159 / 180
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0
            scan_msg.ranges = scan_data
            self.scan_pub.publish(scan_msg)

            # 发布摄像头图像
            frame = self.rm.get_camera_image()
            if frame is not None:
                ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(ros_image)

                # 图像预处理
                processed_image = cv2.GaussianBlur(frame, (5, 5), 0)
                ros_processed_image = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
                ros_processed_image.header.stamp = self.get_clock().now().to_msg()
                self.image_processed_pub.publish(ros_processed_image)

            # 发布电池状态
            battery_state = self.rm.get_battery_state()
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = battery_state['voltage']
            battery_msg.percentage = battery_state['percentage']
            self.battery_pub.publish(battery_msg)

            # 检查电池电量
            if battery_msg.voltage < 9.6:
                self.buzzer_pub.publish(Bool(True))  # 低电量报警

        except Exception as e:
            self.get_logger().error(f'发布传感器数据错误: {str(e)}')

    def __del__(self):
        """清理资源"""
        self.rm.stop()

def main(args=None):
    rclpy.init(args=args)
    driver = TestDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 