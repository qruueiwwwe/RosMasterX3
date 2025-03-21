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
import os
import json
import logging
from datetime import datetime
import sys

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from test_interface import TestInterface

class RosMasterDriver(Node):
    def __init__(self):
        super().__init__('rosmaster_driver')
        
        # 初始化变量
        self.cv_bridge = CvBridge()
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.lock = threading.Lock()
        
        # 检查是否在测试模式
        self.is_test_mode = os.environ.get('ROBOT_TEST_MODE', 'false').lower() == 'true'
        self.get_logger().info(f"当前运行模式: {'测试模式' if self.is_test_mode else '实际模式'}")
        
        # 初始化测试接口
        self.test_interface = TestInterface() if self.is_test_mode else None
        
        # 日志文件路径      
        self.log_file = os.environ.get("LOG_FILE", "rosmaster_operations.log")
        self._init_logging()

        # 发布者
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.image_processed_pub = self.create_publisher(Image, '/camera/image_processed', 10)
        self.led_pub = self.create_publisher(String, '/led_control', 10)
        self.buzzer_pub = self.create_publisher(Bool, '/buzzer_cmd', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_data', 10)
        self.robot_state_pub = self.create_publisher(Twist, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅者
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/keyboard_event', self.keyboard_event_callback, 10)

        # 创建定时器用于定期发布传感器数据
        self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        self.create_timer(0.05, self.publish_encoder_data)  # 20Hz
        self.create_timer(0.1, self.publish_robot_state)  # 10Hz

        # 启动数据接收线程
        self.rm.create_receive_threading()
        
        # 设置自动数据上报
        self.set_auto_report_state(True, forever=False)

        self.get_logger().info('ROSMASTER驱动节点已启动')

    def _init_logging(self):
        """初始化日志记录"""
        logging.basicConfig(
            filename=self.log_file,
            level=logging.INFO,
            format="%(asctime)s - %(message)s",
        )

    def log_operation(self, operation: str, value: float, operation_type: str):
        """记录操作到日志文件"""
        try:
            log_entry = {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "operation": operation,
                "value": value,
                "operation_type": operation_type,
            }
            logging.info(f"JSON_LOG:{json.dumps(log_entry)}")
        except Exception as e:
            logging.error(f"记录操作失败: {str(e)}")

    def cmd_vel_callback(self, msg):
        """处理底盘运动控制指令"""
        try:
            with self.lock:
                # 获取线速度和角速度
                v_x = msg.linear.x
                v_y = msg.linear.y
                v_z = msg.angular.z
                
                # 速度转换和限制
                speed_x = max(min(v_x / 10.0, 1.0), -1.0)
                speed_y = max(min(v_y / 10.0, 1.0), -1.0)
                speed_z = max(min(v_z / 10.0, 5.0), -5.0)
                
                # 调用Rosmaster库的set_car_motion函数
                self.rm.set_car_motion(speed_x, speed_y, speed_z)
                
                self.current_speed = speed_x
                self.current_angular = speed_z
                self.get_logger().info(f"收到cmd_vel: v_x={speed_x}, v_y={speed_y}, v_z={speed_z}")
                self.log_operation("cmd_vel", speed_x, "motion")
        except Exception as e:
            self.get_logger().error(f"cmd_vel_callback错误: {str(e)}")
            self.log_operation("cmd_vel_error", self.current_speed, f"Error: {str(e)}")

    def keyboard_event_callback(self, msg):
        """处理按键事件"""
        key = msg.data
        self.get_logger().info(f"收到键盘事件: {key}")

        # 创建Twist消息
        twist_msg = Twist()
        if key == "w":  # 前进
            twist_msg.linear.x = 0.5
        elif key == "s":  # 后退
            twist_msg.linear.x = -0.5
        elif key == "a":  # 左转
            twist_msg.angular.z = 0.5
        elif key == "d":  # 右转
            twist_msg.angular.z = -0.5
        elif key == "space":  # 停止
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # 发布运动控制指令
        self.cmd_vel_pub.publish(twist_msg)
        self.log_operation("keyboard_event", key, "motion")

        if key == "start":
            with self.lock:
                self.led_pub.publish(String(data="green"))  # 控制LED灯条为绿色
                self.buzzer_pub.publish(Bool(data=True))  # 触发蜂鸣器
                self.log_operation("keyboard_event", 1.0, "led_buzzer")

    def publish_encoder_data(self):
        """发布编码器数据"""
        try:
            # 获取编码器数据
            encoder_data = self.rm.get_encoder_data()
            
            # 创建编码器消息
            encoder_msg = Int32MultiArray()
            encoder_msg.data = encoder_data
            
            # 发布消息
            self.encoder_pub.publish(encoder_msg)
            
            # 清除数据缓存
            self.clear_auto_report_data()
        except Exception as e:
            self.get_logger().error(f"发布编码器数据错误: {str(e)}")

    def publish_robot_state(self):
        """发布机器人状态"""
        try:
            if self.is_test_mode:
                robot_state = self.test_interface.get_robot_state()
            else:
                # TODO: 实际硬件获取机器人状态
                robot_state = {
                    'linear_velocity_x': self.current_speed,
                    'linear_velocity_y': 0.0,
                    'angular_velocity_z': self.current_angular,
                    'battery_voltage': 12.0,
                    'battery_percentage': 100.0
                }
            
            # 创建机器人状态消息
            state_msg = Twist()
            state_msg.linear = Vector3(
                x=robot_state['linear_velocity_x'],
                y=robot_state['linear_velocity_y'],
                z=0.0
            )
            state_msg.angular = Vector3(
                x=0.0,
                y=0.0,
                z=robot_state['angular_velocity_z']
            )
            
            # 发布消息
            self.robot_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f"发布机器人状态错误: {str(e)}")

    def publish_sensor_data(self):
        """发布所有传感器数据"""
        try:
            if self.is_test_mode:
                imu_data = self.test_interface.get_imu_data()
                lidar_data = self.test_interface.get_lidar_data()
                camera_frame = self.test_interface.get_camera_image()
                battery_state = self.test_interface.get_battery_state()
            else:
                # TODO: 实际硬件获取传感器数据
                imu_data = {'acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81},
                           'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
                lidar_data = [10.0] * 360
                camera_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                battery_state = {'voltage': 12.0, 'percentage': 100.0}
            
            # 发布IMU数据
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
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_link"
            scan_msg.angle_min = -3.14159
            scan_msg.angle_max = 3.14159
            scan_msg.angle_increment = 3.14159 / 180
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0
            scan_msg.ranges = lidar_data
            self.scan_pub.publish(scan_msg)

            # 发布摄像头图像
            if camera_frame is not None:
                ros_image = self.cv_bridge.cv2_to_imgmsg(camera_frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(ros_image)

                # 图像预处理
                processed_image = cv2.GaussianBlur(camera_frame, (5, 5), 0)
                ros_processed_image = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
                ros_processed_image.header.stamp = self.get_clock().now().to_msg()
                self.image_processed_pub.publish(ros_processed_image)

            # 发布电池状态
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = battery_state['voltage']
            battery_msg.percentage = battery_state['percentage']
            self.battery_pub.publish(battery_msg)

            # 检查电池电量
            if battery_msg.voltage < 9.6:
                self.buzzer_pub.publish(Bool(True))  # 低电量报警
                self.log_operation("low_battery", battery_msg.voltage, "battery")

        except Exception as e:
            self.get_logger().error(f"发布传感器数据错误: {str(e)}")

    def emergency_stop(self):
        """紧急停止"""
        with self.lock:
            self.current_speed = 0.0
            self.current_angular = 0.0
            if self.is_test_mode:
                self.test_interface.set_car_motion(0.0, 0.0, 0.0)
            else:
                # TODO: 实际硬件紧急停止
                pass
            self.buzzer_pub.publish(Bool(True))  # 触发蜂鸣器
            self.log_operation("emergency_stop", 0.0, "emergency")
            self.get_logger().info("紧急停止已激活")

    def set_auto_report_state(self, enable: bool, forever: bool = False):
        """设置自动数据上报状态"""
        try:
            self.rm.set_auto_report_state(enable, forever)
            self.get_logger().info(f"设置自动数据上报: enable={enable}, forever={forever}")
        except Exception as e:
            self.get_logger().error(f"设置自动数据上报失败: {str(e)}")

    def clear_auto_report_data(self):
        """清除自动上报的数据缓存"""
        try:
            self.rm.clear_auto_report_data()
            self.get_logger().info("清除自动上报数据缓存")
        except Exception as e:
            self.get_logger().error(f"清除自动上报数据失败: {str(e)}")

    def set_motor(self, m1: int, m2: int, m3: int, m4: int):
        """直接控制四个电机"""
        try:
            self.rm.set_motor(m1, m2, m3, m4)
            self.get_logger().info(f"设置电机速度: m1={m1}, m2={m2}, m3={m3}, m4={m4}")
        except Exception as e:
            self.get_logger().error(f"设置电机速度失败: {str(e)}")

    def get_motion_data(self):
        """获取运动数据"""
        try:
            v_x, v_y, v_z = self.rm.get_motion_data()
            self.get_logger().info(f"获取运动数据: v_x={v_x}, v_y={v_y}, v_z={v_z}")
            return v_x, v_y, v_z
        except Exception as e:
            self.get_logger().error(f"获取运动数据失败: {str(e)}")
            return 0.0, 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    driver = RosMasterDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.emergency_stop()
    finally:
        if driver.test_interface:
            driver.test_interface.stop()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 