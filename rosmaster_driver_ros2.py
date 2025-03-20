#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan, Image, BatteryState
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import threading
import os
import json
import logging
from datetime import datetime
from mock_devices import MockIMU, MockLidar, MockCamera, MockBattery
from Rosmaster import Rosmaster
from rosmaster_interfaces.msg import Encoder, RobotState
from rosmaster_interfaces.srv import SetPIDParam

class RosMasterDriver(Node):
    def __init__(self):
        super().__init__('rosmaster_driver')
        
        # 初始化Rosmaster对象
        self.rm = Rosmaster()
        
        # 设置PID参数
        self.rm.set_pid_param(kp=1.0, ki=0.1, kd=0.05, forever=True)
        
        # 初始化变量
        self.cv_bridge = CvBridge()
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.lock = threading.Lock()

        # 初始化模拟设备
        self.imu = MockIMU()
        self.lidar = MockLidar()
        self.camera = MockCamera()
        self.battery = MockBattery()

        # 启动所有模拟设备
        self.imu.start()
        self.lidar.start()
        self.camera.start()
        self.battery.start()

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
        self.encoder_pub = self.create_publisher(Encoder, '/encoder_data', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/robot_state', 10)

        # 订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.keyboard_event_sub = self.create_subscription(
            String,
            '/keyboard_event',
            self.keyboard_event_callback,
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
                
                # 限制速度范围
                v_x = max(min(v_x, 1.0), -1.0)
                v_y = max(min(v_y, 1.0), -1.0)
                v_z = max(min(v_z, 5.0), -5.0)
                
                # 调用Rosmaster库的set_car_motion函数
                self.rm.set_car_motion(v_x, v_y, v_z)
                
                self.current_speed = v_x
                self.current_angular = v_z
                self.get_logger().info(f"收到cmd_vel: v_x={v_x}, v_y={v_y}, v_z={v_z}")
                self.log_operation("cmd_vel", v_x, "motion")
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
        self.cmd_vel_callback(twist_msg)
        self.log_operation("keyboard_event", key, "motion")

        if key == "start":
            with self.lock:
                self.led_pub.publish(String("green"))  # 控制LED灯条为绿色
                self.buzzer_pub.publish(Bool(True))  # 触发蜂鸣器
                self.log_operation("keyboard_event", 1.0, "led_buzzer")

    def publish_encoder_data(self):
        """发布编码器数据"""
        try:
            # 获取编码器数据
            encoder_data = self.rm.get_encoder_data()
            
            # 创建编码器消息
            encoder_msg = Encoder()
            encoder_msg.header.stamp = self.get_clock().now().to_msg()
            encoder_msg.header.frame_id = "base_link"
            
            # 设置编码器数据
            encoder_msg.left_front = encoder_data[0]
            encoder_msg.right_front = encoder_data[1]
            encoder_msg.left_back = encoder_data[2]
            encoder_msg.right_back = encoder_data[3]
            
            # 发布消息
            self.encoder_pub.publish(encoder_msg)
        except Exception as e:
            self.get_logger().error(f"发布编码器数据错误: {str(e)}")

    def publish_robot_state(self):
        """发布机器人状态"""
        try:
            # 获取机器人状态数据
            robot_state = self.rm.get_robot_state()
            
            # 创建机器人状态消息
            state_msg = RobotState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = "base_link"
            
            # 设置机器人状态数据
            state_msg.linear_velocity.x = robot_state['linear_velocity_x']
            state_msg.linear_velocity.y = robot_state['linear_velocity_y']
            state_msg.angular_velocity.z = robot_state['angular_velocity_z']
            state_msg.battery_voltage = robot_state['battery_voltage']
            state_msg.battery_percentage = robot_state['battery_percentage']
            
            # 发布消息
            self.robot_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f"发布机器人状态错误: {str(e)}")

    def set_pid_param_callback(self, request, response):
        """处理PID参数设置请求"""
        try:
            # 设置PID参数
            self.rm.set_pid_param(
                kp=request.kp,
                ki=request.ki,
                kd=request.kd,
                forever=request.forever
            )
            response.success = True
            response.message = "PID参数设置成功"
            self.get_logger().info(f"PID参数已更新: kp={request.kp}, ki={request.ki}, kd={request.kd}")
        except Exception as e:
            response.success = False
            response.message = f"PID参数设置失败: {str(e)}"
            self.get_logger().error(f"PID参数设置错误: {str(e)}")
        return response

    def publish_sensor_data(self):
        """发布所有传感器数据"""
        self.publish_imu_data()
        self.publish_lidar_data()
        self.publish_camera_image()
        self.publish_battery_state()

    def publish_imu_data(self):
        """发布IMU数据"""
        imu_data = self.imu.get_data()
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = imu_data['acceleration']['x']
        imu_msg.linear_acceleration.y = imu_data['acceleration']['y']
        imu_msg.linear_acceleration.z = imu_data['acceleration']['z']
        imu_msg.angular_velocity.x = imu_data['angular_velocity']['x']
        imu_msg.angular_velocity.y = imu_data['angular_velocity']['y']
        imu_msg.angular_velocity.z = imu_data['angular_velocity']['z']

        with self.lock:
            self.imu_pub.publish(imu_msg)

    def publish_lidar_data(self):
        """发布激光雷达数据"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_link"
        scan_msg.angle_min = self.lidar.angle_min
        scan_msg.angle_max = self.lidar.angle_max
        scan_msg.angle_increment = 3.14159 / 180
        scan_msg.range_min = self.lidar.range_min
        scan_msg.range_max = self.lidar.range_max
        scan_msg.ranges = self.lidar.get_scan()

        with self.lock:
            self.scan_pub.publish(scan_msg)

    def publish_camera_image(self):
        """发布摄像头图像"""
        frame = self.camera.get_frame()
        if frame is not None:
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            with self.lock:
                self.image_pub.publish(ros_image)

            # 图像预处理
            processed_image = cv2.GaussianBlur(frame, (5, 5), 0)
            ros_processed_image = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
            ros_processed_image.header.stamp = self.get_clock().now().to_msg()
            with self.lock:
                self.image_processed_pub.publish(ros_processed_image)

    def publish_battery_state(self):
        """发布电池状态"""
        battery_state = self.battery.get_state()
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = battery_state['voltage']
        battery_msg.percentage = battery_state['percentage']

        if battery_msg.voltage < 9.6:
            with self.lock:
                self.buzzer_pub.publish(Bool(True))  # 低电量报警
                self.log_operation("low_battery", battery_msg.voltage, "battery")

        with self.lock:
            self.battery_pub.publish(battery_msg)

    def emergency_stop(self):
        """紧急停止"""
        with self.lock:
            self.current_speed = 0.0
            self.current_angular = 0.0
            self.rm.set_car_motion(0.0, 0.0, 0.0)  # 停止所有电机
            self.buzzer_pub.publish(Bool(True))  # 触发蜂鸣器
            self.log_operation("emergency_stop", 0.0, "emergency")
            self.get_logger().info("紧急停止已激活")

    def __del__(self):
        """清理资源"""
        self.imu.stop()
        self.lidar.stop()
        self.camera.stop()
        self.battery.stop()
        self.rm.set_car_motion(0.0, 0.0, 0.0)  # 确保电机停止

def main(args=None):
    rclpy.init(args=args)
    driver = RosMasterDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.emergency_stop()
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 