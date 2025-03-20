#!/usr/bin/env python3
import rospy
import numpy as np
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


class RosMasterDriver:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('rosmaster_driver', anonymous=True)

        # 初始化变量
        self.cv_bridge = CvBridge()
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.lock = threading.Lock()  # 线程锁，确保线程安全

        # 日志文件路径
        self.log_file = os.environ.get("LOG_FILE", "rosmaster_operations.log")
        self._init_logging()

        # 发布者
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)
        self.image_processed_pub = rospy.Publisher('/camera/image_processed', Image, queue_size=10)
        self.led_pub = rospy.Publisher('/led_control', String, queue_size=10)
        self.buzzer_pub = rospy.Publisher('/buzzer_cmd', Bool, queue_size=10)

        # 订阅者
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/keyboard_event', String, self.keyboard_event_callback)

    def _init_logging(self):
        """初始化日志记录"""
        logging.basicConfig(
            filename=self.log_file,
            level=logging.INFO,
            format="%(asctime)s - %(message)s",
        )

    def log_operation(self, operation: str, value: float, operation_type: str):
        """
        将操作记录到日志文件中。

        参数:
            operation: 操作名称。
            value: 操作值。
            operation_type: 操作类型（如 "steering", "throttle"）。
        """
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
                self.current_speed = msg.linear.x
                self.current_angular = msg.angular.z
                rospy.loginfo(f"Received cmd_vel: linear={self.current_speed}, angular={self.current_angular}")

                # 这里由于不使用GPIO控制电机，所以可以简单打印信息，或者根据需要实现其他逻辑
                rospy.loginfo("Motor control logic skipped as GPIO is not used.")
                self.log_operation("cmd_vel", self.current_speed, "motion")
        except Exception as e:
            rospy.logerr(f"Error in cmd_vel_callback: {str(e)}")
            self.log_operation("cmd_vel_error", self.current_speed, f"Error: {str(e)}")

    def keyboard_event_callback(self, msg):
        """处理按键事件"""
        key = msg.data
        rospy.loginfo(f"Received keyboard event: {key}")

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
                self.led_pub.publish("green")  # 控制 LED 灯条为绿色
                self.buzzer_pub.publish(True)  # 触发蜂鸣器
                self.log_operation("keyboard_event", 1.0, "led_buzzer")

    def publish_imu_data(self):
        """发布 IMU 数据"""
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # 模拟 IMU 数据
        imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.z = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.x = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.y = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.z = np.random.normal(0, 0.1)

        with self.lock:
            self.imu_pub.publish(imu_msg)

    def publish_lidar_data(self):
        """发布激光雷达数据"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "laser_link"
        scan_msg.angle_min = -np.pi / 2
        scan_msg.angle_max = np.pi / 2
        scan_msg.angle_increment = np.pi / 180
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [np.random.uniform(0.1, 10.0) for _ in range(180)]  # 模拟 180 个扫描点

        with self.lock:
            self.scan_pub.publish(scan_msg)

    def publish_camera_image(self):
        """发布摄像头图像"""
        cap = cv2.VideoCapture(0)  # 打开摄像头
        ret, frame = cap.read()
        if ret:
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            with self.lock:
                self.image_pub.publish(ros_image)

            # 图像预处理（如裁剪、滤波）
            processed_image = cv2.GaussianBlur(frame, (5, 5), 0)
            ros_processed_image = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
            with self.lock:
                self.image_processed_pub.publish(ros_processed_image)

        cap.release()

    def publish_battery_state(self):
        """发布电池状态"""
        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = 12.0  # 模拟电压值
        battery_msg.percentage = 0.8  # 模拟剩余电量

        if battery_msg.voltage < 9.6:
            with self.lock:
                self.buzzer_pub.publish(True)  # 低电量报警
                self.log_operation("low_battery", battery_msg.voltage, "battery")

        with self.lock:
            self.battery_pub.publish(battery_msg)

    def emergency_stop(self):
        """紧急停止"""
        with self.lock:
            self.current_speed = 0.0
            self.current_angular = 0.0
            # 这里由于不使用GPIO控制电机，所以可以简单打印信息，或者根据需要实现其他逻辑
            rospy.loginfo("Motor stop logic skipped as GPIO is not used.")
            self.buzzer_pub.publish(True)  # 触发蜂鸣器
            self.log_operation("emergency_stop", 0.0, "emergency")
            rospy.loginfo("Emergency stop activated.")

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_imu_data()
            self.publish_lidar_data()
            self.publish_camera_image()
            self.publish_battery_state()
            rate.sleep()


if __name__ == "__main__":
    try:
        driver = RosMasterDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass