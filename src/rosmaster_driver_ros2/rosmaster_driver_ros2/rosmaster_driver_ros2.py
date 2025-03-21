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
import RPi.GPIO as GPIO
from Rosmaster import Rosmaster

# GPIO引脚定义
class GpioConfig:
    # 电机控制引脚
    MOTOR_LEFT_FRONT = 17  # 左前电机
    MOTOR_RIGHT_FRONT = 18  # 右前电机
    MOTOR_LEFT_BACK = 27  # 左后电机
    MOTOR_RIGHT_BACK = 22  # 右后电机
    
    # 编码器引脚
    ENCODER_LEFT_FRONT = 23  # 左前编码器
    ENCODER_RIGHT_FRONT = 24  # 右前编码器
    ENCODER_LEFT_BACK = 25  # 左后编码器
    ENCODER_RIGHT_BACK = 8  # 右后编码器
    
    # 其他功能引脚
    LED_PIN = 12  # LED控制引脚
    BUZZER_PIN = 13  # 蜂鸣器引脚

class RosMasterDriver(Node):
    def __init__(self):
        super().__init__('rosmaster_driver')
        
        # 初始化变量
        self.cv_bridge = CvBridge()
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.lock = threading.Lock()
        self.is_real_robot = False
        
        # 初始化GPIO
        self._init_gpio()
        
        # 尝试初始化真实机器人
        try:
            self.rm = Rosmaster()
            self.rm.create_receive_threading()
            self.is_real_robot = True
            self.get_logger().info("成功连接到真实机器人")
        except Exception as e:
            self.get_logger().warn(f"无法连接到真实机器人: {str(e)}")
            self.get_logger().info("切换到测试模式")
            self.is_real_robot = False
            # 导入测试接口
            try:
                from test_interface import TestInterface
                self.test_interface = TestInterface()
            except ImportError as e:
                self.get_logger().error(f"无法导入测试接口: {str(e)}")
                raise

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
                
                # 限制速度范围
                v_x = max(min(v_x, 1.0), -1.0)
                v_y = max(min(v_y, 1.0), -1.0)
                v_z = max(min(v_z, 5.0), -5.0)
                
                if self.is_real_robot:
                    # 真实机器人控制
                    self.rm.set_car_motion(v_x, v_y, v_z)
                else:
                    # 测试模式控制
                    self.test_interface.set_car_motion(v_x, v_y, v_z)
                
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
            if self.is_real_robot:
                # 获取真实编码器数据
                encoder_data = self.rm.get_encoder_data()
            else:
                # 获取测试编码器数据
                encoder_data = self.test_interface.get_encoder_data()
            
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
            if self.is_real_robot:
                # 获取真实机器人状态
                robot_state = self.rm.get_robot_state()
            else:
                # 获取测试机器人状态
                robot_state = self.test_interface.get_robot_state()
            
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
            if self.is_real_robot:
                # 发布真实传感器数据
                self.publish_real_sensor_data()
            else:
                # 发布测试传感器数据
                self.publish_test_sensor_data()
        except Exception as e:
            self.get_logger().error(f"发布传感器数据错误: {str(e)}")

    def publish_real_sensor_data(self):
        """发布真实传感器数据"""
        # 实现真实传感器数据发布
        pass

    def publish_test_sensor_data(self):
        """发布测试传感器数据"""
        # 实现测试传感器数据发布
        pass

    def emergency_stop(self):
        """紧急停止"""
        with self.lock:
            self.current_speed = 0.0
            self.current_angular = 0.0
            if self.is_real_robot:
                self.rm.set_car_motion(0.0, 0.0, 0.0)
            else:
                self.test_interface.set_car_motion(0.0, 0.0, 0.0)
            self.buzzer_pub.publish(Bool(True))
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

    def _init_gpio(self):
        """初始化GPIO"""
        try:
            # 设置GPIO模式为BCM
            GPIO.setmode(GPIO.BCM)
            
            # 设置电机控制引脚为输出
            GPIO.setup(GpioConfig.MOTOR_LEFT_FRONT, GPIO.OUT)
            GPIO.setup(GpioConfig.MOTOR_RIGHT_FRONT, GPIO.OUT)
            GPIO.setup(GpioConfig.MOTOR_LEFT_BACK, GPIO.OUT)
            GPIO.setup(GpioConfig.MOTOR_RIGHT_BACK, GPIO.OUT)
            
            # 设置编码器引脚为输入
            GPIO.setup(GpioConfig.ENCODER_LEFT_FRONT, GPIO.IN)
            GPIO.setup(GpioConfig.ENCODER_RIGHT_FRONT, GPIO.IN)
            GPIO.setup(GpioConfig.ENCODER_LEFT_BACK, GPIO.IN)
            GPIO.setup(GpioConfig.ENCODER_RIGHT_BACK, GPIO.IN)
            
            # 设置其他功能引脚
            GPIO.setup(GpioConfig.LED_PIN, GPIO.OUT)
            GPIO.setup(GpioConfig.BUZZER_PIN, GPIO.OUT)
            
            self.get_logger().info("GPIO初始化成功")
        except Exception as e:
            self.get_logger().error(f"GPIO初始化失败: {str(e)}")
            raise

    def set_motor(self, m1: int, m2: int, m3: int, m4: int):
        """直接控制四个电机"""
        try:
            if self.is_real_robot:
                # 使用GPIO控制电机
                GPIO.output(GpioConfig.MOTOR_LEFT_FRONT, GPIO.HIGH if m1 > 0 else GPIO.LOW)
                GPIO.output(GpioConfig.MOTOR_RIGHT_FRONT, GPIO.HIGH if m2 > 0 else GPIO.LOW)
                GPIO.output(GpioConfig.MOTOR_LEFT_BACK, GPIO.HIGH if m3 > 0 else GPIO.LOW)
                GPIO.output(GpioConfig.MOTOR_RIGHT_BACK, GPIO.HIGH if m4 > 0 else GPIO.LOW)
            else:
                # 测试模式使用模拟接口
                self.test_interface.set_motor(m1, m2, m3, m4)
            
            self.get_logger().info(f"设置电机速度: m1={m1}, m2={m2}, m3={m3}, m4={m4}")
        except Exception as e:
            self.get_logger().error(f"设置电机速度失败: {str(e)}")

    def get_encoder_data(self):
        """获取编码器数据"""
        try:
            if self.is_real_robot:
                # 使用GPIO读取编码器数据
                encoder_data = [
                    GPIO.input(GpioConfig.ENCODER_LEFT_FRONT),
                    GPIO.input(GpioConfig.ENCODER_RIGHT_FRONT),
                    GPIO.input(GpioConfig.ENCODER_LEFT_BACK),
                    GPIO.input(GpioConfig.ENCODER_RIGHT_BACK)
                ]
            else:
                # 测试模式使用模拟数据
                encoder_data = self.test_interface.get_encoder_data()
            
            return encoder_data
        except Exception as e:
            self.get_logger().error(f"获取编码器数据失败: {str(e)}")
            return [0, 0, 0, 0]

    def __del__(self):
        """清理资源"""
        try:
            # 停止电机
            if self.is_real_robot:
                self.rm.set_car_motion(0.0, 0.0, 0.0)
            else:
                self.test_interface.set_car_motion(0.0, 0.0, 0.0)
            
            # 清理GPIO
            GPIO.cleanup()
        except Exception as e:
            self.get_logger().error(f"清理资源失败: {str(e)}")

def main(args=None):
    try:
        rclpy.init(args=args)
        driver = RosMasterDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.emergency_stop()
    except Exception as e:
        logging.error(f"驱动程序运行失败: {str(e)}")
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 