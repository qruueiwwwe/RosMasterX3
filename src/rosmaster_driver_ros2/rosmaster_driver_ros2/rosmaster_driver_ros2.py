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

# Check if running in Docker
def is_running_in_docker():
    return os.path.exists('/.dockerenv')

# Try to import GPIO library
GPIO_AVAILABLE = False
if not is_running_in_docker():
    try:
        import Jetson.GPIO as GPIO
        GPIO_AVAILABLE = True
        print("Successfully imported Jetson.GPIO")
    except ImportError:
        try:
            import RPi.GPIO as GPIO
            GPIO_AVAILABLE = True
            print("Successfully imported RPi.GPIO")
        except ImportError:
            try:
                import Adafruit_BBIO.GPIO as GPIO
                GPIO_AVAILABLE = True
                print("Successfully imported Adafruit_BBIO.GPIO")
            except ImportError:
                print("Warning: No GPIO library available")
                print("Running in simulation mode...")
else:
    print("Warning: Running in Docker container, GPIO access is not available")
    print("Running in simulation mode...")

# Try to import Rosmaster library
try:
    from Rosmaster import Rosmaster
    ROSMASTER_AVAILABLE = True
    print("Successfully imported Rosmaster")
except ImportError:
    ROSMASTER_AVAILABLE = False
    print("Warning: Rosmaster library not available: No module named 'Rosmaster'")
    print("Running in simulation mode...")

# Import mock devices
from mock_devices import MockIMU, MockLidar, MockCamera, MockBattery

class GpioConfig:
    # Motor Control Pins
    MOTOR_LEFT_FRONT = 17  # Left Front Motor
    MOTOR_RIGHT_FRONT = 18  # Right Front Motor
    MOTOR_LEFT_BACK = 27  # Left Back Motor
    MOTOR_RIGHT_BACK = 22  # Right Back Motor
    
    # Encoder Pins
    ENCODER_LEFT_FRONT = 23  # Left Front Encoder
    ENCODER_RIGHT_FRONT = 24  # Right Front Encoder
    ENCODER_LEFT_BACK = 25  # Left Back Encoder
    ENCODER_RIGHT_BACK = 8  # Right Back Encoder
    
    # Other Function Pins
    LED_PIN = 12  # LED Control Pin
    BUZZER_PIN = 13  # Buzzer Pin

class RosMasterDriver(Node):
    def __init__(self):
        super().__init__('rosmaster_driver')
        
        # Initialize variables
        self.cv_bridge = CvBridge()
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.lock = threading.Lock()
        self.is_real_robot = False
        
        # Initialize mock devices
        self.imu = MockIMU()
        self.lidar = MockLidar()
        self.camera = MockCamera()
        self.battery = MockBattery()
        
        # Start mock devices
        self.imu.start()
        self.lidar.start()
        self.camera.start()
        self.battery.start()
        
        # Log file path
        self.log_file = os.environ.get("LOG_FILE", "rosmaster_operations.log")
        self._init_logging()
        
        # Publishers
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
        
        # Subscribers
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
        
        # Create timers for publishing sensor data
        self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        self.create_timer(0.05, self.publish_encoder_data)  # 20Hz
        self.create_timer(0.1, self.publish_robot_state)  # 10Hz
        
        self.get_logger().info('ROSMASTER Driver Node Started in Simulation Mode')

    def _init_logging(self):
        """Initialize Logging"""
        logging.basicConfig(
            filename=self.log_file,
            level=logging.INFO,
            format="%(asctime)s - %(message)s",
        )

    def log_operation(self, operation: str, value: float, operation_type: str):
        """Record Operation to Log File"""
        try:
            log_entry = {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "operation": operation,
                "value": value,
                "operation_type": operation_type,
            }
            logging.info(f"JSON_LOG:{json.dumps(log_entry)}")
        except Exception as e:
            logging.error(f"Failed to Record Operation: {str(e)}")

    def cmd_vel_callback(self, msg):
        """Handle Chassis Motion Control Commands"""
        try:
            with self.lock:
                # Get Linear and Angular Velocities
                v_x = msg.linear.x
                v_y = msg.linear.y
                v_z = msg.angular.z
                
                # Limit Speed Range
                v_x = max(min(v_x, 1.0), -1.0)
                v_y = max(min(v_y, 1.0), -1.0)
                v_z = max(min(v_z, 5.0), -5.0)
                
                if self.is_real_robot:
                    # Real Robot Control
                    self.rm.set_car_motion(v_x, v_y, v_z)
                else:
                    # Test Mode Control
                    self.test_interface.set_car_motion(v_x, v_y, v_z)
                
                self.current_speed = v_x
                self.current_angular = v_z
                self.get_logger().info(f"Received cmd_vel: v_x={v_x}, v_y={v_y}, v_z={v_z}")
                self.log_operation("cmd_vel", v_x, "motion")
        except Exception as e:
            self.get_logger().error(f"cmd_vel_callback Error: {str(e)}")
            self.log_operation("cmd_vel_error", self.current_speed, f"Error: {str(e)}")

    def keyboard_event_callback(self, msg):
        """Handle Keyboard Events"""
        key = msg.data
        self.get_logger().info(f"Received Keyboard Event: {key}")

        # Create Twist Message
        twist_msg = Twist()
        if key == "w":  # Forward
            twist_msg.linear.x = 0.5
        elif key == "s":  # Backward
            twist_msg.linear.x = -0.5
        elif key == "a":  # Turn Left
            twist_msg.angular.z = 0.5
        elif key == "d":  # Turn Right
            twist_msg.angular.z = -0.5
        elif key == "space":  # Stop
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # Publish Motion Control Command
        self.cmd_vel_pub.publish(twist_msg)
        self.log_operation("keyboard_event", key, "motion")

        if key == "start":
            with self.lock:
                self.led_pub.publish(String(data="green"))  # Control LED Strip to Green
                self.buzzer_pub.publish(Bool(data=True))  # Trigger Buzzer
                self.log_operation("keyboard_event", 1.0, "led_buzzer")

    def publish_encoder_data(self):
        """Publish Encoder Data"""
        try:
            if self.is_real_robot:
                # Get Real Encoder Data
                encoder_data = self.rm.get_encoder_data()
            else:
                # Get Test Encoder Data
                encoder_data = self.test_interface.get_encoder_data()
            
            # Create Encoder Message
            encoder_msg = Int32MultiArray()
            encoder_msg.data = encoder_data
            
            # Publish Message
            self.encoder_pub.publish(encoder_msg)
            
            # Clear Data Cache
            self.clear_auto_report_data()
        except Exception as e:
            self.get_logger().error(f"Error Publishing Encoder Data: {str(e)}")

    def publish_robot_state(self):
        """Publish Robot State"""
        try:
            if self.is_real_robot:
                # Get Real Robot State
                robot_state = self.rm.get_robot_state()
            else:
                # Get Test Robot State
                robot_state = self.test_interface.get_robot_state()
            
            # Create Robot State Message
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
            
            # Publish Message
            self.robot_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f"Error Publishing Robot State: {str(e)}")

    def publish_sensor_data(self):
        """Publish All Sensor Data"""
        try:
            if self.is_real_robot:
                # Publish Real Sensor Data
                self.publish_real_sensor_data()
            else:
                # Publish Test Sensor Data
                self.publish_test_sensor_data()
        except Exception as e:
            self.get_logger().error(f"Error Publishing Sensor Data: {str(e)}")

    def publish_real_sensor_data(self):
        """Publish Real Sensor Data"""
        try:
            # Get IMU Data
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

            # Get LiDAR Data
            lidar_data = self.rm.get_lidar_data()
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_link"
            scan_msg.angle_min = lidar_data['angle_min']
            scan_msg.angle_max = lidar_data['angle_max']
            scan_msg.angle_increment = lidar_data['angle_increment']
            scan_msg.range_min = lidar_data['range_min']
            scan_msg.range_max = lidar_data['range_max']
            scan_msg.ranges = lidar_data['ranges']
            self.scan_pub.publish(scan_msg)

            # Get Camera Image
            camera_data = self.rm.get_camera_data()
            if camera_data is not None:
                # Convert raw image data to ROS Image message
                image_msg = self.cv_bridge.cv2_to_imgmsg(camera_data, "bgr8")
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = "camera_link"
                self.image_pub.publish(image_msg)

                # Process and publish processed image
                processed_image = cv2.GaussianBlur(camera_data, (5, 5), 0)
                processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
                processed_msg.header.stamp = self.get_clock().now().to_msg()
                processed_msg.header.frame_id = "camera_link"
                self.image_processed_pub.publish(processed_msg)

            # Get Battery State
            battery_data = self.rm.get_battery_data()
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = battery_data['voltage']
            battery_msg.percentage = battery_data['percentage']
            self.battery_pub.publish(battery_msg)

            # Check Battery Level and Trigger Warning if Low
            if battery_msg.percentage < 20:  # 20% threshold
                self.buzzer_pub.publish(Bool(True))
                self.get_logger().warn("Low Battery Warning!")

        except Exception as e:
            self.get_logger().error(f"Error Publishing Real Sensor Data: {str(e)}")

    def publish_test_sensor_data(self):
        """Publish Test Sensor Data"""
        try:
            # Generate and Publish Test IMU Data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
            imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
            imu_msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.1)
            imu_msg.angular_velocity.x = np.random.normal(0, 0.1)
            imu_msg.angular_velocity.y = np.random.normal(0, 0.1)
            imu_msg.angular_velocity.z = np.random.normal(0, 0.1)
            self.imu_pub.publish(imu_msg)

            # Generate and Publish Test LiDAR Data
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_link"
            scan_msg.angle_min = -np.pi
            scan_msg.angle_max = np.pi
            scan_msg.angle_increment = np.pi / 180
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0
            scan_msg.ranges = [np.random.uniform(0.1, 10.0) for _ in range(360)]
            self.scan_pub.publish(scan_msg)

            # Generate and Publish Test Camera Image
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.circle(test_image, (320, 240), 100, (0, 255, 0), -1)
            image_msg = self.cv_bridge.cv2_to_imgmsg(test_image, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_link"
            self.image_pub.publish(image_msg)

            # Process and Publish Test Processed Image
            processed_image = cv2.GaussianBlur(test_image, (5, 5), 0)
            processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header.stamp = self.get_clock().now().to_msg()
            processed_msg.header.frame_id = "camera_link"
            self.image_processed_pub.publish(processed_msg)

            # Generate and Publish Test Battery State
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = np.random.uniform(11.0, 12.6)
            battery_msg.percentage = np.random.uniform(0, 100)
            self.battery_pub.publish(battery_msg)

            # Check Battery Level and Trigger Warning if Low
            if battery_msg.percentage < 20:  # 20% threshold
                self.buzzer_pub.publish(Bool(data=True))
                self.get_logger().warn("Test Mode: Low Battery Warning!")

        except Exception as e:
            self.get_logger().error(f"Error Publishing Test Sensor Data: {str(e)}")

    def emergency_stop(self):
        """Emergency Stop"""
        with self.lock:
            self.current_speed = 0.0
            self.current_angular = 0.0
            if self.is_real_robot:
                self.rm.set_car_motion(0.0, 0.0, 0.0)
            else:
                self.test_interface.set_car_motion(0.0, 0.0, 0.0)
            self.buzzer_pub.publish(Bool(True))
            self.log_operation("emergency_stop", 0.0, "emergency")
            self.get_logger().info("Emergency Stop Activated")

    def set_auto_report_state(self, enable: bool, forever: bool = False):
        """Set Auto Data Report State"""
        try:
            if self.rm:
                self.rm.set_auto_report_state(enable, forever)
                self.get_logger().info(f"Set Auto Data Report: enable={enable}, forever={forever}")
        except Exception as e:
            self.get_logger().error(f"Failed to Set Auto Data Report: {str(e)}")

    def clear_auto_report_data(self):
        """Clear Auto Report Data Cache"""
        try:
            if self.rm:
                self.rm.clear_auto_report_data()
                self.get_logger().info("Cleared Auto Report Data Cache")
        except Exception as e:
            self.get_logger().error(f"Failed to Clear Auto Report Data: {str(e)}")

    def set_motor(self, m1: int, m2: int, m3: int, m4: int):
        """Directly Control Four Motors"""
        try:
            if self.is_real_robot and GPIO_AVAILABLE:
                # Control Motors Using GPIO
                GPIO.output(GpioConfig.MOTOR_LEFT_FRONT, GPIO.HIGH if m1 > 0 else GPIO.LOW)
                GPIO.output(GpioConfig.MOTOR_RIGHT_FRONT, GPIO.HIGH if m2 > 0 else GPIO.LOW)
                GPIO.output(GpioConfig.MOTOR_LEFT_BACK, GPIO.HIGH if m3 > 0 else GPIO.LOW)
                GPIO.output(GpioConfig.MOTOR_RIGHT_BACK, GPIO.HIGH if m4 > 0 else GPIO.LOW)
            else:
                # Use Test Interface in Test Mode
                self.test_interface.set_motor(m1, m2, m3, m4)
            
            self.get_logger().info(f"Set Motor Speed: m1={m1}, m2={m2}, m3={m3}, m4={m4}")
        except Exception as e:
            self.get_logger().error(f"Failed to Set Motor Speed: {str(e)}")

    def get_encoder_data(self):
        """Get Encoder Data"""
        try:
            if self.is_real_robot and GPIO_AVAILABLE:
                # Read Encoder Data Using GPIO
                encoder_data = [
                    GPIO.input(GpioConfig.ENCODER_LEFT_FRONT),
                    GPIO.input(GpioConfig.ENCODER_RIGHT_FRONT),
                    GPIO.input(GpioConfig.ENCODER_LEFT_BACK),
                    GPIO.input(GpioConfig.ENCODER_RIGHT_BACK)
                ]
            else:
                # Use Test Data in Test Mode
                encoder_data = self.test_interface.get_encoder_data()
            
            return encoder_data
        except Exception as e:
            self.get_logger().error(f"Failed to Get Encoder Data: {str(e)}")
            return [0, 0, 0, 0]

    def __del__(self):
        """Cleanup Resources"""
        try:
            # Stop Motors
            if self.is_real_robot:
                self.rm.set_car_motion(0.0, 0.0, 0.0)
            else:
                self.test_interface.set_car_motion(0.0, 0.0, 0.0)
            
            # Cleanup GPIO if available
            if GPIO_AVAILABLE:
                GPIO.cleanup()
        except Exception as e:
            self.get_logger().error(f"Failed to Cleanup Resources: {str(e)}")

def main(args=None):
    driver = None
    try:
        rclpy.init(args=args)
        driver = RosMasterDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        if driver:
            driver.emergency_stop()
    except Exception as e:
        logging.error(f"Driver Runtime Error: {str(e)}")
    finally:
        if driver:
            driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 