#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import yaml
import paho.mqtt.client as mqtt
import threading
from http_server import init_http_server
import json

class DeviceShifuDriver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # Initialize parameters
        self.load_config()
        
        # Create publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
        
        # Create subscriber - for receiving control commands
        self.command_sub = rospy.Subscriber('remote_command', Int32, self.command_callback)
            
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.init_camera()
        
        # Create rate for publishing images
        self.rate = rospy.Rate(self.config['driver']['camera_fps'])
        
        # Initialize MQTT client
        self.init_mqtt()
        
        # Initialize HTTP server
        self.init_http()
        
        rospy.loginfo('DeviceShifu driver initialized')

    def load_config(self):
        """Load configuration file"""
        try:
            with open('/ros_ws/config/params.yaml', 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f'Failed to load config: {str(e)}')
            self.config = {
                'driver': {
                    'linear_speed': 0.5,
                    'angular_speed': 0.2,
                    'camera_fps': 30,
                    'mqtt': {
                        'broker': 'localhost',
                        'port': 1883,
                        'username': 'ros_robot',
                        'password': 'ros_robot'
                    }
                }
            }

    def init_mqtt(self):
        """Initialize MQTT client"""
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(
            self.config['driver']['mqtt']['username'],
            self.config['driver']['mqtt']['password']
        )
        
        # Set callback
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect to broker
        try:
            self.mqtt_client.connect(
                self.config['driver']['mqtt']['broker'],
                self.config['driver']['mqtt']['port']
            )
            self.mqtt_client.loop_start()
            rospy.loginfo('MQTT client started')
        except Exception as e:
            rospy.logerr(f'Failed to connect to MQTT broker: {str(e)}')

    def init_http(self):
        """Initialize HTTP server"""
        try:
            # Start HTTP server in a new thread
            http_thread = threading.Thread(
                target=init_http_server,
                args=(self.command_sub,)
            )
            http_thread.daemon = True
            http_thread.start()
            rospy.loginfo('HTTP server started')
        except Exception as e:
            rospy.logerr(f'Failed to start HTTP server: {str(e)}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            rospy.loginfo('Connected to MQTT broker')
            # Subscribe to command topic
            client.subscribe(self.config['driver']['mqtt']['topics']['command'])
        else:
            rospy.logerr(f'Failed to connect to MQTT broker with code: {rc}')

    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            data = json.loads(msg.payload.decode())
            command = data.get('command')
            if command is not None:
                # Publish command to ROS topic
                ros_msg = Int32()
                ros_msg.data = command
                self.command_callback(ros_msg)
        except Exception as e:
            rospy.logerr(f'Failed to process MQTT message: {str(e)}')

    def init_camera(self):
        """Initialize camera"""
        try:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                rospy.loginfo('Successfully connected to camera')
                self.has_camera = True
            else:
                raise Exception("Failed to open camera")
        except Exception as e:
            rospy.logwarn(f'Camera initialization failed: {str(e)}')
            rospy.loginfo('Using simulated image')
            self.has_camera = False
            self.cap = None

    def publish_image(self):
        """Publish image data"""
        if self.has_camera:
            # Use real camera
            ret, frame = self.cap.read()
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "camera_frame"
                    self.image_pub.publish(msg)
                    
                    # Publish image to MQTT
                    _, img_encoded = cv2.imencode('.jpg', frame)
                    self.mqtt_client.publish(
                        self.config['driver']['mqtt']['topics']['image'],
                        img_encoded.tobytes()
                    )
                    
                    rospy.logdebug('Published real camera image')
                except Exception as e:
                    rospy.logerr(f'Failed to publish image: {str(e)}')
        else:
            # Publish simulated image
            try:
                # Create solid color image (#39c5bb)
                image = np.zeros((480, 720, 3), dtype=np.uint8)
                # Note: OpenCV uses BGR format
                image[:] = [187, 197, 57]  # BGR format of #39c5bb
                
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_frame"
                self.image_pub.publish(msg)
                
                # Publish simulated image to MQTT
                _, img_encoded = cv2.imencode('.jpg', image)
                self.mqtt_client.publish(
                    self.config['driver']['mqtt']['topics']['image'],
                    img_encoded.tobytes()
                )
            except Exception as e:
                rospy.logerr(f'Failed to publish simulated image: {str(e)}')

    def command_callback(self, msg):
        """Process control commands"""
        cmd = Twist()
        
        if msg.data == 0:    # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.loginfo('Executing stop command')
        elif msg.data == 1:  # Forward
            cmd.linear.x = self.config['driver']['linear_speed']
            cmd.angular.z = 0.0
            rospy.loginfo('Executing forward command')
        elif msg.data == 2:  # Backward
            cmd.linear.x = -self.config['driver']['linear_speed']
            cmd.angular.z = 0.0
            rospy.loginfo('Executing backward command')
        elif msg.data == 3:  # Turn left
            cmd.linear.x = 0.0
            cmd.angular.z = self.config['driver']['angular_speed']
            rospy.loginfo('Executing turn left command')
        elif msg.data == 4:  # Turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.config['driver']['angular_speed']
            rospy.loginfo('Executing turn right command')
        else:
            rospy.logwarn(f'Unknown command: {msg.data}')
            return
            
        self.cmd_vel_pub.publish(cmd)
        
        # Publish status to MQTT
        status = {
            'command': msg.data,
            'linear_x': cmd.linear.x,
            'angular_z': cmd.angular.z,
            'timestamp': rospy.Time.now().to_sec()
        }
        self.mqtt_client.publish(
            self.config['driver']['mqtt']['topics']['status'],
            json.dumps(status)
        )

    def run(self):
        """Run node"""
        try:
            while not rospy.is_shutdown():
                self.publish_image()
                self.rate.sleep()
        except Exception as e:
            rospy.logerr(f'Error occurred: {str(e)}')
        finally:
            if self.has_camera and self.cap is not None:
                self.cap.release()
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main():
    try:
        driver = DeviceShifuDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 