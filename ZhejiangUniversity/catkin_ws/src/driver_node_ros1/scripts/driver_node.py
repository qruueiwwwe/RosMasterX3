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

class DeviceShifuDriver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # Initialize parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.camera_fps = rospy.get_param('~camera_fps', 30)
        
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
        self.rate = rospy.Rate(self.camera_fps)
        
        rospy.loginfo('DeviceShifu driver initialized')

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
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            rospy.loginfo('Executing forward command')
        elif msg.data == 2:  # Backward
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
            rospy.loginfo('Executing backward command')
        elif msg.data == 3:  # Turn left
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            rospy.loginfo('Executing turn left command')
        elif msg.data == 4:  # Turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            rospy.loginfo('Executing turn right command')
        else:
            rospy.logwarn(f'Unknown command: {msg.data}')
            return
            
        self.cmd_vel_pub.publish(cmd)

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

def main():
    try:
        driver = DeviceShifuDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 