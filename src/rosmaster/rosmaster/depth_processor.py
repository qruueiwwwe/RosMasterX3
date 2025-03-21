#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建深度图像订阅器
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
            
        # 创建RGB图像订阅器
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
            
        # 创建处理后的图像发布器
        self.processed_pub = self.create_publisher(
            Image,
            '/processed_image',
            10)
            
        # 初始化参数
        self.min_depth = 0.5  # 最小深度值（米）
        self.max_depth = 5.0  # 最大深度值（米）
        
        self.get_logger().info('深度处理节点已启动')
        
    def depth_callback(self, msg):
        """处理深度图像"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 创建深度掩码
            mask = np.logical_and(depth_image > self.min_depth * 1000,  # 转换为毫米
                                depth_image < self.max_depth * 1000)
                                
            # 应用彩色映射以便可视化
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET)
                
            # 应用掩码
            depth_colormap[~mask] = 0
            
            # 发布处理后的图像
            processed_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {str(e)}')
            
    def image_callback(self, msg):
        """处理RGB图像"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 在这里可以添加RGB图像处理逻辑
            # 例如：物体检测、特征提取等
            
        except Exception as e:
            self.get_logger().error(f'处理RGB图像时出错: {str(e)}')
            
    def get_depth_at_point(self, x, y):
        """获取指定点的深度值"""
        if not hasattr(self, 'latest_depth'):
            return None
            
        try:
            return self.latest_depth[y, x]
        except IndexError:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 