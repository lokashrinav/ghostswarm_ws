#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import random

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        
        # Parameters
        self.rate = self.declare_parameter('update_rate', 1.0).value
        
        # Publishers
        self.target_pub = self.create_publisher(Point, '/uav/target_position', 10)
        
        # Subscribers
        self.uav_odom_sub = self.create_subscription(
            PoseStamped,
            '/uav1/pose',
            self.uav_odom_callback,
            10)
            
        self.ugv_odom_sub = self.create_subscription(
            PoseStamped,
            '/ugv1/pose',
            self.ugv_odom_callback,
            10)
            
        self.uav_camera_sub = self.create_subscription(
            Image,
            '/uav1/camera/image_raw',
            self.image_callback,
            10)
        
        # Initialize variables
        self.uav_pose = None
        self.ugv_pose = None
        self.target_position = None
        self.cv_bridge = CvBridge()
        
        # Create timer for main coordination loop
        self.timer = self.create_timer(1.0/self.rate, self.coordination_loop)
        
        self.get_logger().info('Coordinator node initialized')
    
    def uav_odom_callback(self, msg):
        """Callback for UAV odometry"""
        self.uav_pose = msg.pose
    
    def ugv_odom_callback(self, msg):
        """Callback for UGV odometry"""
        self.ugv_pose = msg.pose
    
    def image_callback(self, msg):
        """Process camera images from UAV"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Simple target detection (blue object)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define blue color range (adjust these values based on your target)
            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])
            
            # Create a mask for blue color
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # Publish the target position relative to the UAV
                    target_msg = Point()
                    target_msg.x = float(cX - cv_image.shape[1]//2) / 100.0  # Convert to meters
                    target_msg.y = float(cv_image.shape[0]//2 - cY) / 100.0  # Convert to meters
                    target_msg.z = 0.0  # Assume target is on the ground
                    
                    self.target_pub.publish(target_msg)
                    self.get_logger().info(f'Target detected at: ({target_msg.x}, {target_msg.y})')
            
            # Display the image (for debugging)
            cv2.imshow('UAV Camera', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def coordination_loop(self):
        """Main coordination loop"""
        if self.uav_pose is None or self.ugv_pose is None:
            return
            
        # Simple coordination logic:
        # 1. UAV explores and detects targets
        # 2. When target is found, UGV is directed to it
        # 3. If UGV gets stuck, UAV provides alternative path
        
        # For now, just log the positions
        self.get_logger().debug(f'UAV Position: {self.uav_pose.position}')
        self.get_logger().debug(f'UGV Position: {self.ugv_pose.position}')

def main(args=None):
    rclpy.init(args=args)
    
    coordinator = Coordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
