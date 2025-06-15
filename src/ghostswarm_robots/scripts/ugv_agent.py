#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class UGVAgent(Node):
    def __init__(self):
        super().__init__('ugv_agent')
        
        # Parameters
        self.agent_id = self.declare_parameter('agent_id', 'ugv1').value
        self.rate = self.declare_parameter('update_rate', 10.0).value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.agent_id}/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, f'/{self.agent_id}/goal_pose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.agent_id}/odom',
            self.odom_callback,
            10)
            
        self.lidar_sub = self.create_subscription(
            LaserScan,
            f'/{self.agent_id}/scan',
            self.lidar_callback,
            10)
            
        # Communication subscriber
        self.comm_sub = self.create_subscription(
            PoseStamped,
            '/uav/target_position',
            self.comm_callback,
            10)
        
        # Initialize variables
        self.current_pose = None
        self.lidar_data = None
        self.target_position = None
        
        # Create timer for main control loop
        self.timer = self.create_timer(1.0/self.rate, self.control_loop)
        
        self.get_logger().info(f'UGV Agent {self.agent_id} initialized')
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.current_pose = msg.pose.pose
    
    def lidar_callback(self, msg):
        """Callback for LIDAR data"""
        self.lidar_data = {
            'ranges': np.array(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }
    
    def comm_callback(self, msg):
        """Callback for communication from UAV"""
        self.target_position = msg.pose.position
        self.get_logger().info(f'Received target position: {self.target_position}')
    
    def avoid_obstacles(self):
        """Simple obstacle avoidance"""
        if self.lidar_data is None:
            return Twist()
            
        # Simple obstacle avoidance
        ranges = self.lidar_data['ranges']
        front_ranges = np.concatenate([
            ranges[-15:],  # Wrap around for front
            ranges[:15]    # Front sector
        ])
        
        min_distance = np.min(front_ranges)
        
        cmd_vel = Twist()
        if min_distance < 1.0:  # 1m threshold
            # Turn right if obstacle is too close
            cmd_vel.angular.z = -0.5
        else:
            # Move forward
            cmd_vel.linear.x = 0.3
            
        return cmd_vel
    
    def control_loop(self):
        """Main control loop for the UGV agent"""
        if self.current_pose is None or self.lidar_data is None:
            return
            
        # Simple behavior: if we have a target, move towards it
        # Otherwise, use obstacle avoidance
        if self.target_position:
            # TODO: Implement path planning to target
            cmd_vel = self.avoid_obstacles()
        else:
            # Just do obstacle avoidance
            cmd_vel = self.avoid_obstacles()
            
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    ugv_agent = UGVAgent()
    
    try:
        rclpy.spin(ugv_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ugv_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
