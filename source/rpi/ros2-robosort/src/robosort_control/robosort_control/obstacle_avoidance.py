#!/usr/bin/env python3
"""
Obstacle Avoidance Node for RoboSort
Real-time reactive navigation using LiDAR data
No static maps - purely reactive obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import math
import numpy as np


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Declare parameters
        self.declare_parameter('min_obstacle_distance', 0.3)  # meters
        self.declare_parameter('warning_distance', 0.5)  # meters
        self.declare_parameter('max_linear_speed', 0.3)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('front_angle', 60.0)  # degrees - front detection cone
        self.declare_parameter('side_angle', 30.0)  # degrees - side detection
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.min_distance = self.get_parameter('min_obstacle_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.front_angle = math.radians(self.get_parameter('front_angle').value)
        self.side_angle = math.radians(self.get_parameter('side_angle').value)
        self.enabled = self.get_parameter('enabled').value
        
        # State
        self.obstacle_detected = False
        self.autonomous_mode = False
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/robosort/enable_avoidance',
            self.enable_callback,
            10
        )
        
        self.autonomous_sub = self.create_subscription(
            Bool,
            '/robosort/autonomous_mode',
            self.autonomous_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robosort/avoidance_status', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/robosort/obstacle_detected', 10)
        
        self.get_logger().info('🚧 Obstacle Avoidance Node initialized')
        self.get_logger().info(f'   Min distance: {self.min_distance}m')
        self.get_logger().info(f'   Warning distance: {self.warning_distance}m')
        self.get_logger().info(f'   Front cone: ±{math.degrees(self.front_angle)/2:.0f}°')
        
    def enable_callback(self, msg: Bool):
        """Enable/disable obstacle avoidance"""
        self.enabled = msg.data
        status = 'enabled' if self.enabled else 'disabled'
        self.get_logger().info(f'Obstacle avoidance {status}')
        self.publish_status(f'avoidance_{status}')
        
    def autonomous_callback(self, msg: Bool):
        """Enable/disable autonomous wandering mode"""
        self.autonomous_mode = msg.data
        status = 'enabled' if self.autonomous_mode else 'disabled'
        self.get_logger().info(f'Autonomous mode {status}')
        self.publish_status(f'autonomous_{status}')
    
    def publish_status(self, status: str):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan for obstacle detection and avoidance"""
        if not self.enabled:
            return
        
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Replace inf and nan with max range
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), msg.range_max, ranges)
        
        # Calculate angles for each reading
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        
        # Define regions
        front_mask = np.abs(angles) < (self.front_angle / 2)
        left_mask = (angles > self.front_angle / 2) & (angles < self.front_angle / 2 + self.side_angle)
        right_mask = (angles < -self.front_angle / 2) & (angles > -self.front_angle / 2 - self.side_angle)
        
        # Get minimum distances in each region
        front_min = np.min(ranges[front_mask]) if np.any(front_mask) else msg.range_max
        left_min = np.min(ranges[left_mask]) if np.any(left_mask) else msg.range_max
        right_min = np.min(ranges[right_mask]) if np.any(right_mask) else msg.range_max
        
        # Check for obstacles
        front_obstacle = front_min < self.min_distance
        front_warning = front_min < self.warning_distance
        left_obstacle = left_min < self.min_distance
        right_obstacle = right_min < self.min_distance
        
        self.obstacle_detected = front_obstacle or left_obstacle or right_obstacle
        
        # Publish obstacle status
        obstacle_msg = Bool()
        obstacle_msg.data = self.obstacle_detected
        self.obstacle_pub.publish(obstacle_msg)
        
        # Only send commands in autonomous mode
        if not self.autonomous_mode:
            return
        
        # Generate avoidance command
        cmd = Twist()
        
        if front_obstacle:
            # Obstacle directly ahead - turn in place
            cmd.linear.x = 0.0
            # Turn away from nearest side obstacle
            if left_min < right_min:
                cmd.angular.z = -self.max_angular  # Turn right
            else:
                cmd.angular.z = self.max_angular  # Turn left
            self.publish_status(f'avoiding_front:{front_min:.2f}m')
            
        elif front_warning:
            # Approaching obstacle - slow down and start turning
            speed_factor = (front_min - self.min_distance) / (self.warning_distance - self.min_distance)
            cmd.linear.x = self.max_linear * speed_factor
            
            if left_min < right_min:
                cmd.angular.z = -self.max_angular * 0.5
            else:
                cmd.angular.z = self.max_angular * 0.5
            self.publish_status(f'slowing:{front_min:.2f}m')
            
        elif left_obstacle:
            # Obstacle on left - veer right
            cmd.linear.x = self.max_linear * 0.5
            cmd.angular.z = -self.max_angular * 0.7
            self.publish_status(f'avoiding_left:{left_min:.2f}m')
            
        elif right_obstacle:
            # Obstacle on right - veer left
            cmd.linear.x = self.max_linear * 0.5
            cmd.angular.z = self.max_angular * 0.7
            self.publish_status(f'avoiding_right:{right_min:.2f}m')
            
        else:
            # Clear path - go forward
            cmd.linear.x = self.max_linear
            cmd.angular.z = 0.0
            self.publish_status('clear')
        
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().debug(
            f'Distances - Front: {front_min:.2f}m, Left: {left_min:.2f}m, Right: {right_min:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
