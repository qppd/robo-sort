#!/usr/bin/env python3
"""
Fake Odometry Publisher for Testing Nav2
Publishes static odometry and TF odom->base_link for testing without real wheels
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)
        
        # State (static for fake odom)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.get_logger().info('🤖 Fake Odometry Publisher started')
        self.get_logger().info(f'   Publishing at {self.publish_rate} Hz')
        self.get_logger().info(f'   Frame: {self.odom_frame} -> {self.base_frame}')
        self.get_logger().warn('⚠️  Using FAKE odometry for testing only!')
    
    def publish_odom(self):
        """Publish odometry message and TF transform"""
        now = self.get_clock().now().to_msg()
        
        # Create quaternion from yaw
        quat = self.euler_to_quaternion(0, 0, self.theta)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        
        # Velocity (zero for static fake odom)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
