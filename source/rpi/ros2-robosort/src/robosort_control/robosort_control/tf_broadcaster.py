#!/usr/bin/env python3
"""
TF Broadcaster Node for RoboSort
Publishes TF transforms for the robot frame hierarchy:
  odom -> base_link -> lidar_frame

Uses a single broadcaster to avoid TF issues with multiple publishers
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math


class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_frame')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # LiDAR position on robot (static transform)
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.1)  # 10cm above base
        self.declare_parameter('lidar_roll', 0.0)
        self.declare_parameter('lidar_pitch', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)
        
        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.lidar_x = self.get_parameter('lidar_x').value
        self.lidar_y = self.get_parameter('lidar_y').value
        self.lidar_z = self.get_parameter('lidar_z').value
        self.lidar_roll = self.get_parameter('lidar_roll').value
        self.lidar_pitch = self.get_parameter('lidar_pitch').value
        self.lidar_yaw = self.get_parameter('lidar_yaw').value
        
        # Robot pose (dead reckoning)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0
        self.last_time = self.get_clock().now()
        
        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish static transform for lidar
        self.publish_lidar_static_tf()
        
        # Timer for dynamic TF (odom -> base_link)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_transforms)
        
        self.get_logger().info('📐 TF Broadcaster initialized')
        self.get_logger().info(f'   Frame tree: {self.odom_frame} -> {self.base_frame} -> {self.lidar_frame}')
        self.get_logger().info(f'   Publishing at {self.publish_rate} Hz')
        
    def publish_lidar_static_tf(self):
        """Publish static transform from base_link to lidar_frame"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        
        t.transform.translation.x = self.lidar_x
        t.transform.translation.y = self.lidar_y
        t.transform.translation.z = self.lidar_z
        
        quat = self.euler_to_quaternion(self.lidar_roll, self.lidar_pitch, self.lidar_yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'   Static TF: {self.base_frame} -> {self.lidar_frame}')
    
    def cmd_vel_callback(self, msg: Twist):
        """Update velocity from cmd_vel for dead reckoning"""
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z
    
    def publish_transforms(self):
        """Publish odom -> base_link transform and odometry message"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        # Simple dead reckoning (integrate velocities)
        # This is approximate - for better accuracy use wheel encoders
        if dt > 0 and dt < 1.0:  # Sanity check
            delta_x = self.vx * math.cos(self.theta) * dt
            delta_y = self.vx * math.sin(self.theta) * dt
            delta_theta = self.vtheta * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        quat = self.euler_to_quaternion(0.0, 0.0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vtheta
        
        self.odom_pub.publish(odom)
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """Convert Euler angles to quaternion [x, y, z, w]"""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        
        return [qx, qy, qz, qw]
    
    def reset_odometry(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info('Odometry reset to origin')


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
