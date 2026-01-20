#!/usr/bin/env python3
"""
Initial Odometry Publisher
Publishes identity odom->base_footprint transform on startup
Gets overridden by RF2O once it starts publishing
This prevents RViz from hanging while waiting for RF2O to initialize
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class InitialOdomPublisher(Node):
    def __init__(self):
        super().__init__('initial_odom_publisher')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Publish at 10Hz - will be overridden by RF2O once it starts
        self.timer = self.create_timer(0.1, self.publish_identity_odom)
        
        self.get_logger().info('Initial odometry publisher started - publishing identity transform')
        self.get_logger().info('This will be overridden by RF2O once laser odometry initializes')
    
    def publish_identity_odom(self):
        """Publish identity odom->base_footprint transform"""
        now = self.get_clock().now()
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Identity transform (0, 0, 0)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Identity pose
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # Zero velocity
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = InitialOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
