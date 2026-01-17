#!/usr/bin/env python3
"""
LiDAR TF Publisher Node
Publishes dynamic transform from base_footprint to lidar_link
This bypasses static transforms and ensures SLAM Toolbox can resolve the chain
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class LidarTFPublisher(Node):
    def __init__(self):
        super().__init__('lidar_tf_publisher')
        
        # Declare parameters (from URDF values)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('lidar_x', 0.4008)  # From URDF
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.276)  # wheel_radius + body_height/2 + lidar_offset_z
        
        # Get parameters
        publish_rate = self.get_parameter('publish_rate').value
        self.lidar_x = self.get_parameter('lidar_x').value
        self.lidar_y = self.get_parameter('lidar_y').value
        self.lidar_z = self.get_parameter('lidar_z').value
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish transform
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_transform)
        
        self.get_logger().info(f'🔵 LiDAR TF Publisher started at {publish_rate} Hz')
        self.get_logger().info(f'   Publishing base_footprint -> lidar_link at ({self.lidar_x}, {self.lidar_y}, {self.lidar_z})')
    
    def publish_transform(self):
        """Publish dynamic transform from base_footprint to lidar_link"""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'lidar_link'
        
        # Translation (from URDF calculations)
        t.transform.translation.x = self.lidar_x
        t.transform.translation.y = self.lidar_y
        t.transform.translation.z = self.lidar_z
        
        # Rotation (identity - no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
