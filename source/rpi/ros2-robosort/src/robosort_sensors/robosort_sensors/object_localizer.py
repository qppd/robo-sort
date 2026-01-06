#!/usr/bin/env python3
"""
Object Localizer Node
Combines YOLO detections with LiDAR data to provide accurate 3D pickup coordinates
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from robosort_interfaces.srv import GetObjectPosition
import math


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer')
        
        # Parameters
        self.declare_parameter('camera_height', 0.3)  # Height from ground (meters)
        self.declare_parameter('camera_tilt_angle', 30.0)  # Tilt angle (degrees)
        self.declare_parameter('gripper_offset_x', 0.05)  # Gripper offset from camera
        
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_tilt = math.radians(self.get_parameter('camera_tilt_angle').value)
        self.gripper_offset = self.get_parameter('gripper_offset_x').value
        
        # Subscribers
        self.position_sub = self.create_subscription(
            PointStamped,
            '/robosort/object_position_3d',
            self.position_callback,
            10
        )
        
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            '/robosort/detections',
            self.yolo_callback,
            10
        )
        
        # Publishers
        self.pickup_point_pub = self.create_publisher(
            PointStamped,
            '/robosort/pickup_point',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/robosort/localizer_status',
            10
        )
        
        # Service
        self.get_position_service = self.create_service(
            GetObjectPosition,
            '/robosort/get_object_position',
            self.get_position_callback
        )
        
        # State
        self.latest_position = None
        self.latest_detection = None
        
        self.get_logger().info('📍 Object Localizer initialized')
        self.get_logger().info(f'   Camera height: {self.camera_height}m, Tilt: {math.degrees(self.camera_tilt)}°')

    def position_callback(self, msg: PointStamped):
        """Receive 3D position from LiDAR processor"""
        self.latest_position = msg
        
        # Calculate pickup point (accounting for gripper offset)
        pickup_point = PointStamped()
        pickup_point.header = msg.header
        pickup_point.point.x = msg.point.x + self.gripper_offset
        pickup_point.point.y = msg.point.y
        pickup_point.point.z = 0.0  # Ground level
        
        self.pickup_point_pub.publish(pickup_point)
        
        self.get_logger().info(
            f'🎯 Pickup point: ({pickup_point.point.x:.3f}, '
            f'{pickup_point.point.y:.3f}, {pickup_point.point.z:.3f})'
        )

    def yolo_callback(self, msg: Detection2DArray):
        """Store latest YOLO detection"""
        if msg.detections:
            self.latest_detection = msg.detections[0]  # Use first detection

    def get_position_callback(self, request, response):
        """Service to get latest object position"""
        if self.latest_position is None:
            response.success = False
            response.message = 'No object position available'
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.distance = 0.0
        else:
            response.success = True
            response.message = 'Object position acquired'
            response.x = self.latest_position.point.x
            response.y = self.latest_position.point.y
            response.z = self.latest_position.point.z
            
            # Calculate distance from origin
            response.distance = math.sqrt(
                response.x**2 + response.y**2 + response.z**2
            )
            
            # Include object class if available
            if self.latest_detection and self.latest_detection.results:
                response.object_class = self.latest_detection.results[0].hypothesis.class_id
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
