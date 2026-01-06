#!/usr/bin/env python3
"""
LiDAR LD06 Processor Node
Processes LD06 LiDAR data for object detection and distance measurement
Integrates with YOLO for 3D object localization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32, String
from vision_msgs.msg import Detection2DArray
import numpy as np
import math


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Parameters
        self.declare_parameter('max_range', 12.0)  # LD06 max range in meters
        self.declare_parameter('min_range', 0.02)  # LD06 min range
        self.declare_parameter('roi_angle', 60.0)  # Region of interest angle (degrees)
        self.declare_parameter('object_detection_threshold', 0.1)  # Clustering threshold
        
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.roi_angle = math.radians(self.get_parameter('roi_angle').value)
        self.detection_threshold = self.get_parameter('object_detection_threshold').value
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # LD06 LiDAR scan topic
            self.lidar_callback,
            10
        )
        
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            '/robosort/detections',
            self.yolo_callback,
            10
        )
        
        # Publishers
        self.object_distance_pub = self.create_publisher(
            Float32,
            '/robosort/object_distance',
            10
        )
        
        self.object_position_pub = self.create_publisher(
            PointStamped,
            '/robosort/object_position_3d',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/robosort/lidar_status',
            10
        )
        
        # State variables
        self.latest_scan = None
        self.latest_yolo_detections = None
        self.objects_detected = []
        
        # Timers
        self.create_timer(0.1, self.process_timer_callback)  # 10 Hz processing
        
        self.get_logger().info('🔵 LiDAR LD06 Processor initialized')
        self.get_logger().info(f'   Max range: {self.max_range}m, ROI: ±{math.degrees(self.roi_angle)}°')

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR scan data"""
        self.latest_scan = msg
        
        # Find closest object in front ROI
        front_distances = self.get_front_distances(msg)
        
        if front_distances:
            min_distance = min(front_distances)
            
            # Publish closest object distance
            distance_msg = Float32()
            distance_msg.data = min_distance
            self.object_distance_pub.publish(distance_msg)
            
            # Detect objects using clustering
            self.objects_detected = self.detect_objects_from_scan(msg)

    def get_front_distances(self, scan: LaserScan):
        """Extract distances in front ROI"""
        distances = []
        num_readings = len(scan.ranges)
        
        if num_readings == 0:
            return distances
        
        angle_increment = scan.angle_increment
        angle_min = scan.angle_min
        
        for i, distance in enumerate(scan.ranges):
            angle = angle_min + i * angle_increment
            
            # Check if within front ROI and valid range
            if abs(angle) <= self.roi_angle / 2:
                if self.min_range < distance < self.max_range:
                    distances.append(distance)
        
        return distances

    def detect_objects_from_scan(self, scan: LaserScan):
        """
        Detect individual objects from LiDAR scan using clustering
        Returns list of (distance, angle) tuples
        """
        objects = []
        num_readings = len(scan.ranges)
        
        if num_readings == 0:
            return objects
        
        angle_increment = scan.angle_increment
        angle_min = scan.angle_min
        
        # Get valid points
        valid_points = []
        for i, distance in enumerate(scan.ranges):
            if self.min_range < distance < self.max_range:
                angle = angle_min + i * angle_increment
                # Only consider front hemisphere
                if abs(angle) <= math.pi / 2:
                    valid_points.append((distance, angle, i))
        
        if not valid_points:
            return objects
        
        # Simple clustering: group nearby points
        clusters = []
        current_cluster = [valid_points[0]]
        
        for i in range(1, len(valid_points)):
            prev_dist, prev_angle, _ = valid_points[i-1]
            curr_dist, curr_angle, _ = valid_points[i]
            
            # Calculate Euclidean distance between points
            dx = curr_dist * math.cos(curr_angle) - prev_dist * math.cos(prev_angle)
            dy = curr_dist * math.sin(curr_angle) - prev_dist * math.sin(prev_angle)
            point_distance = math.sqrt(dx**2 + dy**2)
            
            if point_distance < self.detection_threshold:
                current_cluster.append(valid_points[i])
            else:
                if len(current_cluster) >= 3:  # Minimum points for an object
                    clusters.append(current_cluster)
                current_cluster = [valid_points[i]]
        
        # Add last cluster
        if len(current_cluster) >= 3:
            clusters.append(current_cluster)
        
        # Calculate object centers
        for cluster in clusters:
            # Average distance and angle
            avg_distance = np.mean([p[0] for p in cluster])
            avg_angle = np.mean([p[1] for p in cluster])
            objects.append((avg_distance, avg_angle))
        
        return objects

    def yolo_callback(self, msg: Detection2DArray):
        """Receive YOLO detections"""
        self.latest_yolo_detections = msg

    def process_timer_callback(self):
        """Fuse LiDAR and YOLO data for 3D localization"""
        if self.latest_scan is None:
            return
        
        if self.latest_yolo_detections is None or len(self.latest_yolo_detections.detections) == 0:
            return
        
        # For each YOLO detection, find corresponding LiDAR point
        for detection in self.latest_yolo_detections.detections:
            if not detection.results:
                continue
            
            # Get bounding box center
            bbox = detection.bbox
            bbox_center_x = bbox.center.position.x
            bbox_center_y = bbox.center.position.y
            
            # Assume camera FOV and image dimensions
            # LD06 has 360° FOV, camera typically 60-90°
            # Map bbox center to angle
            # For simplicity, assume front-facing camera with 60° horizontal FOV
            image_width = 640  # Typical resolution
            camera_hfov = math.radians(60)
            
            # Normalize bbox center to [-0.5, 0.5]
            normalized_x = (bbox_center_x / image_width) - 0.5
            
            # Calculate angle relative to camera center
            object_angle = normalized_x * camera_hfov
            
            # Find closest LiDAR point at this angle
            distance = self.find_lidar_distance_at_angle(object_angle)
            
            if distance is not None:
                # Calculate 3D position (camera frame)
                x = distance * math.cos(object_angle)
                y = distance * math.sin(object_angle)
                z = 0.0  # Assuming planar ground
                
                # Publish 3D position
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = 'lidar_frame'
                point_msg.point.x = x
                point_msg.point.y = y
                point_msg.point.z = z
                
                self.object_position_pub.publish(point_msg)
                
                # Log detection
                obj_class = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                self.get_logger().info(
                    f'📍 Object localized: {obj_class} at ({x:.2f}m, {y:.2f}m, {z:.2f}m), '
                    f'distance: {distance:.2f}m, confidence: {confidence:.2f}'
                )
                
                # Publish status
                status_msg = String()
                status_msg.data = f'Object at {distance:.2f}m, angle {math.degrees(object_angle):.1f}°'
                self.status_pub.publish(status_msg)

    def find_lidar_distance_at_angle(self, target_angle):
        """Find LiDAR distance measurement at specific angle"""
        if self.latest_scan is None:
            return None
        
        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        
        # Normalize target angle to scan range
        if target_angle < angle_min or target_angle > angle_max:
            return None
        
        # Find closest index
        index = int((target_angle - angle_min) / angle_increment)
        
        if 0 <= index < len(scan.ranges):
            distance = scan.ranges[index]
            if self.min_range < distance < self.max_range:
                return distance
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
