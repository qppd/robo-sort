#!/usr/bin/env python3
"""
YOLO Detector Node for RoboSort
Detects and classifies waste materials using YOLOv8
Publishes detection results and bounding boxes for waste segregation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/robosort/detections')
        self.declare_parameter('annotated_image_topic', '/robosort/annotated_image')
        self.declare_parameter('camera_source', 'usb0')  # usb0, picamera0, or device path
        self.declare_parameter('publish_rate', 30.0)  # Hz
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        annotated_topic = self.get_parameter('annotated_image_topic').value
        self.camera_source = self.get_parameter('camera_source').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path, task='detect')
        self.labels = self.model.names
        self.get_logger().info(f'Model loaded with {len(self.labels)} classes')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, detection_topic, 10)
        self.annotated_image_pub = self.create_publisher(Image, annotated_topic, 10)
        
        # Initialize camera
        self.cap = None
        self.init_camera()
        
        # Create timer for camera capture and detection
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.detection_callback)
        
        self.get_logger().info('YOLO Detector Node initialized')
        
    def init_camera(self):
        """Initialize camera based on source type"""
        source = self.camera_source
        
        if source.startswith('usb'):
            # USB camera
            device_id = int(source.replace('usb', ''))
            self.cap = cv2.VideoCapture(device_id)
            self.get_logger().info(f'Initialized USB camera: /dev/video{device_id}')
            
        elif source.startswith('picamera'):
            # Raspberry Pi Camera
            try:
                from picamera2 import Picamera2
                self.cap = Picamera2()
                config = self.cap.create_video_configuration(
                    main={"format": 'RGB888', "size": (640, 480)}
                )
                self.cap.configure(config)
                self.cap.start()
                self.get_logger().info('Initialized Raspberry Pi Camera')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Pi Camera: {e}')
                self.cap = None
        else:
            # Try as video device path
            self.cap = cv2.VideoCapture(source)
            self.get_logger().info(f'Initialized camera from: {source}')
            
        if self.cap is None or (hasattr(self.cap, 'isOpened') and not self.cap.isOpened()):
            self.get_logger().error('Failed to initialize camera')
            
    def detection_callback(self):
        """Capture frame and run YOLO detection"""
        if self.cap is None:
            return
            
        # Capture frame
        if self.camera_source.startswith('picamera'):
            # Picamera2 capture
            frame = self.cap.capture_array()
        else:
            # OpenCV VideoCapture
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Failed to capture frame')
                return
                
        # Run YOLO inference
        results = self.model(frame, verbose=False)
        detections = results[0].boxes
        
        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_frame'
        
        # Process each detection
        for i in range(len(detections)):
            # Get bounding box
            xyxy_tensor = detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)
            
            # Get class and confidence
            class_id = int(detections[i].cls.item())
            class_name = self.labels[class_id]
            confidence = detections[i].conf.item()
            
            # Filter by confidence threshold
            if confidence < self.confidence_threshold:
                continue
                
            # Create Detection2D message
            detection = Detection2D()
            detection.header = detection_array.header
            
            # Set bounding box center and size
            detection.bbox.center = Pose2D()
            detection.bbox.center.x = float((xmin + xmax) / 2)
            detection.bbox.center.y = float((ymin + ymax) / 2)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(xmax - xmin)
            detection.bbox.size_y = float(ymax - ymin)
            
            # Set object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)
            
            # Store class name in ID field for easy access
            detection.id = class_name
            
            detection_array.detections.append(detection)
            
            # Draw bounding box on frame
            color = (0, 255, 0)  # Green for detected objects
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            label = f'{class_name}: {int(confidence*100)}%'
            cv2.putText(frame, label, (xmin, ymin-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Publish detections
        self.detection_pub.publish(detection_array)
        
        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = detection_array.header
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')
            
        # Log detection count
        if len(detection_array.detections) > 0:
            self.get_logger().info(
                f'Detected {len(detection_array.detections)} objects: '
                f'{[d.id for d in detection_array.detections]}'
            )
            
    def destroy_node(self):
        """Cleanup resources"""
        if self.cap is not None:
            if self.camera_source.startswith('picamera'):
                self.cap.stop()
            else:
                self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
