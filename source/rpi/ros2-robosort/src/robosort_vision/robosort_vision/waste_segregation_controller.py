#!/usr/bin/env python3
"""
Waste Segregation Controller Node for RoboSort
Main logic node that coordinates YOLO detection with robot arm control
Handles waste classification and sorting operations
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray, String
from robosort_interfaces.srv import SetServo, MoveRobotArm, RotateBin
from std_srvs.srv import Trigger
import time


class WasteSegregationController(Node):
    def __init__(self):
        super().__init__('waste_segregation_controller')
        
        # Declare parameters
        self.declare_parameter('bin_capacity_threshold', 15.0)  # cm from sensor
        self.declare_parameter('pickup_height', 100.0)  # Lifter position for pickup
        self.declare_parameter('drop_height', 0.0)  # Lifter position for drop
        
        # Get parameters
        self.bin_threshold = self.get_parameter('bin_capacity_threshold').value
        self.pickup_height = self.get_parameter('pickup_height').value
        self.drop_height = self.get_parameter('drop_height').value
        
        # State variables
        self.current_detections = []
        self.bin_levels = [0.0, 0.0, 0.0, 0.0]  # Distance from 4 ultrasonic sensors
        self.is_processing = False
        self.bin_map = {
            'paper': 0,
            'plastic': 1,
            'metal': 2,
            'other': 3
        }
        
        # Subscribers
        self.create_subscription(
            Detection2DArray, '/robosort/detections',
            self.detection_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, '/robosort/ultrasonic_levels',
            self.ultrasonic_callback, 10
        )
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, '/robosort/controller_status', 10)
        
        # Service clients for robot control
        self.set_servo_client = self.create_client(SetServo, '/robosort/set_servo')
        self.move_arm_client = self.create_client(MoveRobotArm, '/robosort/move_arm')
        self.rotate_bin_client = self.create_client(RotateBin, '/robosort/rotate_bin')
        self.home_arm_client = self.create_client(Trigger, '/robosort/home_arm')
        self.enable_servos_client = self.create_client(Trigger, '/robosort/enable_servos')
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.set_servo_client.wait_for_service(timeout_sec=5.0)
        self.move_arm_client.wait_for_service(timeout_sec=5.0)
        self.rotate_bin_client.wait_for_service(timeout_sec=5.0)
        
        # Enable servos on startup
        self.enable_servos()
        time.sleep(0.5)
        
        # Home the arm
        self.home_arm()
        
        self.get_logger().info('Waste Segregation Controller initialized')
        
    def detection_callback(self, msg):
        """Handle new detection results"""
        if self.is_processing:
            return
            
        if len(msg.detections) > 0:
            # Store detections and process the first one
            self.current_detections = msg.detections
            self.get_logger().info(
                f'Detected {len(msg.detections)} objects: '
                f'{[d.id for d in msg.detections]}'
            )
            
            # Process first detection
            self.process_detection(msg.detections[0])
            
    def ultrasonic_callback(self, msg):
        """Handle ultrasonic sensor readings"""
        if len(msg.data) == 4:
            self.bin_levels = list(msg.data)
            
            # Check for full bins
            for i, level in enumerate(self.bin_levels):
                if level > 0 and level < self.bin_threshold:
                    self.get_logger().warn(
                        f'Bin compartment {i} is near capacity! Level: {level:.1f}cm'
                    )
                    
    def process_detection(self, detection):
        """Process a single detection and sort the waste"""
        self.is_processing = True
        
        try:
            # Get detected class name
            class_name = detection.id.lower()
            self.get_logger().info(f'Processing: {class_name}')
            
            # Determine target bin
            target_bin = self.bin_map.get(class_name, 3)  # Default to 'other' bin
            
            # Check if target bin is full
            if self.bin_levels[target_bin] < self.bin_threshold:
                self.get_logger().warn(
                    f'Target bin {target_bin} is full! Skipping.'
                )
                self.publish_status(f'bin_full:{target_bin}')
                self.is_processing = False
                return
                
            # Execute sorting sequence
            self.publish_status(f'sorting:{class_name}:bin_{target_bin}')
            
            # 1. Move arm to pickup position
            self.get_logger().info('Moving to pickup position...')
            self.move_to_pickup()
            time.sleep(1.0)
            
            # 2. Close gripper to grab object
            self.get_logger().info('Grabbing object...')
            self.grab_object()
            time.sleep(0.5)
            
            # 3. Lift the arm
            self.get_logger().info('Lifting arm...')
            self.lift_arm()
            time.sleep(1.0)
            
            # 4. Rotate bin to correct compartment
            self.get_logger().info(f'Rotating to bin {target_bin}...')
            self.rotate_to_bin(target_bin)
            time.sleep(2.0)
            
            # 5. Lower the arm
            self.get_logger().info('Lowering arm...')
            self.lower_arm()
            time.sleep(1.0)
            
            # 6. Release object
            self.get_logger().info('Releasing object...')
            self.release_object()
            time.sleep(0.5)
            
            # 7. Return to home position
            self.get_logger().info('Returning home...')
            self.home_arm()
            time.sleep(1.5)
            
            self.publish_status(f'sorted:{class_name}')
            self.get_logger().info(f'Successfully sorted {class_name} into bin {target_bin}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing detection: {e}')
            self.publish_status('error')
            self.home_arm()
            
        finally:
            self.is_processing = False
            
    def enable_servos(self):
        """Enable servo motors"""
        req = Trigger.Request()
        future = self.enable_servos_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
    def home_arm(self):
        """Home robot arm to default position"""
        req = Trigger.Request()
        future = self.home_arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
    def move_to_pickup(self):
        """Move arm to pickup position"""
        # Predefined pickup pose: [base, shoulder, elbow, wrist, gripper, lifter]
        pickup_pose = [90.0, 45.0, 45.0, 90.0, 0.0, 0.0]  # Open gripper, low position
        
        req = MoveRobotArm.Request()
        req.joint_angles = pickup_pose
        future = self.move_arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
    def grab_object(self):
        """Close gripper to grab object"""
        req = SetServo.Request()
        req.servo_num = 4  # Gripper servo
        req.angle = 45  # Close gripper
        future = self.set_servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
    def release_object(self):
        """Open gripper to release object"""
        req = SetServo.Request()
        req.servo_num = 4  # Gripper servo
        req.angle = 0  # Open gripper
        future = self.set_servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
    def lift_arm(self):
        """Lift the arm using servo 5 (lifter)"""
        req = SetServo.Request()
        req.servo_num = 5  # Lifter servo
        req.angle = int(self.pickup_height)
        future = self.set_servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
    def lower_arm(self):
        """Lower the arm"""
        req = SetServo.Request()
        req.servo_num = 5  # Lifter servo
        req.angle = int(self.drop_height)
        future = self.set_servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
    def rotate_to_bin(self, bin_number):
        """Rotate trash bin to specific compartment"""
        req = RotateBin.Request()
        req.compartment_number = bin_number
        future = self.rotate_bin_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
    def publish_status(self, status: str):
        """Publish controller status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WasteSegregationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
