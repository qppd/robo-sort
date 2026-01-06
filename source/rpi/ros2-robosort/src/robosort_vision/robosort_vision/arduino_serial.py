#!/usr/bin/env python3
"""
Arduino Serial Bridge Node for RoboSort
Handles serial communication with Arduino Mega for hardware control
Provides services for servo, stepper, and sensor operations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from robosort_interfaces.srv import SetServo, MoveRobotArm, RotateBin, GetDistance, ControlMotor
import serial
import time
from typing import Optional, List


class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('ultrasonic_publish_rate', 10.0)  # Hz
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        ultrasonic_rate = self.get_parameter('ultrasonic_publish_rate').value
        
        # Initialize serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.connect_serial(port, baudrate, timeout)
        
        # Publishers
        self.ultrasonic_pub = self.create_publisher(
            Float32MultiArray, '/robosort/ultrasonic_levels', 10
        )
        self.status_pub = self.create_publisher(String, '/robosort/arduino_status', 10)
        
        # Services for robot control
        self.create_service(SetServo, '/robosort/set_servo', self.set_servo_callback)
        self.create_service(MoveRobotArm, '/robosort/move_arm', self.move_arm_callback)
        self.create_service(RotateBin, '/robosort/rotate_bin', self.rotate_bin_callback)
        self.create_service(GetDistance, '/robosort/get_distance', self.get_distance_callback)
        self.create_service(ControlMotor, '/robosort/control_motor', self.control_motor_callback)
        self.create_service(Trigger, '/robosort/home_arm', self.home_arm_callback)
        self.create_service(Trigger, '/robosort/enable_servos', self.enable_servos_callback)
        self.create_service(Trigger, '/robosort/disable_servos', self.disable_servos_callback)
        self.create_service(Trigger, '/robosort/test_motors', self.test_motors_callback)
        
        # Timer for periodic ultrasonic sensor reading
        timer_period = 1.0 / ultrasonic_rate
        self.ultrasonic_timer = self.create_timer(timer_period, self.read_ultrasonic_levels)
        
        self.get_logger().info('Arduino Serial Node initialized')
        
    def connect_serial(self, port: str, baudrate: int, timeout: float) -> bool:
        """Connect to Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for Arduino reset
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            self.is_connected = True
            self.get_logger().info(f'Connected to Arduino on {port}')
            
            # Publish status
            status_msg = String()
            status_msg.data = 'connected'
            self.status_pub.publish(status_msg)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            self.is_connected = False
            return False
            
    def send_command(self, command: str) -> bool:
        """Send command to Arduino"""
        if not self.is_connected or not self.serial_conn:
            self.get_logger().warn('Not connected to Arduino')
            return False
            
        try:
            if not command.endswith('\n'):
                command += '\n'
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent: {command.strip()}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
            return False
            
    def read_response(self, timeout: float = 1.0) -> Optional[str]:
        """Read response from Arduino"""
        if not self.is_connected or not self.serial_conn:
            return None
            
        try:
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                self.get_logger().debug(f'Received: {response}')
                return response
        except Exception as e:
            self.get_logger().error(f'Error reading response: {e}')
        return None
        
    def read_all_responses(self, wait_time: float = 0.5) -> List[str]:
        """Read all available responses"""
        responses = []
        time.sleep(wait_time)
        
        while self.serial_conn and self.serial_conn.in_waiting > 0:
            response = self.read_response()
            if response:
                responses.append(response)
        return responses
        
    def set_servo_callback(self, request, response):
        """Service callback to set individual servo angle"""
        servo_num = request.servo_num
        angle = request.angle
        
        if 0 <= servo_num <= 5 and 0 <= angle <= 180:
            command = f'S{servo_num} {angle}'
            if self.send_command(command):
                self.read_all_responses(0.2)
                response.success = True
                response.message = f'Set servo {servo_num} to {angle}°'
            else:
                response.success = False
                response.message = 'Failed to send command'
        else:
            response.success = False
            response.message = 'Invalid servo number (0-5) or angle (0-180)'
            
        return response
        
    def move_arm_callback(self, request, response):
        """Service callback to move robot arm to position"""
        # Move servos 0-4 for arm positioning
        angles = request.joint_angles[:5]  # First 5 joints are the arm
        
        success = True
        for i, angle in enumerate(angles):
            command = f'S{i} {int(angle)}'
            if not self.send_command(command):
                success = False
                break
            time.sleep(0.05)  # Small delay between commands
            
        if success:
            self.read_all_responses(0.3)
            response.success = True
            response.message = f'Moved arm to position: {angles}'
        else:
            response.success = False
            response.message = 'Failed to move arm'
            
        return response
        
    def rotate_bin_callback(self, request, response):
        """Service callback to rotate trash bin to specific compartment"""
        # 4 compartments, each 90 degrees apart
        # Calculate steps needed (200 steps per revolution for typical stepper)
        steps_per_rev = 200
        steps_per_compartment = steps_per_rev // 4  # 50 steps per 90°
        
        compartment = request.compartment_number
        if compartment < 0 or compartment >= 4:
            response.success = False
            response.message = 'Invalid compartment (0-3)'
            return response
            
        # Calculate direction and steps from current position
        # Assuming we track current position (simplified here)
        steps = steps_per_compartment * compartment
        direction = 0  # CW
        
        command = f'STEP {steps} {direction}'
        if self.send_command(command):
            time.sleep(steps / 100 + 0.5)  # Wait for movement
            self.read_all_responses()
            response.success = True
            response.message = f'Rotated to compartment {compartment}'
        else:
            response.success = False
            response.message = 'Failed to rotate bin'
            
        return response
        
    def get_distance_callback(self, request, response):
        """Service callback to get distance from specific ultrasonic sensor"""
        sensor_num = request.sensor_id
        
        if sensor_num < 1 or sensor_num > 4:
            response.distance = 0.0
            response.success = False
            return response
            
        command = f'UDIST {sensor_num}'
        if self.send_command(command):
            responses = self.read_all_responses(0.3)
            
            # Parse distance from response
            for resp in responses:
                if 'Distance:' in resp and 'cm' in resp:
                    try:
                        # Extract distance value
                        distance_str = resp.split('Distance:')[1].split('cm')[0].strip()
                        response.distance = float(distance_str)
                        response.success = True
                        return response
                    except:
                        pass
                        
        response.distance = 0.0
        response.success = False
        return response
        
    def home_arm_callback(self, request, response):
        """Service callback to home robot arm to default position"""
        # Home position: all servos to 90 degrees
        home_angles = [90, 90, 90, 90, 90, 0]  # Last servo (lifter) to 0
        
        success = True
        for i, angle in enumerate(home_angles):
            command = f'S{i} {angle}'
            if not self.send_command(command):
                success = False
                break
            time.sleep(0.1)
            
        if success:
            self.read_all_responses(0.5)
            response.success = True
            response.message = 'Arm homed to default position'
        else:
            response.success = False
            response.message = 'Failed to home arm'
            
        return response
        
    def enable_servos_callback(self, request, response):
        """Enable servo motors"""
        if self.send_command('SENABLE'):
            self.read_all_responses()
            response.success = True
            response.message = 'Servos enabled'
        else:
            response.success = False
            response.message = 'Failed to enable servos'
        return response
        
    def disable_servos_callback(self, request, response):
        """Disable servo motors"""
        if self.send_command('SDISABLE'):
            self.read_all_responses()
            response.success = True
            response.message = 'Servos disabled'
        else:
            response.success = False
            response.message = 'Failed to disable servos'
        return response
        
    def read_ultrasonic_levels(self):
        """Periodically read all 4 ultrasonic sensors for bin levels"""
        if not self.is_connected:
            return
            
        levels = Float32MultiArray()
        levels.data = []
        
        # Read all 4 ultrasonic sensors
        for sensor in range(1, 5):
            command = f'UDIST {sensor}'
            if self.send_command(command):
                responses = self.read_all_responses(0.2)
                
                # Parse distance
                distance = 0.0
                for resp in responses:
                    if 'Distance:' in resp and 'cm' in resp:
                        try:
                            distance_str = resp.split('Distance:')[1].split('cm')[0].strip()
                            distance = float(distance_str)
                        except:
                            pass
                levels.data.append(distance)
            else:
                levels.data.append(0.0)
                
        # Publish levels
        self.ultrasonic_pub.publish(levels)
    
    def control_motor_callback(self, request, response):
        """Control DC motor via L298N driver"""
        motor_id = request.motor_id
        direction = request.direction
        speed = request.speed
        
        # Map motor ID to letter
        motor_letter = 'A' if motor_id == 0 else 'B'
        
        # Map direction to Arduino command
        direction_map = {
            0: 'S',  # STOP
            1: 'F',  # FORWARD
            2: 'B',  # BACKWARD
            3: 'R'   # BRAKE
        }
        
        if direction not in direction_map:
            response.success = False
            response.message = 'Invalid direction. Use 0=STOP, 1=FORWARD, 2=BACKWARD, 3=BRAKE'
            return response
        
        # Construct Arduino command: M<motor> <direction> <speed>
        # Example: MA F 200 (Motor A forward at speed 200)
        command = f'M{motor_letter} {direction_map[direction]} {speed}'
        
        if self.send_command(command):
            responses = self.read_all_responses(0.5)
            response.success = True
            response.message = f'Motor {motor_letter} {direction_map[direction]} at speed {speed}'
            self.get_logger().info(f'Motor control: {response.message}')
        else:
            response.success = False
            response.message = 'Failed to send motor command'
            
        return response
    
    def test_motors_callback(self, request, response):
        """Test both DC motors"""
        command = 'MTEST'
        if self.send_command(command):
            time.sleep(10)  # Motor test takes ~10 seconds
            response.success = True
            response.message = 'Motor test sequence completed'
            self.get_logger().info('Motor test completed')
        else:
            response.success = False
            response.message = 'Failed to start motor test'
            
        return response
        
    def destroy_node(self):
        """Cleanup"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
