#!/usr/bin/env python3
"""
Motor Controller Node for RoboSort
Handles serial communication with Arduino for DC motor control
Provides velocity command interface for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from robosort_interfaces.srv import ControlMotor
import serial
import time
from typing import Optional


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_speed', 255)
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels in meters
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        # Publishers (create BEFORE connecting serial)
        self.status_pub = self.create_publisher(String, '/robosort/motor_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Services
        self.create_service(ControlMotor, '/robosort/control_motor', self.control_motor_callback)
        
        # Safety timer - stop motors if no command received
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        self.motors_stopped = False  # Track motor state to avoid spam
        
        # Initialize serial connection (AFTER publishers are created)
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.connect_serial(port, baudrate, timeout)
        
        self.get_logger().info('🤖 Motor Controller initialized')
        
    def connect_serial(self, port: str, baudrate: int, timeout: float) -> bool:
        """Connect to Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for Arduino reset
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            self.is_connected = True
            self.get_logger().info(f'✅ Connected to Arduino on {port}')
            self.publish_status('connected')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to {port}: {e}')
            self.is_connected = False
            self.publish_status('disconnected')
            return False
    
    def publish_status(self, status: str):
        """Publish motor controller status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
            
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
            self.is_connected = False
            return False
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist messages to differential drive motor commands
        linear.x = forward/backward velocity
        angular.z = rotation velocity
        """
        self.last_cmd_time = self.get_clock().now()
        self.motors_stopped = False  # Reset stopped flag when receiving commands
        
        linear = msg.linear.x
        angular = msg.angular.z
        
        self.get_logger().info(f'📥 Received cmd_vel: linear={linear:.3f}, angular={angular:.3f}')
        
        # Differential drive kinematics
        # v_left = linear - (angular * wheel_base / 2)
        # v_right = linear + (angular * wheel_base / 2)
        v_left = linear - (angular * self.wheel_base / 2)
        v_right = linear + (angular * self.wheel_base / 2)
        
        # Normalize to max_speed range (-255 to 255)
        # Assuming linear velocity is in range [-1, 1] m/s
        speed_left = int(max(-self.max_speed, min(self.max_speed, v_left * self.max_speed)))
        speed_right = int(max(-self.max_speed, min(self.max_speed, v_right * self.max_speed)))
        
        # Determine direction and absolute speed for each motor
        dir_left = 1 if speed_left >= 0 else 2  # 1=FORWARD, 2=BACKWARD
        dir_right = 1 if speed_right >= 0 else 2
        
        abs_left = abs(speed_left)
        abs_right = abs(speed_right)
        
        # Stop condition
        if abs_left < 10 and abs_right < 10:
            dir_left = 0  # STOP
            dir_right = 0
            abs_left = 0
            abs_right = 0
        
        # Send motor commands
        # Motor A = Left, Motor B = Right
        self.set_motor('A', dir_left, abs_left)
        self.set_motor('B', dir_right, abs_right)
        
        self.get_logger().info(f'🔧 Motors: L({dir_left},{abs_left}) R({dir_right},{abs_right})')
    
    def set_motor(self, motor: str, direction: int, speed: int):
        """Set individual motor speed and direction"""
        direction_map = {0: 'S', 1: 'F', 2: 'B', 3: 'R'}  # Stop, Forward, Backward, Brake
        cmd = f'M{motor} {direction_map.get(direction, "S")} {speed}'
        self.send_command(cmd)
    
    def stop_motors(self):
        if not self.motors_stopped:  # Only log once
            self.set_motor('A', 0, 0)
            self.set_motor('B', 0, 0)
            self.get_logger().info('⛔ Motors stopped')
            self.motors_stopped = True
        self.get_logger().info('⛔ Motors stopped')
    
    def safety_check(self):
        """Stop motors if no command received recently (safety timeout)"""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if not self.motors_stopped:  # Only log once per stop event
                
        if elapsed > 1.0:  # 1 second timeout
            self.get_logger().debug(f'⏱️  Safety timeout: {elapsed:.2f}s since last command')
            self.stop_motors()
    
    def control_motor_callback(self, request, response):
        """Service callback for direct motor control"""
        motor_id = request.motor_id
        direction = request.direction
        speed = request.speed
        
        motor_letter = 'A' if motor_id == 0 else 'B'
        
        if direction not in [0, 1, 2, 3]:
            response.success = False
            response.message = 'Invalid direction. Use 0=STOP, 1=FORWARD, 2=BACKWARD, 3=BRAKE'
            return response
        
        self.set_motor(motor_letter, direction, speed)
        response.success = True
        response.message = f'Motor {motor_letter}: direction={direction}, speed={speed}'
        self.last_cmd_time = self.get_clock().now()
        
        return response
        
    def destroy_node(self):
        """Cleanup - stop motors and close serial"""
        self.stop_motors()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
