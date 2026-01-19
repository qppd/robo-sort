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
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
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
        
        self.get_logger().info('📡 Subscribed to /cmd_vel topic')
        
        # Services
        self.create_service(ControlMotor, '/robosort/control_motor', self.control_motor_callback)
        
        # Track motor state
        self.last_cmd_time = self.get_clock().now()
        self.motors_stopped = False
        
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
        Uses new continuous commands: FORWARD:, BACKWARD:, LEFT:, RIGHT:, TURN_LEFT:, TURN_RIGHT:
        """
        self.last_cmd_time = self.get_clock().now()
        self.motors_stopped = False  # Reset stopped flag when receiving commands
        
        linear = msg.linear.x
        angular = msg.angular.z
        
        self.get_logger().info(f'📥 Received cmd_vel: linear={linear:.3f}, angular={angular:.3f}')
        
        # Normalize velocities to 0-255 range
        # Assuming linear velocity is in range [-1, 1] m/s
        linear_speed = int(abs(linear) * self.max_speed)
        angular_speed = int(abs(angular) * self.max_speed)
        
        # Constrain to valid range
        linear_speed = max(0, min(255, linear_speed))
        angular_speed = max(0, min(255, angular_speed))
        
        # Determine the appropriate command based on linear and angular velocities
        command = None
        
        # Stop condition
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            command = 'MSTOP'
            self.get_logger().info('⛔ Sending MSTOP')
        # Pure rotation (spot turn)
        elif abs(linear) < 0.01 and abs(angular) >= 0.01:
            if angular > 0:
                command = f'TURN_LEFT:{angular_speed}'
                self.get_logger().info(f'🔄 Spot turn left at speed {angular_speed}')
            else:
                command = f'TURN_RIGHT:{angular_speed}'
                self.get_logger().info(f'🔄 Spot turn right at speed {angular_speed}')
        # Forward/Backward with possible turning
        elif abs(angular) < 0.01:
            # Pure forward or backward
            if linear > 0:
                command = f'FORWARD:{linear_speed}'
                self.get_logger().info(f'⬆️  Forward at speed {linear_speed}')
            else:
                command = f'BACKWARD:{linear_speed}'
                self.get_logger().info(f'⬇️  Backward at speed {linear_speed}')
        else:
            # Combined movement: use differential turning
            # Use the average speed for the turn command
            avg_speed = max(linear_speed, angular_speed)
            
            if linear > 0:
                # Moving forward with turn
                if angular > 0:
                    command = f'LEFT:{avg_speed}'
                    self.get_logger().info(f'↖️  Differential turn left at speed {avg_speed}')
                else:
                    command = f'RIGHT:{avg_speed}'
                    self.get_logger().info(f'↗️  Differential turn right at speed {avg_speed}')
            else:
                # Moving backward with turn (invert turn direction)
                if angular > 0:
                    command = f'RIGHT:{avg_speed}'
                    self.get_logger().info(f'↘️  Backward right at speed {avg_speed}')
                else:
                    command = f'LEFT:{avg_speed}'
                    self.get_logger().info(f'↙️  Backward left at speed {avg_speed}')
        
        # Send the command
        if command:
            self.send_command(command)
    
    def stop_motors(self):
        """Emergency stop both motors"""
        if not self.motors_stopped:  # Only log once
            self.send_command('MSTOP')
            self.get_logger().info('⛔ Motors stopped')
            self.motors_stopped = True
    
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
        
        # Map direction to command string
        direction_map = {0: 'S', 1: 'F', 2: 'B', 3: 'BR'}  # Stop, Forward, Backward, Brake
        cmd = f'M{motor_letter} {direction_map.get(direction, "S")} {speed}'
        self.send_command(cmd)
        
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
