"""
Autonomous Navigation Module for RoboSort
Integrates LIDAR data, obstacle avoidance logic, and Arduino motor control
"""

import sys
import os
import time
import threading
from typing import Optional

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ldrobot-ld06-lidar-python-driver-master'))

from obstacle_avoidance import ObstacleAvoidance
from listen_to_lidar import listen_to_lidar
import nav_config as config


class ArduinoController:
    """
    Interface for controlling Arduino motor controller via serial
    """
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 9600):
        """
        Initialize Arduino controller
        
        Args:
            port: Serial port for Arduino (default: /dev/ttyACM0)
            baudrate: Serial baudrate (default: 9600)
        """
        import serial
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """
        Connect to Arduino via serial
        
        Returns:
            True if successful
        """
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Wait for Arduino reset
            self.connected = True
            print(f"✓ Connected to Arduino on {self.port}")
            
            # Clear buffers
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            
            return True
        except Exception as e:
            print(f"✗ Failed to connect to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.stop()
            self.serial_conn.close()
            self.connected = False
            print("✓ Disconnected from Arduino")
    
    def send_command(self, command: str) -> bool:
        """
        Send command to Arduino
        
        Args:
            command: Command string
            
        Returns:
            True if successful
        """
        if not self.connected or not self.serial_conn:
            print("✗ Not connected to Arduino")
            return False
        
        try:
            if not command.endswith('\n'):
                command += '\n'
            self.serial_conn.write(command.encode('utf-8'))
            return True
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            return False
    
    def forward(self, speed: int = 150):
        """Move forward"""
        self.send_command(f"FORWARD:{speed}")
    
    def backward(self, speed: int = 150):
        """Move backward"""
        self.send_command(f"BACKWARD:{speed}")
    
    def turn_left(self, speed: int = 150):
        """Turn left: only left wheel forward (right wheel stopped)"""
        self.send_command(f"LEFT:{speed}")
    
    def turn_right(self, speed: int = 150):
        """Turn right: only right wheel forward (left wheel stopped)"""
        self.send_command(f"RIGHT:{speed}")
    
    def rotate_left(self, speed: int = 180):
        """Rotate left in place"""
        self.send_command(f"TURN_LEFT:{speed}")
    
    def rotate_right(self, speed: int = 180):
        """Rotate right in place"""
        self.send_command(f"TURN_RIGHT:{speed}")
    
    def stop(self):
        """Stop all motors"""
        self.send_command("MSTOP")


class AutonomousNavigator:
    """
    Main autonomous navigation system
    Combines LIDAR sensing with obstacle avoidance and motor control
    """
    
    def __init__(
        self,
        lidar_port: str = '/dev/ttyUSB0',
        arduino_port: str = '/dev/ttyACM0',
        arduino_baudrate: int = 9600,
        update_rate: float = 0.1,  # seconds between navigation updates
        verbose: bool = True
    ):
        """
        Initialize autonomous navigation system
        
        Args:
            lidar_port: Serial port for LD06 LIDAR
            arduino_port: Serial port for Arduino
            arduino_baudrate: Baudrate for Arduino serial
            update_rate: Time between navigation decision updates (seconds)
            verbose: Enable verbose output
        """
        self.lidar_port = lidar_port
        self.arduino_port = arduino_port
        self.arduino_baudrate = arduino_baudrate
        self.update_rate = update_rate
        self.verbose = verbose
        
        # Initialize components
        self.obstacle_avoidance = ObstacleAvoidance(
            safe_distance=config.SAFE_DISTANCE,
            critical_distance=config.CRITICAL_DISTANCE,
            danger_distance=config.DANGER_DISTANCE,
            front_angle_range=config.FRONT_ANGLE_RANGE,
            left_angle_range=config.LEFT_ANGLE_RANGE,
            right_angle_range=config.RIGHT_ANGLE_RANGE,
            side_weight=config.SIDE_WEIGHT,
            clear_path_threshold=config.CLEAR_PATH_THRESHOLD,
            valid_angle_ranges=config.VALID_ANGLE_RANGES,
            min_valid_distance=config.MIN_VALID_DISTANCE
        )
        self.arduino = ArduinoController(arduino_port, arduino_baudrate)
        
        # LIDAR data and control
        self.lidar_data = None
        self.lidar_stop_func = None
        
        # Navigation state
        self.running = False
        self.paused = False
        self.last_action = None
        self.last_action_time = 0
        self.action_duration = 0.5  # Minimum time to execute an action (seconds)
        
    def start_lidar(self) -> bool:
        """
        Start LIDAR data collection
        
        Returns:
            True if successful
        """
        try:
            print(f"Starting LIDAR on {self.lidar_port}...")
            self.lidar_data, self.lidar_stop_func = listen_to_lidar(port=self.lidar_port)
            time.sleep(1)  # Wait for initial data
            print("✓ LIDAR started successfully")
            return True
        except Exception as e:
            print(f"✗ Failed to start LIDAR: {e}")
            return False
    
    def stop_lidar(self):
        """Stop LIDAR data collection"""
        if self.lidar_stop_func:
            self.lidar_stop_func()
            print("✓ LIDAR stopped")
    
    def start(self) -> bool:
        """
        Start autonomous navigation
        
        Returns:
            True if successful
        """
        # Connect to Arduino
        if not self.arduino.connect():
            return False
        
        # Start LIDAR
        if not self.start_lidar():
            self.arduino.disconnect()
            return False
        
        # Start navigation loop
        self.running = True
        print("✓ Autonomous navigation started")
        print("Press Ctrl+C to stop")
        
        return True
    
    def stop(self):
        """Stop autonomous navigation"""
        self.running = False
        self.arduino.stop()
        self.arduino.disconnect()
        self.stop_lidar()
        print("✓ Autonomous navigation stopped")
    
    def navigate_once(self):
        """
        Perform one navigation decision cycle
        """
        if not self.lidar_data or not self.lidar_data['distances']:
            if self.verbose:
                print("⚠ Waiting for LIDAR data...")
            return
        
        # Get current LIDAR distances
        distances = self.lidar_data['distances'].copy()
        
        # Decide action
        action, speed, info = self.obstacle_avoidance.decide_action(distances)
        
        # Check if we should continue current action or change
        current_time = time.time()
        time_since_last_action = current_time - self.last_action_time
        
        # If same action and minimum duration not met, skip
        if action == self.last_action and time_since_last_action < self.action_duration:
            return
        
        # Execute action
        if action == 'forward':
            self.arduino.forward(speed)
        elif action == 'backward':
            self.arduino.backward(speed)
        elif action == 'turn_left':
            self.arduino.turn_left(speed)
        elif action == 'turn_right':
            self.arduino.turn_right(speed)
        elif action == 'rotate_left':
            self.arduino.rotate_left(speed)
        elif action == 'rotate_right':
            self.arduino.rotate_right(speed)
        elif action == 'stop':
            self.arduino.stop()
        
        # Update state
        self.last_action = action
        self.last_action_time = current_time
        
        # Print info
        if self.verbose:
            print(f"[{time.strftime('%H:%M:%S')}] {action.upper():15s} @ {speed:3d} | {info}")
    
    def run(self):
        """
        Main navigation loop
        """
        if not self.start():
            return
        
        try:
            while self.running:
                if not self.paused:
                    self.navigate_once()
                time.sleep(self.update_rate)
        except KeyboardInterrupt:
            print("\n⚠ Interrupted by user")
        finally:
            self.stop()
    
    def pause(self):
        """Pause navigation (stop motors but keep sensors active)"""
        self.paused = True
        self.arduino.stop()
        print("⏸ Navigation paused")
    
    def resume(self):
        """Resume navigation"""
        self.paused = False
        print("▶ Navigation resumed")
    
    def get_status(self) -> str:
        """
        Get current navigation status
        
        Returns:
            Status string
        """
        if not self.lidar_data or not self.lidar_data['distances']:
            return "No LIDAR data available"
        
        distances = self.lidar_data['distances'].copy()
        return self.obstacle_avoidance.get_status_report(distances)


def main():
    """
    Main entry point
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='RoboSort Autonomous Navigation')
    parser.add_argument('--lidar-port', type=str, default='/dev/ttyUSB0',
                        help='LIDAR serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--arduino-port', type=str, default='/dev/ttyACM0',
                        help='Arduino serial port (default: /dev/ttyACM0)')
    parser.add_argument('--arduino-baudrate', type=int, default=9600,
                        help='Arduino serial baudrate (default: 9600)')
    parser.add_argument('--update-rate', type=float, default=0.1,
                        help='Navigation update rate in seconds (default: 0.1)')
    parser.add_argument('--quiet', action='store_true',
                        help='Disable verbose output')
    
    args = parser.parse_args()
    
    # Create navigator
    navigator = AutonomousNavigator(
        lidar_port=args.lidar_port,
        arduino_port=args.arduino_port,
        arduino_baudrate=args.arduino_baudrate,
        update_rate=args.update_rate,
        verbose=not args.quiet
    )
    
    # Run navigation
    navigator.run()


if __name__ == '__main__':
    main()
