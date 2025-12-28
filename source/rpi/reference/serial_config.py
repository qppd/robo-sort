"""
Serial Configuration Module for RoboSort
Handles serial communication between Raspberry Pi and Arduino
"""

import serial
import time
from typing import Optional, List


class SerialConfig:
    """
    Manages serial communication with Arduino over USB
    """
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 9600, timeout: int = 1):
        """
        Initialize serial connection
        
        Args:
            port: Serial port (default: /dev/ttyACM0 for Arduino Mega on RPi)
            baudrate: Communication speed (default: 9600)
            timeout: Read timeout in seconds (default: 1)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
    
    def connect(self) -> bool:
        """
        Establish serial connection with Arduino
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset after connection
            self.is_connected = True
            print(f"✓ Connected to Arduino on {self.port}")
            
            # Clear any initial data
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """
        Close serial connection
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.is_connected = False
            print("✓ Disconnected from Arduino")
    
    def send_command(self, command: str) -> bool:
        """
        Send command to Arduino
        
        Args:
            command: Command string to send
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected or not self.serial_conn:
            print("✗ Not connected to Arduino")
            return False
        
        try:
            # Add newline if not present
            if not command.endswith('\n'):
                command += '\n'
            
            self.serial_conn.write(command.encode('utf-8'))
            print(f"→ Sent: {command.strip()}")
            return True
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            return False
    
    def read_response(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read response from Arduino
        
        Args:
            timeout: Optional timeout override
            
        Returns:
            str: Response from Arduino, or None if no response
        """
        if not self.is_connected or not self.serial_conn:
            return None
        
        try:
            if timeout:
                original_timeout = self.serial_conn.timeout
                self.serial_conn.timeout = timeout
            
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                
                if timeout:
                    self.serial_conn.timeout = original_timeout
                
                return response
            
            if timeout:
                self.serial_conn.timeout = original_timeout
                
            return None
        except Exception as e:
            print(f"✗ Error reading response: {e}")
            return None
    
    def read_all_responses(self, wait_time: float = 0.5) -> List[str]:
        """
        Read all available responses from Arduino
        
        Args:
            wait_time: Time to wait for responses in seconds
            
        Returns:
            List[str]: All responses received
        """
        responses = []
        time.sleep(wait_time)
        
        while self.serial_conn and self.serial_conn.in_waiting > 0:
            response = self.read_response()
            if response:
                responses.append(response)
                print(f"← {response}")
        
        return responses
    
    def send_and_receive(self, command: str, wait_time: float = 0.5) -> List[str]:
        """
        Send command and read all responses
        
        Args:
            command: Command to send
            wait_time: Time to wait for responses
            
        Returns:
            List[str]: All responses received
        """
        if self.send_command(command):
            return self.read_all_responses(wait_time)
        return []
    
    # Servo Commands
    def test_servos(self) -> List[str]:
        """Run servo test sequence"""
        return self.send_and_receive("TEST", wait_time=15)
    
    def set_servo(self, servo_num: int, angle: int) -> List[str]:
        """Set servo to specific angle"""
        if 0 <= servo_num < 5 and 0 <= angle <= 180:
            return self.send_and_receive(f"S{servo_num} {angle}")
        else:
            print("✗ Invalid servo number (0-4) or angle (0-180)")
            return []
    
    # Motor Commands
    def test_motors(self) -> List[str]:
        """Run motor test sequence"""
        return self.send_and_receive("MTEST", wait_time=20)
    
    def control_motor(self, motor: str, direction: str, speed: int) -> List[str]:
        """
        Control DC motor
        
        Args:
            motor: 'A' or 'B'
            direction: 'F' (forward), 'B' (backward), 'S' (stop), 'BR' (brake)
            speed: 0-255
        """
        if motor.upper() in ['A', 'B'] and direction.upper() in ['F', 'B', 'S', 'BR'] and 0 <= speed <= 255:
            return self.send_and_receive(f"M{motor.upper()} {direction.upper()} {speed}")
        else:
            print("✗ Invalid motor command parameters")
            return []
    
    def stop_all_motors(self) -> List[str]:
        """Stop all motors"""
        return self.send_and_receive("MSTOP")
    
    # Ultrasonic Commands
    def test_ultrasonic(self) -> List[str]:
        """Run ultrasonic sensor test"""
        return self.send_and_receive("UTEST", wait_time=10)
    
    def get_distance(self) -> List[str]:
        """Get single distance measurement"""
        return self.send_and_receive("UDIST")
    
    def get_average_distance(self, samples: int = 3) -> List[str]:
        """Get average distance from multiple samples"""
        if 1 <= samples <= 10:
            return self.send_and_receive(f"UAVG {samples}", wait_time=1)
        else:
            print("✗ Invalid sample count (1-10)")
            return []
    
    def detect_object(self, threshold: int) -> List[str]:
        """Detect object within threshold distance"""
        if 1 <= threshold <= 400:
            return self.send_and_receive(f"UDETECT {threshold}")
        else:
            print("✗ Invalid threshold (1-400 cm)")
            return []
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
