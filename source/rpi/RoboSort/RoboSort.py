"""
RoboSort Main Application
Automated Paper and Plastic Waste Segregation System
Integrates serial communication with Arduino for control and testing
"""

import time
import sys
from serial_config import SerialConfig


def print_banner():
    """Display application banner"""
    print("=" * 60)
    print("  RoboSort - Automated Waste Segregation System")
    print("  Paper and Plastic Waste Sorting Robot")
    print("=" * 60)
    print()


def print_menu():
    """Display command menu"""
    print("\n" + "=" * 60)
    print("COMMAND MENU")
    print("=" * 60)
    print("\nSERVO COMMANDS:")
    print("  1. Test all servos")
    print("  2. Set specific servo angle")
    print("\nSTEPPER MOTOR COMMANDS:")
    print("  3. Test stepper motor")
    print("  4. Control stepper motor movement")
    print("  5. Home stepper motor")
    print("  6. Stop stepper motor")
    print("\nULTRASONIC COMMANDS:")
    print("  7. Test ultrasonic sensor")
    print("  8. Get distance measurement")
    print("  9. Get average distance")
    print("  10. Detect object (with threshold)")
    print("\nSYSTEM COMMANDS:")
    print("  11. Send custom command")
    print("  0. Exit")
    print("=" * 60)


def test_servos(serial_conn: SerialConfig):
    """Test servo functionality"""
    print("\n--- Testing Servos ---")
    responses = serial_conn.test_servos()
    if not responses:
        print("No response received from Arduino")


def set_servo_angle(serial_conn: SerialConfig):
    """Set specific servo to angle"""
    try:
        servo_num = int(input("Enter servo number (0-4): "))
        angle = int(input("Enter angle (0-180): "))
        print(f"\n--- Setting Servo {servo_num} to {angle}° ---")
        serial_conn.set_servo(servo_num, angle)
    except ValueError:
        print("✗ Invalid input. Please enter numbers only.")


def test_motors(serial_conn: SerialConfig):
    """Test motor functionality"""
    print("\n--- Testing Motors ---")
    responses = serial_conn.test_motors()
    if not responses:
        print("No response received from Arduino")


def control_motor(serial_conn: SerialConfig):
    """Control specific motor"""
    try:
        motor = input("Enter motor (A/B): ").upper()
        print("Directions: F=Forward, B=Backward, S=Stop, BR=Brake")
        direction = input("Enter direction: ").upper()
        speed = int(input("Enter speed (0-255): "))
        print(f"\n--- Controlling Motor {motor} ---")
        serial_conn.control_motor(motor, direction, speed)
    except ValueError:
        print("✗ Invalid input.")


def stop_motors(serial_conn: SerialConfig):
    """Stop all motors"""
    print("\n--- Stopping All Motors ---")
    serial_conn.stop_all_motors()


def test_ultrasonic(serial_conn: SerialConfig):
    """Test ultrasonic sensor"""
    print("\n--- Testing Ultrasonic Sensor ---")
    responses = serial_conn.test_ultrasonic()
    if not responses:
        print("No response received from Arduino")


def get_distance(serial_conn: SerialConfig):
    """Get distance measurement"""
    print("\n--- Getting Distance Measurement ---")
    serial_conn.get_distance()


def get_average_distance(serial_conn: SerialConfig):
    """Get average distance"""
    try:
        samples = int(input("Enter number of samples (1-10, default 3): ") or "3")
        print(f"\n--- Getting Average Distance ({samples} samples) ---")
        serial_conn.get_average_distance(samples)
    except ValueError:
        print("✗ Invalid input. Using default (3 samples).")
        serial_conn.get_average_distance(3)


def detect_object(serial_conn: SerialConfig):
    """Detect object with threshold"""
    try:
        threshold = int(input("Enter detection threshold in cm (1-400): "))
        print(f"\n--- Detecting Object (threshold: {threshold} cm) ---")
        serial_conn.detect_object(threshold)
    except ValueError:
        print("✗ Invalid input.")


def send_custom_command(serial_conn: SerialConfig):
    """Send custom command to Arduino"""
    command = input("\nEnter custom command: ")
    print(f"\n--- Sending Custom Command: {command} ---")
    serial_conn.send_and_receive(command, wait_time=2)


def main():
    """Main application loop"""
    print_banner()
    
    # Serial port configuration
    # Common ports: /dev/ttyACM0, /dev/ttyUSB0 for Raspberry Pi
    # For testing on Windows, use: COM3, COM4, etc.
    port = input("Enter serial port (default: /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
    
    # Create serial connection
    serial_conn = SerialConfig(port=port, baudrate=9600)
    
    # Connect to Arduino
    if not serial_conn.connect():
        print("\n✗ Failed to connect to Arduino. Please check:")
        print("  - Arduino is connected via USB")
        print("  - Correct serial port")
        print("  - Arduino firmware is uploaded")
        print("  - User has permission to access serial port")
        print("\nOn Raspberry Pi, you may need to run:")
        print("  sudo usermod -a -G dialout $USER")
        print("  (then logout and login again)")
        sys.exit(1)
    
    try:
        # Wait for Arduino to initialize
        time.sleep(2)
        
        # Read initial messages from Arduino
        print("\n--- Arduino Messages ---")
        initial_messages = serial_conn.read_all_responses(wait_time=1)
        if not initial_messages:
            print("No initial messages received")
        
        # Main command loop
        while True:
            print_menu()
            choice = input("\nEnter command number: ").strip()
            
            if choice == "0":
                print("\n✓ Exiting RoboSort...")
                break
            elif choice == "1":
                test_servos(serial_conn)
            elif choice == "2":
                set_servo_angle(serial_conn)
            elif choice == "3":
                # Test stepper motor - send STEPTEST
                print("\n--- Testing Stepper Motor ---")
                serial_conn.send_and_receive("STEPTEST", wait_time=2)
            elif choice == "4":
                # Control stepper motor movement
                try:
                    steps = int(input("Enter steps to move: "))
                    direction = int(input("Enter direction (0=CW, 1=CCW): "))
                    print(f"\n--- Moving Stepper {steps} steps {'CW' if direction == 0 else 'CCW'} ---")
                    serial_conn.send_and_receive(f"STEP {steps} {direction}", wait_time=2)
                except ValueError:
                    print("✗ Invalid input.")
            elif choice == "5":
                # Home stepper motor
                print("\n--- Homing Stepper Motor ---")
                serial_conn.send_and_receive("BIN_HOME", wait_time=5)  # Longer wait for homing
            elif choice == "6":
                # Stop stepper motor
                print("\n--- Stopping Stepper Motor ---")
                serial_conn.send_and_receive("STEPSTOP", wait_time=1)
            elif choice == "7":
                test_ultrasonic(serial_conn)
            elif choice == "8":
                get_distance(serial_conn)
            elif choice == "9":
                get_average_distance(serial_conn)
            elif choice == "10":
                detect_object(serial_conn)
            elif choice == "11":
                send_custom_command(serial_conn)
            else:
                print("\n✗ Invalid choice. Please try again.")
            
            # Small delay between commands
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\n✓ Program interrupted by user")
    
    finally:
        # Clean up
        serial_conn.disconnect()
        print("\n✓ RoboSort shutdown complete")


if __name__ == "__main__":
    main()

