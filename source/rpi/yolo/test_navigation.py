"""
Test Script for RoboSort Obstacle Avoidance System
Provides various testing modes for validating the navigation system
"""

import sys
import os
import time
import argparse

# Add paths for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ldrobot-ld06-lidar-python-driver-master'))

from obstacle_avoidance import ObstacleAvoidance
from listen_to_lidar import listen_to_lidar
from navigate import ArduinoController, AutonomousNavigator
import nav_config as config


def test_lidar_connection(port: str = '/dev/ttyUSB0'):
    """
    Test LIDAR connection and display data
    
    Args:
        port: LIDAR serial port
    """
    print("=" * 50)
    print("LIDAR CONNECTION TEST")
    print("=" * 50)
    print(f"Connecting to LIDAR on {port}...")
    
    try:
        lidar_data, stop_func = listen_to_lidar(port=port)
        print("✓ LIDAR connected successfully")
        print("\nWaiting for data (5 seconds)...")
        time.sleep(5)
        
        distances = lidar_data['distances']
        if distances:
            print(f"✓ Receiving data: {len(distances)} distance readings")
            print("\nSample readings (first 10):")
            for i, (angle, dist) in enumerate(list(distances.items())[:10]):
                print(f"  Angle {angle:6.2f}° → Distance: {dist:6.2f} cm")
        else:
            print("✗ No data received")
        
        stop_func()
        print("\n✓ LIDAR test completed")
        return True
        
    except Exception as e:
        print(f"✗ LIDAR test failed: {e}")
        return False


def test_arduino_connection(port: str = '/dev/ttyACM0', baudrate: int = 9600):
    """
    Test Arduino connection and motor commands
    
    Args:
        port: Arduino serial port
        baudrate: Serial baudrate
    """
    print("\n" + "=" * 50)
    print("ARDUINO CONNECTION TEST")
    print("=" * 50)
    print(f"Connecting to Arduino on {port}...")
    
    try:
        arduino = ArduinoController(port, baudrate)
        if not arduino.connect():
            print("✗ Failed to connect to Arduino")
            return False
        
        print("✓ Arduino connected successfully")
        
        # Test commands
        print("\nTesting motor commands (2 seconds each)...")
        
        print("  → Forward...")
        arduino.forward(100)
        time.sleep(2)
        
        print("  → Stop...")
        arduino.stop()
        time.sleep(1)
        
        print("  → Backward...")
        arduino.backward(100)
        time.sleep(2)
        
        print("  → Stop...")
        arduino.stop()
        time.sleep(1)
        
        print("  → Rotate left...")
        arduino.rotate_left(100)
        time.sleep(2)
        
        print("  → Stop...")
        arduino.stop()
        time.sleep(1)
        
        print("  → Rotate right...")
        arduino.rotate_right(100)
        time.sleep(2)
        
        print("  → Stop...")
        arduino.stop()
        
        arduino.disconnect()
        print("\n✓ Arduino test completed")
        return True
        
    except Exception as e:
        print(f"✗ Arduino test failed: {e}")
        return False


def test_obstacle_detection(port: str = '/dev/ttyUSB0', duration: int = 30):
    """
    Test obstacle detection and avoidance logic without motors
    
    Args:
        port: LIDAR serial port
        duration: Test duration in seconds
    """
    print("\n" + "=" * 50)
    print("OBSTACLE DETECTION TEST")
    print("=" * 50)
    print(f"Testing obstacle detection for {duration} seconds...")
    print("Place obstacles in front of the robot to test detection\n")
    
    try:
        # Connect to LIDAR
        lidar_data, stop_func = listen_to_lidar(port=port)
        print("✓ LIDAR connected")
        time.sleep(2)
        
        # Create obstacle avoidance system
        oa = ObstacleAvoidance(
            safe_distance=config.SAFE_DISTANCE,
            critical_distance=config.CRITICAL_DISTANCE,
            danger_distance=config.DANGER_DISTANCE
        )
        
        print("Starting detection loop (Ctrl+C to stop early)...\n")
        
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                distances = lidar_data['distances'].copy()
                
                if distances:
                    # Get recommended action
                    action, speed, info = oa.decide_action(distances)
                    
                    # Analyze obstacles
                    analysis = oa.analyze_obstacles(distances)
                    
                    # Print status
                    print(f"[{time.strftime('%H:%M:%S')}]")
                    print(f"  Front: {analysis['front_min_distance']:6.1f}cm "
                          f"Left: {analysis['left_min_distance']:6.1f}cm "
                          f"Right: {analysis['right_min_distance']:6.1f}cm")
                    print(f"  Action: {action.upper():15s} @ speed {speed:3d}")
                    print(f"  Info: {info}")
                    print()
                
                time.sleep(1)
        
        except KeyboardInterrupt:
            print("\n⚠ Test interrupted by user")
        
        stop_func()
        print("\n✓ Obstacle detection test completed")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False


def test_full_system(
    lidar_port: str = '/dev/ttyUSB0',
    arduino_port: str = '/dev/ttyACM0',
    duration: int = 60
):
    """
    Test complete autonomous navigation system
    
    Args:
        lidar_port: LIDAR serial port
        arduino_port: Arduino serial port
        duration: Test duration in seconds
    """
    print("\n" + "=" * 50)
    print("FULL SYSTEM TEST")
    print("=" * 50)
    print(f"Running autonomous navigation for {duration} seconds")
    print("⚠ WARNING: Robot will move! Ensure clear area.")
    print("Press Ctrl+C to emergency stop\n")
    
    input("Press ENTER to start...")
    
    try:
        navigator = AutonomousNavigator(
            lidar_port=lidar_port,
            arduino_port=arduino_port,
            update_rate=0.1,
            verbose=True
        )
        
        if not navigator.start():
            print("✗ Failed to start navigation system")
            return False
        
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                navigator.navigate_once()
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n⚠ Emergency stop!")
        
        navigator.stop()
        print("\n✓ Full system test completed")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False


def test_configuration():
    """
    Test configuration loading and behavior modes
    """
    print("\n" + "=" * 50)
    print("CONFIGURATION TEST")
    print("=" * 50)
    
    print("\nCurrent Configuration:")
    print(f"  LIDAR Port: {config.LIDAR_PORT}")
    print(f"  Arduino Port: {config.ARDUINO_PORT}")
    print(f"  Safe Distance: {config.SAFE_DISTANCE} cm")
    print(f"  Critical Distance: {config.CRITICAL_DISTANCE} cm")
    print(f"  Danger Distance: {config.DANGER_DISTANCE} cm")
    print(f"  Forward Speed: {config.FORWARD_SPEED}")
    print(f"  Update Rate: {config.UPDATE_RATE} seconds")
    
    print("\nAvailable Behavior Modes:")
    for mode_name, mode_config in config.BEHAVIOR_MODES.items():
        print(f"\n  {mode_name.upper()}:")
        for key, value in mode_config.items():
            print(f"    {key}: {value}")
    
    print("\n✓ Configuration test completed")
    return True


def main():
    """
    Main test interface
    """
    parser = argparse.ArgumentParser(
        description='RoboSort Obstacle Avoidance Test Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Test Modes:
  lidar       - Test LIDAR connection and data reception
  arduino     - Test Arduino connection and motor control
  detection   - Test obstacle detection logic (no motor movement)
  full        - Test complete autonomous navigation (robot will move!)
  config      - Display current configuration
  all         - Run all tests (except full navigation)

Examples:
  python test_navigation.py lidar
  python test_navigation.py arduino --arduino-port /dev/ttyACM0
  python test_navigation.py detection --duration 60
  python test_navigation.py full --lidar-port /dev/ttyUSB0
        """
    )
    
    parser.add_argument('mode', 
                       choices=['lidar', 'arduino', 'detection', 'full', 'config', 'all'],
                       help='Test mode to run')
    parser.add_argument('--lidar-port', type=str, default=config.LIDAR_PORT,
                       help=f'LIDAR serial port (default: {config.LIDAR_PORT})')
    parser.add_argument('--arduino-port', type=str, default=config.ARDUINO_PORT,
                       help=f'Arduino serial port (default: {config.ARDUINO_PORT})')
    parser.add_argument('--arduino-baudrate', type=int, default=config.ARDUINO_BAUDRATE,
                       help=f'Arduino baudrate (default: {config.ARDUINO_BAUDRATE})')
    parser.add_argument('--duration', type=int, default=30,
                       help='Test duration in seconds (default: 30)')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("ROBO-SORT NAVIGATION TEST SUITE")
    print("=" * 50)
    
    success = True
    
    if args.mode == 'lidar' or args.mode == 'all':
        success = test_lidar_connection(args.lidar_port) and success
    
    if args.mode == 'arduino' or args.mode == 'all':
        success = test_arduino_connection(args.arduino_port, args.arduino_baudrate) and success
    
    if args.mode == 'detection' or args.mode == 'all':
        success = test_obstacle_detection(args.lidar_port, args.duration) and success
    
    if args.mode == 'full':
        success = test_full_system(args.lidar_port, args.arduino_port, args.duration)
    
    if args.mode == 'config' or args.mode == 'all':
        success = test_configuration() and success
    
    print("\n" + "=" * 50)
    if success:
        print("✓ ALL TESTS PASSED")
    else:
        print("✗ SOME TESTS FAILED")
    print("=" * 50)
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
