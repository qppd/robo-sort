"""
Test Script for RoboSort Obstacle Avoidance System
Provides various testing modes for validating the navigation system
"""

import sys
import os
import time
import argparse

# Add matplotlib imports for visualization
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for better real-time updating
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

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
        print("LIDAR connected successfully")
        print("\nWaiting for data (5 seconds)...")
        time.sleep(5)
        
        distances = lidar_data['distances']
        if distances:
            print(f" Receiving data: {len(distances)} distance readings")
            print("\nSample readings (first 10):")
            for i, (angle, dist) in enumerate(list(distances.items())[:10]):
                print(f"  Angle {angle:6.2f}° → Distance: {dist:6.2f} cm")
        else:
            print("✗ No data received")
        
        stop_func()
        print("\n LIDAR test completed")
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
        
        print(" Arduino connected successfully")
        
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
        print("\n Arduino test completed")
        return True
        
    except Exception as e:
        print(f"✗ Arduino test failed: {e}")
        return False


def update_visualization_plot(frame, lidar_data, ax):
    """Update function for matplotlib animation"""
    # Properly clear all previous artists
    ax.clear()
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)  # Clockwise
    ax.set_thetamin(0)
    ax.set_thetamax(360)
    ax.set_rlim(0, 200)
    ax.set_title("LIDAR Real-Time Visualization (Full Navigation Test)", va='bottom')
    ax.grid(True, alpha=0.3)
    
    distances = lidar_data['distances'].copy()
    angles_deg = []
    distances_cm = []
    for angle, dist in distances.items():
        if dist > 45:  # Only plot distances greater than 45cm
            angles_deg.append(angle)
            distances_cm.append(dist)
    
    # Convert angles to radians for polar plot
    angles_rad = np.deg2rad(angles_deg)
    
    # Plot scatter
    if len(angles_rad) > 0 and len(distances_cm) > 0:
        ax.scatter(angles_rad, distances_cm, s=1, c='red', alpha=0.7)
    
    return ax,


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
        print(" LIDAR connected")
        time.sleep(2)
        
        # Create obstacle avoidance system
        oa = ObstacleAvoidance(
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
        print("\n Obstacle detection test completed")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False


def test_full_system(
    lidar_port: str = '/dev/ttyUSB0',
    arduino_port: str = '/dev/ttyACM0'
):
    """
    Test complete autonomous navigation system with real-time visualization (continuous)
    
    Args:
        lidar_port: LIDAR serial port
        arduino_port: Arduino serial port
    """
    print("\n" + "=" * 50)
    print("FULL SYSTEM TEST WITH VISUALIZATION (CONTINUOUS)")
    print("=" * 50)
    print(f"Running autonomous navigation with visualization continuously")
    print("⚠ WARNING: Robot will move! Close visualization window or Ctrl+C to stop\n")
    
    input("Press ENTER to start...")
    
    try:
        navigator = AutonomousNavigator(
            lidar_port=lidar_port,
            arduino_port=arduino_port,
            update_rate=0.0,
            verbose=True
        )
        
        if not navigator.start():
            print("✗ Failed to start navigation system")
            return False
        
        # Set up the polar plot for visualization
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, polar=True)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)  # Clockwise
        ax.set_thetamin(0)
        ax.set_thetamax(360)
        ax.set_rlim(0, 200)
        ax.set_title("LIDAR Real-Time Visualization (Full Navigation Test)", va='bottom')
        ax.grid(True, alpha=0.3)
        
        # Animation
        ani = animation.FuncAnimation(fig, update_visualization_plot, fargs=(navigator.lidar_data, ax), 
                                     interval=100, blit=False, cache_frame_data=False)
        
        # Start navigation in a separate thread
        import threading
        
        navigation_running = [True]  # Use list to make it mutable from thread
        
        def navigation_thread():
            try:
                navigation_count = 0
                
                while navigation_running[0]:
                    try:
                        # Check if matplotlib window is still open
                        if not plt.get_fignums():
                            print("\n⚠ Matplotlib window closed, stopping navigation...")
                            navigation_running[0] = False
                            break
                            
                        # Get current LIDAR data (always fresh)
                        if not navigator.lidar_data or not navigator.lidar_data['distances']:
                            continue
                            
                        distances = navigator.lidar_data['distances'].copy()
                        
                        # Analyze obstacles with fresh data
                        analysis = navigator.obstacle_avoidance.analyze_obstacles(distances)
                        front_dist = analysis['front_min_distance']
                        left_dist = analysis['left_min_distance'] 
                        right_dist = analysis['right_min_distance']
                        
                        # PURELY REACTIVE NAVIGATION - No delays, no state machines
                        # React immediately to current sensor readings
                        
                        # Priority 1: CRITICAL front obstacle - backup immediately
                        if front_dist < config.CRITICAL_DISTANCE:
                            print(f"[NAV] CRITICAL front at {front_dist:.1f}cm - backing up!")
                            navigator.arduino.backward(255)
                        
                        # Priority 2: Front obstacle - backup
                        elif front_dist < config.SAFE_DISTANCE:
                            print(f"[NAV] Front obstacle at {front_dist:.1f}cm - backing up")
                            navigator.arduino.backward(200)
                        
                        # Priority 3: Left obstacle close - turn right to avoid
                        elif left_dist < 60:
                            print(f"[NAV] Left obstacle at {left_dist:.1f}cm - turning right")
                            # Decide rotation vs turn based on how close
                            if left_dist < config.CRITICAL_DISTANCE:
                                navigator.arduino.rotate_right(255)  # Rotate in place if very close
                            else:
                                navigator.arduino.turn_right(200)  # Gentle turn if moderate distance
                        
                        # Priority 4: Right obstacle close - turn left to avoid
                        elif right_dist < 60:
                            print(f"[NAV] Right obstacle at {right_dist:.1f}cm - turning left")
                            # Decide rotation vs turn based on how close
                            if right_dist < config.CRITICAL_DISTANCE:
                                navigator.arduino.rotate_left(255)  # Rotate in place if very close
                            else:
                                navigator.arduino.turn_left(200)  # Gentle turn if moderate distance
                        
                        # Priority 5: Path clear - move forward
                        else:
                            # All clear, move forward at speed based on front clearance
                            if front_dist > config.CLEAR_PATH_THRESHOLD:
                                navigator.arduino.forward(255)  # Full speed when very clear
                            elif front_dist > config.SAFE_DISTANCE + 20:
                                navigator.arduino.forward(200)  # Medium speed with good clearance
                            else:
                                navigator.arduino.forward(150)  # Slow speed near threshold
                        
                        navigation_count += 1
                        if navigation_count % 50 == 0:  # Log every 50 cycles
                            print(f"[NAV] Completed {navigation_count} navigation cycles - F:{front_dist:.0f} L:{left_dist:.0f} R:{right_dist:.0f}")
                            
                    except Exception as nav_error:
                        print(f"[NAV] Navigation error (continuing): {nav_error}")
                    
            except KeyboardInterrupt:
                print("\n⚠ Emergency stop! Sending MSTOP to Arduino...")
                navigator.arduino.stop()
                print("Motors stopped via MSTOP command")
            except Exception as thread_error:
                print(f"[NAV] Thread error: {thread_error}")
            finally:
                navigation_running[0] = False
                print(f"[NAV] Navigation thread stopped after {navigation_count} cycles")
        
        # Start navigation thread
        nav_thread = threading.Thread(target=navigation_thread)
        nav_thread.start()
        
        # Set up signal handler for graceful shutdown (Unix-like systems)
        try:
            import signal
            def signal_handler(signum, frame):
                print("\n⚠ Received SIGINT (Ctrl+C) - Sending MSTOP to Arduino...")
                navigator.arduino.stop()
                print("Motors stopped via MSTOP command")
                plt.close('all')
            
            signal.signal(signal.SIGINT, signal_handler)
        except (ImportError, AttributeError):
            # Signal handling not available on this platform
            pass
        
        # Set up window close event handler
        def on_close(event):
            print("\n⚠ Visualization window closed by user")
            navigation_running[0] = False
        
        fig.canvas.mpl_connect('close_event', on_close)
        
        try:
            # Show visualization (blocking)
            plt.show()
        except KeyboardInterrupt:
            print("\n⚠ Visualization interrupted by user - Sending MSTOP to Arduino...")
            navigator.arduino.stop()
            print("Motors stopped via MSTOP command")
        finally:
            # Stop navigation
            navigation_running[0] = False
            nav_thread.join(timeout=2.0)
            navigator.stop()
            plt.close('all')  # Ensure all plots are closed
            print("\n Full system test with visualization stopped")
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
    
    print("\n Configuration test completed")
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
  full        - Test complete autonomous navigation with real-time visualization (continuous, robot will move!)
  config      - Display current configuration
  all         - Run all tests (except full navigation)

Examples:
  python test_navigation.py lidar /dev/ttyUSB0
  python test_navigation.py arduino /dev/ttyUSB0 /dev/ttyACM0
  python test_navigation.py detection /dev/ttyUSB0 --duration 60
  python test_navigation.py full /dev/ttyUSB0 /dev/ttyACM0
        """
    )
    
    parser.add_argument('mode', 
                       choices=['lidar', 'arduino', 'detection', 'full', 'config', 'all'],
                       help='Test mode to run')
    parser.add_argument('lidar_port', nargs='?', type=str, default=config.LIDAR_PORT,
                       help=f'LIDAR serial port (positional, default: {config.LIDAR_PORT})')
    parser.add_argument('arduino_port', nargs='?', type=str, default=config.ARDUINO_PORT,
                       help=f'Arduino serial port (positional, default: {config.ARDUINO_PORT})')
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
        success = test_full_system(args.lidar_port, args.arduino_port)
    
    if args.mode == 'config' or args.mode == 'all':
        success = test_configuration() and success
    
    print("\n" + "=" * 50)
    if success:
        print(" ALL TESTS PASSED")
    else:
        print("✗ SOME TESTS FAILED")
    print("=" * 50)
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
