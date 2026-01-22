"""
Configuration file for RoboSort Autonomous Navigation
Centralized configuration for LIDAR, obstacle avoidance, and motor control
"""

# ===== SERIAL PORT CONFIGURATION =====
# LIDAR (LD06) Serial Port
LIDAR_PORT = '/dev/ttyUSB1'  # Default USB serial port for LIDAR
LIDAR_BAUDRATE = 230400  # LD06 LIDAR baudrate

# Arduino Serial Port
ARDUINO_PORT = '/dev/ttyACM0'  # Default Arduino Mega port on Raspberry Pi
ARDUINO_BAUDRATE = 9600  # Arduino serial communication baudrate

# ===== OBSTACLE AVOIDANCE PARAMETERS =====
# Distance thresholds (in centimeters)
SAFE_DISTANCE = 50.0  # Minimum safe distance from obstacles
CRITICAL_DISTANCE = 30.0  # Distance requiring immediate turning
DANGER_DISTANCE = 15.0  # Distance requiring stop/reverse
CLEAR_PATH_THRESHOLD = 80.0  # Distance considered as clear path

# Angular zones (in degrees)
# 0° is forward, angles increase counterclockwise
FRONT_ANGLE_RANGE = 30  # ±30° from forward is considered "front"
LEFT_ANGLE_RANGE = (30, 100)  # Left side detection zone (adjusted to avoid rear objects)
RIGHT_ANGLE_RANGE = (260, 330)  # Right side detection zone (adjusted to avoid rear objects)

# Valid LIDAR angle ranges (to filter out readings from behind the robot)
# Only use angles: 0-100° and 260-360° to avoid detecting objects mounted on the robot
VALID_ANGLE_RANGES = [(0, 100), (260, 360)]  # List of (min, max) angle ranges to use

# Scoring weights
SIDE_WEIGHT = 0.7  # Weight for side obstacle scoring (0-1)

# ===== MOTOR CONTROL PARAMETERS =====
# Speed values (0-255)
FORWARD_SPEED = 180  # Default forward movement speed
BACKWARD_SPEED = 150  # Default backward movement speed
TURN_SPEED = 120  # Speed for single motor turns
ROTATE_SPEED = 180  # Speed for spot rotations

# Speed adjustments based on distance
SPEED_CLOSE = 100  # Speed when obstacles are close
SPEED_MEDIUM = 150  # Speed for moderate distances
SPEED_FAR = 180  # Speed when path is clear

# ===== NAVIGATION TIMING =====
UPDATE_RATE = 0.1  # Navigation decision update rate (seconds)
ACTION_DURATION = 0.5  # Minimum time to execute an action (seconds)
HEARTBEAT_INTERVAL = 2.0  # Interval for sending heartbeat to Arduino (seconds)

# ===== SAFETY PARAMETERS =====
AUTONOMOUS_TIMEOUT = 5000  # Arduino autonomous timeout (milliseconds)
ENABLE_SAFETY_STOP = True  # Enable safety stop on LIDAR data loss
MAX_CONSECUTIVE_ERRORS = 3  # Maximum consecutive LIDAR errors before stop

# ===== LOGGING AND DEBUG =====
VERBOSE = True  # Enable verbose output
LOG_LIDAR_DATA = False  # Log raw LIDAR data (creates large output)
LOG_DECISIONS = True  # Log navigation decisions
LOG_INTERVAL = 1.0  # Status logging interval (seconds)

# ===== DISPLAY CONFIGURATION =====
SHOW_STATUS_REPORT = True  # Show detailed status reports periodically
STATUS_REPORT_INTERVAL = 5.0  # Interval for status reports (seconds)
USE_COLORS = True  # Use ANSI colors in terminal output (if supported)

# ===== ADVANCED PARAMETERS =====
# Obstacle detection sensitivity
MIN_VALID_DISTANCE = 5.0  # Minimum valid LIDAR reading (cm)
MAX_VALID_DISTANCE = 1000.0  # Maximum valid LIDAR reading (cm)

# Path planning
PREFER_RIGHT_TURN = False  # Prefer right turns when scores are equal
BIAS_FACTOR = 1.0  # Multiplier for preferred direction (1.0 = no bias)

# Emergency behavior
EMERGENCY_REVERSE_TIME = 1.0  # Time to reverse in emergency (seconds)
EMERGENCY_ROTATE_TIME = 1.5  # Time to rotate after emergency (seconds)

# ===== FIELD MAPPING (OPTIONAL) =====
# For mapping known environments
USE_MAPPING = False  # Enable environment mapping
MAP_RESOLUTION = 10.0  # Map grid resolution (cm)
MAP_SIZE = (500, 500)  # Map size in cm (width, height)

# ===== BEHAVIOR MODES =====
# Different navigation behavior profiles
BEHAVIOR_MODES = {
    'cautious': {
        'safe_distance': 60.0,
        'critical_distance': 40.0,
        'danger_distance': 20.0,
        'forward_speed': 120,
        'rotate_speed': 150,
    },
    'normal': {
        'safe_distance': 50.0,
        'critical_distance': 30.0,
        'danger_distance': 15.0,
        'forward_speed': 180,
        'rotate_speed': 180,
    },
    'aggressive': {
        'safe_distance': 40.0,
        'critical_distance': 25.0,
        'danger_distance': 12.0,
        'forward_speed': 200,
        'rotate_speed': 200,
    }
}

# Default behavior mode
DEFAULT_BEHAVIOR_MODE = 'normal'


def get_behavior_mode(mode_name: str = None):
    """
    Get behavior mode configuration
    
    Args:
        mode_name: Name of behavior mode ('cautious', 'normal', 'aggressive')
                  If None, returns default mode
    
    Returns:
        Dictionary with behavior parameters
    """
    if mode_name is None:
        mode_name = DEFAULT_BEHAVIOR_MODE
    
    return BEHAVIOR_MODES.get(mode_name, BEHAVIOR_MODES[DEFAULT_BEHAVIOR_MODE])


def apply_behavior_mode(mode_name: str):
    """
    Apply a behavior mode to global configuration
    
    Args:
        mode_name: Name of behavior mode to apply
    """
    global SAFE_DISTANCE, CRITICAL_DISTANCE, DANGER_DISTANCE
    global FORWARD_SPEED, ROTATE_SPEED
    
    mode = get_behavior_mode(mode_name)
    
    SAFE_DISTANCE = mode['safe_distance']
    CRITICAL_DISTANCE = mode['critical_distance']
    DANGER_DISTANCE = mode['danger_distance']
    FORWARD_SPEED = mode['forward_speed']
    ROTATE_SPEED = mode['rotate_speed']
    
    print(f"Applied behavior mode: {mode_name}")


# ===== PLATFORM-SPECIFIC OVERRIDES =====
# These will be auto-detected or can be manually set

def detect_platform():
    """
    Detect platform and adjust configuration accordingly
    """
    global LIDAR_PORT, ARDUINO_PORT
    
    import platform
    import os
    
    system = platform.system()
    
    if system == 'Windows':
        # Windows platform - adjust serial ports
        LIDAR_PORT = 'COM3'  # Adjust as needed
        ARDUINO_PORT = 'COM4'  # Adjust as needed
        print("Detected Windows platform")
    
    elif system == 'Linux':
        # Check if running on Raspberry Pi
        try:
            with open('/proc/cpuinfo', 'r') as f:
                if 'Raspberry Pi' in f.read():
                    print("Detected Raspberry Pi")
                else:
                    print("Detected Linux platform")
        except:
            print("Detected Linux platform")
    
    elif system == 'Darwin':
        # macOS
        LIDAR_PORT = '/dev/tty.usbserial-0001'
        ARDUINO_PORT = '/dev/tty.usbmodem14201'
        print("Detected macOS platform")


# Auto-detect platform on import
if __name__ != '__main__':
    detect_platform()
