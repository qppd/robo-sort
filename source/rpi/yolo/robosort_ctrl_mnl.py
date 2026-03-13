#!/usr/bin/env python3

import cv2
import serial
import argparse
import threading
import json
import pyrebase
import time
from typing import Optional

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("⚠ RPi.GPIO not available - GPIO button functionality disabled")

try:
    # Optional: only needed when camera streaming is enabled
    from flask import Flask, Response
except Exception:
    Flask = None
    Response = None

# Firebase Configuration
firebase_config = {
    "apiKey": "AIzaSyCqx9VuRO3F5DICYS4xDlM0fjUyL-TlGPw",
    "authDomain": "robosortcontrol.firebaseapp.com",
    "databaseURL": "https://robosortcontrol-default-rtdb.firebaseio.com",
    "storageBucket": "robosortcontrol.appspot.com"
}

class RoboSortRemoteControl:
    def __init__(
        self,
        arduino_port,
        camera_source,
        arduino_baud,
        stream_enabled: bool,
        stream_host: str,
        stream_port: int,
    ):
        self.arduino_port = arduino_port
        self.arduino_baud = arduino_baud
        self.camera_source = camera_source
        self.stream_enabled = stream_enabled
        self.stream_host = stream_host
        self.stream_port = stream_port
        self.arduino = None
        self.camera = None
        self.firebase = None
        self.db = None
        self.stream = None
        self.running = True

        # Latest camera frame shared to MJPEG stream
        self._latest_frame: Optional[any] = None
        self._frame_lock = threading.Lock()

        # Flask server
        self._flask_app = None
        self._flask_thread = None

        # Current state
        self.current_command = "STOP"
        
        self.servo_angles = {"servo1": 180, "servo2": 110, "servo3": 90, "servo4": 180, "servo5": 180}
        self.motor_state = "STOP"
        self.detected_object = "None"  # Store detected object for display
        self.first_stream_event = True  # Flag to skip auto-detect on first stream event
        
        # GPIO button configuration
        self.GPIO_BUTTON_PIN = 17
        self.button_last_press_time = 0
        self.button_debounce_delay = 0.3  # 300ms debounce
        
        # Ultrasonic monitoring
        self.ultrasonic_threshold = 22  # cm
        self.ultrasonic_check_interval = 0.5  # seconds

        # Autonomous obstacle avoidance mode
        self.autonomous_mode = False
        self._autonomous_stream = None
        self._heartbeat_thread = None
        self._heartbeat_interval = 2.0  # seconds between AUTO_HEARTBEAT commands (< 5s Arduino timeout)

        # Live sensor readings (updated by read_arduino_feedback)
        self.front_left_dist: Optional[int] = None   # cm, None = no reading yet
        self.front_right_dist: Optional[int] = None  # cm, None = no reading yet
        self.rear_dist: Optional[int] = None         # sensor 1 (rear/top), cm
        self.avoid_action = ""                       # last AVOID: log line from Arduino

        # Initialize connections
        self.init_arduino()
        self.init_camera()
        self.init_firebase()
        self.init_gpio_button()

        if self.stream_enabled:
            self.init_camera_stream_server()

    def init_arduino(self):
        """Initialize serial connection to Arduino"""
        try:
            # IMPORTANT: Arduino sketch in this repo uses Serial.begin(9600)
            # Using the wrong baud rate will make Arduino receive garbage.
            self.arduino = serial.Serial(
                self.arduino_port,
                self.arduino_baud,
                timeout=0.05,
                write_timeout=0.2,
            )

            # Opening the port resets many Arduino boards; give it a moment to boot.
            time.sleep(2.0)
            try:
                self.arduino.reset_input_buffer()
                self.arduino.reset_output_buffer()
            except Exception:
                pass

            print(f"✓ Arduino connected on {self.arduino_port} @ {self.arduino_baud} baud")
        except Exception as e:
            print(f"✗ Arduino connection failed: {e}")
            raise

    def init_camera(self):
        """Initialize camera for local viewing"""
        try:
            # Try USB camera first
            if self.camera_source.startswith('usb'):
                cam_id = int(self.camera_source.replace('usb', ''))
                self.camera = cv2.VideoCapture(cam_id)
            else:
                self.camera = cv2.VideoCapture(int(self.camera_source))

            if not self.camera.isOpened():
                raise Exception("Camera not accessible")

            # Set camera properties for lower latency
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)

            print(f"✓ Camera initialized: {self.camera_source}")
        except Exception as e:
            print(f"✗ Camera initialization failed: {e}")
            self.camera = None

    def init_camera_stream_server(self):
        """Start Flask MJPEG camera streaming server in a background thread."""
        if Flask is None or Response is None:
            print("✗ Flask is not installed. Install it with: pip install Flask")
            print("  Or disable streaming with: --no-stream")
            self.stream_enabled = False
            return

        if not self.camera:
            print("✗ Camera is not available; MJPEG stream will not start")
            self.stream_enabled = False
            return

        app = Flask(__name__)

        def gen_frames():
            """Generate MJPEG frames from the latest captured frame."""
            while self.running:
                with self._frame_lock:
                    frame = None if self._latest_frame is None else self._latest_frame.copy()

                if frame is None:
                    time.sleep(0.02)
                    continue

                ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    continue

                jpg = buffer.tobytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: "
                    + str(len(jpg)).encode("ascii")
                    + b"\r\n\r\n"
                    + jpg
                    + b"\r\n"
                )

                # Limit to 10 FPS to reduce network congestion
                time.sleep(0.1)

        @app.get("/")
        def index():
            return (
                "<html><head><title>RoboSort Camera</title></head>"
                "<body style='margin:0;background:#111;display:flex;align-items:center;justify-content:center;height:100vh'>"
                "<img src='/video' style='max-width:100%;max-height:100%' />"
                "</body></html>"
            )

        @app.get("/video")
        def video():
            return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

        def run_server():
            # use_reloader=False is critical when running inside threads
            app.run(host=self.stream_host, port=self.stream_port, threaded=True, use_reloader=False)

        self._flask_app = app
        self._flask_thread = threading.Thread(target=run_server, daemon=True)
        self._flask_thread.start()
        print(f"✓ MJPEG stream started: http://{self.stream_host}:{self.stream_port}/video")

    def init_firebase(self):
        """Initialize Firebase connection"""
        try:
            self.firebase = pyrebase.initialize_app(firebase_config)
            self.db = self.firebase.database()
            print("✓ Firebase connected")

            # Set initial state
            self.db.child("robosort").child("status").update({
                "motor_state": "STOP",
                "connected": True,
                "servo1": self.servo_angles["servo1"],
                "servo2": self.servo_angles["servo2"],
                "servo3": self.servo_angles["servo3"],
                "servo4": self.servo_angles["servo4"],
                "servo5": self.servo_angles["servo5"],
                "detected_object": self.detected_object,
            })

            # Start listening to commands
            print("Starting Firebase stream listener...")
            self.stream = self.db.child("robosort").child("commands").stream(self.on_command_change)
            print("✓ Firebase stream listener started")

            # Start listening to autonomous mode flag
            self.init_autonomous_listener()

        except Exception as e:
            print(f"✗ Firebase connection failed: {e}")
            raise

    def init_autonomous_listener(self):
        """Listen for autonomous mode flag at robot/autonomous_mode in Firebase."""
        try:
            self._autonomous_stream = self.db.child("robot").child("autonomous_mode").stream(
                self.on_autonomous_mode_change
            )
            print("✓ Autonomous mode listener started (robot/autonomous_mode)")
        except Exception as e:
            print(f"✗ Autonomous mode listener failed: {e}")

    def on_autonomous_mode_change(self, message):
        """Handle changes to robot/autonomous_mode Firebase flag (0=manual, 1=autonomous)."""
        if message["event"] != "put":
            return
        data = message.get("data")
        if data is None:
            return
        try:
            value = int(data)
        except (TypeError, ValueError):
            return
        if value == 1 and not self.autonomous_mode:
            self.autonomous_mode = True
            self.send_text_command("AUTO_ON")
            print("✓ Autonomous mode ON - sent AUTO_ON to Arduino")
            self._start_heartbeat()
        elif value == 0 and self.autonomous_mode:
            self.autonomous_mode = False
            self.send_text_command("AUTO_OFF")
            print("✓ Autonomous mode OFF - sent AUTO_OFF to Arduino")
            # heartbeat thread checks self.autonomous_mode and will stop itself

    def _start_heartbeat(self):
        """Start a background thread that keeps sending AUTO_HEARTBEAT to Arduino
        every _heartbeat_interval seconds while autonomous_mode is True."""
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return  # already running

        def _heartbeat_loop():
            print(f"✓ Heartbeat thread started (interval: {self._heartbeat_interval}s)")
            while self.running and self.autonomous_mode:
                try:
                    self.send_text_command("AUTO_HEARTBEAT")
                except Exception as e:
                    print(f"✗ Heartbeat send error: {e}")
                time.sleep(self._heartbeat_interval)
            print("✓ Heartbeat thread stopped")

        self._heartbeat_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def init_gpio_button(self):
        """Initialize GPIO button on pin 17 for alert beep"""
        if not GPIO_AVAILABLE:
            print("⚠ GPIO initialization skipped - RPi.GPIO not available")
            return
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.GPIO_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Add event detection with callback
            GPIO.add_event_detect(
                self.GPIO_BUTTON_PIN,
                GPIO.FALLING,
                callback=self.on_button_press,
                bouncetime=300
            )
            
            print(f"✓ GPIO button initialized on pin {self.GPIO_BUTTON_PIN}")
        except Exception as e:
            print(f"✗ GPIO initialization failed: {e}")

    def on_button_press(self, channel):
        """Callback when GPIO17 button is pressed - sends beep command to Android"""
        current_time = time.time()
        
        # Software debounce
        if current_time - self.button_last_press_time < self.button_debounce_delay:
            return
        
        self.button_last_press_time = current_time
        
        print("🔘 GPIO17 button pressed - sending beep alert to Android")
        
        # Send beep command to Firebase for Android to play sound
        try:
            self.db.child("robosort").child("commands").child("alert").set({
                "type": "beep",
                "timestamp": int(current_time * 1000),
                "message": "Control machine alert"
            })
            print("✓ Beep alert sent to Android")
        except Exception as e:
            print(f"✗ Failed to send beep alert: {e}")

    def on_command_change(self, message):
        """
        Firebase stream callback - handles incoming commands
        Event-driven, NO delays
        """
        print(f"Stream event received: {message['event']}")
        print(f"Stream path: {message.get('path', '')}")
        print(f"Stream data: {message.get('data', 'No data')}")
        
        is_first_event = self.first_stream_event
        self.first_stream_event = False
        
        if message["event"] == "put":
            data = message["data"]
            path = str(message.get("path", ""))
            
            # Check if data is valid (not None)
            if data is None:
                print("Data is None, skipping")
                return

            # Re-arm heartbeat on every real command while autonomous mode is active
            if self.autonomous_mode:
                self._start_heartbeat()

            # Some stream events deliver nested snapshots, e.g. {"bin": {...}}
            # when listening at /robosort/commands and only one child changed.
            if isinstance(data, dict) and "bin" in data and isinstance(data.get("bin"), dict):
                # Only unwrap if it looks like a bin payload
                inner = data.get("bin")
                if inner.get("type") == "bin" or str(inner.get("command", "")).upper().startswith("BIN_"):
                    data = inner

            # BIN fast-path (supports both full object writes and leaf updates)
            if (
                isinstance(data, dict)
                and (data.get("type") == "bin" or str(data.get("command", "")).upper().startswith("BIN_"))
            ):
                bin_cmd = str(data.get("command", "")).strip().upper()
                if bin_cmd in {"BIN_HOME", "BIN_1", "BIN_2", "BIN_3", "BIN_4"}:
                    self.send_text_command(bin_cmd)
                    print(f"✓ Sent BIN command: {bin_cmd}")
                    
                    # Automatically set detected object based on BIN position
                    if not is_first_event:
                        bin_to_object = {
                            "BIN_1": "plastic_bottle",
                            "BIN_2": "plastic_wrapper",
                            "BIN_3": "paper",
                            "BIN_4": "other"
                        }
                        if bin_cmd in bin_to_object:
                            self.detected_object = bin_to_object[bin_cmd]
                            print(f"📦 Auto-detected object for {bin_cmd}: {self.detected_object}")
                        elif bin_cmd == "BIN_HOME":
                            self.detected_object = "None"
                            print(f"🏠 BIN_HOME - Reset detected object to None")
                    
                    return
                print(f"Unknown BIN command payload: {data}")
                return

            # PLACE fast-path (check path for /place)
            if "place" in path.lower() and isinstance(data, dict) and data.get("type") == "place_sequence":
                print(f"🤖 PLACE sequence initiated (via path: {path})")
                threading.Thread(target=self.execute_place_sequence, daemon=True).start()
                return

            if isinstance(data, str) and "bin" in path.lower():
                bin_cmd = data.strip().upper()
                if bin_cmd in {"BIN_HOME", "BIN_1", "BIN_2", "BIN_3", "BIN_4"}:
                    self.send_text_command(bin_cmd)
                    print(f"✓ Sent BIN command: {bin_cmd}")
                    
                    # Automatically set detected object based on BIN position
                    if not is_first_event:
                        bin_to_object = {
                            "BIN_1": "plastic_bottle",
                            "BIN_2": "plastic_wrapper",
                            "BIN_3": "paper",
                            "BIN_4": "other"
                        }
                        if bin_cmd in bin_to_object:
                            self.detected_object = bin_to_object[bin_cmd]
                            print(f"📦 Auto-detected object for {bin_cmd}: {self.detected_object}")
                        elif bin_cmd == "BIN_HOME":
                            self.detected_object = "None"
                            print(f"🏠 BIN_HOME - Reset detected object to None")
                    
                    return

            # Firebase stream sometimes delivers a full snapshot of /commands on first connect:
            # {
            #   "motor": {direction,speed,...},
            #   "servo1": {type:"servo", servo:1, angle:...},
            #   ...
            # }
            is_snapshot = (
                isinstance(data, dict)
                and (
                    "motor" in data
                    or any(k in data for k in ("servo1", "servo2", "servo3", "servo4", "servo5"))
                )
                and not ("direction" in data and "speed" in data)
                and data.get("type") not in {"servo", "lifter"}
            )

            if is_snapshot and "motor" in data and isinstance(data.get("motor"), dict):
                motor = data.get("motor")
                if isinstance(motor, dict) and "direction" in motor and "speed" in motor:
                    self.current_command = motor.get("direction", "STOP")
                    self.send_motor_command(motor)

            if is_snapshot:
                for key in ("servo1", "servo2", "servo3", "servo4", "servo5"):
                    if key in data and isinstance(data.get(key), dict):
                        servo_cmd = data.get(key)
                        # legacy android: {type:'servo', servo:n, angle:x}
                        if servo_cmd.get("type") == "servo" and "servo" in servo_cmd and "angle" in servo_cmd:
                            self.send_servo_command(int(servo_cmd.get("servo")), {"angle": servo_cmd.get("angle")})
                        # new schema: {type:'servo', cmd:'ARM-ROTATE', angle:x}
                        elif servo_cmd.get("type") == "servo" and "cmd" in servo_cmd and "angle" in servo_cmd:
                            self.send_named_servo_command(str(servo_cmd.get("cmd")), servo_cmd.get("angle"))

                # Snapshot handled; don't fall through and re-handle keys below.
                return
            
            # Handle motor commands - data comes directly as motor command
            if "direction" in data and "speed" in data:
                command = data  # The data IS the motor command
                direction = command.get("direction", "STOP")
                print(f"Motor command found: {direction}")
                
                # Always send motor commands for real-time control
                self.current_command = direction
                self.send_motor_command(command)
                print(f"Command processed: {direction}")

            # New servo command schema: {type:'servo'|'lifter', cmd/action, angle}
            elif isinstance(data, dict) and data.get("type") == "lifter":
                action = str(data.get("action", "")).upper()
                if action in {"UP", "DOWN", "STOP"}:
                    # Safety check: Prevent LIFTER DOWN if ARM-EXTEND is below 90 degrees
                    if action == "DOWN":
                        arm_extend_angle = self.servo_angles.get("servo4", 110)  # servo4 = ARM-EXTEND
                        if arm_extend_angle < 90:
                            print(f"⚠ SAFETY BLOCK: LIFTER DOWN blocked - ARM-EXTEND is at {arm_extend_angle}° (must be ≥90°)")
                            print("  Extend arm first before lowering lifter!")
                            return  # Don't send the command
                    
                    self.send_text_command(f"LIFTER {action}")
                else:
                    print(f"Unknown lifter action: {data}")

            elif isinstance(data, dict) and data.get("type") == "detected":
                # Handle object detection command from Android
                detected_object = str(data.get("object", "")).strip()
                if detected_object in {"plastic_bottle", "plastic_wrapper", "paper", "other", "none"}:
                    if detected_object == "none":
                        print(f"🧹 CLEAR command received - resetting detected object")
                        self.detected_object = "None"
                    else:
                        print(f"📦 DETECTED command received: {detected_object}")
                        # Display on terminal/console
                        print(f"✓ Object detected: DETECTED:{detected_object}")
                        # Store for camera display
                        self.detected_object = detected_object
                    # You can add additional processing here (e.g., trigger sorting logic)
                else:
                    print(f"Unknown detected object: {data}")

            elif isinstance(data, dict) and data.get("type") == "place_sequence":
                # Handle PLACE sequence command from Android
                print(f"🤖 PLACE sequence initiated")
                threading.Thread(target=self.execute_place_sequence, daemon=True).start()

            elif isinstance(data, dict) and data.get("type") == "servo":
                # Support BOTH:
                # 1) legacy android: {type:'servo', servo:1, angle:180}
                # 2) new schema:     {type:'servo', cmd:'ARM-ROTATE', angle:180}
                if "cmd" in data and "angle" in data:
                    cmd = str(data.get("cmd", "")).upper()
                    angle = data.get("angle")
                    self.send_named_servo_command(cmd, angle)
                elif "servo" in data and "angle" in data:
                    try:
                        servo_num = int(data.get("servo"))
                    except Exception:
                        print(f"Invalid servo number: {data}")
                        return
                    self.send_servo_command(servo_num, {"angle": data.get("angle")})
                else:
                    print(f"Unknown servo payload: {data}")

            # Handle servo commands - check for servo keys
            elif "servo1" in data:
                self.send_servo_command(1, data["servo1"])
            elif "servo2" in data:
                self.send_servo_command(2, data["servo2"])
            elif "servo3" in data:
                self.send_servo_command(3, data["servo3"])
            elif "servo4" in data:
                self.send_servo_command(4, data["servo4"])
            elif "servo5" in data:
                self.send_servo_command(5, data["servo5"])
            else:
                print(f"Unknown data structure: {data}")

    def send_text_command(self, text: str):
        """Send a raw text command to Arduino (newline will be appended)."""
        try:
            cmd = text.strip() + "\n"
            print(f"DEBUG: About to send to Arduino: '{cmd.strip()}'")
            self.arduino.write(cmd.encode())
            self.arduino.flush()
            print(f"✓ Sent to Arduino: {cmd.strip()}")
        except Exception as e:
            print(f"✗ Arduino write error: {e}")

    def send_named_servo_command(self, cmd: str, angle_value):
        """Send one of the high-level servo commands the Arduino sketch supports."""
        if cmd in {"ARM-ROTATE", "ARM_EXTEND", "ARM-EXTEND", "LOOK", "GRIP", "GRIP-ROTATE", "GRIP_ROTATE"}:
            try:
                angle = int(angle_value)
            except Exception:
                print(f"Invalid angle for {cmd}: {angle_value}")
                return

            # Canonicalize command spelling
            if cmd == "ARM_EXTEND":
                cmd = "ARM-EXTEND"
            if cmd == "GRIP_ROTATE":
                cmd = "GRIP-ROTATE"

            # Clamp angles to valid ranges
            if cmd == "ARM-EXTEND":
                angle = max(90, min(180, angle))  # ARM-EXTEND range: 90-180 degrees
            else:
                angle = max(0, min(180, angle))  # Default range: 0-180 degrees

            # Update local status mirror (for Android slider sync)
            if cmd == "ARM-ROTATE":
                self.servo_angles["servo1"] = angle
                self.update_status_async("servo1", angle)
            elif cmd == "GRIP":
                self.servo_angles["servo2"] = angle
                self.update_status_async("servo2", angle)
            elif cmd == "GRIP-ROTATE":
                self.servo_angles["servo3"] = angle
                self.update_status_async("servo3", angle)
            elif cmd == "ARM-EXTEND":
                self.servo_angles["servo4"] = angle
                self.update_status_async("servo4", angle)
            elif cmd == "LOOK":
                self.servo_angles["servo5"] = angle
                self.update_status_async("servo5", angle)

            self.send_text_command(f"{cmd}:{angle}")
        else:
            print(f"Unknown servo cmd: {cmd}")

    def send_motor_command(self, command):
        """Send motor command to Arduino"""
        try:
            direction = command.get("direction", "STOP")
            speed = command.get("speed", 0)
            
            # Format command for Arduino: FORWARD:255, BACKWARD:255, etc.
            if direction == "STOP":
                arduino_cmd = "MSTOP\n"
            else:
                arduino_cmd = f"{direction}:{speed}\n"
            
            print(f"DEBUG: About to send to Arduino: '{arduino_cmd.strip()}' (length: {len(arduino_cmd)})")
            print(f"DEBUG: Arduino port: {self.arduino.port}, is_open: {self.arduino.is_open}")
            
            bytes_written = self.arduino.write(arduino_cmd.encode())
            print(f"DEBUG: Bytes written to Arduino: {bytes_written}")
            
            self.arduino.flush()  # Ensure data is sent
            print(f"DEBUG: Flushed Arduino serial buffer")
            
            # Wait a bit for Arduino to process and respond
            import time
            time.sleep(0.1)  # 100ms delay
            
            # Read Arduino response (if any)
            if self.arduino.in_waiting > 0:
                response = self.arduino.read(self.arduino.in_waiting).decode('utf-8', errors='ignore').strip()
                print(f"← Arduino response: '{response}'")
            else:
                print(f"DEBUG: No response from Arduino (in_waiting: {self.arduino.in_waiting})")
            
            self.motor_state = direction
            
            # Update Firebase status (non-blocking)
            self.update_status_async("motor_state", direction)
            print(f"✓ Sent to Arduino: {arduino_cmd.strip()}")

        except Exception as e:
            print(f"✗ Motor command error: {e}")
            import traceback
            traceback.print_exc()

    def send_servo_command(self, servo_num, command_data):
        """Send servo command to Arduino"""
        try:
            angle = command_data.get("angle", 90)
            # Clamp angle between 0-180
            angle = max(0, min(180, int(angle)))

            # Backward-compat path: map servo numbers to the Arduino high-level commands
            if servo_num == 1:
                self.send_named_servo_command("ARM-ROTATE", angle)
            elif servo_num == 2:
                self.send_named_servo_command("GRIP", angle)
            elif servo_num == 3:
                self.send_named_servo_command("GRIP-ROTATE", angle)
            elif servo_num == 4:
                self.send_named_servo_command("ARM-EXTEND", angle)
            elif servo_num == 5:
                self.send_named_servo_command("LOOK", angle)
            else:
                print(f"Unsupported servo_num: {servo_num}")

        except Exception as e:
            print(f"Servo command error: {e}")

    def update_status_async(self, key, value):
        """Update Firebase status in background thread (non-blocking)"""
        def update():
            try:
                self.db.child("robosort").child("status").update({key: value})
            except:
                pass

        threading.Thread(target=update, daemon=True).start()

    def execute_place_sequence(self):
        """
        Execute the PLACE sequence:
        1. LIFTER UP (until limit switch triggers)
        2. ARM-EXTEND:90
        3. ARM-ROTATE:0
        4. GRIP:110 (release)
        5. ARM-ROTATE:180
        6. ARM-EXTEND:180
        7. LOOK:180
        8. LIFTER DOWN (40 rotations)
        """
        try:
            print("🤖 PLACE Step 1: LIFTER UP (waiting for limit switch)")
            self.send_text_command("LIFTER UP")
            
            # Wait for ARM limit switch to trigger (monitor Arduino feedback)
            # For now, use a timeout approach (adjust timing as needed)
            time.sleep(3.0)  # Wait for lifter to reach top
            
            print("🤖 PLACE Step 2: ARM-EXTEND:90")
            self.send_named_servo_command("ARM-EXTEND", 90)
            time.sleep(1.0)
            
            print("🤖 PLACE Step 3: ARM-ROTATE:0")
            self.send_named_servo_command("ARM-ROTATE", 0)
            time.sleep(1.5)
            
            print("🤖 PLACE Step 4: GRIP:110 (release)")
            self.send_named_servo_command("GRIP", 110)
            time.sleep(1.0)
            
            print("🤖 PLACE Step 5: ARM-ROTATE:180")
            self.send_named_servo_command("ARM-ROTATE", 180)
            time.sleep(1.5)
            
            print("🤖 PLACE Step 6: ARM-EXTEND:180")
            self.send_named_servo_command("ARM-EXTEND", 180)
            time.sleep(1.0)
            
            print("🤖 PLACE Step 7: LOOK:180")
            self.send_named_servo_command("LOOK", 180)
            time.sleep(1.0)
            
            print("🤖 PLACE Step 8: LIFTER DOWN (40 rotations)")
            self.send_text_command("LIFTER DOWN")
            time.sleep(60.0)  # 40 rotations @ ~1.5s per rotation = ~60 seconds
            
            print("✅ PLACE sequence complete!")
            
        except Exception as e:
            print(f"❌ PLACE sequence error: {e}")
            import traceback
            traceback.print_exc()

    def monitor_ultrasonic(self):
        """
        Continuously monitor ultrasonic sensor and control Arduino buzzer
        <22cm: buzzer ON, >=22cm: buzzer OFF
        Runs in separate thread - non-blocking
        """
        print("✓ Ultrasonic monitoring started")
        
        while self.running:
            try:
                # Request distance from Arduino ultrasonic sensor 1
                self.arduino.write(b"UDIST 1\n")
                time.sleep(0.05)  # Give Arduino time to respond
                
                # Read response
                if self.arduino.in_waiting > 0:
                    response = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Parse distance from response like "Ultrasonic 1 Distance: 15 cm"
                    if "Distance:" in response:
                        parts = response.split()
                        try:
                            # Find the distance value (number before "cm")
                            for i, part in enumerate(parts):
                                if part.isdigit() and i + 1 < len(parts) and parts[i + 1] == "cm":
                                    distance = int(part)
                                    
                                    # Control buzzer based on threshold
                                    if distance < self.ultrasonic_threshold:
                                        # Object detected close - sound buzzer
                                        self.arduino.write(b"BWARNING\n")
                                        print(f"⚠ Obstacle detected: {distance}cm - Buzzer ON")
                                    break
                        except (ValueError, IndexError):
                            pass
                
                # Wait before next check
                time.sleep(self.ultrasonic_check_interval)
                
            except Exception as e:
                if self.running:
                    print(f"Ultrasonic monitoring error: {e}")
                    time.sleep(1)  # Back off on error
    
    def read_arduino_feedback(self):
        """
        Continuously read feedback from Arduino.
        Also parses AVOID: log lines and distance reports to keep
        self.front_left_dist / front_right_dist / rear_dist up to date.
        Runs in separate thread - non-blocking.
        """
        import re
        while self.running:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        print(f"\u2190 Arduino response: '{line}'")

                        # --- Structured feedback: "S:1:90" or "M:FORWARD" ---
                        parts = line.split(':')
                        if parts[0] == 'S' and len(parts) == 3:
                            servo_id = parts[1]
                            angle = parts[2]
                            self.update_status_async(f"servo{servo_id}", int(angle))
                        elif parts[0] == 'M' and len(parts) == 2:
                            self.update_status_async("motor_state", parts[1])

                        # --- AVOID state-machine log lines ---
                        # e.g. "AVOID: collision L=18 R=25 -> BACKWARD:255"
                        if line.startswith("AVOID:"):
                            self.avoid_action = line[6:].strip()
                            # extract L= and R= values if present
                            m = re.search(r'L=(\d+)', line)
                            if m:
                                self.front_left_dist = int(m.group(1))
                            m = re.search(r'R=(\d+)', line)
                            if m:
                                self.front_right_dist = int(m.group(1))

                        # --- Front distance queries: "FL_DIST"/"FR_DIST" replies ---
                        # "Distance from Front Left Sensor: 32 cm"
                        if 'Front Left' in line or 'front left' in line:
                            m = re.search(r'(\d+)\s*cm', line)
                            if m:
                                self.front_left_dist = int(m.group(1))
                        if 'Front Right' in line or 'front right' in line:
                            m = re.search(r'(\d+)\s*cm', line)
                            if m:
                                self.front_right_dist = int(m.group(1))

                        # --- Generic ultrasonic report: "Ultrasonic 1 Distance: 45 cm" ---
                        if 'Ultrasonic 1' in line and 'Distance:' in line:
                            m = re.search(r'(\d+)\s*cm', line)
                            if m:
                                self.rear_dist = int(m.group(1))

            except Exception as e:
                if self.running:
                    print(f"Arduino read error: {e}")

    # ------------------------------------------------------------------
    # Camera display helpers
    # ------------------------------------------------------------------
    _MOVEMENT_LABEL = {
        "FORWARD":    ("\u2191 FORWARD",    (0, 220, 0)),
        "BACKWARD":   ("\u2193 BACKWARD",   (0, 100, 255)),
        "LEFT":       ("\u21b0 ARC LEFT",   (0, 220, 220)),
        "RIGHT":      ("\u21b1 ARC RIGHT",  (0, 180, 255)),
        "TURN_LEFT":  ("\u21ba TURN LEFT",  (255, 180, 0)),
        "TURN_RIGHT": ("\u21bb TURN RIGHT", (255, 100, 0)),
        "STOP":       ("\u25a0 STOP",        (0, 0, 220)),
    }

    @staticmethod
    def _dist_color(dist, warn_cm=35, crit_cm=20):
        """Return BGR colour for a sensor distance reading."""
        if dist is None:
            return (160, 160, 160)   # grey  - no data
        if dist <= crit_cm:
            return (0, 0, 255)       # red   - collision zone
        if dist <= warn_cm:
            return (0, 165, 255)     # orange - warning zone
        return (0, 220, 0)           # green - clear

    def display_camera(self):
        """
        Display camera feed locally on Raspberry Pi.
        Overlay: movement arrow, front ultrasonic bars, rear distance,
                 autonomous mode indicator, servo positions, detected object.
        """
        if not self.camera:
            return

        font       = cv2.FONT_HERSHEY_SIMPLEX
        font_sm    = 0.55
        font_md    = 0.70
        thick_sm   = 1
        thick_md   = 2

        while self.running:
            ret, frame = self.camera.read()
            if not ret:
                continue

            h, w = frame.shape[:2]

            # ── Crosshair ──────────────────────────────────────────────
            cx, cy = w // 2, h // 2 + 100
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

            # Share frame with MJPEG stream before text overlays
            # (stream gets clean crosshair frame; text overlays on local copy)
            with self._frame_lock:
                self._latest_frame = frame.copy()

            # ── Semi-transparent dark sidebar (left) ───────────────────
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (260, h), (20, 20, 20), -1)
            cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

            # ── Movement label ─────────────────────────────────────────
            mv   = self.motor_state.upper() if self.motor_state else "STOP"
            lbl, lbl_color = self._MOVEMENT_LABEL.get(mv, (mv, (200, 200, 200)))
            cv2.putText(frame, lbl, (8, 30), font, font_md, lbl_color, thick_md)

            # ── Autonomous mode badge ──────────────────────────────────
            if self.autonomous_mode:
                cv2.putText(frame, "AUTO", (8, 56), font, font_md, (0, 255, 180), thick_md)
            else:
                cv2.putText(frame, "MANUAL", (8, 56), font, font_sm, (160, 160, 160), thick_sm)

            # ── Front ultrasonic bars ──────────────────────────────────
            bar_y    = 75
            bar_max  = 80     # pixel width for 100 cm
            max_cm   = 100.0

            # Left sensor
            dL  = self.front_left_dist
            cL  = self._dist_color(dL)
            lL  = f"FL: {'---' if dL is None else f'{dL:3d} cm'}"
            bwL = 0 if dL is None else int(min(dL, max_cm) / max_cm * bar_max)
            cv2.rectangle(frame, (8, bar_y + 4), (8 + bar_max, bar_y + 16), (60, 60, 60), -1)
            if bwL > 0:
                cv2.rectangle(frame, (8, bar_y + 4), (8 + bwL, bar_y + 16), cL, -1)
            cv2.putText(frame, lL, (8, bar_y), font, font_sm, cL, thick_sm)

            # Right sensor
            bar_y2 = bar_y + 30
            dR  = self.front_right_dist
            cR  = self._dist_color(dR)
            lR  = f"FR: {'---' if dR is None else f'{dR:3d} cm'}"
            bwR = 0 if dR is None else int(min(dR, max_cm) / max_cm * bar_max)
            cv2.rectangle(frame, (8, bar_y2 + 4), (8 + bar_max, bar_y2 + 16), (60, 60, 60), -1)
            if bwR > 0:
                cv2.rectangle(frame, (8, bar_y2 + 4), (8 + bwR, bar_y2 + 16), cR, -1)
            cv2.putText(frame, lR, (8, bar_y2), font, font_sm, cR, thick_sm)

            # Rear / top sensor
            bar_y3 = bar_y2 + 30
            dS  = self.rear_dist
            cS  = self._dist_color(dS, warn_cm=22, crit_cm=10)
            lS  = f"S1: {'---' if dS is None else f'{dS:3d} cm'}"
            cv2.putText(frame, lS, (8, bar_y3), font, font_sm, cS, thick_sm)

            # Obstacle status summary
            bar_y4 = bar_y3 + 22
            if dL is not None and dR is not None:
                if dL <= 20 or dR <= 20:
                    obs_txt, obs_col = "!! COLLISION !!", (0, 0, 255)
                elif dL <= 35 or dR <= 35:
                    obs_txt, obs_col = "WARN: close",   (0, 165, 255)
                else:
                    obs_txt, obs_col = "PATH CLEAR",    (0, 220, 0)
            else:
                obs_txt, obs_col = "sensors init...", (160, 160, 160)
            cv2.putText(frame, obs_txt, (8, bar_y4), font, font_sm, obs_col, thick_sm)

            # Avoid action (last line from Arduino AVOID: log)
            if self.autonomous_mode and self.avoid_action:
                short = self.avoid_action[:30]
                cv2.putText(frame, short, (8, bar_y4 + 20), font, 0.42, (200, 200, 100), 1)

            # ── Servos & detected object ───────────────────────────────
            sv_y = bar_y4 + 50
            cv2.putText(frame,
                f"S1:{self.servo_angles['servo1']} S2:{self.servo_angles['servo2']}",
                (8, sv_y), font, font_sm, (180, 220, 180), thick_sm)
            cv2.putText(frame,
                f"S3:{self.servo_angles['servo3']} S4:{self.servo_angles['servo4']}",
                (8, sv_y + 20), font, font_sm, (180, 220, 180), thick_sm)
            cv2.putText(frame,
                f"Obj: {self.detected_object}",
                (8, sv_y + 40), font, font_sm, (220, 220, 100), thick_sm)

            cv2.imshow("RoboSort RC View", frame)

            # Non-blocking key check
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False

    def run(self):
        """Start all threads and run the control system"""
        print("\n=== RoboSort Remote Control Started ===")
        print("Press 'q' in camera window to quit\n")

        # Start Arduino feedback thread
        arduino_thread = threading.Thread(target=self.read_arduino_feedback, daemon=True)
        arduino_thread.start()
        
        # Start ultrasonic monitoring thread
        # ultrasonic_thread = threading.Thread(target=self.monitor_ultrasonic, daemon=True)
        # ultrasonic_thread.start()

        # Start camera display thread (blocking on main thread for cv2)
        self.display_camera()

        # Cleanup
        self.cleanup()

    def cleanup(self):
        """Clean shutdown"""
        print("\nShutting down...")
        self.running = False

        # Close Firebase stream
        if self.stream:
            try:
                self.stream.close()
            except:
                pass

        # Stop motors
        if self.arduino:
            try:
                self.arduino.write(b"M:S\n")
            except:
                pass

        # Update Firebase
        try:
            self.db.child("robosort").child("status").update({
                "motor_state": "STOP",
                "connected": False
            })
        except:
            pass

        # Close connections
        if self.arduino:
            self.arduino.close()
        if self.camera:
            self.camera.release()
        
        # Cleanup GPIO
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
                print("✓ GPIO cleaned up")
            except:
                pass

        cv2.destroyAllWindows()
        print("✓ Shutdown complete")


def main():
    parser = argparse.ArgumentParser(description='RoboSort Real-Time Remote Control')
    parser.add_argument('--source', default='usb0', help='Camera source (usb0, usb1, or device id)')
    parser.add_argument('--arduino', default='/dev/ttyUSB1', help='Arduino serial port')
    parser.add_argument('--baud', type=int, default=9600, help='Arduino serial baud rate (default: 9600)')

    stream_group = parser.add_mutually_exclusive_group()
    stream_group.add_argument('--stream', dest='stream', action='store_true', help='Enable MJPEG camera stream (default)')
    stream_group.add_argument('--no-stream', dest='stream', action='store_false', help='Disable MJPEG camera stream')
    parser.set_defaults(stream=True)

    parser.add_argument('--stream-host', default='0.0.0.0', help='MJPEG stream bind host (default: 0.0.0.0)')
    parser.add_argument('--stream-port', type=int, default=5000, help='MJPEG stream port (default: 5000)')

    args = parser.parse_args()

    try:
        controller = RoboSortRemoteControl(
            arduino_port=args.arduino,
            camera_source=args.source,
            arduino_baud=args.baud,
            stream_enabled=args.stream,
            stream_host=args.stream_host,
            stream_port=args.stream_port,
        )
        controller.run()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")


if __name__ == "__main__":
    main()
