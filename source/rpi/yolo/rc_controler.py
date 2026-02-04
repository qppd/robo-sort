#!/usr/bin/env python3
"""
Real-Time Remote Control System for RoboSort
Controls DC motors and servo motors via Firebase Realtime Database
No artificial delays - pure event-driven architecture
"""

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
        # Match Arduino SERVO_CONFIG defaults (channels 1-5):
        # 1: ARM-ROTATE=180, 2: GRIP=110, 3: GRIP-ROTATE=90, 4: ARM-EXTEND=180, 5: LOOK=180
        self.servo_angles = {"servo1": 180, "servo2": 110, "servo3": 90, "servo4": 180, "servo5": 180}
        self.motor_state = "STOP"
        self.detected_object = "None"  # Store detected object for display
        
        # GPIO button configuration
        self.GPIO_BUTTON_PIN = 17
        self.button_last_press_time = 0
        self.button_debounce_delay = 0.3  # 300ms debounce

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
            self.db.child("robosortv2").child("status").update({
                "motor_state": "STOP",
                "connected": True,
                "servo1": self.servo_angles["servo1"],
                "servo2": self.servo_angles["servo2"],
                "servo3": self.servo_angles["servo3"],
                "servo4": self.servo_angles["servo4"],
                "servo5": self.servo_angles["servo5"],
            })

            # Start listening to commands
            print("Starting Firebase stream listener...")
            self.stream = self.db.child("robosortv2").child("commands").stream(self.on_command_change)
            print("✓ Firebase stream listener started")

        except Exception as e:
            print(f"✗ Firebase connection failed: {e}")
            raise

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
            self.db.child("robosortv2").child("commands").child("alert").set({
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
        
        if message["event"] == "put":
            data = message["data"]
            path = str(message.get("path", ""))
            
            # Check if data is valid (not None)
            if data is None:
                print("Data is None, skipping")
                return

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
                self.db.child("robosortv2").child("status").update({key: value})
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

    def read_arduino_feedback(self):
        """
        Continuously read feedback from Arduino
        Runs in separate thread - non-blocking
        """
        while self.running:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8').strip()

                    if line:
                        print(f"← Arduino response: '{line}'")
                        
                        # Parse feedback: format "S:1:90" or "M:FORWARD"
                        parts = line.split(':')

                        if parts[0] == 'S' and len(parts) == 3:
                            # Servo feedback
                            servo_id = parts[1]
                            angle = parts[2]
                            self.update_status_async(f"servo{servo_id}", int(angle))

                        elif parts[0] == 'M' and len(parts) == 2:
                            # Motor feedback
                            state = parts[1]
                            self.update_status_async("motor_state", state)

            except Exception as e:
                if self.running:
                    print(f"Arduino read error: {e}")

    def display_camera(self):
        """
        Display camera feed locally on Raspberry Pi
        Runs in separate thread - non-blocking
        """
        if not self.camera:
            return

        while self.running:
            ret, frame = self.camera.read()

            if ret:
                # Define crosshair position (center of frame)
                crosshair_x = frame.shape[1] // 2
                crosshair_y = (frame.shape[0] // 2) + 100  # Move crosshair down by 50 pixels

                # Draw crosshair
                cv2.line(frame, (crosshair_x - 20, crosshair_y), (crosshair_x + 20, crosshair_y), (0, 255, 0), 2)
                cv2.line(frame, (crosshair_x, crosshair_y - 20), (crosshair_x, crosshair_y + 20), (0, 255, 0), 2)

                # Share latest frame to stream (after crosshair so stream matches what you see)
                with self._frame_lock:
                    self._latest_frame = frame

                # Add status overlay
                cv2.putText(frame, f"Motor: {self.motor_state}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(frame, f"S1:{self.servo_angles['servo1']} S2:{self.servo_angles['servo2']}",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.putText(frame, f"S3:{self.servo_angles['servo3']} S4:{self.servo_angles['servo4']}",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Display detected object
                cv2.putText(frame, f"Detected: {self.detected_object}", (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

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
            self.db.child("robosortv2").child("status").update({
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
