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

# Firebase Configuration
firebase_config = {
    "apiKey": "AIzaSyCqx9VuRO3F5DICYS4xDlM0fjUyL-TlGPw",
    "authDomain": "robosortcontrol.firebaseapp.com",
    "databaseURL": "https://robosortcontrol-default-rtdb.firebaseio.com",
    "storageBucket": "robosortcontrol.appspot.com"
}

class RoboSortRemoteControl:
    def __init__(self, arduino_port, camera_source):
        self.arduino_port = arduino_port
        self.camera_source = camera_source
        self.arduino = None
        self.camera = None
        self.firebase = None
        self.db = None
        self.stream = None
        self.running = True

        # Current state
        self.current_command = "STOP"
        self.servo_angles = {"servo1": 90, "servo2": 90, "servo3": 90, "servo4": 90}
        self.motor_state = "STOP"

        # Initialize connections
        self.init_arduino()
        self.init_camera()
        self.init_firebase()

    def init_arduino(self):
        """Initialize serial connection to Arduino"""
        try:
            self.arduino = serial.Serial(self.arduino_port, 115200, timeout=0.01)
            print(f"✓ Arduino connected on {self.arduino_port}")
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

    def init_firebase(self):
        """Initialize Firebase connection"""
        try:
            self.firebase = pyrebase.initialize_app(firebase_config)
            self.db = self.firebase.database()
            print("✓ Firebase connected")

            # Set initial state
            self.db.child("robosort").child("status").update({
                "motor_state": "STOP",
                "connected": True
            })

            # Start listening to commands
            print("Starting Firebase stream listener...")
            self.stream = self.db.child("robosort").child("commands").stream(self.on_command_change)
            print("✓ Firebase stream listener started")

        except Exception as e:
            print(f"✗ Firebase connection failed: {e}")
            raise

    def on_command_change(self, message):
        """
        Firebase stream callback - handles incoming commands
        Event-driven, NO delays
        """
        print(f"Stream event received: {message['event']}")
        print(f"Stream data: {message.get('data', 'No data')}")
        
        if message["event"] == "put":
            data = message["data"]
            
            # Check if data is valid (not None)
            if data is None:
                print("Data is None, skipping")
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

            # Handle servo commands - check for servo keys
            elif "servo1" in data:
                self.send_servo_command(1, data["servo1"])
            elif "servo2" in data:
                self.send_servo_command(2, data["servo2"])
            elif "servo3" in data:
                self.send_servo_command(3, data["servo3"])
            elif "servo4" in data:
                self.send_servo_command(4, data["servo4"])
            else:
                print(f"Unknown data structure: {data}")

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
            
            self.motor_state = direction
            
            # Update Firebase status (non-blocking)
            self.update_status_async("motor_state", direction)
            print(f"Sent to Arduino: {arduino_cmd.strip()}")

        except Exception as e:
            print(f"Motor command error: {e}")
            import traceback
            traceback.print_exc()

    def send_servo_command(self, servo_num, command_data):
        """Send servo command to Arduino"""
        try:
            angle = command_data.get("angle", 90)
            # Clamp angle between 0-180
            angle = max(0, min(180, int(angle)))

            # Format: S<servo> <angle> (e.g., S1 90)
            arduino_cmd = f"S{servo_num} {angle}\n"
            self.arduino.write(arduino_cmd.encode())

            # Update local state
            self.servo_angles[f"servo{servo_num}"] = angle

            # Update Firebase status (non-blocking)
            self.update_status_async(f"servo{servo_num}", angle)
            print(f"Sent to Arduino: {arduino_cmd.strip()}")

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
                # Add status overlay
                cv2.putText(frame, f"Motor: {self.motor_state}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(frame, f"S1:{self.servo_angles['servo1']} S2:{self.servo_angles['servo2']}",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.putText(frame, f"S3:{self.servo_angles['servo3']} S4:{self.servo_angles['servo4']}",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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

        cv2.destroyAllWindows()
        print("✓ Shutdown complete")


def main():
    parser = argparse.ArgumentParser(description='RoboSort Real-Time Remote Control')
    parser.add_argument('--source', default='usb0', help='Camera source (usb0, usb1, or device id)')
    parser.add_argument('--arduino', default='/dev/ttyUSB1', help='Arduino serial port')

    args = parser.parse_args()

    try:
        controller = RoboSortRemoteControl(
            arduino_port=args.arduino,
            camera_source=args.source
        )
        controller.run()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")


if __name__ == "__main__":
    main()
