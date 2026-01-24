"""
Test Script for RoboSort Object Pickup System
Tests YOLO object detection integrated with Arduino arm control
"""

import sys
import os
import time
import argparse
import cv2
import numpy as np
from ultralytics import YOLO

# Add paths for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import Arduino controller from navigate module
from navigate import ArduinoController


class ObjectPickupTester:
    """
    Test object detection and pickup functionality
    """
    
    def __init__(self, model_path: str, arduino_port: str = '/dev/ttyACM0', arduino_baudrate: int = 9600):
        """
        Initialize object pickup tester
        
        Args:
            model_path: Path to YOLO model file
            arduino_port: Arduino serial port
            arduino_baudrate: Arduino baudrate
        """
        # Load YOLO model
        print(f"Loading YOLO model from {model_path}...")
        self.model = YOLO(model_path, task='detect')
        self.labels = self.model.names
        print(f"✓ Model loaded with {len(self.labels)} classes")
        
        # Initialize Arduino controller
        print(f"Connecting to Arduino on {arduino_port}...")
        self.arduino = ArduinoController(arduino_port, arduino_baudrate)
        if not self.arduino.connect():
            raise Exception("Failed to connect to Arduino")
        
        # Pickup parameters
        self.detection_threshold = 0.5
        self.pickup_in_progress = False
        
    def arm_rotate(self, angle: int):
        """Rotate arm to specified angle (0-180)"""
        self.arduino.send_command(f"ARM-ROTATE:{angle}")
        print(f"  → ARM-ROTATE to {angle}°")
        
    def lifter_up(self):
        """Move lifter up"""
        self.arduino.send_command("LIFTER:UP")
        print("  → LIFTER UP")
        
    def lifter_down(self):
        """Move lifter down"""
        self.arduino.send_command("LIFTER:DOWN")
        print("  → LIFTER DOWN")
        
    def arm_extend(self):
        """Extend arm forward"""
        self.arduino.send_command("ARM-EXTEND")
        print("  → ARM-EXTEND")
        
    def arm_retract(self):
        """Retract arm (return to home)"""
        self.arduino.send_command("ARM-RETRACT")
        print("  → ARM-RETRACT")
        
    def look(self):
        """Move arm to look/scan position"""
        self.arduino.send_command("LOOK")
        print("  → LOOK")
        
    def grip_rotate(self, angle: int):
        """Rotate gripper to specified angle (0-180)"""
        self.arduino.send_command(f"GRIP-ROTATE:{angle}")
        print(f"  → GRIP-ROTATE to {angle}°")
        
    def grip_open(self):
        """Open gripper"""
        self.arduino.send_command("GRIP:OPEN")
        print("  → GRIP OPEN")
        
    def grip_close(self):
        """Close gripper"""
        self.arduino.send_command("GRIP:CLOSE")
        print("  → GRIP CLOSE")
        
    def reset_arm(self):
        """Reset arm to home position"""
        print("\n[RESET] Resetting arm to home position...")
        self.grip_open()
        time.sleep(0.5)
        self.arm_retract()
        time.sleep(1.0)
        self.lifter_down()
        time.sleep(0.8)
        self.arm_rotate(90)  # Center position
        time.sleep(0.5)
        self.grip_rotate(90)  # Center gripper
        time.sleep(0.5)
        print("✓ Arm reset complete\n")
        
    def pickup_sequence(self, object_name: str, x_center: int, frame_width: int):
        """
        Execute pickup sequence for detected object
        
        Args:
            object_name: Name of detected object
            x_center: X coordinate of object center
            frame_width: Width of camera frame
        """
        if self.pickup_in_progress:
            return
            
        self.pickup_in_progress = True
        print(f"\n{'='*60}")
        print(f"[PICKUP] Starting pickup sequence for: {object_name}")
        print(f"{'='*60}")
        
        try:
            # Calculate rotation angle based on object position
            # Center of frame = 90°, left edge = 0°, right edge = 180°
            rotation_angle = int((x_center / frame_width) * 180)
            rotation_angle = max(30, min(150, rotation_angle))  # Limit to safe range
            
            print(f"[PICKUP] Step 1: Position arm (object at x={x_center}, angle={rotation_angle}°)")
            self.arm_rotate(rotation_angle)
            time.sleep(1.5)
            
            print("[PICKUP] Step 2: Look at object")
            self.look()
            time.sleep(1.0)
            
            print("[PICKUP] Step 3: Open gripper")
            self.grip_open()
            time.sleep(0.8)
            
            print("[PICKUP] Step 4: Extend arm to object")
            self.arm_extend()
            time.sleep(1.5)
            
            print("[PICKUP] Step 5: Lower lifter")
            self.lifter_down()
            time.sleep(1.0)
            
            print("[PICKUP] Step 6: Close gripper to grab object")
            self.grip_close()
            time.sleep(1.0)
            
            print("[PICKUP] Step 7: Lift object")
            self.lifter_up()
            time.sleep(1.0)
            
            print("[PICKUP] Step 8: Retract arm")
            self.arm_retract()
            time.sleep(1.5)
            
            print(f"✓ Pickup sequence complete for {object_name}!")
            print(f"{'='*60}\n")
            
            # Wait before next pickup
            time.sleep(2.0)
            
        except Exception as e:
            print(f"✗ Error during pickup sequence: {e}")
        finally:
            self.pickup_in_progress = False
            
    def test_camera_detection(self, camera_source: str = 'usb0'):
        """
        Test object detection with camera and pickup sequences
        
        Args:
            camera_source: Camera source (usb0, usb1, picamera0, etc.)
        """
        print("\n" + "="*60)
        print("OBJECT DETECTION AND PICKUP TEST")
        print("="*60)
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Pause/Resume")
        print("  'p' - Capture screenshot")
        print("  'r' - Reset arm to home position")
        print("  SPACE - Trigger pickup on detected object")
        print("="*60 + "\n")
        
        # Initialize camera
        if 'usb' in camera_source:
            cam_idx = int(camera_source.replace('usb', ''))
            cap = cv2.VideoCapture(cam_idx)
        elif 'picamera' in camera_source:
            from picamera2 import Picamera2
            cap = Picamera2()
            cap.configure(cap.create_video_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
            cap.start()
            use_picamera = True
        else:
            print(f"Invalid camera source: {camera_source}")
            return
            
        # Reset arm to home position
        self.reset_arm()
        
        # Detection loop
        use_picamera = 'picamera' in camera_source
        frame_count = 0
        fps_buffer = []
        
        try:
            while True:
                t_start = time.perf_counter()
                
                # Capture frame
                if use_picamera:
                    frame_bgra = cap.capture_array()
                    frame = cv2.cvtColor(np.copy(frame_bgra), cv2.COLOR_BGRA2BGR)
                else:
                    ret, frame = cap.read()
                    if not ret:
                        print("Failed to capture frame")
                        break
                
                frame_height, frame_width = frame.shape[:2]
                
                # Run YOLO detection
                results = self.model(frame, verbose=False)
                detections = results[0].boxes
                
                # Track detected objects
                detected_objects = []
                
                # Process detections
                for i in range(len(detections)):
                    # Get bounding box
                    xyxy = detections[i].xyxy.cpu().numpy().squeeze()
                    xmin, ymin, xmax, ymax = xyxy.astype(int)
                    
                    # Get class and confidence
                    classidx = int(detections[i].cls.item())
                    classname = self.labels[classidx]
                    conf = detections[i].conf.item()
                    
                    # Draw if confidence is high enough
                    if conf > self.detection_threshold:
                        # Calculate center
                        x_center = (xmin + xmax) // 2
                        y_center = (ymin + ymax) // 2
                        
                        detected_objects.append({
                            'name': classname,
                            'conf': conf,
                            'x': x_center,
                            'y': y_center,
                            'bbox': (xmin, ymin, xmax, ymax)
                        })
                        
                        # Draw bounding box
                        color = (0, 255, 0) if not self.pickup_in_progress else (0, 165, 255)
                        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                        
                        # Draw center point
                        cv2.circle(frame, (x_center, y_center), 5, (0, 0, 255), -1)
                        
                        # Draw label
                        label = f'{classname}: {int(conf*100)}%'
                        cv2.putText(frame, label, (xmin, ymin-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Display info
                fps = np.mean(fps_buffer) if fps_buffer else 0
                cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f'Objects: {len(detected_objects)}', (10, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                if self.pickup_in_progress:
                    cv2.putText(frame, 'PICKUP IN PROGRESS', (10, 90), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                
                # Show frame
                cv2.imshow('Object Detection and Pickup Test', frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("\nExiting...")
                    break
                elif key == ord('s') or key == ord('S'):
                    print("Paused. Press any key to resume...")
                    cv2.waitKey()
                elif key == ord('p') or key == ord('P'):
                    filename = f'pickup_test_{int(time.time())}.png'
                    cv2.imwrite(filename, frame)
                    print(f"Screenshot saved: {filename}")
                elif key == ord('r') or key == ord('R'):
                    self.reset_arm()
                elif key == 32:  # SPACE key
                    if detected_objects and not self.pickup_in_progress:
                        # Pick up the first detected object
                        obj = detected_objects[0]
                        self.pickup_sequence(obj['name'], obj['x'], frame_width)
                
                # Calculate FPS
                t_stop = time.perf_counter()
                frame_rate = 1.0 / (t_stop - t_start)
                fps_buffer.append(frame_rate)
                if len(fps_buffer) > 30:
                    fps_buffer.pop(0)
                
                frame_count += 1
                
        except KeyboardInterrupt:
            print("\n\n⚠ Interrupted by user")
        finally:
            # Cleanup
            print("\nCleaning up...")
            if use_picamera:
                cap.stop()
            else:
                cap.release()
            cv2.destroyAllWindows()
            self.arduino.disconnect()
            print("✓ Test complete")


def main():
    """Main test interface"""
    parser = argparse.ArgumentParser(
        description='RoboSort Object Pickup Test Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test_object_pickup.py --model my_model.pt --camera usb0
  python test_object_pickup.py --model best.pt --camera usb1 --arduino /dev/ttyACM0
  python test_object_pickup.py --model my_model.pt --camera picamera0
        """
    )
    
    parser.add_argument('--model', type=str, required=True,
                       help='Path to YOLO model file (e.g., "my_model.pt")')
    parser.add_argument('--camera', type=str, default='usb0',
                       help='Camera source: usb0, usb1, picamera0, etc. (default: usb0)')
    parser.add_argument('--arduino', type=str, default='/dev/ttyACM0',
                       help='Arduino serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baudrate', type=int, default=9600,
                       help='Arduino baudrate (default: 9600)')
    parser.add_argument('--threshold', type=float, default=0.5,
                       help='Detection confidence threshold (default: 0.5)')
    
    args = parser.parse_args()
    
    try:
        # Initialize tester
        tester = ObjectPickupTester(
            model_path=args.model,
            arduino_port=args.arduino,
            arduino_baudrate=args.baudrate
        )
        
        tester.detection_threshold = args.threshold
        
        # Run camera detection test
        tester.test_camera_detection(camera_source=args.camera)
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
