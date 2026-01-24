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

# Try to import YOLO libraries
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

try:
    import ncnn
    NCNN_AVAILABLE = True
except ImportError:
    NCNN_AVAILABLE = False

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
            model_path: Path to model file (.pt for YOLO, .param for NCNN)
            arduino_port: Arduino serial port
            arduino_baudrate: Arduino baudrate
        """
        # Detect model type
        self.model_type = self._detect_model_type(model_path)
        self.model_path = model_path
        
        # Load model based on type
        if self.model_type == 'yolo':
            if not ULTRALYTICS_AVAILABLE:
                raise Exception("Ultralytics YOLO not available. Install with: pip install ultralytics")
            print(f"Loading YOLO model from {model_path}...")
            self.model = YOLO(model_path, task='detect')
            self.labels = self.model.names
            print(f"✓ YOLO model loaded with {len(self.labels)} classes")
            
        elif self.model_type == 'ncnn':
            if not NCNN_AVAILABLE:
                raise Exception("NCNN not available. Install with: pip install ncnn")
            print(f"Loading NCNN model from {model_path}...")
            # Initialize default labels (will be updated by _load_ncnn_model if metadata available)
            self.labels = {0: 'other', 1: 'paper', 2: 'plastic_bottle', 3: 'plastic_wrapper'}
            self._load_ncnn_model()
            print(f"✓ NCNN model loaded with {len(self.labels)} classes")
            
        else:
            raise Exception(f"Unsupported model type: {self.model_type}")
        
        # Initialize Arduino controller
        print(f"Connecting to Arduino on {arduino_port}...")
        self.arduino = ArduinoController(arduino_port, arduino_baudrate)
        if not self.arduino.connect():
            raise Exception("Failed to connect to Arduino")
        
        # Pickup parameters
        self.detection_threshold = 0.5
        self.pickup_in_progress = False
        
    def _run_ncnn_inference(self, image):
        """
        Run inference using NCNN model
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            List of detections in format similar to Ultralytics
        """
        # Preprocess image
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.input_size)
        img = img.astype(np.float32)
        
        # Normalize
        img = (img - self.mean_vals) * self.norm_vals
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        
        # Create NCNN mat
        mat_in = ncnn.Mat(img)
        
        # Run inference
        with self.ncnn_net.create_extractor() as ex:
            ex.input("in0", mat_in)
            _, mat_out = ex.extract("out0")
            
            # Convert to numpy
            output = np.array(mat_out)
            
        # Parse YOLOv8 output format (simplified)
        # This is a basic implementation - you may need to adjust based on your model's output format
        detections = []
        
        # YOLOv8 output is [batch, 84, 8400] for COCO dataset
        # We need to parse this into bounding boxes
        if len(output.shape) == 3:
            output = output[0]  # Remove batch dimension
            
            # Each detection is 84 values: 4 bbox + 80 classes
            num_classes = len(self.labels)
            num_detections = output.shape[1] if len(output.shape) > 1 else 0
            
            for i in range(min(num_detections, 100)):  # Limit to prevent too many detections
                detection = output[:, i] if len(output.shape) > 1 else output
                
                if len(detection) < 4 + num_classes:
                    continue
                    
                # Extract bbox and confidence
                bbox = detection[:4]
                confs = detection[4:4+num_classes]
                
                # Find best class
                class_id = np.argmax(confs)
                confidence = confs[class_id]
                
                if confidence > self.detection_threshold:
                    # Convert bbox from center-x, center-y, width, height to xmin, ymin, xmax, ymax
                    cx, cy, w, h = bbox
                    
                    # Scale to original image size (assuming 640x640 input)
                    cx *= image.shape[1] / self.input_size[0]
                    cy *= image.shape[0] / self.input_size[1]
                    w *= image.shape[1] / self.input_size[0]
                    h *= image.shape[0] / self.input_size[1]
                    
                    xmin = int(cx - w/2)
                    ymin = int(cy - h/2)
                    xmax = int(cx + w/2)
                    ymax = int(cy + h/2)
                    
                    detections.append({
                        'bbox': [xmin, ymin, xmax, ymax],
                        'class_id': int(class_id),
                        'confidence': float(confidence),
                        'class_name': self.labels[class_id]
                    })
        
        return detections
        
    def _detect_model_type(self, model_path: str) -> str:
        """Detect model type based on file extension and path"""
        if model_path.endswith('.pt') or model_path.endswith('.torchscript'):
            return 'yolo'
        elif 'ncnn' in model_path.lower() or model_path.endswith('.param'):
            return 'ncnn'
        elif os.path.isdir(model_path) and any(f.endswith('.param') for f in os.listdir(model_path)):
            return 'ncnn'
        else:
            # Default to YOLO if can't determine
            return 'yolo'
    
    def _load_ncnn_model(self):
        """Load NCNN model"""
        try:
            import yaml
        except ImportError:
            print("Warning: PyYAML not available, using default class names")
            yaml = None

        # Find model files
        model_dir = self.model_path if os.path.isdir(self.model_path) else os.path.dirname(self.model_path)

        param_file = None
        bin_file = None
        metadata_file = None

        for file in os.listdir(model_dir):
            if file.endswith('.param'):
                param_file = os.path.join(model_dir, file)
            elif file.endswith('.bin'):
                bin_file = os.path.join(model_dir, file)
            elif file == 'metadata.yaml':
                metadata_file = os.path.join(model_dir, file)

        if not param_file or not bin_file:
            raise Exception(f"NCNN model files not found in {model_dir}")

        # Load class names from metadata if available
        if metadata_file and yaml:
            try:
                with open(metadata_file, 'r') as f:
                    metadata = yaml.safe_load(f)
                    self.labels = metadata.get('names', self.labels)
                    print(f"Loaded class names from metadata: {self.labels}")
            except Exception as e:
                print(f"Warning: Could not load metadata: {e}")

        # Load NCNN model
        self.ncnn_net = ncnn.Net()
        self.ncnn_net.load_param(param_file)
        self.ncnn_net.load_model(bin_file)

        # Model parameters
        self.input_size = (640, 640)  # From metadata
        self.mean_vals = [0, 0, 0]    # RGB mean
        self.norm_vals = [1/255.0, 1/255.0, 1/255.0]  # Normalization
        
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
        
    def arm_extend(self, angle: int = 180):
        """Extend arm to specified angle (100=home, 180=fully extended)"""
        self.arduino.send_command(f"ARM-EXTEND:{angle}")
        print(f"  → ARM-EXTEND to {angle}°")
        
    def look(self, angle: int = 160):
        """Move head to look at specified angle (160=object, 180=BIN)"""
        self.arduino.send_command(f"LOOK:{angle}")
        print(f"  → LOOK at {angle}°")
        
    def grip_rotate(self, angle: int):
        """Rotate gripper to specified angle (0-180)"""
        self.arduino.send_command(f"GRIP-ROTATE:{angle}")
        print(f"  → GRIP-ROTATE to {angle}°")
        
    def grip(self, angle: int):
        """Set gripper position (110=open, 180=closed)"""
        self.arduino.send_command(f"GRIP:{angle}")
        print(f"  → GRIP to {angle}°")
        
    def reset_arm(self):
        """Initial homing sequence"""
        print("\n[HOMING] Starting initial homing sequence...")
        print("[HOMING] Step 1: LIFTER UP until limit switch triggers")
        self.lifter_up()
        time.sleep(3.0)  # Wait for limit switch
        
        print("[HOMING] Step 2: ARM-ROTATE to 180 (front)")
        self.arm_rotate(180)
        time.sleep(1.5)
        
        print("[HOMING] Step 3: ARM-EXTEND to 100 (home)")
        self.arm_extend(100)
        time.sleep(1.0)
        
        print("[HOMING] Step 4: LOOK at 180 (BIN)")
        self.look(180)
        time.sleep(1.0)
        
        print("[HOMING] Step 5: GRIP-ROTATE to 0")
        self.grip_rotate(0)
        time.sleep(1.0)
        
        print("[HOMING] Step 6: GRIP to 110 (open)")
        self.grip(110)
        time.sleep(0.5)
        
        print("✓ Homing sequence complete\n")
        
    def pickup_and_drop_sequence(self, object_name: str, x_center: int, frame_width: int):
        """
        Execute complete pickup and drop sequence for detected object
        
        Args:
            object_name: Name of detected object
            x_center: X coordinate of object center
            frame_width: Width of camera frame
        """
        if self.pickup_in_progress:
            return
            
        self.pickup_in_progress = True
        print(f"\n{'='*60}")
        print(f"[PICKUP & DROP] Starting sequence for: {object_name}")
        print(f"{'='*60}")
        
        try:
            # Object is centered in camera, arm should already be at 180 (front)
            # No rotation needed initially - arm stays at 180
            
            print("[STEP 1] LIFTER DOWN for 60 seconds (slowly lower to pickup height)")
            self.lifter_down()
            time.sleep(60.0)  # 60 seconds to lower slowly
            
            print("[STEP 2] ARM-EXTEND to 180 (reach toward object)")
            self.arm_extend(180)
            time.sleep(2.0)
            
            print("[STEP 3] LOOK at 160 (tilt head to look at object)")
            self.look(160)
            time.sleep(1.0)
            
            print("[STEP 4] GRIP-ROTATE to 90 (rotate gripper to align with object)")
            self.grip_rotate(90)
            time.sleep(1.5)
            
            print("[STEP 5] GRIP to 180 (close gripper to grasp object)")
            self.grip(180)
            time.sleep(1.5)
            
            print("[STEP 6] LIFTER UP until limit switch triggers (lift entire arm)")
            self.lifter_up()
            time.sleep(3.0)  # Wait for limit switch
            
            print("[STEP 7] LOOK at 180 (look at BIN)")
            self.look(180)
            time.sleep(1.0)
            
            print("[STEP 8] ARM-EXTEND to 100 (retract arm)")
            self.arm_extend(100)
            time.sleep(1.5)
            
            print("[STEP 9] Wait for LIFTER UP to finish (limit switch triggered)")
            time.sleep(1.0)
            
            print("[STEP 10] ARM-ROTATE to 0 (rotate base toward BIN)")
            self.arm_rotate(0)
            time.sleep(2.0)
            
            print("[STEP 11] GRIP to 110 (open gripper to drop object)")
            self.grip(110)
            time.sleep(1.0)
            
            print(f"✓ Pickup and drop sequence complete for {object_name}!")
            print(f"{'='*60}\n")
            
            # Return to home position
            print("[RETURN] Returning to home position...")
            self.arm_rotate(180)  # Return to front
            time.sleep(2.0)
            self.grip_rotate(0)  # Reset gripper rotation
            time.sleep(1.0)
            
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
                
                # Run detection based on model type
                if self.model_type == 'yolo':
                    results = self.model(frame, verbose=False)
                    detections = results[0].boxes
                elif self.model_type == 'ncnn':
                    detections = self._run_ncnn_inference(frame)
                else:
                    detections = []
                
                # Track detected objects
                detected_objects = []
                
                # Process detections
                if self.model_type == 'yolo':
                    # Ultralytics YOLO format
                    for i in range(len(detections)):
                        # Get bounding box
                        xyxy = detections[i].xyxy.cpu().numpy().squeeze()
                        xmin, ymin, xmax, ymax = xyxy.astype(int)
                        
                        # Get class and confidence
                        classidx = int(detections[i].cls.item())
                        classname = self.labels[classidx]
                        conf = detections[i].conf.item()
                        
                        if conf > self.detection_threshold:
                            x_center = (xmin + xmax) // 2
                            detected_objects.append({
                                'name': classname,
                                'conf': conf,
                                'x': x_center,
                                'y': (ymin + ymax) // 2,
                                'bbox': (xmin, ymin, xmax, ymax)
                            })
                            
                            # Draw bounding box
                            color = (0, 255, 0) if not self.pickup_in_progress else (0, 165, 255)
                            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                            cv2.circle(frame, (x_center, (ymin + ymax) // 2), 5, (0, 0, 255), -1)
                            
                            # Draw label
                            label = f'{classname}: {int(conf*100)}%'
                            cv2.putText(frame, label, (xmin, ymin-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                elif self.model_type == 'ncnn':
                    # NCNN format
                    for det in detections:
                        xmin, ymin, xmax, ymax = det['bbox']
                        classname = det['class_name']
                        conf = det['confidence']
                        
                        x_center = (xmin + xmax) // 2
                        detected_objects.append({
                            'name': classname,
                            'conf': conf,
                            'x': x_center,
                            'y': (ymin + ymax) // 2,
                            'bbox': (xmin, ymin, xmax, ymax)
                        })
                        
                        # Draw bounding box
                        color = (0, 255, 0) if not self.pickup_in_progress else (0, 165, 255)
                        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                        cv2.circle(frame, (x_center, (ymin + ymax) // 2), 5, (0, 0, 255), -1)
                        
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
                        self.pickup_and_drop_sequence(obj['name'], obj['x'], frame_width)
                
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
  python test_object_pickup.py --model my_model_ncnn_model --camera picamera0
        """
    )
    
    parser.add_argument('--model', type=str, required=True,
                       help='Path to model file (.pt for YOLO, .param or directory for NCNN)')
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
