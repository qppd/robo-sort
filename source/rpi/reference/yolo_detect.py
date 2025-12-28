import os
import sys
import argparse
import glob
import time
import serial
import threading

import cv2
import numpy as np
from ultralytics import YOLO

# Define and parse user input arguments

parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                    required=True)
parser.add_argument('--source', help='Image source, can be image file ("test.jpg"), \
                    image folder ("test_dir"), video file ("testvid.mp4"), or index of USB camera ("usb0")', 
                    required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                    default=0.5)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                    otherwise, match source resolution',
                    default=None)
parser.add_argument('--record', help='Record results from video or webcam and save it as "demo1.avi". Must specify --resolution argument to record.',
                    action='store_true')
parser.add_argument('--lidar', help='Enable LIDAR distance sensor integration (example: "--lidar /dev/ttyUSB0" or "--lidar COM3")',
                    default=None)
parser.add_argument('--lidar-type', help='Type of LIDAR sensor: "ld06" (LDRobot LD06 360deg)',
                    default='ld06', choices=['ld06'])

args = parser.parse_args()


# Parse user inputs
model_path = args.model
img_source = args.source
min_thresh = args.thresh
user_res = args.resolution
record = args.record
lidar_port = args.lidar
lidar_type = args.lidar_type

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

# LIDAR sensor class for LDRobot LD06 (360-degree scanning LIDAR)
class LD06Lidar:
    def __init__(self, port, baudrate=230400):
        """Initialize LD06 LIDAR (0.02-12m range, 360° FOV, 4500Hz sample rate)"""
        self.serial = serial.Serial(port, baudrate, timeout=1.0)
        self.point_cloud = []  # Full 360° point cloud
        self.center_distance = 0  # Distance in front (0° direction)
        self.min_distance = float('inf')
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
    
    def _read_loop(self):
        """Read and parse LD06 data packets"""
        buffer = bytearray()
        while self.running:
            try:
                # LD06 sends packets of 47 bytes
                if self.serial.in_waiting > 0:
                    buffer.extend(self.serial.read(self.serial.in_waiting))
                
                # Look for packet header (0x54)
                while len(buffer) >= 47:
                    if buffer[0] == 0x54:  # Header byte
                        packet = buffer[:47]
                        buffer = buffer[47:]
                        
                        # Parse packet: contains 12 measurement points
                        radar_speed = packet[2] + (packet[3] << 8)  # deg/s
                        start_angle = (packet[4] + (packet[5] << 8)) / 100.0  # degrees
                        
                        points = []
                        for i in range(12):
                            offset = 6 + i * 3
                            distance = packet[offset] + (packet[offset + 1] << 8)  # mm
                            intensity = packet[offset + 2]
                            
                            if distance > 0:  # Valid measurement
                                # Calculate angle for this point
                                angle_step = packet[42] / 100.0  # End angle
                                angle = (start_angle + (angle_step - start_angle) * i / 12.0) % 360
                                points.append({
                                    'angle': angle,
                                    'distance': distance / 10.0,  # Convert to cm
                                    'intensity': intensity
                                })
                        
                        with self.lock:
                            self.point_cloud = points
                            
                            # Find closest point overall
                            if points:
                                self.min_distance = min(p['distance'] for p in points)
                                
                                # Get distance in forward direction (0° ± 15°)
                                forward_points = [p for p in points if p['angle'] < 15 or p['angle'] > 345]
                                if forward_points:
                                    self.center_distance = min(p['distance'] for p in forward_points)
                                else:
                                    self.center_distance = self.min_distance
                    else:
                        buffer = buffer[1:]  # Skip byte and look for next header
                        
            except Exception as e:
                print(f"LD06 LIDAR read error: {e}")
                time.sleep(0.1)
    
    def get_distance(self):
        """Get distance in forward direction (center of view)"""
        with self.lock:
            return self.center_distance if self.center_distance < float('inf') else 0
    
    def get_min_distance(self):
        """Get closest distance from all 360° points"""
        with self.lock:
            return self.min_distance if self.min_distance < float('inf') else 0
    
    def get_point_cloud(self):
        """Get full 360° point cloud data"""
        with self.lock:
            return self.point_cloud.copy()
    
    def get_distance_at_angle(self, target_angle, tolerance=10):
        """Get distance at specific angle (±tolerance)"""
        with self.lock:
            nearby = [p for p in self.point_cloud 
                     if abs(p['angle'] - target_angle) < tolerance or 
                        abs(p['angle'] - target_angle + 360) < tolerance or
                        abs(p['angle'] - target_angle - 360) < tolerance]
            if nearby:
                return min(p['distance'] for p in nearby)
            return 0
    
    def get_strength(self):
        return 0  # Return avg intensity if needed
    
    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        if self.serial.is_open:
            self.serial.close()

# Initialize LIDAR if enabled
lidar = None
if lidar_port:
    print(f"Initializing {lidar_type.upper()} LIDAR sensor on {lidar_port}...")
    try:
        if lidar_type == 'ld06':
            lidar = LD06Lidar(lidar_port)
            print("LD06: 360° scanning LIDAR, 0.02-12m range, 4500Hz sample rate")
        time.sleep(1.0)  # Let LIDAR stabilize and collect initial data
        print(f"LIDAR initialized successfully. Distance: {lidar.get_distance():.1f} cm")
    except Exception as e:
        print(f"ERROR: Failed to initialize LIDAR: {e}")
        print("Continuing without LIDAR...")
        lidar = None

# Parse input to determine if image source is a file, folder, video, or USB camera
img_ext_list = ['.jpg','.JPG','.jpeg','.JPEG','.png','.PNG','.bmp','.BMP']
vid_ext_list = ['.avi','.mov','.mp4','.mkv','.wmv']

if os.path.isdir(img_source):
    source_type = 'folder'
elif os.path.isfile(img_source):
    _, ext = os.path.splitext(img_source)
    if ext in img_ext_list:
        source_type = 'image'
    elif ext in vid_ext_list:
        source_type = 'video'
    else:
        print(f'File extension {ext} is not supported.')
        sys.exit(0)
elif 'usb' in img_source:
    source_type = 'usb'
    usb_idx = int(img_source[3:])
elif 'picamera' in img_source:
    source_type = 'picamera'
    picam_idx = int(img_source[8:])
else:
    print(f'Input {img_source} is invalid. Please try again.')
    sys.exit(0)

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Check if recording is valid and set up recording
if record:
    if source_type not in ['video','usb']:
        print('Recording only works for video and camera sources. Please try again.')
        sys.exit(0)
    if not user_res:
        print('Please specify resolution to record video at.')
        sys.exit(0)
    
    # Set up recording
    record_name = 'demo1.avi'
    record_fps = 30
    recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))

# Load or initialize image source
if source_type == 'image':
    imgs_list = [img_source]
elif source_type == 'folder':
    imgs_list = []
    filelist = glob.glob(img_source + '/*')
    for file in filelist:
        _, file_ext = os.path.splitext(file)
        if file_ext in img_ext_list:
            imgs_list.append(file)
elif source_type == 'video' or source_type == 'usb':

    if source_type == 'video': cap_arg = img_source
    elif source_type == 'usb': cap_arg = usb_idx
    cap = cv2.VideoCapture(cap_arg)

    # Set camera or video resolution if specified by user
    if user_res:
        ret = cap.set(3, resW)
        ret = cap.set(4, resH)

elif source_type == 'picamera':
    from picamera2 import Picamera2
    cap = Picamera2()
    cap.configure(cap.create_video_configuration(main={"format": 'XRGB8888', "size": (resW, resH)}))
    cap.start()

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

# Begin inference loop
while True:

    t_start = time.perf_counter()

    # Load frame from image source
    if source_type == 'image' or source_type == 'folder': # If source is image or image folder, load the image using its filename
        if img_count >= len(imgs_list):
            print('All images have been processed. Exiting program.')
            sys.exit(0)
        img_filename = imgs_list[img_count]
        frame = cv2.imread(img_filename)
        img_count = img_count + 1
    
    elif source_type == 'video': # If source is a video, load next frame from video file
        ret, frame = cap.read()
        if not ret:
            print('Reached end of the video file. Exiting program.')
            break
    
    elif source_type == 'usb': # If source is a USB camera, grab frame from camera
        ret, frame = cap.read()
        if (frame is None) or (not ret):
            print('Unable to read frames from the camera. This indicates the camera is disconnected or not working. Exiting program.')
            break

    elif source_type == 'picamera': # If source is a Picamera, grab frames using picamera interface
        frame_bgra = cap.capture_array()
        frame = cv2.cvtColor(np.copy(frame_bgra), cv2.COLOR_BGRA2BGR)
        if (frame is None):
            print('Unable to read frames from the Picamera. This indicates the camera is disconnected or not working. Exiting program.')
            break

    # Resize frame to desired display resolution
    if resize == True:
        frame = cv2.resize(frame,(resW,resH))

    # Get LIDAR distance reading if available
    lidar_distance = 0
    lidar_min_distance = 0
    lidar_strength = 0
    if lidar:
        lidar_distance = lidar.get_distance()  # Forward/center distance
        lidar_min_distance = lidar.get_min_distance()  # Closest point (360°)
        lidar_strength = lidar.get_strength()

    # Run inference on frame
    results = model(frame, verbose=False)

    # Extract results
    detections = results[0].boxes

    # Initialize variable for basic object counting example
    object_count = 0

    # Go through each detection and get bbox coords, confidence, and class
    for i in range(len(detections)):

        # Get bounding box coordinates
        # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
        xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
        xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
        xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

        # Get bounding box class ID and name
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]

        # Get bounding box confidence
        conf = detections[i].conf.item()

        # Draw box if confidence threshold is high enough
        if conf > 0.5:

            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

            # Create label with confidence and LIDAR distance if available
            if lidar and lidar_distance > 0:
                label = f'{classname}: {int(conf*100)}% | {lidar_distance:.1f}cm'
            else:
                label = f'{classname}: {int(conf*100)}%'
            
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

            # Basic example: count the number of objects in the image
            object_count = object_count + 1

    # Calculate and draw framerate (if using video, USB, or Picamera source)
    if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
    
    # Display detection results
    cv2.putText(frame, f'Number of objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
    
    # Display LIDAR information if available
    if lidar:
        if lidar_type == 'ld06':
            # Show both forward distance and closest obstacle for 360° LIDAR
            lidar_text = f'LIDAR Forward: {lidar_distance:.1f}cm | Closest: {lidar_min_distance:.1f}cm'
        else:
            lidar_text = f'LIDAR Distance: {lidar_distance:.1f} cm'
            if lidar_strength > 0:
                lidar_text += f' | Strength: {lidar_strength}'
        cv2.putText(frame, lidar_text, (10,60), cv2.FONT_HERSHEY_SIMPLEX, .7, (255,100,255), 2)
    cv2.imshow('YOLO detection results',frame) # Display image
    if record: recorder.write(frame)

    # If inferencing on individual images, wait for user keypress before moving to next image. Otherwise, wait 5ms before moving to next frame.
    if source_type == 'image' or source_type == 'folder':
        key = cv2.waitKey()
    elif source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        key = cv2.waitKey(5)
    
    if key == ord('q') or key == ord('Q'): # Press 'q' to quit
        break
    elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
        cv2.waitKey()
    elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
        cv2.imwrite('capture.png',frame)
    
    # Calculate FPS for this frame
    t_stop = time.perf_counter()
    frame_rate_calc = float(1/(t_stop - t_start))

    # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
    if len(frame_rate_buffer) >= fps_avg_len:
        temp = frame_rate_buffer.pop(0)
        frame_rate_buffer.append(frame_rate_calc)
    else:
        frame_rate_buffer.append(frame_rate_calc)

    # Calculate average FPS for past frames
    avg_frame_rate = np.mean(frame_rate_buffer)


# Clean up
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
if lidar:
    print("Closing LIDAR sensor...")
    lidar.close()
if source_type == 'video' or source_type == 'usb':
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record: recorder.release()
cv2.destroyAllWindows()
