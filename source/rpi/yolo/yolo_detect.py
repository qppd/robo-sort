import os
import sys
import argparse
import glob
import time
import serial
import threading

import cv2
import numpy as np
import pygame
from pynput import keyboard

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
parser.add_argument('--arduino', help='Serial port for Arduino communication (example: "/dev/ttyUSB0")',
                    default=None)

args = parser.parse_args()


# Parse user inputs
model_path = args.model
img_source = args.source
min_thresh = args.thresh
user_res = args.resolution
record = args.record
arduino_port = args.arduino

# Parse user-specified display resolution
resize = False
resW, resH = 640, 480  # default
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

# Initialize pressed keys set
pressed_keys = set()

# Keyboard listener functions
def on_press(key):
    try:
        pressed_keys.add(key.char)
    except AttributeError:
        pressed_keys.add(str(key))

def on_release(key):
    try:
        pressed_keys.discard(key.char)
    except AttributeError:
        pressed_keys.discard(str(key))

# Start keyboard listener in a thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((resW, resH))
pygame.display.set_caption('YOLO Detection with Motor Control')

# Initialize Arduino serial connection if port is provided
arduino_ser = None
if arduino_port:
    try:
        arduino_ser = serial.Serial(arduino_port, 9600, timeout=1)
        print(f"Connected to Arduino on {arduino_port}")
    except serial.SerialException as e:
        print(f"Failed to connect to Arduino on {arduino_port}: {e}")
        arduino_ser = None

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

# Initialize centering detection variables
last_centered_class = None
consecutive_centered_count = 0
centering_threshold = 50  # pixels
sent_commands = False

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

    # Define crosshair position (center of frame)
    crosshair_x = frame.shape[1] // 2
    crosshair_y = frame.shape[0] // 2

    # Run inference on frame
    results = model(frame, verbose=False)

    # Extract results
    detections = results[0].boxes

    # Initialize variable for basic object counting example
    object_count = 0

    # Initialize centering check for this frame
    centered = False
    centered_class = None

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

            label = f'{classname}: {int(conf*100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

            # Basic example: count the number of objects in the image
            object_count = object_count + 1

            # Check if object center is close to crosshair
            center_x = (xmin + xmax) // 2
            center_y = (ymin + ymax) // 2
            dx = center_x - crosshair_x
            dy = center_y - crosshair_y
            if dx * dx + dy * dy < centering_threshold * centering_threshold:
                if not centered:
                    centered = True
                    centered_class = classidx

    # Update consecutive centering count
    if centered:
        if centered_class == last_centered_class:
            consecutive_centered_count += 1
        else:
            last_centered_class = centered_class
            consecutive_centered_count = 1
    else:
        consecutive_centered_count = 0
        last_centered_class = None
        sent_commands = False  # Reset when centering breaks

    # Send commands to Arduino if centered for 5 consecutive frames and not already sent
    if consecutive_centered_count >= 5 and not sent_commands and arduino_ser:
        try:
            arduino_ser.write(b"ARM-EXTEND:170\n")
            time.sleep(0.1)
            arduino_ser.write(b"LOOK:140\n")
            print("Sent ARM-EXTEND:170 and LOOK:140 to Arduino")
            sent_commands = True
        except serial.SerialException as e:
            print(f"Failed to send commands to Arduino: {e}")

    # Calculate and draw framerate (if using video, USB, or Picamera source)
    if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
    
    # Display detection results
    cv2.putText(frame, f'Number of objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
    
    # Display centered message if consecutive count >= 5
    if consecutive_centered_count >= 5:
        cv2.putText(frame, 'Object is centered', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw crosshair in the middle
    cv2.line(frame, (crosshair_x - 10, crosshair_y), (crosshair_x + 10, crosshair_y), (255, 255, 255), 2)
    cv2.line(frame, (crosshair_x, crosshair_y - 10), (crosshair_x, crosshair_y + 10), (255, 255, 255), 2)
    
    # Convert frame to RGB and display with Pygame
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    surf = pygame.surfarray.make_surface(np.transpose(frame_rgb, (1, 0, 2)))
    screen.blit(surf, (0, 0))
    pygame.display.flip()
    
    if record: recorder.write(frame)

    # Handle Pygame events and key presses
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    if 'q' in pressed_keys:  # Quit
        running = False
    elif 's' in pressed_keys:  # Pause inference
        paused = True
        while paused:
            if 's' in pressed_keys:
                paused = False
            time.sleep(0.1)
    elif 'p' in pressed_keys:  # Save picture
        cv2.imwrite('capture.png', frame)
    
    # Motor control with arrow keys
    if arduino_ser:
        if 'Key.up' in pressed_keys:
            arduino_ser.write(b"FORWARD\n")
        elif 'Key.down' in pressed_keys:
            arduino_ser.write(b"BACKWARD\n")
        elif 'Key.left' in pressed_keys:
            arduino_ser.write(b"LEFT\n")
        elif 'Key.right' in pressed_keys:
            arduino_ser.write(b"RIGHT\n")
        elif ' ' in pressed_keys:  # Space
            arduino_ser.write(b"STOP\n")
    
    if not running:
        break
    
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
if source_type == 'video' or source_type == 'usb':
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record: recorder.release()
pygame.quit()
listener.stop()
if arduino_ser:
    arduino_ser.close()
