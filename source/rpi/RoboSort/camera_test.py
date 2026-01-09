#!/usr/bin/env python3
"""
Camera Test Script for RoboSort
Tests USB camera connectivity and display using OpenCV
"""

import cv2
import sys
import time

def test_camera(camera_index=1):
    """
    Test USB camera at specified index

    Args:
        camera_index (int): Camera device index (0 for /dev/video0, 1 for /dev/video1, etc.)
    """
    print(f"🔍 Testing camera at index {camera_index}...")
    print(f"   This corresponds to /dev/video{camera_index}")

    # Open camera
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"❌ Failed to open camera at index {camera_index}")
        print("   Possible issues:")
        print(f"   - Camera not connected to /dev/video{camera_index}")
        print("   - Camera already in use by another application")
        print("   - Insufficient permissions (try: sudo chmod 666 /dev/video*)")
        print("   - Wrong camera index")
        return False

    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print("✅ Camera opened successfully!"    print(f"   Resolution: {width}x{height}")
    print(f"   FPS: {fps}")
    print("   Press 'q' or ESC to exit"
    print("   Press 's' to save a snapshot"

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("❌ Failed to read frame from camera")
                break

            frame_count += 1

            # Calculate and display FPS
            elapsed_time = time.time() - start_time
            if elapsed_time > 0:
                current_fps = frame_count / elapsed_time
                cv2.putText(frame, f"FPS: {current_fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Add camera info
            cv2.putText(frame, f"Camera /dev/video{camera_index}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Resolution: {width}x{height}", (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Display frame
            cv2.imshow(f'Camera Test - /dev/video{camera_index}', frame)

            # Handle key presses
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:  # 'q' or ESC
                print("👋 Exiting camera test...")
                break
            elif key == ord('s'):  # 's' to save snapshot
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"camera_{camera_index}_snapshot_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"📸 Snapshot saved as: {filename}")

    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")

    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        print("🧹 Camera resources cleaned up")

    return True

def list_available_cameras():
    """
    List all available camera devices
    """
    print("🔍 Scanning for available cameras...")
    available_cameras = []

    for i in range(10):  # Check first 10 indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            available_cameras.append((i, width, height))
            cap.release()

    if available_cameras:
        print("📹 Available cameras:")
        for idx, width, height in available_cameras:
            print(f"   /dev/video{idx} - {width}x{height}")
    else:
        print("❌ No cameras found")
        print("   Make sure cameras are connected and not in use")

    return available_cameras

def main():
    """
    Main function
    """
    print("🤖 RoboSort Camera Test Utility")
    print("=" * 40)

    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
            test_camera(camera_index)
        except ValueError:
            print(f"❌ Invalid camera index: {sys.argv[1]}")
            print("   Usage: python camera_test.py [camera_index]")
            sys.exit(1)
    else:
        # No argument provided, list available cameras
        available = list_available_cameras()

        if available:
            print("\n💡 Usage examples:")
            print("   python camera_test.py 0    # Test /dev/video0")
            print("   python camera_test.py 1    # Test /dev/video1")
            print("   python camera_test.py 2    # Test /dev/video2")

            # Test the first available camera
            first_camera = available[0][0]
            print(f"\n🚀 Testing first available camera (/dev/video{first_camera})...")
            test_camera(first_camera)
        else:
            print("\n❌ No cameras available to test")
            sys.exit(1)

if __name__ == "__main__":
    main()