import sys
import os
import time

# Add LIDAR driver path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ldrobot-ld06-lidar-python-driver-master'))
from listen_to_lidar import listen_to_lidar

def main(port='/dev/ttyUSB1', duration=20):
    import csv
    print("="*50)
    print(f"LIDAR SOLO ANGLE TEST ({port})")
    print("="*50)
    print("Scanning for angles 0-360°...\n")
    lidar_data, stop_func = listen_to_lidar(port=port)
    start_time = time.time()
    csv_filename = f"lidar_angle_test_{int(start_time)}.csv"
    with open(csv_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["angle", "distance_cm", "zone"])
        try:
            while time.time() - start_time < duration:
                distances = lidar_data['distances'].copy()
                for angle, dist in sorted(distances.items()):
                    zone = None
                    if 0 <= angle <= 90 or 270 <= angle <= 360:
                        if 1 <= dist <= 30:
                            zone = "FRONT"
                    else:
                        if 1 <= dist <= 30:
                            zone = "BACK"
                    if zone:
                        print(f"Angle: {angle:6.1f}° | Distance: {dist:7.2f} cm | Zone: {zone}")
                        writer.writerow([angle, dist, zone])
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            stop_func()
            print(f"\nLIDAR test completed. Data saved to {csv_filename}.")

if __name__ == "__main__":
    # You can change port and duration here
    main(port='/dev/ttyUSB1', duration=20)