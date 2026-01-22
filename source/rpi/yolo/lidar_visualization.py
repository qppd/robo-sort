import sys
import os
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Add LIDAR driver path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ldrobot-ld06-lidar-python-driver-master'))
from listen_to_lidar import listen_to_lidar

def update_plot(frame, lidar_data, ax):
    ax.clear()
    ax.set_thetamin(0)
    ax.set_thetamax(360)
    ax.set_rlim(0, 200)
    ax.set_title("LIDAR Real-Time Visualization", va='bottom')
    
    distances = lidar_data['distances'].copy()
    angles_deg = []
    distances_cm = []
    for angle, dist in distances.items():
        if dist > 30:  # Only plot distances greater than 5cm
            angles_deg.append(angle)
            distances_cm.append(dist)
    
    # Convert angles to radians for polar plot
    angles_rad = np.deg2rad(angles_deg)
    
    # Plot scatter
    ax.scatter(angles_rad, distances_cm, s=1, c='red', alpha=0.7)
    
    return ax,

def main(port='/dev/ttyUSB1'):
    print("="*50)
    print(f"LIDAR REAL-TIME VISUALIZATION ({port})")
    print("="*50)
    print("Starting real-time LIDAR visualization...\n")
    
    # Start LIDAR
    lidar_data, stop_func = listen_to_lidar(port=port)
    
    # Set up the polar plot
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, polar=True)
    ax.set_thetamin(0)
    ax.set_thetamax(360)
    ax.set_rlim(0, 200)  # Adjust max distance as needed
    ax.set_title("LIDAR Real-Time Visualization", va='bottom')
    
    # Animation
    ani = animation.FuncAnimation(fig, update_plot, fargs=(lidar_data, ax), 
                                 interval=100, blit=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nVisualization interrupted by user.")
    finally:
        stop_func()
        print("LIDAR visualization stopped.")

if __name__ == "__main__":
    # You can change port here
    main(port='/dev/ttyUSB1')