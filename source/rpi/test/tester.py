from gpiozero import Button
from signal import pause
import time

# Define callback function for start button
def start_pressed():
    print(f"{time.time():.2f}: Start button pressed")

# Create start button instance with dedicated callback
start_button = Button(17, pull_up=True, bounce_time=0.1)

# Assign callback
start_button.when_pressed = start_pressed

print("Waiting for start button press on GPIO 17...")
pause()
