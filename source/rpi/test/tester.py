from gpiozero import Button
from signal import pause
import time

# Define separate callback functions for each button
def start_pressed():
    print(f"{time.time():.2f}: Start button pressed")

def bin_pressed():
    print(f"{time.time():.2f}: Bin button pressed")

def arm_pressed():
    print(f"{time.time():.2f}: Arm button pressed")

# Create separate button instances with dedicated callbacks
start_button = Button(17, pull_up=True, bounce_time=0.1)
bin_button = Button(27, pull_up=True, bounce_time=0.1)
arm_button = Button(22, pull_up=True, bounce_time=0.1)

# Assign callbacks
start_button.when_pressed = start_pressed
bin_button.when_pressed = bin_pressed
arm_button.when_pressed = arm_pressed

print("Waiting for button presses...")
pause()
