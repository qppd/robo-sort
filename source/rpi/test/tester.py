from gpiozero import Button
from signal import pause
import time

pins = [17, 27, 22]

# Map pins to their functions
pin_names = {
    17: "Start",
    27: "Bin",
    22: "Arm"
}

buttons = []
for p in pins:
    b = Button(p, pull_up=True, bounce_time=0.1)
    b.when_pressed = lambda p=p: print(f"{time.time():.2f}: {pin_names[p]} button pressed")
    buttons.append(b)

print("Waiting for button presses...")
pause()
