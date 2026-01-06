from gpiozero import Button
from signal import pause

pins = [17, 27, 22]

buttons = []
for p in pins:
    b = Button(p, pull_up=True, bounce_time=0.1)
    b.when_pressed = lambda p=p: print(f"GPIO {p} pressed")
    buttons.append(b)

print("Waiting for button presses...")
pause()
