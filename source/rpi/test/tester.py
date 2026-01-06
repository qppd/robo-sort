import RPi.GPIO as GPIO
import time

# GPIO pin numbers
pins = [17, 27, 22]

# Setup
GPIO.setmode(GPIO.BCM)  # Use BCM numbering
for pin in pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Assuming switches connect to GND when pressed

print("Limit switch test started. Press the buttons...")

try:
    while True:
        for pin in pins:
            state = GPIO.input(pin)
            print(f"GPIO {pin}: {'LOW (pressed)' if state == GPIO.LOW else 'HIGH (not pressed)'}")
            if state == GPIO.LOW:  # Button pressed
                print(f"Button on GPIO {pin} pressed!")
                # Wait until button is released to avoid multiple prints
                while GPIO.input(pin) == GPIO.LOW:
                    time.sleep(0.01)
        time.sleep(0.1)  # Small delay to reduce CPU usage and make output readable

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    GPIO.cleanup()
