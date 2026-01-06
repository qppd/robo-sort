import RPi.GPIO as GPIO
import time

# GPIO pins for the limit switches
LIMIT_SWITCH_PINS = [17, 27, 22]

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(LIMIT_SWITCH_PINS, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Enable pull-up resistors

try:
    print("Testing limit switches. Press Ctrl+C to exit.")
    while True:
        for pin in LIMIT_SWITCH_PINS:
            if GPIO.input(pin) == GPIO.LOW:  # Switch pressed
                print(f"Limit switch on GPIO {pin} is PRESSED")
            else:
                print(f"Limit switch on GPIO {pin} is NOT pressed")
        time.sleep(0.2)  # Short delay to avoid flooding the console
        print("---")
except KeyboardInterrupt:
    print("Exiting test.")
finally:
    GPIO.cleanup()
