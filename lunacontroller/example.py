import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) # Use Broadcom pin-numbering scheme
GPIO_PIN = 18 # Example GPIO pin

GPIO.setup(GPIO_PIN, GPIO.OUT) # Set the pin as an output

try:
    while True:
        GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn the pin on (send a high signal)
        time.sleep(1)
        GPIO.output(GPIO_PIN, GPIO.LOW) # Turn the pin off (send a low signal)
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup() # Clean up GPIO settings on exit