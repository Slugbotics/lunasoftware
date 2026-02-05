import RPi.GPIO as GPIO
import time
import subprocess

GPIO.setmode(GPIO.BCM) # Use Broadcom pin-numbering scheme
BUTTON_PIN = 17 # Replace with your chosen GPIO pin

GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set as input with pull-up resistor

def run_my_script():
    print("Button pressed! Running main.py...")
    #subprocess.run(["python3", "/lunasoftware/lunacontroller/main.py"]) # Execute your script
    subprocess.run(["python3", "main.py"]) # Execute your script

try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW: # Button is pressed (pulled to ground)
            run_my_script()
            time.sleep(0.5) # Debounce delay
        time.sleep(0.1) # Check button state periodically

except KeyboardInterrupt:
    GPIO.cleanup() # Clean up GPIO settings on exit