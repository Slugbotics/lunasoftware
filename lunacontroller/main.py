import time

from teleop import Teleop
from command import Command

# Open serial connection to Arduino
# arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1) #On Pi
# on Windows:arduino = serial.Serial('COM3', 9600, timeout=1)
# on macOS:arduino = serial.Serial('/dev/tty.usbmodem...', 9600, timeout=1)
# time.sleep(2)  # Give Arduino time to reset

# print("Connected to Arduino")

# Example: send motor command
# arduino.write(b"FORWARD\n")
# print("Sent: FORWARD")
# time.sleep(3)

# arduino.write(b"STOP\n")
# print("Sent: STOP")

# Example: read response
# while arduino.in_waiting > 0:
#     print("Arduino says:", arduino.readline().decode().strip())

# Create and initialize Teleop command
command: Command = Teleop()
command.initialize()
try:
    # Run the command loop at about 50 Hz
    while True:
        command.execute()
        time.sleep(0.02)  # 50 Hz update rate
except KeyboardInterrupt:
    # On Ctrl-C, end the command and exit the loop
    command.end()

# arduino.close()
