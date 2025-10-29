import serial
import time

# Open serial connection to Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1) #On Pi
# on Windows:arduino = serial.Serial('COM3', 9600, timeout=1)
# on macOS:arduino = serial.Serial('/dev/tty.usbmodem...', 9600, timeout=1)
time.sleep(2)  # Give Arduino time to reset

print("Connected to Arduino")

# Example: send motor command
arduino.write(b"FORWARD\n")
print("Sent: FORWARD")
time.sleep(3)

arduino.write(b"STOP\n")
print("Sent: STOP")

# Example: read response
while arduino.in_waiting > 0:
    print("Arduino says:", arduino.readline().decode().strip())

arduino.close()
