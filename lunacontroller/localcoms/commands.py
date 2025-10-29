# Pi side
CMD_FORWARD = "FORWARD"
CMD_REVERSE = "REVERSE"
CMD_STOP    = "STOP"
CMD_LEFT    = "LEFT"
CMD_RIGHT   = "RIGHT"
#########################
import serial # pip install serial in base terminal
import time

arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # allow Arduino to reset

def send_command(cmd):
    arduino.write(cmd.encode())

# Example usage (kept here for quick manual testing; remove or guard in production):
if __name__ == '__main__':
    send_command('F')  # move forward (example)
    time.sleep(2)
    send_command('S')  # stop
