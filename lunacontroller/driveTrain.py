"""Drivetrain hardware interface with optional console simulation.

This file provides a Drivetrain class with a small simulation mode so code
that depends on it (like the tester or teleop) can run without a serial
device.
"""

from typing import Optional
from gpiozero import Motor

def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


class Drivetrain:
    def __init__(self):
        self.leftMotor = Motor(forward=27, backward=22)
        self.rightMotor = Motor(forward=23, backward=24)

    def tank_drive(self, left: float, right: float) -> None:
        """Send left/right tank values to the drivetrain.

        left/right are expected in [-1.0, 1.0]; we format them to two
        decimals when sending over serial.
        """
        left = _clamp(left)
        right = _clamp(right)
        if left > 0:
            self.leftMotor.forward(left)
        else:
            self.leftMotor.backward(-left)
        if right > 0:
            self.rightMotor.forward(right)
        else:
            self.rightMotor.backward(-right)

    def drive(self, speed: float, rotation: float) -> None:
        """Helper used by teleop: convert speed/rotation -> left/right.

        A simple mixing: left = speed - rotation, right = speed + rotation.
        Counterclockwise rotation (left from robot perspective) is positive
        """
        left = speed - rotation
        right = speed + rotation
        self.tank_drive(left, right)

    def stop(self) -> None:
        self.drive(0.0, 0.0)

# Use this instead of creating multiple instances of Drivetrain
driveTrainInstance = Drivetrain()    
