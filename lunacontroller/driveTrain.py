"""Drivetrain hardware interface with optional console simulation.

This file provides a Drivetrain class with a small simulation mode so code
that depends on it (like the tester or teleop) can run without a serial
device.
"""

from typing import Optional
from gpiozero import Motor

def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    """Clamp v to the range [lo, hi]"""
    return max(lo, min(hi, v))


class Drivetrain:
    """
    Represents the drivetrain subsystem.
    """
    def __init__(self):
        # Create motors
        self.leftMotor = Motor(forward=2, backward=4, enable=3)
        self.rightMotor = Motor(forward=23, backward=24, enable=25)

    def tank_drive(self, left: float, right: float) -> None:
        """Drives the robot using left/right speed values.

        left/right are expected in [-1.0, 1.0]
        """
        left = _clamp(left)
        right = _clamp(right)
        self.leftMotor.value = left
        self.rightMotor.value = right

    def drive(self, speed: float, rotation: float) -> None:
        """Drives the robot using speed and rotation values.
        speed is forward/backward motion in [-1.0, 1.0]
        rotation is rotational motion in [-1.0, 1.0]
        Counterclockwise rotation (left from robot perspective) is positive
        """
        left = speed - rotation
        right = speed + rotation
        self.tank_drive(left, right)

    def stop(self) -> None:
        """Stops the drivetrain motors."""
        self.tank_drive(0.0, 0.0)

# Use this instead of creating multiple instances of Drivetrain
driveTrainInstance = Drivetrain()    
