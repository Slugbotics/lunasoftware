"""Drivetrain hardware interface with optional console simulation.

This file provides a Drivetrain class with a small simulation mode so code
that depends on it (like the tester or teleop) can run without a serial
device.
"""

from typing import Optional
from gpiozero import Motor

BOARD_TO_BCM = {
    1: None, 2: None, 3: 2,  4: None, 5: 3,  6: None, 7: 4,  8: 14,
    9: None, 10: 15, 11: 17, 12: 18, 13: 27, 14: None, 15: 22, 16: 23,
    17: None, 18: 24, 19: 10, 20: None, 21: 9, 22: 25, 23: 11, 24: 8,
    25: None, 26: 7, 27: 0, 28: 1, 29: 5, 30: None, 31: 6, 32: 12,
    33: 13, 34: None, 35: 19, 36: 16, 37: 26, 38: 20, 39: None, 40: 21
}

def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    """Clamp v to the range [lo, hi]"""
    return max(lo, min(hi, v))


class Drivetrain:
    """
    Represents the drivetrain subsystem.
    """
    def __init__(self):
        # Create motors
        self.leftMotor1 = Motor(forward=BOARD_TO_BCM[10], backward=BOARD_TO_BCM[12], enable=BOARD_TO_BCM[8])
        self.rightMotor1 = Motor(forward=BOARD_TO_BCM[13], backward=BOARD_TO_BCM[15], enable=BOARD_TO_BCM[11])
        self.leftMotor2 = Motor(forward=BOARD_TO_BCM[38], backward=BOARD_TO_BCM[40], enable=BOARD_TO_BCM[36])
        self.rightMotor2 = Motor(forward=BOARD_TO_BCM[31], backward=BOARD_TO_BCM[33], enable=BOARD_TO_BCM[29])
        self.stop()

    def tank_drive(self, left: float, right: float) -> None:
        """Drives the robot using left/right speed values.

        left/right are expected in [-1.0, 1.0]
        """
        left = _clamp(left)
        right = _clamp(right)
        self.leftMotor1.value = left
        self.rightMotor1.value = right
        self.leftMotor2.value = left
        self.rightMotor2.value = right

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
drivetrainInstance = Drivetrain()    
