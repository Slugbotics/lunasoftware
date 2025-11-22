from gpiozero import AngularServo
from lunacontroller.drivetrain import _clamp, BOARD_TO_BCM

class Arm:
    """Represents the robot's arm subsystem."""
    def __init__(self):
        # Create servo
        self.angle = 90
        self.leftServo = AngularServo(pin=BOARD_TO_BCM[22], min_angle=0, max_angle=180, initial_angle=self.angle)
        self.rightServo = AngularServo(pin=BOARD_TO_BCM[27], min_angle=0, max_angle=180, initial_angle=self.angle)
        self.set_angle(self.angle)
    
    def set_angle(self, angle):
        """
        Sets the arm servo to the specified angle.
        angle: Desired angle in degrees (0 to 180)
        If angle is None, sets servo to free mode.
        """
        if angle is None:
            self.leftServo.angle = None
            self.rightServo.angle = None
            return
        self.angle = _clamp(angle, 0, 180)
        self.leftServo.angle = self.angle
        self.rightServo.angle = self.angle

armInstance = Arm()