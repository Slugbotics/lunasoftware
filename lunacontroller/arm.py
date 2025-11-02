from gpiozero import AngularServo
from driveTrain import _clamp

class Arm:
    """Represents the robot's arm subsystem."""
    def __init__(self):
        # Create servo
        self.angle = 90
        self.servo = AngularServo(pin=17, min_angle=0, max_angle=180, initial_angle=self.angle)
    
    def set_angle(self, angle):
        """
        Sets the arm servo to the specified angle.
        angle: Desired angle in degrees (0 to 180)
        """
        self.angle = _clamp(angle, 0, 180)
        self.servo.angle = self.angle

armInstance = Arm()