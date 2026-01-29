import warnings
try:
    from gpiozero import Device
    from gpiozero.pins.pigpio import PiGPIOFactory
    Device.pin_factory = PiGPIOFactory()
except Exception as e:
    warnings.warn(f'PiGPIOFactory unavailable, gpiozero will select default pin factory: {e}')

from gpiozero import AngularServo
from lunacontroller.drivetrain import _clamp, BOARD_TO_BCM
from rclpy.node import Node
from std_msgs.msg import Float64

class Arm:
    """Represents the robot's arm subsystem."""
    def __init__(self, node):
        self.node = node

        # Create servos
        self.angle = 90
        try:
            self.leftServo = AngularServo(pin=BOARD_TO_BCM[22], min_angle=0, max_angle=180, initial_angle=self.angle)
            self.rightServo = AngularServo(pin=BOARD_TO_BCM[27], min_angle=0, max_angle=180, initial_angle=self.angle)
        except Exception:
            self.leftServo = None
            self.rightServo = None
            self.node.get_logger().warning('Arm is running as simulation')
            self.angle_publisher = self.node.create_publisher(Float64, '/arm_angle', 10)
        self.set_angle(None)
    
    def set_angle(self, angle: float | None):
        """
        Sets the arm servo to the specified angle.
        angle: Desired angle in degrees (0 to 180)
        If angle is None, sets servo to free mode.
        """
        if angle is None:
            if self.leftServo is not None:
                self.leftServo.angle = None
                self.rightServo.angle = None
            return
        self.angle = _clamp(angle, 0, 180)
        if self.leftServo is not None:
            self.leftServo.angle = self.angle
            self.rightServo.angle = self.angle
        else:
            msg = Float64()
            msg.data = float(self.angle)
            self.angle_publisher.publish(msg)

armInstance = None
def create_arm(node):
    global armInstance
    armInstance = Arm(node)
