from lunacontroller.command import Command
from lunacontroller.drivetrain import drivetrainInstance
from lunacontroller.arm import armInstance
from messages.msg import ControllerInput

MIN_ARM_ANGLE = 0
MAX_ARM_ANGLE = 180
ARM_ANGLE_RATE = 2

class Teleop(Command):
    """
    Teleop command for controlling the robot using a joystick or keyboard.
    The controls are:
    - Joystick:
        - Left stick Y axis: forward/backward
        - Left stick X axis: rotation
        - Right stick Y axis: arm up/down
    initialize() should be called once when the command starts.
    execute() should be called repeatedly while the command is active.
    end() should be called once when the command ends.
    """
    def __init__(self, node):
        super().__init__(node)
        # Initialize all variables
        self.drivetrain = drivetrainInstance
        self.arm = armInstance
        self.subscriber = None
        self.input = ControllerInput()
    
    def initialize(self):
        """Runs once when the command starts"""
        self.armAngle = 90
        self.arm.set_angle(self.armAngle)
        self.subscriber = self.node.create_subscription("controller_input", ControllerInput, self.joystick_callback, 10)
        # ensure we have a default message so execute() can read fields immediately
        if self.input is None:
            self.input = ControllerInput()
    
    def execute(self):
        """Runs repeatedly while the command is active"""
        # Guard against missing input
        if self.input is None:
            return

        # Invert Y axis, down is positive
        speed = -self.input.left_y
        # Multiply by absolute value to make control less sensitive near 0
        # All speeds are still possible, since speed * abs(speed) covers [-1, 1]
        speed = speed * abs(speed)
        # Same thing for robot rotation and arm movement
        rotation = -self.input.left_x
        rotation = rotation * abs(rotation)
        armSpeed = -self.input.right_y
        armSpeed = ARM_ANGLE_RATE * armSpeed * abs(armSpeed)
        # Store the arm angle to make the servo trun to
        self.armAngle += armSpeed
        # Clamp the arm angle and send commands to the motors
        self.armAngle = max(MIN_ARM_ANGLE, min(MAX_ARM_ANGLE, self.armAngle))
        self.arm.set_angle(self.armAngle)
        self.drivetrain.drive(speed, rotation)
    
    def end(self):
        """Runs once when the command ends"""
        # Set arm to free mode
        self.arm.set_angle(None)
        # Stop the drivetrain and keyboard listener
        self.drivetrain.stop()
        if self.subscriber is not None:
            self.node.destroy_subscription(self.subscriber)
            self.subscriber = None

    def joystick_callback(self, msg):
        self.input = msg