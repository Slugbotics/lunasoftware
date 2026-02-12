from lunacontroller.command import Command
from lunacontroller import arm, drivetrain
from messages.msg import ControllerInput
from std_msgs.msg import String

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
        self.drivetrain = drivetrain.drivetrainInstance
        self.arm = arm.armInstance
        self.subscriber = None
        self.input = ControllerInput()
    
    def initialize(self):
        """Runs once when the command starts"""
        self.armAngle = 90
        self.arm.set_angle(self.armAngle)
        self.subscriber = self.node.create_subscription(ControllerInput, "controller_input", self.joystick_callback, 10)
        self.keyboard_subscriber = self.node.create_subscription(String, "/keyboard", self.keyboard_callback, 10)
        self.input = None
        self.keys = None
    
    def execute(self):
        """Runs repeatedly while the command is active"""
        speed = 0
        rotation = 0
        armSpeed = 0
        if self.input is not None:
            speed = -self.input.left_y
            # Multiply by absolute value to make control less sensitive near 0
            # All speeds are still possible, since speed * abs(speed) covers [-1, 1]
            speed = speed * abs(speed)
            # Same thing for robot rotation and arm movement
            rotation = -self.input.left_x
            rotation = rotation * abs(rotation)
            armSpeed = -self.input.right_y
            armSpeed = ARM_ANGLE_RATE * armSpeed * abs(armSpeed)
        elif self.keys is not None:
            # Keyboard controls
            if 'w' in self.keys:
                speed += 1
            if 's' in self.keys:
                speed -= 1
            if 'a' in self.keys:
                rotation += 1
            if 'd' in self.keys:
                rotation -= 1
            if 'i' in self.keys:
                armSpeed += ARM_ANGLE_RATE
            if 'k' in self.keys:
                armSpeed -= ARM_ANGLE_RATE
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
    def keyboard_callback(self, msg):
        self.keys = msg.data
    
    """
    Other button mapping:
    A = dig
    B = dump
    X = drive across obstacle field
    Y = autonomous mode
    """
    def dig_selected(self):
        return self.input is not None and self.input.a
    def dump_selected(self):
        return self.input is not None and self.input.b
    def drive_selected(self):
        return self.input is not None and self.input.x
    def auto_selected(self):
        return self.input is not None and self.input.y