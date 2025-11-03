from command import Command
from driveTrain import driveTrainInstance
from arm import armInstance
import pygame
import threading

# Importing pynput might cause an exception if there is no keyboard
try:
    from pynput import keyboard
except ImportError:
    print("Error importing pynput. Keyboard control will not be available.")
    keyboard = None

MIN_ARM_ANGLE = 0
MAX_ARM_ANGLE = 180
ARM_ANGLE_RATE = 2

# Initialize pygame and try to connect to a joystick
# Joystick refers to an entire controller, not just a stick
pygame.init()
pygame.joystick.init()
joystick = None
if pygame.joystick.get_count() > 0:
    # A joystick exists; store it and print its name
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick connected: {joystick.get_name()}")
elif keyboard is not None:
    # No joystick, but keyboard is available
    print("No joystick connected. Using keyboard.")
else:
    # No joystick and no keyboard
    print("No joystick connected. Connect a joystick to drive the robot.")

class Teleop(Command):
    """
    Teleop command for controlling the robot using a joystick or keyboard.
    The controls are:
    - Joystick:
        - Left stick Y axis: forward/backward
        - Left stick X axis: rotation
        - Right stick Y axis: arm up/down
    - Keyboard:
        - W/S: forward/backward
        - A/D: rotation
        - J/K: arm down/up
    initialize() should be called once when the command starts.
    execute() should be called repeatedly while the command is active.
    end() should be called once when the command ends.
    """
    def __init__(self):
        # Initialize all variables
        self.drivetrain = driveTrainInstance
        self.arm = armInstance
        self.listener = None
        self.keys = {'w': False}
        self.lock = threading.Lock()
        self.armAngle = 90
    
    def initialize(self):
        """Runs once when the command starts"""
        # Create keyboard listener if keyboard is available
        # This creates and runs a separate thread that handles key events
        if keyboard is not None:
            self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
            self.listener.start()

    def execute(self):
        """Runs repeatedly while the command is active"""
        # Same code as before to try to connect to the joystick
        global joystick
        if joystick is None:
            # Try to connect to joystick
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                print(f"Joystick connected: {joystick.get_name()}")

        speed = 0
        rotation = 0
        if joystick is not None:
            pygame.event.pump()
            # If the joystick disconnected, set it to None and call execute again
            if not joystick.get_init():
                print("Joystick disconnected.")
                joystick = None
                self.execute()
                return
            # Joystick controls.
            speed = -joystick.get_axis(1)  # Invert Y axis, down is positive
            # Multiply by absolute value to make control less sensitive near 0
            # All speeds are still possible, since speed * abs(speed) covers [-1, 1]
            speed = speed * abs(speed)
            # Same thing for robot rotation and arm movement
            rotation = -joystick.get_axis(0)
            rotation = rotation * abs(rotation)
            armSpeed = -joystick.get_axis(3)
            armSpeed = ARM_ANGLE_RATE * armSpeed * abs(armSpeed)
            # Store the arm angle to make the servo trun to
            self.armAngle += armSpeed
        elif keyboard is not None:
            # Keyboard controls
            # with self.lock prevents this code and the key events from running at the same time
            with self.lock:
                # These speeds are slower than the maximum to make control easier without analog input
                if self.keys.get('w'):
                    speed = 0.5
                if self.keys.get('s'):
                    speed += -0.5
                if self.keys.get('a'):
                    rotation = 0.5
                if self.keys.get('d'):
                    rotation += -0.5
                if self.keys.get('k'):
                    self.armAngle += ARM_ANGLE_RATE/2
                if self.keys.get('j'):
                    self.armAngle -= ARM_ANGLE_RATE/2
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
        if self.listener:
            self.listener.stop()
            self.listener = None
        self.keys = {'w': False}

    def on_press(self, key):
        # Stores the key that was pressed in self.keys
        k = None
        try:
            # Make key lowercase
            k = key.char.lower()
        except AttributeError:
            return
        with self.lock:
            self.keys[k] = True

    def on_release(self, key):
        # Removes the key that was released from self.keys
        k = None
        try:
            # Make key lowercase
            k = key.char.lower()
        except AttributeError:
            return
        with self.lock:
            self.keys[k] = False
