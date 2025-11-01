from command import Command
from driveTrain import driveTrainInstance
from arm import armInstance
import pygame

try:
    from pynput import keyboard
except ImportError:
    print("Error importing pynput. Keyboard control will not be available.")
    keyboard = None

minArmAngle = 0
maxArmAngle = 180
armAngleRate = 2

pygame.init()
pygame.joystick.init()
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick connected: {joystick.get_name()}")
else:
    print("No joystick connected. Using keyboard.")

class Teleop(Command):
    def __init__(self):
        self.drivetrain = driveTrainInstance
        self.arm = armInstance
        self.listener = None
        self.keys = {'w': False}
        self.armAngle = 90
    
    def initialize(self):
        if joystick is None and keyboard is not None:
            self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
            self.listener.start()

    def execute(self):
        speed = 0
        rotation = 0
        if joystick is not None:
            pygame.event.pump()
            speed = -joystick.get_axis(1)  # Invert Y axis, down is positive
            speed = speed * abs(speed)
            rotation = -joystick.get_axis(0)
            rotation = rotation * abs(rotation)
            armSpeed = -joystick.get_axis(3)
            armSpeed = armAngleRate * armSpeed * abs(armSpeed)
            self.armAngle += armSpeed
        elif keyboard is not None:
            if self.keys['w']:
                speed = 0.5
            if self.keys['s']:
                speed += -0.5
            if self.keys['a']:
                rotation = 0.5
            if self.keys['d']:
                rotation += -0.5
            if self.keys['k']:
                self.armAngle += armAngleRate/2
            if self.keys['j']:
                self.armAngle -= armAngleRate/2
        self.armAngle = max(minArmAngle, min(maxArmAngle, self.armAngle))
        self.arm.set_angle(self.armAngle)
        self.drivetrain.drive(speed, rotation)
    
    def end(self):
        self.drivetrain.stop()
        if self.listener:
            self.listener.stop()
            self.listener = None
        self.keys = {'w': False}

    def on_press(self, key):
        try:
            # Make key lowercase
            self.keys[key.char.lower()] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.keys[key.char.lower()] = False
        except AttributeError:
            pass
