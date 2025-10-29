#from command import Command
from driveTrain import Drivetrain
from pynput import keyboard  # pip install pynput in base terminal
'''
class Teleop(Command):
    def __init__(self):
        self.drivetrain = Drivetrain()

    def initialize(self):
        print("[TELEOP] Initialized")

    def execute(self):
        speed = 0.0
        rotation = 0.0

        # Basic WASD-style control
        if keyboard.is_pressed('w'):
            speed = 1.0
        elif keyboard.is_pressed('s'):
            speed = -1.0
        if keyboard.is_pressed('a'):
            rotation = -0.5
        elif keyboard.is_pressed('d'):
            rotation = 0.5

        self.drivetrain.drive(speed, rotation)

    def end(self):
        self.drivetrain.stop()
        print("[TELEOP] Ended")

    def isFinished(self):
        return False  # stays active until state changes

'''
class Teleop:
    def __init__(self):
        self.drivetrain = Drivetrain()
        self.speed = 0
        self.rotation = 0

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.speed = 1
            elif key.char == 's':
                self.speed = -1
            elif key.char == 'a':
                self.rotation = -0.5
            elif key.char == 'd':
                self.rotation = 0.5
        except AttributeError:
            pass
        self.drivetrain.drive(self.speed, self.rotation)

    def on_release(self, key):
        if key.char in ['w', 's', 'a', 'd']:
            self.speed = 0
            self.rotation = 0
            self.drivetrain.stop()

    def run(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()