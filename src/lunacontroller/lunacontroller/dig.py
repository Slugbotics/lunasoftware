from lunacontroller.command import Command
from lunacontroller.drivetrain import drivetrainInstance
from lunacontroller.arm import armInstance
import time

class Dig(Command):
    """
    Autonomously digs
    """
    def __init__(self, node):
        super().__init__(node)
        self.drivetrain = drivetrainInstance
        self.arm = armInstance
    
    def initialize(self):
        self.startTime = time.time()
    
    def execute(self):
        # This is an example
        # This could either be timed or based on sensors (once they exist)
        elapsed = time.time() - self.startTime
        if elapsed < 1.0:
            self.arm.set_angle(90)
            self.drivetrain.stop()
        elif elapsed < 4.0:
            self.arm.set_angle(30)
            self.drivetrain.drive(0.5, 0)
        else:
            self.arm.set_angle(90)
            self.drivetrain.stop()
    
    def end(self):
        self.arm.set_angle(None)
        self.drivetrain.stop()

    def isFinished(self) -> bool:
        return (time.time() - self.startTime) >= 6.0