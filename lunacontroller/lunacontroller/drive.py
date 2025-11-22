from lunacontroller.command import Command
from lunacontroller.drivetrain import drivetrainInstance
from lunacontroller.arm import armInstance
from lunacontroller.geometry import Pose2d

class Drive(Command):
    """
    Drives to a specified pose
    """
    def __init__(self, node, target_pose):
        super().__init__(node)
        self.drivetrain = drivetrainInstance
        self.arm = armInstance
        self.target_pose = target_pose
    
    def initialize(self):
        self.arm.set_angle(90)
    
    def execute(self):
        # Drives in circles as an example
        self.drivetrain.drive(0.5, 0.5)
    
    def end(self):
        self.drivetrain.stop()

    def isFinished(self) -> bool:
        return self.drivetrain.get_pose().distance(self.target_pose) < 0.05