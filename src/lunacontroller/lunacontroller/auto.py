from lunacontroller.command import Command
from lunacontroller.drivetrain import drivetrainInstance
from lunacontroller.arm import armInstance
from lunacontroller.drive import Drive
from lunacontroller.dig import Dig
from lunacontroller.dump import Dump
import lunacontroller.constants as constants

class Auto(Command):
    """
    Autonomously performs a sequence of actions
    """
    def __init__(self, node):
        super().__init__(node)
        self.drivetrain = drivetrainInstance
        self.arm = armInstance
        self.commands = [
            Drive(self.node, constants.DIG_POSE),
            Dig(self.node),
            Drive(self.node, constants.DUMP_POSE),
            Dump(self.node)
        ]
        self.index = 0
    
    def initialize(self):
        self.index = 0
        self.commands[self.index].initialize()
    
    def execute(self):
        current_command = self.commands[self.index]
        current_command.execute()
        if current_command.isFinished():
            current_command.end()
            self.index = (self.index + 1) % len(self.commands)
            self.commands[self.index].initialize()
    
    def end(self):
        self.arm.set_angle(None)
        self.drivetrain.stop()

    def isFinished(self) -> bool:
        return False # This should loop infinitely