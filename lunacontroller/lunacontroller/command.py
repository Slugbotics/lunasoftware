class Command:
    def __init__(self, node):
        self.node = node
    
    """
    Base class for robot commands.
    """
    def initialize(self):
        pass
    def execute(self):
        pass
    def end(self):
        pass
    def isFinished(self) -> bool:
        return False