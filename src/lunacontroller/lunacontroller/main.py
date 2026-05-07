import rclpy
from rclpy.node import Node
from lunacontroller.command import Command
from lunacontroller.teleop import Teleop
from std_srvs.srv import Empty
from lunacontroller.dig import Dig
from lunacontroller.dump import Dump
from lunacontroller.drive import Drive
from lunacontroller.auto import Auto
import lunacontroller.constants as constants
from lunacontroller import arm, drivetrain
from lunacontroller.camera import Camera

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        arm.create_arm(self)
        drivetrain.create_drivetrain(self)
        self.commands = {
            'disabled': Command(self),
            'teleop': Teleop(self),
            'dig': Dig(self),
            'dump': Dump(self),
            'drive': Drive(self, constants.DUMP_POSE),
            'auto': Auto(self)
        }
        self.command = self.commands['disabled']
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.cancel_service = self.create_service(
            Empty,
            'cancel_command',
            self.cancel_command_callback
        )
        self.camera = Camera(self, '/camera', constants.CAMERA_ID)
        self.get_logger().info('MainNode initialized')

    def set_command(self, command):
        self.command.end()
        self.command = command
        self.command.initialize()

    def cancel_command_callback(self, request, response):
        if self.command != self.commands['disabled']:
            self.set_command(self.commands['teleop'])

    def timer_callback(self):
        if not self.is_enabled():
            self.set_command(self.commands['disabled'])
        elif self.command == self.commands['disabled']:
            self.set_command(self.commands['teleop'])
        self.command.execute()
        self.camera.update()
        if type(self.command) == Teleop:
            if self.command.dig_selected():
                self.set_command(self.commands['dig'])
            elif self.command.dump_selected():
                self.set_command(self.commands['dump'])
            elif self.command.drive_selected():
                self.set_command(self.commands['drive'])
            elif self.command.auto_selected():
                self.set_command(self.commands['auto'])
        elif self.command.isFinished():
            self.set_command(self.commands['teleop'])
    
    def is_enabled(self):
        return True  # Placeholder for actual enabled check

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.camera.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
