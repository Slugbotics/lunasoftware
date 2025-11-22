import rclpy
from rclpy.node import Node
from lunacontroller.command import Command
from lunacontroller.teleop import Teleop
from std_srvs.srv import Empty

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.commands = {'disabled': Command(self), 'teleop': Teleop(self)}
        self.command = self.commands['disabled']
        self.get_logger().info('MainNode initialized')
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.cancel_service = self.create_service(
            Empty,
            'cancel_command',
            self.cancel_command_callback
        )

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
        if self.command.isFinished():
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
