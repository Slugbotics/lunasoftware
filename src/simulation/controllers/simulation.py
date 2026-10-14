#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

from controller import Robot, Keyboard

TIME_STEP_MS = 32

MAX_WHEEL_SPEED = 25.0  # radians per second


class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        # ROS topics
        self.keyboard_pub = self.create_publisher(String, '/keyboard', 10)
        self.left_sub = self.create_subscription(Float64, '/left_speed', self._left_cb, 10)
        self.right_sub = self.create_subscription(Float64, '/right_speed', self._right_cb, 10)

        # latest commanded speeds (float)
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Webots devices
        self.robot = Robot()
        self.motors = []
        self.turn_motors = []

        time_step = int(self.robot.getBasicTimeStep()) if self.robot.getBasicTimeStep() else TIME_STEP_MS
        self.keyboard = Keyboard()
        self.keyboard.enable(time_step)

        motor_names = ["FrontLeftWheel", "FrontRightWheel", "BackLeftWheel", "BackRightWheel"]
        arm_names = ["FrontLeftArm", "FrontRightArm", "BackLeftArm", "BackRightArm"]

        for name in motor_names:
            m = self.robot.getDevice(name)
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors.append(m)

        for name in arm_names:
            t = self.robot.getDevice(name)
            t.setPosition(0.0)
            self.turn_motors.append(t)

        # periodic timer: step Webots and apply speeds
        self.timer = self.create_timer(TIME_STEP_MS / 1000.0, self._timer_cb)

    def _left_cb(self, msg: Float64):
        try:
            self.left_speed = float(msg.data)
        except Exception:
            pass

    def _right_cb(self, msg: Float64):
        try:
            self.right_speed = float(msg.data)
        except Exception:
            pass

    def publish_keys(self, keys: str):
        m = String()
        m.data = keys
        self.keyboard_pub.publish(m)

    def _timer_cb(self):
        # handle keyboard polling and publish keys
        if self.keyboard is not None:
            keys = ''
            k = self.keyboard.getKey()
            while k != -1:
                if k == Keyboard.UP:
                    keys += 'w'
                elif k == Keyboard.DOWN:
                    keys += 's'
                elif k == Keyboard.LEFT:
                    keys += 'a'
                elif k == Keyboard.RIGHT:
                    keys += 'd'
                else:
                    try:
                        ch = chr(k).lower()
                        if ch.isalnum() or ch == ' ':
                            keys += ch
                    except Exception:
                        pass
                k = self.keyboard.getKey()
            self.publish_keys(keys)

        # apply commanded left/right speeds to motors
        # left -> FrontLeft + BackLeft (indices 0,2), right -> FrontRight + BackRight (1,3)
        for i, m in enumerate(self.motors):
            if i in (0, 2):
                m.setVelocity(-self.left_speed * MAX_WHEEL_SPEED)
            else:
                m.setVelocity(-self.right_speed * MAX_WHEEL_SPEED)

        # keep turn motors at zero in this outline
        for t in self.turn_motors:
            t.setPosition(0.0)

        # step the Webots world if running
        if self.robot.step(TIME_STEP_MS) == -1:
            # if Webots requested termination, shut down ROS
            raise KeyboardInterrupt

def main():
    rclpy.init()
    node = SimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
