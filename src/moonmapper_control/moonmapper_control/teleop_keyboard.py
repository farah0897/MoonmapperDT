#!/usr/bin/env python3
"""
Simple keyboard teleop for MoonMapper rover.
Publishes geometry_msgs/Twist to /cmd_vel.
Can be swapped for teleop_twist_keyboard or joystick on real hardware.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Key bindings (same as teleop_twist_keyboard)
KEY_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, -1),
    '.': (-1, 1),
    'k': (0, 0),
}


def get_key(timeout_sec=0.1):
    """Non-blocking key read. Returns None if no key pressed."""
    fd = sys.stdin.fileno()
    if select.select([sys.stdin], [], [], timeout_sec)[0]:
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch
    return None


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_step = 0.2
        self.angular_step = 0.5
        self.linear = 0.0
        self.angular = 0.0

    def run(self):
        self.get_logger().info(
            'Teleop: i/,=forward/back, j/l=left/right, k=stop, q=quit'
        )
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = get_key(0.05)
                if key == 'q':
                    break
                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    self.linear = lin * self.linear_step
                    self.angular = ang * self.angular_step
                msg = Twist()
                msg.linear.x = float(self.linear)
                msg.angular.z = float(self.angular)
                self.pub.publish(msg)
        except KeyboardInterrupt:
            pass
        # Stop
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
