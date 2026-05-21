"""Moving obstacle publisher for lab worlds (/obstacle/cmd_vel)."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Chaos(Node):
    def __init__(self):
        super().__init__("chaos_node")
        self.pub = self.create_publisher(Twist, "/obstacle/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.move)
        self.dir = 1.0
        self.start = time.time()

    def move(self):
        msg = Twist()
        if time.time() - self.start > 3.0:
            self.dir *= -1.0
            self.start = time.time()
        msg.linear.y = 0.5 * self.dir
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Chaos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
