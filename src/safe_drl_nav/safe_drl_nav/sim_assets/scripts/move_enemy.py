import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Chaos(Node):
    def __init__(self):
        super().__init__('chaos_node')
        # This talks directly to the red cylinder
        self.pub = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)
        self.dir = 1.0
        self.start = time.time()

    def move(self):
        msg = Twist()
        # Change direction every 3 seconds
        if time.time() - self.start > 3.0:
            self.dir *= -1.0
            self.start = time.time()
            
        msg.linear.y = 0.5 * self.dir # Slide left and right
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Chaos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()