#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MagnetGenerator(Node):
    def __init__(self):
        super().__init__('magnet_generator')
        self.pub = self.create_publisher(Float64MultiArray, '/magnet', 10)
        # 10 Hz timer
        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        msg = Float64MultiArray()
        msg.data = [random.randint(0, 1000) for _ in range(3)]
        self.pub.publish(msg)
        self.get_logger().info(f'Published /magnet: {msg.data}')


def main():
    rclpy.init()
    node = MagnetGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
