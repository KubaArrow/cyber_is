#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray


class LineGenerator(Node):
    def __init__(self):
        super().__init__('line_generator')
        self.pub = self.create_publisher(UInt16MultiArray, '/line_detector', 10)
        # 1 Hz timer
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = UInt16MultiArray()
        msg.data = [2200, 3100, 0, 3100, 2800]
        self.get_logger().info(f'Publishing: {msg.data}')
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LineGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
