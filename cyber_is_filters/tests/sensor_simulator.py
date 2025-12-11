#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, UInt16MultiArray
from visualization_msgs.msg import Marker


class LineDetectorSimulator(Node):
    def __init__(self):
        super().__init__('line_detector_simulator')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.pub_line = self.create_publisher(UInt16MultiArray, '/line_detector', 10)
        self.pub_magnet = self.create_publisher(Float64MultiArray, '/magnet', 10)
        self.pub_marker = self.create_publisher(Marker, '/line_marker', 10)

        self.w = 3.03 / 2.0
        self.h = 6.06
        self.w2 = 0.10
        self.h2 = 0.60
        self.thresh = 0.015

        self._create_line_marker()
        self._create_magnet_marker()

        self.create_timer(1.0, self._timer_publish_markers)
        self.get_logger().info("Line + Magnet Simulator started")

    def _create_line_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.ns = 'line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [
            Point(x=-self.w, y=0.0, z=0.0),
            Point(x=self.w, y=0.0, z=0.0),
            Point(x=self.w, y=self.h, z=0.0),
            Point(x=-self.w, y=self.h, z=0.0),
            Point(x=-self.w, y=0.0, z=0.0),
        ]
        marker.pose.orientation.z = -0.7071068
        marker.pose.orientation.w = 0.7071068
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_line = marker

    def _create_magnet_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.ns = 'magnet'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        x0 = self.w - self.w2
        y0 = self.h - self.h2
        marker.points = [
            Point(x=x0, y=y0, z=0.0),
            Point(x=x0 + self.w2, y=y0, z=0.0),
            Point(x=x0 + self.w2, y=y0 + self.h2, z=0.0),
            Point(x=x0, y=y0 + self.h2, z=0.0),
            Point(x=x0, y=y0, z=0.0),
        ]
        marker.pose.orientation.z = -0.7071068
        marker.pose.orientation.w = 0.7071068
        marker.scale.x = 0.03
        marker.color.g = 1.0
        marker.color.a = 1.0
        self.marker_magnet = marker

    def _timer_publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.marker_line.header.stamp = now
        self.marker_magnet.header.stamp = now
        self.pub_marker.publish(self.marker_line)
        self.pub_marker.publish(self.marker_magnet)

    def odom_callback(self, msg: Odometry):
        xg = msg.pose.pose.position.x
        yg = msg.pose.pose.position.y
        lx = -yg
        ly = xg

        on_line = (
            (abs(ly - 0.0) <= self.thresh and abs(lx) <= self.w) or
            (abs(ly - self.h) <= self.thresh and abs(lx) <= self.w) or
            (abs(lx + self.w) <= self.thresh and 0.0 <= ly <= self.h) or
            (abs(lx - self.w) <= self.thresh and 0.0 <= ly <= self.h)
        )
        if on_line:
            arr = UInt16MultiArray()
            arr.data = [4000] * 5
            self.pub_line.publish(arr)

        if (self.w - self.w2) <= lx <= self.w and (self.h - self.h2) <= ly <= self.h:
            mag = Float64MultiArray()
            mag.data = [10000.0] * 3
            self.pub_magnet.publish(mag)


def main():
    rclpy.init()
    node = LineDetectorSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
