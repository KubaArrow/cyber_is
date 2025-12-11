#!/usr/bin/env python3
"""
ROS 2 odometry plotter for /low_level_odom.
"""
from collections import deque
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

WINDOW_SIZE = 10.0  # seconds


class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/low_level_odom',
            self.odom_callback,
            10,
        )
        self._start_time = time.monotonic()
        self._time_buffer = deque()
        self._linear_buffer = deque()
        self._angular_buffer = deque()

    def odom_callback(self, msg: Odometry) -> None:
        now = time.monotonic() - self._start_time
        self._time_buffer.append(now)
        self._linear_buffer.append(msg.twist.twist.linear.x)
        self._angular_buffer.append(msg.twist.twist.angular.z)

        while self._time_buffer and (now - self._time_buffer[0]) > WINDOW_SIZE:
            self._time_buffer.popleft()
            self._linear_buffer.popleft()
            self._angular_buffer.popleft()

    def has_data(self) -> bool:
        return bool(self._time_buffer)

    def data(self):
        return list(self._time_buffer), list(self._linear_buffer), list(self._angular_buffer)


def main():
    rclpy.init()
    node = OdomPlotter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

    def update_plot(_frame):
        if not node.has_data():
            return
        time_vals, linear_vals, angular_vals = node.data()
        ax1.clear()
        ax2.clear()
        ax1.plot(time_vals, linear_vals, label='Linear Velocity (x)', color='green')
        ax2.plot(time_vals, angular_vals, label='Angular Velocity (z)', color='orange')
        ax1.set_ylabel("Linear Velocity [m/s]")
        ax2.set_ylabel("Angular Velocity [rad/s]")
        ax2.set_xlabel("Time [s]")
        ax1.legend()
        ax2.legend()
        ax1.grid()
        ax2.grid()

    FuncAnimation(fig, update_plot, interval=100)
    plt.tight_layout()

    try:
        plt.show()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
