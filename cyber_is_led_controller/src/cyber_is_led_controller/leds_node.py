#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import UInt8MultiArray, String

from .leds_controller import LedsController


class LedsControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('leds_controller')

        # Declare parameters with defaults
        self.declare_parameter('leds_topic', '/leds')
        self.declare_parameter('state_topic', '/leds_mode')
        self.declare_parameter('side_leds', 9)
        self.declare_parameter('front_leds', 5)
        self.declare_parameter('frequency', 12)

        leds_topic: str = self.get_parameter('leds_topic').value
        state_topic: str = self.get_parameter('state_topic').value
        side_leds: int = int(self.get_parameter('side_leds').value)
        front_leds: int = int(self.get_parameter('front_leds').value)
        frequency: float = float(self.get_parameter('frequency').value)

        self.get_logger().info(
            f"Starting with leds_topic={leds_topic}, state_topic={state_topic}, "
            f"side_leds={side_leds}, front_leds={front_leds}, freq={frequency}Hz"
        )

        # Controller
        self.ctrl = LedsController(side_leds=side_leds, front_leds=front_leds)

        qos = QoSProfile(depth=10)
        self.leds_pub = self.create_publisher(UInt8MultiArray, leds_topic, qos)
        self.state_sub = self.create_subscription(String, state_topic, self._mode_cb, qos)

        # Timer for periodic publish
        self._timer_period = 1.0 / max(frequency, 0.1)
        self.timer = self.create_timer(self._timer_period, self._on_timer)

        # Parameter change callback (allow updating frequency at runtime)
        self.add_on_set_parameters_callback(self._on_set_params)

    # ------------------------ callbacks ------------------------
    def _mode_cb(self, msg: String) -> None:
        try:
            self.ctrl.set_mode(msg.data)
            self.get_logger().info(f"[Subscriber] Received mode: {msg.data}")
        except Exception as e:
            self.get_logger().warning(f"Bad mode '{msg.data}': {e}")

    def _on_timer(self) -> None:
        self.ctrl.tick()
        out = UInt8MultiArray()
        out.data = self.ctrl.get_leds()
        self.leds_pub.publish(out)

    def _on_set_params(self, params: List[Parameter]):
        new_freq = None
        for p in params:
            if p.name == 'frequency' and p.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                new_freq = float(p.value)
        if new_freq is not None and new_freq > 0.0:
            try:
                self.timer.cancel()
            except Exception:
                pass
            self._timer_period = 1.0 / new_freq
            self.timer = self.create_timer(self._timer_period, self._on_timer)
            self.get_logger().info(f"Updated frequency to {new_freq} Hz")
        return SetParametersResult(successful=True)


def main() -> None:
    rclpy.init()
    node = LedsControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
