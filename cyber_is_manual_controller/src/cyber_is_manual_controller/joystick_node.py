import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickTeleop(Node):
    def __init__(self) -> None:
        super().__init__('joystick_teleop')

        # Parameters
        self.declare_parameter('axis_linear', 1)        # typically left stick vertical
        self.declare_parameter('axis_angular', 0)       # typically left stick horizontal or right stick
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('deadman_button', -1)    # -1 disables deadman
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')

        self.axis_linear: int = int(self.get_parameter('axis_linear').value)
        self.axis_angular: int = int(self.get_parameter('axis_angular').value)
        self.scale_linear: float = float(self.get_parameter('scale_linear').value)
        self.scale_angular: float = float(self.get_parameter('scale_angular').value)
        self.deadman_button: int = int(self.get_parameter('deadman_button').value)
        output_topic: str = str(self.get_parameter('output_topic').value)
        joy_topic: str = str(self.get_parameter('joy_topic').value)

        # Publisher with default QoS
        self.pub = self.create_publisher(Twist, output_topic, 10)

        # Subscription with sensor data QoS (best effort, volatile)
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Joy, joy_topic, self.on_joy, sensor_qos)

        self.get_logger().info(
            f"Joystick teleop ready: joy='{joy_topic}' -> twist='{output_topic}', "
            f"axes (lin,ang)=({self.axis_linear},{self.axis_angular}), scales (lin,ang)="
            f"({self.scale_linear},{self.scale_angular}), deadman={self.deadman_button}"
        )

    def on_joy(self, msg: Joy) -> None:
        # Check axis bounds safely
        lin = self._axis_value(msg, self.axis_linear)
        ang = self._axis_value(msg, self.axis_angular)

        # Optional deadman
        if self.deadman_button >= 0:
            if not self._button_pressed(msg, self.deadman_button):
                # When deadman not pressed, publish zero twist to stop
                twist = Twist()
                self.pub.publish(twist)
                return

        twist = Twist()
        twist.linear.x = lin * self.scale_linear
        twist.angular.z = ang * self.scale_angular
        self.pub.publish(twist)

    @staticmethod
    def _axis_value(msg: Joy, index: int) -> float:
        if 0 <= index < len(msg.axes):
            return float(msg.axes[index])
        return 0.0

    @staticmethod
    def _button_pressed(msg: Joy, index: int) -> bool:
        if 0 <= index < len(msg.buttons):
            return msg.buttons[index] != 0
        return False


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
