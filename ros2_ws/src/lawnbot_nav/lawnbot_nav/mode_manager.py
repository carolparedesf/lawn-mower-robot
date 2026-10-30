#!/usr/bin/env python3
"""Decide el modo activo a partir de /robot_mode y lo publica en /nav/active_mode.

/robot_mode lo republica el bridge cada ~1 s (heartbeat). Si pasan `heartbeat_timeout`
segundos sin recibirlo, el modo efectivo pasa a 'idle'.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from lawnbot_nav.mode_logic import DEFAULT_VALID_MODES, resolve_mode

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.declare_parameter('heartbeat_timeout', 3.0)
        self.declare_parameter('valid_modes', list(DEFAULT_VALID_MODES))
        self.declare_parameter('publish_period', 1.0)

        self.timeout = self.get_parameter('heartbeat_timeout').value
        self.valid_modes = tuple(self.get_parameter('valid_modes').value)
        period = self.get_parameter('publish_period').value

        self.requested = 'idle'
        self.last_heartbeat = None
        self.active = None
        self.last_publish = 0.0

        self.pub = self.create_publisher(String, 'nav/active_mode', LATCHED)
        self.create_subscription(String, 'robot_mode', self.on_mode, LATCHED)
        self.create_timer(0.1, self.update)
        self.period = period
        self.get_logger().info(
            f'mode_manager listo (heartbeat_timeout={self.timeout} s)')

    def on_mode(self, msg):
        self.requested = msg.data
        self.last_heartbeat = time.monotonic()

    def update(self):
        now = time.monotonic()
        age = None if self.last_heartbeat is None else now - self.last_heartbeat
        mode = resolve_mode(self.requested, age, self.timeout, self.valid_modes)

        changed = mode != self.active
        if changed:
            if age is not None and age > self.timeout:
                self.get_logger().warn(f'sin heartbeat de /robot_mode ({age:.1f} s): modo -> idle')
            elif self.requested not in self.valid_modes and age is not None:
                self.get_logger().warn(f"modo desconocido '{self.requested}': modo -> idle")
            else:
                self.get_logger().info(f'modo activo: {mode}')
            self.active = mode

        # Publica al cambiar y periodicamente (el bridge lo usa para informar el estado)
        if changed or now - self.last_publish >= self.period:
            self.last_publish = now
            self.pub.publish(String(data=self.active))


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
