#!/usr/bin/env python3
"""Stub de navegacion: publica un patron fijo a ritmo constante mientras su modo esta activo.

Un mismo ejecutable sirve para los tres patrones (parametro `pattern`). Publica en
/cmd_vel_<pattern> a `rate_hz` (el firmware corta el movimiento si pasan 500 ms sin comando).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from lawnbot_nav.nav_patterns import make_pattern

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class NavStub(Node):
    def __init__(self):
        super().__init__('nav_stub')

        self.declare_parameter('pattern', 'parallel')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('linear', 0.3)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('straight_s', 3.0)
        self.declare_parameter('turn_s', 1.0)
        self.declare_parameter('min_s', 1.0)
        self.declare_parameter('max_s', 3.0)

        self.name_ = self.get_parameter('pattern').value
        rate = self.get_parameter('rate_hz').value
        linear = self.get_parameter('linear').value
        turn = self.get_parameter('turn').value

        if self.name_ == 'parallel':
            params = dict(linear=linear, turn=turn,
                          straight_s=self.get_parameter('straight_s').value,
                          turn_s=self.get_parameter('turn_s').value)
        elif self.name_ == 'random':
            params = dict(linear=linear, turn=turn,
                          min_s=self.get_parameter('min_s').value,
                          max_s=self.get_parameter('max_s').value)
        else:
            params = dict(linear=linear)
        self.params = params
        self.pattern = make_pattern(self.name_, params)

        self.mode_name = f'auto_{self.name_}'
        self.active = False
        self.t0 = 0.0

        self.pub = self.create_publisher(Twist, f'cmd_vel_{self.name_}', 10)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'stub {self.name_} listo (modo {self.mode_name}, {rate} Hz)')

    def on_mode(self, msg):
        now_active = msg.data == self.mode_name
        if now_active and not self.active:
            self.t0 = time.monotonic()
            self.pattern = make_pattern(self.name_, self.params)
            self.get_logger().info('activado')
        elif self.active and not now_active:
            self.get_logger().info('desactivado')
        self.active = now_active

    def tick(self):
        if not self.active:
            return
        v, w = self.pattern.command(time.monotonic() - self.t0)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
