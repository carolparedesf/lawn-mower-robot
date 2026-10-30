#!/usr/bin/env python3
"""Publica en /robot/events (String) los eventos del robot: lifted, cutting_started, navigation_started,
perimeter_started y low_battery (ver events_logic.py). Cada mensaje es el nombre del evento, una vez por flanco.

Lo escuchan el bridge (lo pasa a la app como `alert`) y quien maneje el buzzer. El nodo no mueve nada ni toca el firmware.
"""
import time
from dataclasses import fields

from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

from lawnbot_nav.blade_logic import BUMPERS, LIFT_PAIRS
from lawnbot_nav.events_logic import EventCore, EventParams
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.sensor_cache import SensorCache

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RobotEvents(Node):
    def __init__(self):
        super().__init__('robot_events')
        d = self.declare_parameter
        d('rate_hz', 10.0)
        d('input_timeout', 1.0)         # s: bumpers y /cutter/cmd mas viejos que esto no cuentan
        d('mode_timeout', 3.0)          # s: sin /nav/active_mode reciente el modo es 'idle'
        d('battery_timeout', 5.0)       # s
        d('battery_min_valid_v', 1.0)   # el firmware publica 0 V sin INA228
        defaults = EventParams()
        names = [f.name for f in fields(EventParams)]
        for n in names:
            d(n, getattr(defaults, n))
        gp = self.get_parameter
        self.core = EventCore(EventParams(**{n: gp(n).value for n in names}))
        self.input_timeout, self.mode_timeout = gp('input_timeout').value, gp('mode_timeout').value
        self.battery_timeout, self.min_v = gp('battery_timeout').value, gp('battery_min_valid_v').value

        self.cache = SensorCache()
        self.mode, self.mode_time = None, 0.0
        self.pub = self.create_publisher(String, 'robot/events', 10)

        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.create_subscription(BatteryState, 'battery/state', self.on_battery, qos_profile_sensor_data)
        for topic in ('cutter/cmd', 'trimmer/cmd'):
            self.create_subscription(Float32, topic, lambda m, t=topic: self.cache.set(t, m.data, time.monotonic()), 10)
        for name in BUMPERS:
            self.create_subscription(Bool, name, lambda m, n=name: self.cache.set(n, 1.0 if m.data else 0.0, time.monotonic()),
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.get_logger().info(f'robot_events listo: low_battery < {self.core.p.low_battery_threshold} %')

    def on_mode(self, msg):
        self.mode, self.mode_time = msg.data, time.monotonic()

    def on_battery(self, msg):
        pct = msg.percentage * 100.0
        if msg.voltage > self.min_v and 0.0 <= pct <= 100.0:
            self.cache.set('battery', pct, time.monotonic())

    def tick(self):
        now = time.monotonic()
        mode = self.mode if self.mode is not None and now - self.mode_time <= self.mode_timeout else 'idle'
        cutting = any((self.cache.get(t, now, self.input_timeout) or 0.0) > 0.0 for t in ('cutter/cmd', 'trimmer/cmd'))
        b = {n: (self.cache.get(n, now, self.input_timeout) or 0.0) > 0.5 for n in BUMPERS}
        lifted = any(b[x] and b[y] for x, y in LIFT_PAIRS)
        pct = self.cache.get('battery', now, self.battery_timeout)
        for event in self.core.update(now, mode, cutting, lifted, pct):
            self.get_logger().info(f'evento: {event}')
            self.pub.publish(String(data=event))


def main(args=None):
    spin_with_cleanup(RobotEvents, args)


if __name__ == '__main__':
    main()
