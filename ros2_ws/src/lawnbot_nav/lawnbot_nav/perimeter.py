#!/usr/bin/env python3
"""Seguimiento de borde en /cmd_vel_perimeter. Solo actua con el modo activo auto_perimeter.

Modo base: por contacto con el bumper lateral (B3 izquierda, B4 derecha). Al tocar retrocede derecho un
poco y se aleja girando. El sonar lateral es un refinamiento opcional (use_side_sonar) que solo se usa con
side_sonar_angle_deg valido. Si pierde el borde o se traba, se detiene y avisa por /nav/alert_in
(perimeter_lost, perimeter_blocked). La salida pasa por twist_mux y safety_gate.
"""
import time
from dataclasses import fields

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.perimeter_logic import (PerimeterCore, PerimeterInputs,
                                         PerimeterParams)
from lawnbot_nav.sensor_cache import (BUMPER_BACK, BUMPER_FRONT, SONARS,
                                      SensorCache)

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

MODE = 'auto_perimeter'
MODE_STALE_S = 3.0

PARAM_FIELDS = tuple(f.name for f in fields(PerimeterParams))


class Perimeter(Node):
    def __init__(self):
        super().__init__('perimeter')

        d = self.declare_parameter
        d('rate_hz', 20.0)
        d('sensor_timeout', 1.0)
        d('sonar_topic_prefix', 'sonar')       # 'sonar_filtered' con el filtro de sonares (nav.launch.py)
        d('output_topic', 'cmd_vel_perimeter')
        defaults = PerimeterParams()
        for field in PARAM_FIELDS:
            d(field, getattr(defaults, field))

        gp = self.get_parameter
        params = PerimeterParams(**{f: gp(f).value for f in PARAM_FIELDS})
        if params.side not in ('left', 'right'):
            raise ValueError("side debe ser 'left' o 'right'")
        self.core = PerimeterCore(params)
        if params.use_side_sonar and not self.core.sonar_enabled:
            self.get_logger().warn(
                f'use_side_sonar con side_sonar_angle_deg={params.side_sonar_angle_deg} (minimo 20): '
                'se ignora el sonar lateral y se sigue por bumper')
        self.sensor_timeout = gp('sensor_timeout').value
        self.side_sonar = f'sonar/{params.side}'
        self.side_bumper = 'bumper/B3_left' if params.side == 'left' else 'bumper/B4_right'

        self.cache = SensorCache()
        self.active_mode = None
        self.mode_time = 0.0
        self.was_active = False
        self.zeros_pending = 0
        self.alert_sent = None
        self.last_alert_pub = 0.0

        self.pub = self.create_publisher(Twist, gp('output_topic').value, 10)
        self.alert_pub = self.create_publisher(String, 'nav/alert_in', 10)
        for s in SONARS:
            self.create_subscription(Range, f'{gp("sonar_topic_prefix").value}/{s}', lambda m, s=s: self.on_sonar(s, m),
                                     qos_profile_sensor_data)
        for name in BUMPER_FRONT + ('bumper/B3_left', 'bumper/B4_right', BUMPER_BACK):
            self.create_subscription(Bool, name, lambda m, n=name: self.on_bumper(n, m),
                                     qos_profile_sensor_data)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.get_logger().info(
            f'perimeter listo (modo {MODE}, pared a la {params.side}, sonar lateral a '
            f'{params.side_sonar_angle_deg} grados)')

    def on_sonar(self, name, msg):
        self.cache.set(f'sonar/{name}', msg.range, time.monotonic())

    def on_bumper(self, name, msg):
        self.cache.set(name, 1.0 if msg.data else 0.0, time.monotonic())

    def on_mode(self, msg):
        self.active_mode = msg.data
        self.mode_time = time.monotonic()

    def mode_active(self, now):
        return self.active_mode == MODE and now - self.mode_time <= MODE_STALE_S

    def build_inputs(self, now):
        t = self.sensor_timeout
        use_sonar = self.core.sonar_enabled
        needed = (['sonar/center'] if self.core.p.use_center_sonar else []) + \
                 ([self.side_sonar] if use_sonar else [])
        sonar_ok = all(self.cache.get(n, now, t) is not None for n in needed)
        return PerimeterInputs(
            sensors_ok=sonar_ok and self.cache.fresh(BUMPER_FRONT, now, t),
            side_range=self.cache.get(self.side_sonar, now, t) if use_sonar else None,
            center_range=self.cache.get('sonar/center', now, t),
            side_bumper=self.cache.pressed(self.side_bumper, now, t),
            front_bumper=self.cache.pressed(BUMPER_FRONT, now, t),
            back_bumper=self.cache.pressed(BUMPER_BACK, now, t))

    def tick(self):
        now = time.monotonic()
        active = self.mode_active(now)
        if active and not self.was_active:
            self.core.reset(now)
            self.get_logger().info('activado')
        if not active and self.was_active:
            self.zeros_pending = 3
            self.get_logger().info('desactivado')
            self.set_alert(None, now, force=True)
        self.was_active = active

        if active:
            v, w = self.core.update(self.build_inputs(now), now)
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.pub.publish(cmd)
            self.set_alert(self.core.alert, now)
        elif self.zeros_pending > 0:
            self.zeros_pending -= 1
            self.pub.publish(Twist())

    def set_alert(self, alert, now, force=False):
        if alert == self.alert_sent and not (alert and now - self.last_alert_pub >= 1.0) and not force:
            return
        if alert != self.alert_sent and alert:
            self.get_logger().warn(f'alerta: {alert}')
        self.alert_sent = alert
        self.last_alert_pub = now
        self.alert_pub.publish(String(data=f'perimeter:{alert or "none"}'))

    def shutdown_cleanup(self):
        for _ in range(5):
            try:
                self.pub.publish(Twist())
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    spin_with_cleanup(Perimeter, args)


if __name__ == '__main__':
    main()
