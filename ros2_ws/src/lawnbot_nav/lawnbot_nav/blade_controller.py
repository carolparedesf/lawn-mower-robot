#!/usr/bin/env python3
"""Controla los motores de corte: publica el duty en /cutter/cmd y /trimmer/cmd (Float32, 0..1).

Esos topics los escucha el nodo cutting_motors (que corta solo a los 0,5 s sin ordenes), por eso se publica a
10 Hz. Cada motor solo recibe duty > 0 mientras TODAS las condiciones se cumplan (ver blade_logic.py); si falla
alguna, o un dato esta viejo, se publica 0 de inmediato. Nunca pasa de max_duty. Los pedidos de encendido llegan
por /cutter/enable y /trimmer/enable (Bool); sin pedido, o al arrancar, esta todo apagado.
"""
import time
from dataclasses import fields

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int32, String

from lawnbot_nav.blade_logic import (BUMPERS, BladeCore, BladeInputs,
                                     BladeParams, tilt_from_quaternion)
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.sensor_cache import SensorCache


class BladeController(Node):
    def __init__(self):
        super().__init__('blade_controller')

        d = self.declare_parameter
        d('rate_hz', 10.0)
        d('input_timeout', 1.0)        # s: un dato mas viejo que esto cuenta como faltante
        d('imu_timeout', 0.5)
        d('cutter_topic', 'cutter/cmd')
        d('trimmer_topic', 'trimmer/cmd')
        d('grass_topic', 'vision/grass_present')
        defaults = BladeParams()
        names = [f.name for f in fields(BladeParams)]
        for name in names:
            d(name, getattr(defaults, name))

        gp = self.get_parameter
        self.core = BladeCore(BladeParams(**{n: gp(n).value for n in names}))
        self.input_timeout = gp('input_timeout').value
        self.imu_timeout = gp('imu_timeout').value

        self.cache = SensorCache()
        self.request_cutter = False
        self.request_trimmer = False
        self.estop = False
        self.last_status = None
        self.last_status_pub = 0.0

        self.cutter_pub = self.create_publisher(Float32, gp('cutter_topic').value, 10)
        self.trimmer_pub = self.create_publisher(Float32, gp('trimmer_topic').value, 10)
        self.status_pub = self.create_publisher(String, 'blade/status', 10)

        # Pedidos y estop: volatiles a proposito (un reinicio del nodo no debe encender nada)
        self.create_subscription(Bool, 'cutter/enable', lambda m: setattr(self, 'request_cutter', bool(m.data)), 10)
        self.create_subscription(Bool, 'trimmer/enable', lambda m: setattr(self, 'request_trimmer', bool(m.data)), 10)
        self.create_subscription(Bool, 'estop', lambda m: setattr(self, 'estop', bool(m.data)), 10)
        self.create_subscription(Bool, 'geofence/inside', lambda m: self.stamp('inside', m.data), 10)
        self.create_subscription(Bool, gp('grass_topic').value, lambda m: self.stamp('grass', m.data), 10)
        self.create_subscription(Int32, 'coverage/cell_passes', lambda m: self.cache.set('passes', m.data, time.monotonic()), 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        for name in BUMPERS:
            self.create_subscription(Bool, name, lambda m, n=name: self.stamp(n, m.data), qos_profile_sensor_data)

        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.get_logger().info(
            f'blade_controller listo: max_duty cuchilla {self.core.p.max_duty_cutter}, trimmer '
            f'{self.core.p.max_duty_trimmer}, pasadas maximas {self.core.p.max_passes}')

    # ── Entradas ─────────────────────────────────────────────

    def stamp(self, name, value):
        self.cache.set(name, 1.0 if value else 0.0, time.monotonic())

    def on_imu(self, msg):
        q = msg.orientation
        self.cache.set('tilt', tilt_from_quaternion(q.x, q.y, q.z, q.w), time.monotonic())

    def flag(self, name, now):
        v = self.cache.get(name, now, self.input_timeout)
        return None if v is None else v > 0.5

    # ── Ciclo ────────────────────────────────────────────────

    def tick(self):
        now = time.monotonic()
        passes = self.cache.get('passes', now, self.input_timeout)
        inputs = BladeInputs(
            request_cutter=self.request_cutter, request_trimmer=self.request_trimmer,
            inside=self.flag('inside', now), grass=self.flag('grass', now),
            passes=None if passes is None else int(passes),
            tilt_deg=self.cache.get('tilt', now, self.imu_timeout),
            bumpers={n: self.flag(n, now) for n in BUMPERS}, estop=self.estop)
        decision = self.core.evaluate(inputs, now)

        # Se publica siempre (los ceros tambien): el watchdog de cutting_motors queda alimentado y apagado
        self.cutter_pub.publish(Float32(data=float(decision.cutter)))
        self.trimmer_pub.publish(Float32(data=float(decision.trimmer)))

        status = decision.status()
        if status != self.last_status:
            self.get_logger().info(status)
        if status != self.last_status or now - self.last_status_pub >= 1.0:
            self.last_status, self.last_status_pub = status, now
            self.status_pub.publish(String(data=status))

    def shutdown_cleanup(self):
        """Apaga los dos motores varias veces antes de cerrar."""
        for _ in range(5):
            try:
                self.cutter_pub.publish(Float32(data=0.0))
                self.trimmer_pub.publish(Float32(data=0.0))
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    spin_with_cleanup(BladeController, args)


if __name__ == '__main__':
    main()
