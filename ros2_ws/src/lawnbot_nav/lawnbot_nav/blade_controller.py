#!/usr/bin/env python3
"""Controla los motores de corte: publica el duty en /cutter/cmd y /trimmer/cmd (Float32, 0..1).

Esos topics los escucha el nodo cutting_motors (que corta solo a los 0,5 s sin ordenes), por eso se publica a
10 Hz. Cada motor solo recibe duty > 0 mientras TODAS las condiciones se cumplan (ver blade_logic.py); si falla
alguna, o un dato esta viejo, se publica 0 de inmediato. Nunca pasa de max_duty.

Los pedidos llegan por /cutter/enable, /trimmer/enable y /blade/force_cut (Bool). El bridge los republica a 2-5 Hz
mientras la app los mantenga: un `true` vale solo `enable_timeout` (2 s) desde su ultimo refresco, un `false`
apaga de inmediato, y sin pedidos (o al arrancar) esta todo apagado. force_cut anula SOLO la condicion de pasto.

La inclinacion llega ya calculada por /imu/tilt_deg (la publica imu_watchdog a 10 Hz con cada muestra nueva del IMU):
suscribirse a /imu/data (entonces a 50 Hz) costaba ~16 % de un nucleo de la Pi solo en recibir. Si imu_watchdog no corre o el
IMU se pierde, el dato envejece y a los imu_timeout s la cuchilla se apaga, como antes.
"""
import time
from dataclasses import fields

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Int32, String

from lawnbot_nav.blade_logic import BUMPERS, BladeCore, BladeInputs, BladeParams
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.sensor_cache import SensorCache


class BladeController(Node):
    def __init__(self):
        super().__init__('blade_controller')

        d = self.declare_parameter
        d('rate_hz', 10.0)
        d('input_timeout', 1.0)        # s: un dato mas viejo que esto cuenta como faltante
        d('imu_timeout', 0.5)
        d('enable_timeout', 2.0)       # s: un pedido sin refresco por este tiempo vence
        d('cutter_topic', 'cutter/cmd')
        d('trimmer_topic', 'trimmer/cmd')
        d('grass_topic', 'vision/grass_present')
        d('tilt_topic', 'imu/tilt_deg')    # grados; lo publica imu_watchdog
        defaults = BladeParams()
        names = [f.name for f in fields(BladeParams)]
        for name in names:
            d(name, getattr(defaults, name))

        gp = self.get_parameter
        self.core = BladeCore(BladeParams(**{n: gp(n).value for n in names}))
        self.input_timeout = gp('input_timeout').value
        self.imu_timeout = gp('imu_timeout').value
        self.enable_timeout = gp('enable_timeout').value

        self.cache = SensorCache()
        self.estop = False
        self.last_status = None
        self.last_status_pub = 0.0

        self.cutter_pub = self.create_publisher(Float32, gp('cutter_topic').value, 10)
        self.trimmer_pub = self.create_publisher(Float32, gp('trimmer_topic').value, 10)
        self.status_pub = self.create_publisher(String, 'blade/status', 10)

        # Pedidos y estop: volatiles a proposito (un reinicio del nodo no debe encender nada)
        for name, topic in (('cutter', 'cutter/enable'), ('trimmer', 'trimmer/enable'), ('force_cut', 'blade/force_cut')):
            self.create_subscription(Bool, topic, lambda m, n=name: self.stamp(f'req_{n}', m.data), 10)
        self.create_subscription(Bool, 'estop', lambda m: setattr(self, 'estop', bool(m.data)), 10)
        self.create_subscription(Bool, 'geofence/inside', lambda m: self.stamp('inside', m.data), 10)
        self.create_subscription(Bool, gp('grass_topic').value, lambda m: self.stamp('grass', m.data), 10)
        self.create_subscription(Int32, 'coverage/cell_passes', lambda m: self.cache.set('passes', m.data, time.monotonic()), 10)
        self.create_subscription(Float32, gp('tilt_topic').value,
                                 lambda m: self.cache.set('tilt', m.data, time.monotonic()), 10)
        for name in BUMPERS:
            self.create_subscription(Bool, name, lambda m, n=name: self.stamp(n, m.data), qos_profile_sensor_data)

        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.get_logger().info(
            f'blade_controller listo: max_duty cuchilla {self.core.p.max_duty_cutter}, trimmer '
            f'{self.core.p.max_duty_trimmer}, pasadas maximas {self.core.p.max_passes}')

    # ── Entradas ─────────────────────────────────────────────

    def stamp(self, name, value):
        self.cache.set(name, 1.0 if value else 0.0, time.monotonic())

    def requested(self, name, now):
        """Pedido vigente: el ultimo mensaje es true y se refresco dentro de enable_timeout."""
        return bool(self.flag(f'req_{name}', now, self.enable_timeout))

    def flag(self, name, now, timeout=None):
        v = self.cache.get(name, now, self.input_timeout if timeout is None else timeout)
        return None if v is None else v > 0.5

    # ── Ciclo ────────────────────────────────────────────────

    def tick(self):
        now = time.monotonic()
        passes = self.cache.get('passes', now, self.input_timeout)
        inputs = BladeInputs(
            request_cutter=self.requested('cutter', now), request_trimmer=self.requested('trimmer', now),
            force_cut=self.requested('force_cut', now),
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
