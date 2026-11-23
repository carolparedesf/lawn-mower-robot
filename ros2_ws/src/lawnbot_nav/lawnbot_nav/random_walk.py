#!/usr/bin/env python3
"""Caminata aleatoria en /cmd_vel_random. Solo actua con el modo activo auto_random.

AVANZAR -> (obstaculo por sonar, bumper o borde del cerco) -> RETROCEDER -> GIRAR al azar -> AVANZAR.
Publica a 20 Hz mientras el modo esta activo y cero al desactivarse o apagarse. La salida pasa por
twist_mux y safety_gate: este nodo nunca publica en /cmd_vel.

Inactivo no recibe sonares, bumpers, distancia al cerco ni TF, y no corre el timer de 20 Hz (CPU de la Pi):
los crea al llegar auto_random por /nav/active_mode (ver active_resources.py).
"""
import math
import time
from dataclasses import fields

from geometry_msgs.msg import PolygonStamped, Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer

from lawnbot_nav.active_resources import ActiveResources, Sub
from lawnbot_nav.geofence_logic import polygon_centroid
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.random_walk_logic import (RandomWalkCore, RandomWalkInputs,
                                           RandomWalkParams)
from lawnbot_nav.sensor_cache import (BUMPER_BACK, BUMPER_FRONT, SONARS,
                                      SensorCache)
from lawnbot_nav.tf_utils import lookup_pose

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

MODE = 'auto_random'
MODE_STALE_S = 3.0
SIDE_BUMPERS = ('bumper/B3_left', 'bumper/B4_right')


class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk')

        d = self.declare_parameter
        d('rate_hz', 20.0)
        d('seed', -1)                          # -1: semilla al azar; otro valor: reproducible
        d('sensor_timeout', 1.0)
        d('sonar_topic_prefix', 'sonar')       # 'sonar_filtered' con el filtro de sonares (nav.launch.py)
        d('output_topic', 'cmd_vel_random')
        for s in SONARS:
            d(f'sonar_{s}_enabled', True)
        d('frame_id', 'map')                   # frame del cerco, para girar hacia su centroide
        d('base_frame', 'base_footprint')
        d('pose_timeout', 0.5)
        defaults = RandomWalkParams()
        names = [f.name for f in fields(RandomWalkParams)]
        for name in names:
            d(name, getattr(defaults, name))

        gp = self.get_parameter
        params = RandomWalkParams(**{n: gp(n).value for n in names})
        seed = gp('seed').value
        self.core = RandomWalkCore(params, None if seed < 0 else seed)
        self.sensor_timeout = gp('sensor_timeout').value
        self.sonars = [s for s in SONARS if gp(f'sonar_{s}_enabled').value]

        self.frame_id = gp('frame_id').value
        self.base_frame = gp('base_frame').value
        self.pose_timeout = gp('pose_timeout').value
        self.centroid = None
        self.tf_buffer = Buffer()

        self.cache = SensorCache()
        self.active_mode = None
        self.mode_time = 0.0
        self.geofence_ok = False
        self.was_active = False
        self.zeros_pending = 0
        self.alert_sent = None
        self.last_alert_pub = 0.0

        self.pub = self.create_publisher(Twist, gp('output_topic').value, 10)
        self.alert_pub = self.create_publisher(String, 'nav/alert_in', 10)
        # Solo mientras auto_random esta activo: sensores, distancia al cerco, TF y el timer de 20 Hz
        fast = [Sub(Range, f'{gp("sonar_topic_prefix").value}/{s}', lambda m, s=s: self.on_sonar(s, m),
                    qos_profile_sensor_data) for s in self.sonars]
        fast += [Sub(Bool, name, lambda m, n=name: self.on_bumper(n, m), qos_profile_sensor_data)
                 for name in BUMPER_FRONT + SIDE_BUMPERS + (BUMPER_BACK,)]
        fast.append(Sub(Float32, 'geofence/distance_to_edge', self.on_edge, 10))
        # La TF solo se usa para girar hacia el centroide al llegar al borde: no se espera por ella al activarse
        self.res = ActiveResources(self, 1.0 / gp('rate_hz').value, self.tick, fast, tf_buffer=self.tf_buffer)
        # Siempre: latcheados y de baja frecuencia
        self.create_subscription(String, 'geofence/status', self.on_geofence_status, LATCHED)
        self.create_subscription(PolygonStamped, 'geofence/polygon', self.on_polygon, LATCHED)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.get_logger().info(
            f'random_walk listo (modo {MODE}, semilla {"al azar" if seed < 0 else seed}, '
            f'cerco requerido: {params.require_geofence})')

    # ── Entradas ─────────────────────────────────────────────

    def on_sonar(self, name, msg):
        self.cache.set(f'sonar/{name}', msg.range, time.monotonic())

    def on_bumper(self, name, msg):
        self.cache.set(name, 1.0 if msg.data else 0.0, time.monotonic())

    def on_edge(self, msg):
        self.cache.set('edge', msg.data, time.monotonic())

    def on_geofence_status(self, msg):
        self.geofence_ok = msg.data == 'OK'

    def on_polygon(self, msg):
        points = [(p.x, p.y) for p in msg.polygon.points]
        frame_ok = (not msg.header.frame_id) or msg.header.frame_id == self.frame_id
        self.centroid = polygon_centroid(points) if len(points) >= 3 and frame_ok else None

    def on_mode(self, msg):
        self.active_mode = msg.data
        self.mode_time = time.monotonic()
        if msg.data == MODE:
            self.res.start()

    def interior_angle(self):
        """Giro que deja al robot mirando al centroide del cerco, o None si falta el cerco o la pose."""
        if self.centroid is None:
            return None
        pose = lookup_pose(self, self.tf_buffer, self.frame_id, self.base_frame, self.pose_timeout)
        if pose is None:
            return None
        heading = math.atan2(self.centroid[1] - pose.y, self.centroid[0] - pose.x)
        return math.atan2(math.sin(heading - pose.yaw), math.cos(heading - pose.yaw))

    # ── Ciclo ────────────────────────────────────────────────

    def mode_active(self, now):
        return self.active_mode == MODE and now - self.mode_time <= MODE_STALE_S

    def build_inputs(self, now):
        t = self.sensor_timeout
        ranges = [self.cache.get(f'sonar/{s}', now, t) for s in self.sonars]
        sonar_ok = all(r is not None for r in ranges)
        front_ok = self.cache.fresh(BUMPER_FRONT, now, t)
        edge = self.cache.get('edge', now, t)
        return RandomWalkInputs(
            sonar_min=min(ranges) if sonar_ok and ranges else None,
            sensors_ok=sonar_ok and front_ok,
            front_bumper=self.cache.pressed(BUMPER_FRONT, now, t),
            side_bumper=self.cache.pressed(SIDE_BUMPERS, now, t),
            back_bumper=self.cache.pressed(BUMPER_BACK, now, t),
            edge_distance=edge, geofence_ok=self.geofence_ok,
            interior_angle=self.interior_angle())

    def tick(self):
        now = time.monotonic()
        active = self.mode_active(now)
        if active and not self.was_active and not self.res.ready(now):
            return                               # recien suscripto: esperando el primer dato de cada sensor
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
        else:
            self.res.stop()                      # inactivo y ceros enviados: deja de recibir y de despertar

    def set_alert(self, alert, now, force=False):
        if alert == self.alert_sent and not (alert and now - self.last_alert_pub >= 1.0) and not force:
            return
        self.alert_sent = alert
        self.last_alert_pub = now
        self.alert_pub.publish(String(data=f'random:{alert or "none"}'))

    def shutdown_cleanup(self):
        for _ in range(5):
            try:
                self.pub.publish(Twist())
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    spin_with_cleanup(RandomWalk, args)


if __name__ == '__main__':
    main()
