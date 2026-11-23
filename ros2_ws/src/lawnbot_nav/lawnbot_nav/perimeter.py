#!/usr/bin/env python3
"""Seguimiento de borde en /cmd_vel_perimeter. Solo actua con el modo activo auto_perimeter.

Modo base: por contacto con el bumper lateral (B3 izquierda, B4 derecha). Al tocar retrocede derecho un
poco y se aleja girando. El sonar lateral es un refinamiento opcional (use_side_sonar) que solo se usa con
side_sonar_angle_deg valido. Sin pared a la vista avanza recto hasta el primer contacto o sonar (girando hacia
el interior del cerco al acercarse a su borde) y se rinde tras search_timeout_s. Si pierde el borde o se traba,
se detiene y avisa por /nav/alert_in (perimeter_lost, perimeter_blocked). La salida pasa por twist_mux y safety_gate.
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
from lawnbot_nav.perimeter_logic import (PerimeterCore, PerimeterInputs,
                                         PerimeterParams)
from lawnbot_nav.robot_geometry import CORNER_SONAR_ANGLE_DEG
from lawnbot_nav.sensor_cache import (BUMPER_BACK, BUMPER_FRONT, SONARS,
                                      SensorCache)
from lawnbot_nav.tf_utils import lookup_pose

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
        # Pose real para medir el avance entre contactos: odom (robot real) o map (fake_robot con frame_id: map)
        d('frame_id', 'odom')
        d('base_frame', 'base_footprint')
        d('pose_timeout', 0.5)
        d('seed', -1)                          # -1: semilla al azar; otro valor: reproducible (giros hacia el interior del cerco)
        d('corner_sonar_angle_deg', CORNER_SONAR_ANGLE_DEG)   # config/robot_geometry.yaml: es el side_sonar_angle_deg de PerimeterParams
        d('geofence_frame_id', 'map')          # frame del cerco, para girar hacia su centroide en la busqueda inicial
        defaults = PerimeterParams()
        for field in PARAM_FIELDS:
            if field != 'side_sonar_angle_deg':                # sale de corner_sonar_angle_deg
                d(field, getattr(defaults, field))

        gp = self.get_parameter
        params = PerimeterParams(**{f: gp(f).value for f in PARAM_FIELDS if f != 'side_sonar_angle_deg'},
                                 side_sonar_angle_deg=gp('corner_sonar_angle_deg').value)
        if params.side not in ('left', 'right'):
            raise ValueError("side debe ser 'left' o 'right'")
        seed = gp('seed').value
        self.core = PerimeterCore(params, None if seed < 0 else seed)
        if params.use_side_sonar and not self.core.sonar_enabled:
            self.get_logger().warn(
                f'use_side_sonar con corner_sonar_angle_deg={params.side_sonar_angle_deg} (minimo 20): '
                'se ignora el sonar lateral y se sigue por bumper')
        self.sensor_timeout = gp('sensor_timeout').value
        self.frame_id = gp('frame_id').value
        self.base_frame = gp('base_frame').value
        self.pose_timeout = gp('pose_timeout').value
        self.tf_buffer = Buffer()
        self.tf_warned = False
        self.geofence_frame = gp('geofence_frame_id').value
        self.centroid = None
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
        # Solo mientras auto_perimeter esta activo (CPU de la Pi, ver active_resources.py): sensores, distancia al
        # cerco, TF y el timer de 20 Hz. Al activarse espera el primer dato de cada sensor y la pose antes de arrancar.
        fast = [Sub(Range, f'{gp("sonar_topic_prefix").value}/{s}', lambda m, s=s: self.on_sonar(s, m),
                    qos_profile_sensor_data) for s in SONARS]
        fast += [Sub(Bool, name, lambda m, n=name: self.on_bumper(n, m), qos_profile_sensor_data)
                 for name in BUMPER_FRONT + ('bumper/B3_left', 'bumper/B4_right', BUMPER_BACK)]
        # opcional: perimeter funciona sin cerco (sin geofence_node no espera por este dato)
        fast.append(Sub(Float32, 'geofence/distance_to_edge', self.on_edge, 10, required=False))
        self.res = ActiveResources(
            self, 1.0 / gp('rate_hz').value, self.tick, fast, tf_buffer=self.tf_buffer,
            tf_ready=lambda: lookup_pose(self, self.tf_buffer, self.frame_id, self.base_frame,
                                         self.pose_timeout) is not None)
        # Siempre: latcheados
        self.create_subscription(PolygonStamped, 'geofence/polygon', self.on_polygon, LATCHED)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.get_logger().info(
            f'perimeter listo (modo {MODE}, pared a la {params.side}, sonar lateral a '
            f'{params.side_sonar_angle_deg} grados)')

    def on_sonar(self, name, msg):
        self.cache.set(f'sonar/{name}', msg.range, time.monotonic())

    def on_bumper(self, name, msg):
        self.cache.set(name, 1.0 if msg.data else 0.0, time.monotonic())

    def on_edge(self, msg):
        self.cache.set('edge', msg.data, time.monotonic())

    def on_polygon(self, msg):
        points = [(p.x, p.y) for p in msg.polygon.points]
        frame_ok = (not msg.header.frame_id) or msg.header.frame_id == self.geofence_frame
        self.centroid = polygon_centroid(points) if len(points) >= 3 and frame_ok else None

    def interior_angle(self):
        """Giro que deja al robot mirando al centroide del cerco, o None si falta el cerco o la pose en su frame."""
        if self.centroid is None:
            return None
        pose = lookup_pose(self, self.tf_buffer, self.geofence_frame, self.base_frame, self.pose_timeout)
        if pose is None:
            return None
        heading = math.atan2(self.centroid[1] - pose.y, self.centroid[0] - pose.x)
        return math.atan2(math.sin(heading - pose.yaw), math.cos(heading - pose.yaw))

    def on_mode(self, msg):
        self.active_mode = msg.data
        self.mode_time = time.monotonic()
        if msg.data == MODE:
            self.res.start()

    def mode_active(self, now):
        return self.active_mode == MODE and now - self.mode_time <= MODE_STALE_S

    def build_inputs(self, now):
        t = self.sensor_timeout
        use_sonar = self.core.sonar_enabled
        needed = (['sonar/center'] if self.core.p.use_center_sonar else []) + \
                 ([self.side_sonar] if use_sonar else [])
        pose = lookup_pose(self, self.tf_buffer, self.frame_id, self.base_frame, self.pose_timeout)
        if pose is None and not self.tf_warned:
            self.tf_warned = True
            self.get_logger().warn(
                f'sin TF {self.frame_id} -> {self.base_frame}: el avance entre contactos se mide integrando la velocidad '
                'ordenada v*dt (un robot trabado que recibe v > 0 parece avanzar)')
        sonar_ok = all(self.cache.get(n, now, t) is not None for n in needed)
        return PerimeterInputs(
            sensors_ok=sonar_ok and self.cache.fresh(BUMPER_FRONT, now, t),
            side_range=self.cache.get(self.side_sonar, now, t) if use_sonar else None,
            center_range=self.cache.get('sonar/center', now, t),
            side_bumper=self.cache.pressed(self.side_bumper, now, t),
            front_bumper=self.cache.pressed(BUMPER_FRONT, now, t),
            back_bumper=self.cache.pressed(BUMPER_BACK, now, t),
            pose=None if pose is None else (pose.x, pose.y),
            edge_distance=self.cache.get('edge', now, t),
            interior_angle=self.interior_angle())

    def tick(self):
        now = time.monotonic()
        active = self.mode_active(now)
        if active and not self.was_active and not self.res.ready(now):
            return                               # recien suscripto: esperando el primer dato de cada sensor y la pose
        if active and not self.was_active:
            self.core.reset(now)
            self.tf_warned = False
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
