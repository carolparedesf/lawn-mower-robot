#!/usr/bin/env python3
"""Mapa de cobertura: pasadas por celda y obstaculos (sonar y camara), sobre la caja del cerco.

Publica /coverage/cell_passes (Int32: pasadas completadas de la celda donde esta el robot, -1 si se desconoce;
lo lee blade_controller) y /coverage/map (String con JSON, ver CoverageGrid.to_dict) para que el bridge lo
mande por MQTT a robot/map. Una pasada cuenta solo si la cuchilla estaba cortando (/cutter/cmd > 0), salvo
`count_only_when_cutting: false`. No mueve el robot.

El JSON lleva ademas `origin_ll` [lat, lng]: la latitud/longitud del origen (0, 0) del frame `map`, para que la app ubique la
grilla en el mapa (x = este, y = norte). Sale del servicio /toLL de navsat_transform y se vuelve a consultar cada
`origin_refresh_s`; si cambia (otro datum) se actualiza y el mapa se reenvia de inmediato. Sin dato la clave se omite.
"""
import json
import math
import time

from geometry_msgs.msg import PoseArray, PolygonStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformListener

from lawnbot_nav.coverage_map_logic import CoverageGrid
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.sensor_cache import SONARS, SensorCache
from lawnbot_nav.robot_geometry import CORNER_SONAR_ANGLE_DEG, SONAR_MOUNTS, sonar_yaws
from lawnbot_nav.tf_utils import lookup_pose

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CoverageMap(Node):
    def __init__(self):
        super().__init__('coverage_map')

        d = self.declare_parameter
        d('resolution', 0.5)                   # m por celda (el GPS tiene ~2-3 m de error: ver el doc)
        d('padding', 1.0)                      # m alrededor de la caja del cerco
        d('publish_period_s', 2.0)
        d('pose_rate_hz', 10.0)
        d('count_only_when_cutting', True)
        d('cutting_timeout', 0.5)              # s: /cutter/cmd mas viejo que esto = no esta cortando
        d('sonar_max_range', 1.5)
        d('corner_sonar_angle_deg', CORNER_SONAR_ANGLE_DEG)   # yaw de los sonares de las esquinas (config/robot_geometry.yaml)
        d('use_sonars', True)
        d('sonar_topic_prefix', 'sonar')       # 'sonar_filtered' con el filtro de sonares (nav.launch.py)
        d('frame_id', 'map')
        d('base_frame', 'base_footprint')
        d('pose_timeout', 0.5)
        d('hysteresis', 0.15)
        d('origin_source', 'navsat')           # navsat: servicio toLL | param: origin_lat/origin_lng | none: sin origin_ll
        d('to_ll_service', 'toLL')
        d('origin_lat', 0.0)
        d('origin_lng', 0.0)
        d('origin_refresh_s', 10.0)
        d('origin_change_deg', 1.0e-6)         # ~0,1 m: un cambio menor es ruido numerico

        gp = self.get_parameter
        self.res = gp('resolution').value
        if self.res < 0.5:
            self.get_logger().warn(
                f'resolution {self.res} m es menor que 0,5 m: con el error tipico del GPS (2-3 m) las pasadas por celda no son confiables')
        self.pad = gp('padding').value
        self.count_only_cutting = gp('count_only_when_cutting').value
        self.cutting_timeout = gp('cutting_timeout').value
        self.max_range = gp('sonar_max_range').value
        self.yaw = sonar_yaws(gp('corner_sonar_angle_deg').value)
        self.use_sonars = gp('use_sonars').value
        self.frame_id = gp('frame_id').value
        self.base_frame = gp('base_frame').value
        self.pose_timeout = gp('pose_timeout').value
        self.hysteresis = gp('hysteresis').value
        self.publish_period = gp('publish_period_s').value
        self.origin_source = gp('origin_source').value
        if self.origin_source not in ('navsat', 'param', 'none'):
            raise ValueError("origin_source debe ser 'navsat', 'param' o 'none'")
        self.origin_refresh = gp('origin_refresh_s').value
        self.origin_tol = gp('origin_change_deg').value
        self.origin_ll = None
        self.origin_pending = False
        self.last_origin_query = -math.inf
        self.origin_client = None
        if self.origin_source == 'param':
            if (gp('origin_lat').value, gp('origin_lng').value) == (0.0, 0.0):
                raise ValueError('origin_source param requiere origin_lat y origin_lng')
            self.origin_ll = (gp('origin_lat').value, gp('origin_lng').value)
        elif self.origin_source == 'navsat':
            from robot_localization.srv import ToLL
            self.ToLL = ToLL
            self.origin_client = self.create_client(ToLL, gp('to_ll_service').value)

        self.grid = None
        self.polygon = None
        self.cache = SensorCache()
        self.last_map_pub = 0.0
        self.last_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.passes_pub = self.create_publisher(Int32, 'coverage/cell_passes', 10)
        self.map_pub = self.create_publisher(String, 'coverage/map', 10)
        self.create_subscription(PolygonStamped, 'geofence/polygon', self.on_polygon, LATCHED)
        self.create_subscription(Float32, 'cutter/cmd', lambda m: self.cache.set('cutter', m.data, time.monotonic()), 10)
        self.create_subscription(PoseArray, 'vision/obstacles', self.on_camera, 10)
        if self.use_sonars:
            for s in SONARS:
                self.create_subscription(Range, f'{gp("sonar_topic_prefix").value}/{s}', lambda m, s=s: self.on_sonar(s, m),
                                         qos_profile_sensor_data)
        self.create_timer(1.0 / gp('pose_rate_hz').value, self.tick)
        self.get_logger().info(f'coverage_map listo (celdas de {self.res} m, cuenta pasadas {"solo cortando" if self.count_only_cutting else "siempre"})')

    # ── Entradas ─────────────────────────────────────────────

    def on_polygon(self, msg):
        points = [(p.x, p.y) for p in msg.polygon.points]
        frame_ok = (not msg.header.frame_id) or msg.header.frame_id == self.frame_id
        poly = points if len(points) >= 3 and frame_ok else None
        if poly == self.polygon:
            return
        self.polygon = poly
        if poly is None:
            self.grid = None
            self.get_logger().info('sin cerco: se descarta el mapa')
        else:
            self.grid = CoverageGrid.from_polygon(poly, self.res, self.pad, hysteresis=self.hysteresis)
            self.get_logger().info(f'cerco nuevo: grilla de {self.grid.w} x {self.grid.h} celdas')

    def get_pose(self):
        return lookup_pose(self, self.tf_buffer, self.frame_id, self.base_frame, self.pose_timeout)

    def on_sonar(self, name, msg):
        pose = self.last_pose
        if self.grid is None or pose is None:
            return
        mx, my = SONAR_MOUNTS[name]
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        self.grid.add_sonar(pose.x + c * mx - s * my, pose.y + s * mx + c * my,
                            pose.yaw + math.radians(self.yaw[name]), float(msg.range), self.max_range)

    def on_camera(self, msg):
        pose = self.last_pose
        if self.grid is None or pose is None:
            return
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        self.grid.add_camera((pose.x + c * p.position.x - s * p.position.y,
                              pose.y + s * p.position.x + c * p.position.y) for p in msg.poses)

    # ── Origen en lat/lng ────────────────────────────────────

    def maintain_origin(self, now):
        """Pregunta a navsat_transform por el lat/lng del origen del frame map, cada origin_refresh_s."""
        if self.origin_client is None or self.origin_pending or now - self.last_origin_query < self.origin_refresh:
            return
        if not self.origin_client.service_is_ready():
            return
        self.last_origin_query = now
        self.origin_pending = True
        req = self.ToLL.Request()
        req.map_point.x = req.map_point.y = req.map_point.z = 0.0
        self.origin_client.call_async(req).add_done_callback(self.on_origin)

    def on_origin(self, future):
        self.origin_pending = False
        try:
            ll = future.result().ll_point
            lat, lng = float(ll.latitude), float(ll.longitude)
        except Exception:
            return
        # navsat_transform responde (0, 0) sin error si su transformacion aun no esta lista
        if (lat, lng) == (0.0, 0.0) or math.isnan(lat) or math.isnan(lng):
            self.get_logger().warn('toLL sin transformacion lista; se reintenta', throttle_duration_sec=30.0)
            self.last_origin_query = -math.inf if self.origin_ll is None else self.last_origin_query
            return
        old = self.origin_ll
        if old is None or abs(lat - old[0]) > self.origin_tol or abs(lng - old[1]) > self.origin_tol:
            self.origin_ll = (lat, lng)
            self.last_map_pub = -math.inf                 # reenviar el mapa ya con el origen nuevo
            self.get_logger().info(f'origen del mapa: {lat:.7f}, {lng:.7f}' + ('' if old is None else ' (cambio de datum)'))

    # ── Ciclo ────────────────────────────────────────────────

    def cutting(self, now):
        if not self.count_only_cutting:
            return True
        value = self.cache.get('cutter', now, self.cutting_timeout)
        return value is not None and value > 0.0

    def tick(self):
        now = time.monotonic()
        pose = self.get_pose()
        self.last_pose = pose
        passes = -1
        if self.grid is not None and pose is not None:
            result = self.grid.update_pose(pose.x, pose.y, self.cutting(now))
            passes = -1 if result is None else int(result)
        self.passes_pub.publish(Int32(data=passes))
        self.maintain_origin(now)

        if self.grid is not None and now - self.last_map_pub >= self.publish_period:
            self.last_map_pub = now
            robot = (pose.x, pose.y, pose.yaw) if pose is not None else None
            payload = self.grid.to_dict(robot, stamp=time.time())
            if self.origin_ll is not None:
                payload['origin_ll'] = [self.origin_ll[0], self.origin_ll[1]]
            self.map_pub.publish(String(data=json.dumps(payload)))

    def shutdown_cleanup(self):
        self.passes_pub.publish(Int32(data=-1))          # sin nodo no hay dato: el cortador se apaga


def main(args=None):
    spin_with_cleanup(CoverageMap, args)


if __name__ == '__main__':
    main()
