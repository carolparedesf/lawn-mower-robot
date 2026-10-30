#!/usr/bin/env python3
"""Cobertura en lineas paralelas: al ENTRAR a auto_parallel genera la ruta sobre el cerco.

Lee /geofence/polygon (poligono convexo en `map`) y publica la ruta en /follower/path, una vista
para RViz en /coverage/markers y el resultado en /coverage/status. Fuera de auto_parallel no hace
nada. Si el cerco no sirve (no convexo, region vacia, sin cerco) publica ERROR con la causa y una
ruta vacia (que borra cualquier ruta vieja del seguidor). No mueve el robot: lo hace el seguidor.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PolygonStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from lawnbot_nav.coverage_logic import (CoverageParams, format_status,
                                        generate_coverage)
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.tf_utils import lookup_pose

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

MODE = 'auto_parallel'


class CoverageParallel(Node):
    def __init__(self):
        super().__init__('coverage_parallel')

        d = self.declare_parameter
        d('cutting_width', 0.30)           # m (provisorio)
        d('overlap', 0.10)                 # fraccion de solape entre pasadas
        d('margin', 1.0)                   # m desde el borde del cerco (mayor que el error del GPS)
        d('sweep_angle_auto', True)        # True: paralelo al lado mas largo del cerco
        d('sweep_angle_deg', 0.0)          # usado solo si sweep_angle_auto es False
        d('start_nearest', True)
        d('turn_extension', 0.15)          # m; no puede superar a margin
        d('est_linear', 0.2)
        d('est_angular', 0.6)
        d('frame_id', 'map')
        d('base_frame', 'base_footprint')
        d('pose_timeout', 0.5)

        gp = self.get_parameter
        self.params = CoverageParams(
            cutting_width=gp('cutting_width').value, overlap=gp('overlap').value,
            margin=gp('margin').value,
            sweep_angle_deg=None if gp('sweep_angle_auto').value else gp('sweep_angle_deg').value,
            start_nearest=gp('start_nearest').value, turn_extension=gp('turn_extension').value,
            est_linear=gp('est_linear').value, est_angular=gp('est_angular').value)
        self.frame_id = gp('frame_id').value
        self.base_frame = gp('base_frame').value
        self.pose_timeout = gp('pose_timeout').value

        self.polygon = None
        self.polygon_frame_ok = True
        self.prev_mode = None
        self.active = False
        self.pending = False
        self.status = 'IDLE'
        self.last_alert = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, 'follower/path', LATCHED)
        self.markers_pub = self.create_publisher(MarkerArray, 'coverage/markers', LATCHED)
        self.status_pub = self.create_publisher(String, 'coverage/status', LATCHED)
        self.alert_pub = self.create_publisher(String, 'nav/alert_in', 10)
        self.create_subscription(PolygonStamped, 'geofence/polygon', self.on_polygon, LATCHED)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.create_timer(1.0, self.maintain)

        self.publish_status('IDLE')
        self.get_logger().info(
            f'coverage_parallel listo: ancho {self.params.cutting_width} m, solape {self.params.overlap}, '
            f'margen {self.params.margin} m (se activa al entrar a {MODE})')

    # ── Entradas ─────────────────────────────────────────────

    def on_polygon(self, msg):
        points = [(p.x, p.y) for p in msg.polygon.points]
        self.polygon_frame_ok = (not msg.header.frame_id) or msg.header.frame_id == self.frame_id
        self.polygon = points if points and self.polygon_frame_ok else None
        if not self.polygon_frame_ok and points:
            self.get_logger().warn(
                f"cerco en el frame '{msg.header.frame_id}', se esperaba '{self.frame_id}'")
        if self.pending:
            self.attempt()                  # el cerco llego (o cambio) mientras se esperaba

    def on_mode(self, msg):
        mode = msg.data
        prev, self.prev_mode = self.prev_mode, mode
        if prev is None:                    # primer mensaje: solo toma la referencia
            self.active = mode == MODE
            return
        if mode == MODE and prev != MODE:
            self.active = True
            self.pending = True
            self.get_logger().info(f'modo {MODE}: generando la ruta de cobertura')
            self.attempt()
        elif mode != MODE and prev == MODE:
            self.active = False
            self.pending = False
            self.clear()

    # ── Generacion ───────────────────────────────────────────

    def get_pose(self):
        return lookup_pose(self, self.tf_buffer, self.frame_id, self.base_frame, self.pose_timeout)

    def attempt(self):
        pose = self.get_pose()
        robot_xy = (pose.x, pose.y) if pose is not None else None
        result = generate_coverage(self.polygon or [], self.params, robot_xy)
        self.publish_status(format_status(result))
        if not result.ok:
            self.get_logger().warn(f'cobertura no generada: {result.error}')
            self.path_pub.publish(self.make_path([]))        # sin ruta: borra la anterior
            self.markers_pub.publish(self.make_markers(None))
            return
        self.pending = False
        self.path_pub.publish(self.make_path(result.path))
        self.markers_pub.publish(self.make_markers(result))
        self.send_alert('none')
        self.get_logger().info(self.status)

    def clear(self):
        self.path_pub.publish(self.make_path([]))
        self.markers_pub.publish(self.make_markers(None))
        self.publish_status('IDLE')
        self.send_alert('none')

    # ── Salidas ──────────────────────────────────────────────

    def make_path(self, points):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            nxt = points[i + 1] if i + 1 < len(points) else None
            prv = points[i - 1] if i > 0 else None
            ref = (nxt[0] - x, nxt[1] - y) if nxt else ((x - prv[0], y - prv[1]) if prv else (1.0, 0.0))
            yaw = math.atan2(ref[1], ref[0])
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        return msg

    def make_markers(self, result):
        array = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        array.markers.append(wipe)
        if result is None:
            return array

        def strip(mid, ns, pts, rgb, width, closed=False):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = ns, mid, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
            m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in (pts + pts[:1] if closed else pts)]
            return m

        array.markers.append(strip(1, 'fence', self.polygon or [], (0.2, 0.8, 0.2), 0.10, closed=True))
        array.markers.append(strip(2, 'region', result.region, (0.95, 0.8, 0.1), 0.06, closed=True))
        array.markers.append(strip(3, 'path', result.path, (0.2, 0.4, 0.95), 0.06))
        dots = Marker()
        dots.header.frame_id = self.frame_id
        dots.header.stamp = self.get_clock().now().to_msg()
        dots.ns, dots.id, dots.type, dots.action = 'waypoints', 4, Marker.SPHERE_LIST, Marker.ADD
        dots.pose.orientation.w = 1.0
        dots.scale.x = dots.scale.y = dots.scale.z = 0.18
        dots.color.r, dots.color.g, dots.color.b, dots.color.a = 1.0, 0.55, 0.0, 1.0
        dots.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in result.path]
        array.markers.append(dots)
        return array

    def publish_status(self, text):
        self.status = text
        self.status_pub.publish(String(data=text))

    def send_alert(self, alert):
        self.alert_pub.publish(String(data=f'coverage:{alert}'))

    def maintain(self):
        """1 Hz: mientras el modo esta activo y no hay ruta, mantiene viva la alerta de error."""
        if self.active and self.pending and self.status.startswith('ERROR'):
            now = time.monotonic()
            if now - self.last_alert >= 1.0:
                self.last_alert = now
                self.send_alert('coverage_error')
        if self.active:
            self.status_pub.publish(String(data=self.status))

    def shutdown_cleanup(self):
        """Sin generador no hay ruta vigente: se borra la del seguidor."""
        self.path_pub.publish(self.make_path([]))
        self.markers_pub.publish(self.make_markers(None))


def main(args=None):
    spin_with_cleanup(CoverageParallel, args)


if __name__ == '__main__':
    main()
