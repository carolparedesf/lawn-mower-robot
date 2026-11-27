#!/usr/bin/env python3
"""Cerco virtual: /geofence_raw (lat/lon de la app) -> poligono en `map` + dentro/fuera.

Publica /geofence/polygon (PolygonStamped), /geofence/inside (Bool), /geofence/distance_to_edge
(Float32, positiva dentro y negativa fuera, NaN si se desconoce) y /geofence/status (String).
Con el robot a menos de inner_margin_m del borde durante breach_cycles lecturas seguidas (o ya
afuera, de inmediato) avisa 'geofence:geofence_breach' por /nav/alert_in; es solo un aviso (llega a
la app en robot/status.alert): inside y distance_to_edge siguen midiendo el borde real.
El cerco se guarda siempre en lat/lon y se reconvierte con el servicio fromLL de
navsat_transform cada vez que este vuelve a estar disponible. No mueve el robot, asi que corre en
cualquier modo. Sin pose, sin poligono o con el poligono sin reconvertir, `inside` es false.
"""
import math
import time
from functools import partial

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

from lawnbot_nav.geofence_logic import (ALERT_BREACH, STATUS_INVALID, STATUS_NO_POLYGON,
                                        STATUS_NO_POSE, STATUS_OK, STATUS_WAITING,
                                        BreachMonitor, Hysteresis,
                                        drop_closing_vertex, lat_lon_to_local_xy,
                                        load_geofence, parse_geofence_json, save_geofence,
                                        signed_distance, validate_polygon)
from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.tf_utils import lookup_pose

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')

        d = self.declare_parameter
        d('conversion_mode', 'navsat')      # navsat: servicio fromLL | datum: plano local (pruebas)
        d('service_name', 'fromLL')
        d('datum_lat', 0.0)
        d('datum_lng', 0.0)
        d('frame_id', 'map')
        d('base_frame', 'base_footprint')
        d('pose_timeout', 0.5)
        d('rate_hz', 10.0)
        d('hysteresis_in', 5)               # lecturas consecutivas "dentro" para pasar a true
        d('hysteresis_out', 2)              # lecturas consecutivas "fuera" para pasar a false
        d('inner_margin_m', 1.0)            # franja interior que dispara geofence_breach [m]
        d('breach_cycles', 15)              # lecturas seguidas en la franja antes de avisar
        d('min_vertices', 4)
        d('min_edge_length', 0.5)
        d('min_area', 2.0)
        d('reconvert_period_s', 10.0)
        d('persist_path', '~/.ros/lawnbot_geofence.json')   # '' = no guardar

        gp = self.get_parameter
        self.mode = gp('conversion_mode').value
        if self.mode not in ('navsat', 'datum'):
            raise ValueError("conversion_mode debe ser 'navsat' o 'datum'")
        self.datum = (gp('datum_lat').value, gp('datum_lng').value)
        if self.mode == 'datum' and self.datum == (0.0, 0.0):
            raise ValueError('conversion_mode datum requiere datum_lat y datum_lng')
        self.frame_id = gp('frame_id').value
        self.base_frame = gp('base_frame').value
        self.pose_timeout = gp('pose_timeout').value
        self.min_vertices = gp('min_vertices').value
        self.min_edge = gp('min_edge_length').value
        self.min_area = gp('min_area').value
        self.reconvert_period = gp('reconvert_period_s').value
        self.persist_path = gp('persist_path').value
        self.hyst = Hysteresis(gp('hysteresis_in').value, gp('hysteresis_out').value)
        self.breach = BreachMonitor(gp('inner_margin_m').value, gp('breach_cycles').value)
        self.breach_reported = False

        # Estado del cerco
        self.raw = []                 # [(lat, lng)], siempre en lat/lon
        self.poly = None              # [(x, y)] validado en frame_id, o None
        self.error = None             # motivo si el cerco es invalido
        self.stale = True             # el poligono no corresponde al servicio actual
        self.generation = 0
        self.converting = False
        self.last_convert = 0.0
        self.service_was_ready = False
        self.last_status = None
        self.last_status_pub = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client = None
        if self.mode == 'navsat':
            from robot_localization.srv import FromLL
            self.FromLL = FromLL
            self.client = self.create_client(FromLL, gp('service_name').value)

        self.polygon_pub = self.create_publisher(PolygonStamped, 'geofence/polygon', LATCHED)
        self.inside_pub = self.create_publisher(Bool, 'geofence/inside', 10)
        self.distance_pub = self.create_publisher(Float32, 'geofence/distance_to_edge', 10)
        self.status_pub = self.create_publisher(String, 'geofence/status', LATCHED)
        self.alert_pub = self.create_publisher(String, 'nav/alert_in', 10)
        self.create_subscription(String, 'geofence_raw', self.on_raw, LATCHED)

        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.create_timer(1.0, self.maintain)

        saved = load_geofence(self.persist_path) if self.persist_path else None
        if saved:
            self.accept_raw(saved, persist=False)
            self.get_logger().info(f'cerco restaurado de {self.persist_path} ({len(saved)} vertices)')
        self.publish_polygon()
        self.get_logger().info(f'geofence_node listo (conversion: {self.mode}, frame {self.frame_id})')

    # ── Entrada ──────────────────────────────────────────────

    def on_raw(self, msg):
        try:
            points = drop_closing_vertex(parse_geofence_json(msg.data))
        except ValueError as exc:
            self.set_invalid(f'json: {exc}')
            return
        if len(points) < self.min_vertices:
            self.set_invalid(f'menos de {self.min_vertices} vertices ({len(points)})')
            return
        self.accept_raw(points, persist=True)

    def set_invalid(self, reason):
        self.raw = []
        self.poly = None
        self.error = reason
        self.generation += 1
        self.hyst.update(None)
        self.get_logger().warn(f'cerco invalido: {reason}')
        self.publish_polygon()

    def accept_raw(self, points, persist):
        self.raw = list(points)
        self.poly = None
        self.error = None
        self.stale = True
        self.generation += 1
        self.converting = False
        self.hyst.update(None)
        if persist and self.persist_path:
            try:
                save_geofence(self.persist_path, self.raw)
            except OSError as exc:
                self.get_logger().warn(f'no se pudo guardar el cerco: {exc}')
        self.publish_polygon()
        self.maybe_convert(force=True)

    # ── Conversion lat/lon -> metros ─────────────────────────

    def service_ready(self):
        return self.mode == 'datum' or self.client.service_is_ready()

    def maybe_convert(self, force=False):
        if not self.raw or self.converting or self.error:
            return
        if not force and time.monotonic() - self.last_convert < self.reconvert_period:
            return
        if self.mode == 'datum':
            xy = [lat_lon_to_local_xy(lat, lng, *self.datum) for lat, lng in self.raw]
            self.finish_conversion(xy)
            return
        if not self.client.service_is_ready():
            return
        self.converting = True
        gen = self.generation
        self.results = [None] * len(self.raw)
        self.pending = len(self.raw)
        for i, (lat, lng) in enumerate(self.raw):
            req = self.FromLL.Request()
            req.ll_point.latitude = lat
            req.ll_point.longitude = lng
            req.ll_point.altitude = 0.0
            self.client.call_async(req).add_done_callback(partial(self.on_response, i, gen))

    def on_response(self, index, gen, future):
        if gen != self.generation:
            return                                   # llego tarde: el cerco cambio
        try:
            point = future.result().map_point
            self.results[index] = (point.x, point.y)
        except Exception:
            self.results[index] = None
        self.pending -= 1
        if self.pending > 0:
            return
        self.converting = False
        self.last_convert = time.monotonic()
        # navsat_transform responde (0, 0) sin error si su transformacion aun no esta lista
        if any(r is None for r in self.results) or all(r == (0.0, 0.0) for r in self.results):
            self.get_logger().warn('fromLL sin transformacion lista; se reintenta')
            return
        self.finish_conversion(list(self.results))

    def finish_conversion(self, xy):
        self.last_convert = time.monotonic()
        poly, reason = validate_polygon(xy, self.min_vertices, self.min_edge, self.min_area)
        if poly is None:
            self.poly = None
            self.error = reason
            self.get_logger().warn(f'cerco invalido: {reason}')
            self.publish_polygon()
            return
        changed = (self.poly is None or len(poly) != len(self.poly) or
                   any(math.dist(a, b) > 0.02 for a, b in zip(poly, self.poly)))
        self.poly = poly
        self.stale = False
        if changed:
            self.get_logger().info(f'cerco convertido: {len(poly)} vertices en {self.frame_id}')
            self.publish_polygon()

    # ── Mantenimiento (1 Hz) ─────────────────────────────────

    def maintain(self):
        ready = self.service_ready()
        if self.mode == 'navsat':
            if ready and not self.service_was_ready and self.raw:
                self.maybe_convert(force=True)       # el servicio volvio: reconvertir
            elif not ready and self.service_was_ready:
                self.stale = True                    # el origen de map pudo cambiar
                self.get_logger().warn('fromLL no disponible: cerco en espera de reconversion')
            elif ready:
                self.maybe_convert()
            self.service_was_ready = ready
        self.publish_status(self.compute_status(self.get_pose()), force=True)

    def get_pose(self):
        return lookup_pose(self, self.tf_buffer, self.frame_id, self.base_frame, self.pose_timeout)

    def usable(self):
        return self.poly is not None and not self.stale and not self.error

    def compute_status(self, pose):
        if self.error:
            return f'{STATUS_INVALID}:{self.error}'
        if not self.raw:
            return STATUS_NO_POLYGON
        if not self.usable():
            return STATUS_WAITING
        if pose is None:
            return STATUS_NO_POSE
        return STATUS_OK

    # ── Ciclo (10 Hz) ────────────────────────────────────────

    def tick(self):
        pose = self.get_pose()
        distance = math.nan
        raw_inside = None
        if self.usable() and pose is not None:
            distance = signed_distance((pose.x, pose.y), self.poly)
            raw_inside = distance >= 0.0
        inside = self.hyst.update(raw_inside)
        self.inside_pub.publish(Bool(data=bool(inside)))
        self.distance_pub.publish(Float32(data=float(distance)))
        self.publish_status(self.compute_status(pose))
        self.publish_breach(None if raw_inside is None else distance)

    # ── Salidas ──────────────────────────────────────────────

    def publish_polygon(self):
        msg = PolygonStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.poly is not None and not self.stale:
            msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in self.poly]
        self.polygon_pub.publish(msg)

    def publish_breach(self, distance):
        """geofence_breach se renueva en cada ciclo mientras dure (vence en safety_gate a los
        alert_ttl s); al terminar se publica 'none' una vez para borrarla enseguida."""
        active = self.breach.update(distance)
        if active:
            self.alert_pub.publish(String(data=f'geofence:{ALERT_BREACH}'))
        elif self.breach_reported:
            self.alert_pub.publish(String(data='geofence:none'))
        if active != self.breach_reported:
            if active:
                where = 'afuera' if distance < 0.0 else f'a {distance:.2f} m del borde'
                self.get_logger().warn(f'geofence_breach: robot {where} '
                                       f'(margen interno {self.breach.margin:.2f} m)')
            else:
                self.get_logger().info('geofence_breach terminado')
        self.breach_reported = active

    def publish_status(self, status, force=False):
        now = time.monotonic()
        if status != self.last_status or (force and now - self.last_status_pub >= 1.0):
            self.last_status = status
            self.last_status_pub = now
            self.status_pub.publish(String(data=status))

    def shutdown_cleanup(self):
        """Sin nodo no hay cerco: inside=false."""
        self.inside_pub.publish(Bool(data=False))
        self.distance_pub.publish(Float32(data=math.nan))


def main(args=None):
    spin_with_cleanup(GeofenceNode, args)


if __name__ == '__main__':
    main()
