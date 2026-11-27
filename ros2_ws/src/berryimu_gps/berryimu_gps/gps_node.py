#!/usr/bin/env python3
"""
Lee gpsd (JSON sobre TCP, puerto 2947) y publica sensor_msgs/NavSatFix en /gps/fix.

Antes de publicar filtra los fixes malos (2D, eph o HDOP altos, saltos imposibles) y pasa los
errores de gpsd (95 %) a sigma para la covarianza: ver gps_filter.py. Sin fix publica
STATUS_NO_FIX; un fix descartado no se publica.
"""
import json
import math
import socket
import threading
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from berryimu_gps.gps_filter import REJECT_NO_FIX, FilterParams, FixFilter

REJECT_LOG_PERIOD_S = 10.0      # resumen de fixes descartados en el log


class GpsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('berryimu_gps_node', **kwargs)

        self.declare_parameter('gpsd_host', '127.0.0.1')
        self.declare_parameter('gpsd_port', 2947)
        self.declare_parameter('frame_id', 'BerryGPS_Link')
        defaults = FilterParams()
        # epx/epy/eph/epv de gpsd son al 95 %: sigma = ep * ep_to_sigma
        self.declare_parameter('ep_to_sigma', defaults.ep_to_sigma)
        # Piso del sigma [m]: evita covarianza ~0 si gpsd reporta epx/epy muy chicos
        self.declare_parameter('min_sigma_m', defaults.min_sigma_m)
        # Sigma usado si gpsd no entrega epx/epy ni eph [m]
        self.declare_parameter('fallback_sigma_m', defaults.fallback_sigma_m)
        # Varianza vertical cuando no hay epv [m^2]
        self.declare_parameter('fallback_vertical_variance', defaults.fallback_vertical_variance)
        # Fixes que se descartan: 2D, eph mayor que esto [m], HDOP mayor que esto
        # (<= 0 desactiva cada limite)
        self.declare_parameter('require_3d', defaults.require_3d)
        self.declare_parameter('max_eph_m', defaults.max_eph_m)
        self.declare_parameter('max_hdop', defaults.max_hdop)
        # Saltos: se rechaza si se aleja del ultimo fix aceptado mas que
        # max_speed * dt + jump_sigmas * sigma; tras jump_accept_after rechazos seguidos se acepta
        # igual (0 desactiva el rechazo de saltos)
        self.declare_parameter('max_speed', defaults.max_speed)
        self.declare_parameter('jump_sigmas', defaults.jump_sigmas)
        self.declare_parameter('jump_accept_after', defaults.jump_accept_after)

        gp = self.get_parameter
        self.host = gp('gpsd_host').value
        self.port = gp('gpsd_port').value
        self.frame_id = gp('frame_id').value
        self.filter = FixFilter(FilterParams(
            ep_to_sigma=float(gp('ep_to_sigma').value),
            min_sigma_m=float(gp('min_sigma_m').value),
            fallback_sigma_m=float(gp('fallback_sigma_m').value),
            fallback_vertical_variance=float(gp('fallback_vertical_variance').value),
            require_3d=bool(gp('require_3d').value),
            max_eph_m=float(gp('max_eph_m').value),
            max_hdop=float(gp('max_hdop').value),
            max_speed=float(gp('max_speed').value),
            jump_sigmas=float(gp('jump_sigmas').value),
            jump_accept_after=int(gp('jump_accept_after').value),
        ))
        self.rejected = Counter()
        self.last_reject_log = time.monotonic()

        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'gps_node leyendo gpsd en {self.host}:{self.port}, frame {self.frame_id}')

    # ── Conexion a gpsd ──────────────────────────────────────

    def _run(self):
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    sock.settimeout(5.0)
                    sock.sendall(b'?WATCH={"enable":true,"json":true}\n')
                    self.get_logger().info('Conectado a gpsd')
                    buf = b''
                    while not self._stop.is_set():
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionError('gpsd cerro la conexion')
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            self._handle_line(line)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'gpsd: {e}; reintentando en 2 s')
                time.sleep(2.0)

    def _handle_line(self, line):
        try:
            report = json.loads(line)
        except ValueError:
            return
        if report.get('class') == 'TPV':
            self.publish_fix(report)
        elif report.get('class') == 'SKY':
            self.filter.on_sky(report)

    # ── NavSatFix ────────────────────────────────────────────

    def publish_fix(self, tpv, now=None):
        now = time.monotonic() if now is None else now
        decision = self.filter.check(tpv, now)
        if decision.accepted and decision.detail:
            self.get_logger().warn(f'GPS: {decision.detail}')
        if not decision.accepted and decision.reason != REJECT_NO_FIX:
            self.note_rejected(decision, now)
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS

        if not decision.accepted:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = math.nan
            msg.longitude = math.nan
            msg.altitude = math.nan
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.fix_pub.publish(msg)
            return

        # gpsd 'status': 2 = DGPS, 3/4 = RTK fijo/flotante
        gpsd_status = tpv.get('status', 1)
        if gpsd_status in (3, 4):
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif gpsd_status == 2:
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_FIX

        msg.latitude = float(tpv['lat'])
        msg.longitude = float(tpv['lon'])
        msg.altitude = float(tpv.get('altMSL', tpv.get('alt', 0.0)) or 0.0)

        # Covarianza ENU: x = este (epx, longitud), y = norte (epy, latitud)
        sx, sy = decision.sigma_x, decision.sigma_y
        msg.position_covariance = [
            sx * sx, 0.0, 0.0,
            0.0, sy * sy, 0.0,
            0.0, 0.0, decision.vertical_variance,
        ]
        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN if decision.known
            else NavSatFix.COVARIANCE_TYPE_APPROXIMATED)

        self.fix_pub.publish(msg)

    def note_rejected(self, decision, now):
        """Cuenta los descartes y los resume en el log cada REJECT_LOG_PERIOD_S."""
        self.rejected[decision.reason] += 1
        self.last_reject_detail = decision.detail or decision.reason
        if now - self.last_reject_log >= REJECT_LOG_PERIOD_S:
            summary = ', '.join(f'{k}: {v}' for k, v in sorted(self.rejected.items()))
            elapsed = now - self.last_reject_log
            self.get_logger().warn(f'GPS: fixes descartados en {elapsed:.0f} s ({summary}); '
                                   f'ultimo: {self.last_reject_detail}')
            self.rejected.clear()
            self.last_reject_log = now

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
