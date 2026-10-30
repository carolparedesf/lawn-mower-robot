#!/usr/bin/env python3
"""Lee gpsd (JSON sobre TCP, puerto 2947) y publica sensor_msgs/NavSatFix en /gps/fix."""
import json
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsNode(Node):
    def __init__(self):
        super().__init__('berryimu_gps_node')

        self.declare_parameter('gpsd_host', '127.0.0.1')
        self.declare_parameter('gpsd_port', 2947)
        self.declare_parameter('frame_id', 'BerryGPS_Link')
        # Piso del sigma [m]: evita covarianza ~0 si gpsd reporta epx/epy muy chicos
        self.declare_parameter('min_sigma_m', 0.5)
        # Sigma usado si gpsd no entrega epx/epy ni eph [m]
        self.declare_parameter('fallback_sigma_m', 10.0)
        # Varianza vertical cuando no hay epv (fix 2D) [m^2]
        self.declare_parameter('fallback_vertical_variance', 1.0e4)

        self.host = self.get_parameter('gpsd_host').value
        self.port = self.get_parameter('gpsd_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_sigma = self.get_parameter('min_sigma_m').value
        self.fallback_sigma = self.get_parameter('fallback_sigma_m').value
        self.fallback_vvar = self.get_parameter('fallback_vertical_variance').value

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

    # ── NavSatFix ────────────────────────────────────────────

    def publish_fix(self, tpv):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS

        mode = tpv.get('mode', 0)
        if mode < 2 or 'lat' not in tpv or 'lon' not in tpv:
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

        # Covarianza ENU: epx = error este (longitud), epy = error norte (latitud)
        epx = tpv.get('epx')
        epy = tpv.get('epy')
        known = epx is not None and epy is not None
        if not known:
            eph = tpv.get('eph')
            epx = epy = eph if eph is not None else self.fallback_sigma
        sx = max(float(epx), self.min_sigma)
        sy = max(float(epy), self.min_sigma)

        epv = tpv.get('epv')
        vvar = max(float(epv), self.min_sigma) ** 2 if epv is not None else self.fallback_vvar

        msg.position_covariance = [
            sx * sx, 0.0, 0.0,
            0.0, sy * sy, 0.0,
            0.0, 0.0, vvar,
        ]
        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN if known
            else NavSatFix.COVARIANCE_TYPE_APPROXIMATED)

        self.fix_pub.publish(msg)

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
