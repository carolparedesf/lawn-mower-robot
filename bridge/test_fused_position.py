"""robot/position fusionada con respaldo en el GPS crudo (fused_position.py) y su uso en bridge.py.

La logica corre sin ROS; los casos del bridge llaman los metodos reales de RobotBridge sobre un objeto de mentira
(necesitan rclpy y paho, si no se saltean)."""
import importlib.util
import json
import math
import os
import sys
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fused_position as fp  # noqa: E402

try:
    _spec = importlib.util.spec_from_file_location('bridge_node_fp', os.path.join(HERE, 'bridge.py'))
    _module = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_module)
    RobotBridge = _module.RobotBridge
    HAVE_ROS = True
except (ImportError, SystemExit):
    HAVE_ROS = False

LAT, LNG = -25.2637, -57.5759


class Rumbo(unittest.TestCase):
    def test_yaw_ros_a_rumbo_gps(self):
        self.assertAlmostEqual(fp.heading_deg(0.0), 90.0)              # mirando al este
        self.assertAlmostEqual(fp.heading_deg(math.pi / 2), 0.0)       # norte
        self.assertAlmostEqual(fp.heading_deg(math.pi), 270.0)         # oeste
        self.assertAlmostEqual(fp.heading_deg(-math.pi / 2), 180.0)    # sur

    def test_yaw_del_cuaternion(self):
        for yaw in (0.0, 0.7, -2.5, math.pi - 1e-6):
            q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
            self.assertAlmostEqual(fp.yaw_from_quaternion(q), yaw, places=6)


class Validez(unittest.TestCase):
    def test_cero_cero_no_es_posicion(self):
        # navsat_transform responde (0, 0) si todavia no tiene la transformacion
        self.assertFalse(fp.valid_lat_lng(0.0, 0.0))
        self.assertFalse(fp.valid_lat_lng(math.nan, LNG))
        self.assertFalse(fp.valid_lat_lng(91.0, LNG))
        self.assertFalse(fp.valid_lat_lng(None, LNG))
        self.assertTrue(fp.valid_lat_lng(LAT, LNG))


class Seleccion(unittest.TestCase):
    def test_sin_nada_no_publica(self):
        sel = fp.PositionSelector()
        self.assertEqual(sel.payload(10.0), (fp.SOURCE_NONE, None))

    def test_fusionada_gana_al_gps_crudo(self):
        sel = fp.PositionSelector()
        sel.on_raw(LAT + 0.001, LNG, 45.0, 10.0)
        sel.on_fused(LAT, LNG, math.pi / 2, 10.0)
        src, pos = sel.payload(10.5)
        self.assertEqual(src, fp.SOURCE_FUSED)
        self.assertEqual(pos, {'lat': LAT, 'lng': LNG, 'hdg': 0.0})   # rumbo desde el yaw, no el track del GPS

    def test_sin_fusionada_reciente_vuelve_al_gps_crudo(self):
        sel = fp.PositionSelector(fused_stale_s=2.5, raw_stale_s=3.0)
        sel.on_fused(LAT, LNG, 0.0, 10.0)
        sel.on_raw(LAT + 0.001, LNG, 45.0, 12.0)
        self.assertEqual(sel.source(12.5), fp.SOURCE_FUSED)
        src, pos = sel.payload(12.6)
        self.assertEqual(src, fp.SOURCE_GPS_RAW)
        self.assertEqual(pos, {'lat': LAT + 0.001, 'lng': LNG, 'hdg': 45.0})
        self.assertEqual(sel.source(15.1), fp.SOURCE_NONE)                # el crudo tambien vencio

    def test_respuesta_invalida_de_toll_no_cuenta(self):
        sel = fp.PositionSelector()
        self.assertFalse(sel.on_fused(0.0, 0.0, 0.0, 10.0))
        self.assertEqual(sel.source(10.0), fp.SOURCE_NONE)

    def test_track_faltante_es_cero(self):
        sel = fp.PositionSelector()
        sel.on_raw(LAT, LNG, None, 1.0)
        self.assertEqual(sel.payload(1.0)[1]['hdg'], 0.0)

    def test_mensaje_solo_al_cambiar(self):
        self.assertIsNone(fp.source_change_message(fp.SOURCE_FUSED, fp.SOURCE_FUSED))
        self.assertIn('CRUDO', fp.source_change_message(fp.SOURCE_FUSED, fp.SOURCE_GPS_RAW))


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))


class _Future:
    def __init__(self, lat, lng):
        self.lat, self.lng = lat, lng

    def result(self):
        return SimpleNamespace(ll_point=SimpleNamespace(latitude=self.lat, longitude=self.lng))


@unittest.skipUnless(HAVE_ROS, 'rclpy/paho no disponibles')
class PuenteFusionado(unittest.TestCase):
    def make(self, pose=(1.0, 2.0, math.pi / 2), ready=True):
        log = _Log()
        published = []
        statuses = []
        requests = []

        class Client:
            def service_is_ready(self):
                return ready

            def call_async(self, req):
                requests.append(req)
                return SimpleNamespace(add_done_callback=lambda cb: requests.append(cb))

        class Fake:
            publish_position = RobotBridge.publish_position
            on_toll = RobotBridge.on_toll
            send_position = RobotBridge.send_position
            get_logger = lambda s: log
            publish_status = lambda s: statuses.append(s.position.source(_module.time.monotonic()))
            lookup_fused_pose = lambda s: pose

        f = Fake()
        f.position = fp.PositionSelector()
        f.position_source = None
        f.toll_pending, f.toll_sent = False, 0.0
        f.toll_client = Client()
        f.ToLL = SimpleNamespace(Request=lambda: SimpleNamespace(map_point=SimpleNamespace(x=0.0, y=0.0, z=0.0)))
        f.mqtt = SimpleNamespace(publish=lambda topic, payload, **kw: published.append((topic, json.loads(payload))))
        f.log, f.published, f.statuses, f.requests = log, published, statuses, requests
        return f

    def test_publica_la_pose_fusionada_al_llegar_toll(self):
        f = self.make()
        f.publish_position()
        req, callback = f.requests
        self.assertEqual((req.map_point.x, req.map_point.y), (1.0, 2.0))
        self.assertEqual(f.published, [])                     # espera la respuesta
        callback(_Future(LAT, LNG))
        self.assertEqual(f.published, [('robot/position', {'lat': LAT, 'lng': LNG, 'hdg': 0.0})])
        self.assertEqual(f.statuses, [fp.SOURCE_FUSED])       # robot/status se entera del cambio de fuente

    def test_sin_tf_publica_el_gps_crudo_y_avisa(self):
        f = self.make(pose=None)
        f.position.on_raw(LAT, LNG, 30.0, _module.time.monotonic())
        f.publish_position()
        self.assertEqual(f.requests, [])
        self.assertEqual(f.published, [('robot/position', {'lat': LAT, 'lng': LNG, 'hdg': 30.0})])
        self.assertEqual(f.statuses, [fp.SOURCE_GPS_RAW])
        self.assertEqual(f.log.lines[0][0], 'warn')

    def test_sin_servicio_toll_publica_el_gps_crudo(self):
        f = self.make(ready=False)
        f.position.on_raw(LAT, LNG, 30.0, _module.time.monotonic())
        f.publish_position()
        self.assertEqual(f.requests, [])
        self.assertEqual(f.published[0][1]['hdg'], 30.0)

    def test_llamada_sin_respuesta_no_traba_la_fusion(self):
        f = self.make()
        f.publish_position()
        self.assertTrue(f.toll_pending)
        f.toll_sent -= fp.FUSED_STALE_S + 0.1                # navsat se cayo a mitad de la llamada
        f.publish_position()
        self.assertEqual(len(f.requests), 4)                 # se volvio a pedir


if __name__ == '__main__':
    unittest.main()
