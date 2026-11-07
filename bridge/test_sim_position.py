import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'lawnbot_nav'))

import sim_position as sp  # noqa: E402
from lawnbot_nav.geofence_logic import lat_lon_to_local_xy  # noqa: E402

LAT, LNG = -25.3, -57.6


class Opciones(unittest.TestCase):
    def test_apagada_por_defecto(self):
        self.assertFalse(sp.requested([], {}))
        self.assertIsNone(sp.parse_options([], {}))
        self.assertIsNone(sp.parse_options(['--datum-lat', '1', '--datum-lng', '2'], {}))   # el datum solo no la activa
        self.assertIsNone(sp.parse_options([], {sp.ENV_ENABLE: '0', sp.ENV_LAT: '1', sp.ENV_LNG: '2'}))

    def test_por_argumento(self):
        self.assertEqual(sp.parse_options(['--sim-position', '--datum-lat', str(LAT), '--datum-lng', str(LNG)], {}), (LAT, LNG))

    def test_por_variable_de_entorno(self):
        env = {sp.ENV_ENABLE: 'True', sp.ENV_LAT: str(LAT), sp.ENV_LNG: str(LNG)}
        self.assertEqual(sp.parse_options([], env), (LAT, LNG))

    def test_el_argumento_gana_a_la_variable(self):
        env = {sp.ENV_ENABLE: '1', sp.ENV_LAT: '1.0', sp.ENV_LNG: '2.0'}
        self.assertEqual(sp.parse_options(['--datum-lat', '-25.0'], env), (-25.0, 2.0))

    def test_ignora_argumentos_de_ros(self):
        argv = ['--sim-position', '--datum-lat', '-25.3', '--datum-lng', '-57.6', '--ros-args', '-r', '__node:=x']
        self.assertEqual(sp.parse_options(argv, {}), (-25.3, -57.6))

    def test_sin_datum_es_error(self):
        with self.assertRaises(ValueError) as cm:
            sp.parse_options(['--sim-position'], {})
        self.assertIn('datum_from_fence', str(cm.exception))
        with self.assertRaises(ValueError):
            sp.parse_options(['--sim-position', '--datum-lat', '-25.3'], {})

    def test_datum_invalido(self):
        for lat, lng in (('0', '0'), ('95', '10'), ('10', '200')):
            with self.assertRaises(ValueError):
                sp.parse_options(['--sim-position', '--datum-lat', lat, '--datum-lng', lng], {})
        with self.assertRaises(ValueError):
            sp.parse_options(['--sim-position'], {sp.ENV_LAT: 'abc', sp.ENV_LNG: '1'})


class Conversion(unittest.TestCase):
    def test_ida_y_vuelta_con_la_conversion_del_geofence_node(self):
        for x, y in ((0.0, 0.0), (15.0, 15.0), (-12.5, 7.25), (30.0, -4.0)):
            lat, lng = sp.xy_to_lat_lon(x, y, LAT, LNG)
            gx, gy = lat_lon_to_local_xy(lat, lng, LAT, LNG)
            self.assertAlmostEqual(gx, x, places=6)
            self.assertAlmostEqual(gy, y, places=6)

    def test_el_origen_es_el_datum(self):
        self.assertEqual(sp.xy_to_lat_lon(0.0, 0.0, LAT, LNG), (LAT, LNG))

    def test_este_y_norte(self):
        lat, lng = sp.xy_to_lat_lon(10.0, 20.0, LAT, LNG)
        self.assertGreater(lat, LAT)
        self.assertGreater(lng, LNG)

    def test_rumbo_como_gpsd(self):
        # yaw 0 = mirando al este (90 grados), pi/2 = norte (0), pi = oeste (270), -pi/2 = sur (180)
        for yaw, hdg in ((0.0, 90.0), (math.pi / 2, 0.0), (math.pi, 270.0), (-math.pi / 2, 180.0)):
            self.assertAlmostEqual(sp.heading_deg(yaw), hdg, places=6)
        self.assertTrue(all(0.0 <= sp.heading_deg(math.radians(a)) < 360.0 for a in range(-720, 720, 7)))

    def test_mismo_json_que_gpsd(self):
        p = sp.position_payload(15.0, 15.0, math.pi / 2, LAT, LNG)
        self.assertEqual(set(p), {'lat', 'lng', 'hdg'})
        self.assertAlmostEqual(p['hdg'], 0.0)
        self.assertTrue(all(isinstance(v, float) for v in p.values()))


if __name__ == '__main__':
    unittest.main()
