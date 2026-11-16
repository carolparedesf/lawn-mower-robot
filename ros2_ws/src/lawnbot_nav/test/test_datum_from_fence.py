import importlib.util
import json
import os
import tempfile
import unittest

from lawnbot_nav.geofence_logic import lat_lon_to_local_xy, polygon_centroid, signed_distance
from lawnbot_nav.sim_world import SimWorld

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config')

spec = importlib.util.spec_from_file_location('datum_from_fence', os.path.join(HERE, '..', 'tools', 'datum_from_fence.py'))
dff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dff)


def rect(lat, lng, w_m, h_m):
    """Rectangulo de w_m x h_m metros con la esquina sur-oeste en (lat, lng), como lista de (lat, lng)."""
    dlat = h_m / 111000.0
    dlng = w_m / (111000.0 * 0.9)
    return [(lat, lng), (lat, lng + dlng), (lat + dlat, lng + dlng), (lat + dlat, lng)]


def world_cfg(name):
    import yaml
    with open(os.path.join(CONFIG, f'sim_{name}.yaml')) as f:
        return yaml.safe_load(f)['fake_robot']['ros__parameters']


def sim_world(name):
    p = world_cfg(name)
    w, o = p['walls'], p['obstacles']
    walls = [((w[i], w[i + 1]), (w[i + 2], w[i + 3])) for i in range(0, len(w) - 3, 4)]
    circles = [(o[i], o[i + 1], o[i + 2]) for i in range(0, len(o) - 2, 3)]
    return SimWorld.from_config(p), p


class MundoCampoTests(unittest.TestCase):
    def test_campo_de_30_m_con_borde_lejano_y_arranque_al_centro(self):
        p = world_cfg('field')
        self.assertEqual((p['x'], p['y']), (15.0, 15.0))
        self.assertEqual(len(p['walls']), 28)                      # borde exterior (4 segmentos), pared recta (1) y L (2)
        xs, ys = p['walls'][0::2], p['walls'][1::2]
        self.assertLessEqual(min(xs), -5.0)
        self.assertGreaterEqual(max(xs), 35.0)
        self.assertLessEqual(min(ys), -5.0)
        self.assertGreaterEqual(max(ys), 35.0)
        self.assertIn(len(p['obstacles']) // 3, (2, 3))
        self.assertTrue(all(r <= 0.5 for r in p['obstacles'][2::3]))

    def test_el_arranque_no_choca_con_nada(self):
        world, p = sim_world('field')
        self.assertFalse(world.collides((p['x'], p['y'], p['yaw'])))

    def test_el_centro_del_mundo(self):
        self.assertEqual(dff.load_world('field')[0], (15.0, 15.0))
        self.assertEqual(dff.load_world('room')[0], (5.0, 4.0))

    def test_los_obstaculos_estan_dentro_del_campo(self):
        o = world_cfg('field')['obstacles']
        for x, y, r in zip(o[0::3], o[1::3], o[2::3]):
            self.assertTrue(0.0 < x < 30.0 and 0.0 < y < 30.0)


class DatumTests(unittest.TestCase):
    def test_el_centro_del_cerco_cae_en_el_centro_del_mundo(self):
        for world, size in (('field', (24.0, 20.0)), ('room', (9.0, 7.0))):
            pts = rect(-25.3, -57.6, *size)
            r = dff.compute(pts, world)
            xy = [lat_lon_to_local_xy(la, ln, r['datum_lat'], r['datum_lng']) for la, ln in pts]
            cx, cy = polygon_centroid(xy)
            center = dff.load_world(world)[0]
            self.assertAlmostEqual(cx, center[0], delta=0.001)
            self.assertAlmostEqual(cy, center[1], delta=0.001)

    def test_cerco_no_rectangular_y_con_vertice_de_cierre(self):
        pts = [(-25.3, -57.6), (-25.3, -57.5998), (-25.2999, -57.5997), (-25.29975, -57.5999), (-25.3, -57.6)]
        r = dff.compute(pts, 'field')
        center = dff.load_world('field')[0]
        self.assertAlmostEqual(r['fence_center'][0], center[0], delta=0.001)
        self.assertEqual(len(r['polygon']), 4)

    def test_el_arranque_queda_dentro_con_margen(self):
        r = dff.compute(rect(-25.3, -57.6, 24.0, 20.0), 'field')
        self.assertGreaterEqual(r['start_margin'], 0.4)
        self.assertAlmostEqual(signed_distance(r['start'], r['polygon']), r['start_margin'], places=6)

    def test_arranque_a_menos_de_0_4_m_del_borde_es_error(self):
        # cerco de 0.5 m de ancho en x... el centro queda a 0.25 m del borde: rechazado (ademas, arista minima 0.5)
        pts = rect(-25.3, -57.6, 6.0, 0.7)
        with self.assertRaises(dff.DatumError) as cm:
            dff.compute(pts, 'field')
        self.assertIn('minimo', str(cm.exception))

    def test_el_margen_es_configurable(self):
        pts = rect(-25.3, -57.6, 6.0, 3.0)                          # el centro queda a 1.5 m del borde
        self.assertEqual(dff.compute(pts, 'field', margin=1.4)['world'], 'field')
        with self.assertRaises(dff.DatumError):
            dff.compute(pts, 'field', margin=1.6)

    def test_arranque_del_cuarto_fuera_del_cerco(self):
        # sim_room arranca en (2, 1) y un cerco chico centrado en (5, 4) no lo incluye
        with self.assertRaises(dff.DatumError) as cm:
            dff.compute(rect(-25.3, -57.6, 2.0, 2.0), 'room')
        self.assertIn('(2, 1)', str(cm.exception))

    def test_cerco_mas_grande_que_el_mundo_es_error(self):
        with self.assertRaises(dff.DatumError) as cm:
            dff.compute(rect(-25.3, -57.6, 24.0, 20.0), 'room')
        self.assertIn('no entra', str(cm.exception))

    def test_cerco_invalido(self):
        with self.assertRaises(dff.DatumError):
            dff.compute([(-25.3, -57.6), (-25.3, -57.5999), (-25.2999, -57.5999)], 'field')

    def test_mundo_desconocido(self):
        with self.assertRaises(dff.DatumError):
            dff.load_world('jardin')

    def test_la_inversa_recupera_el_punto(self):
        lat, lng = dff.local_to_lat_lon(12.0, -7.0, -25.3, -57.6)
        x, y = lat_lon_to_local_xy(lat, lng, -25.3, -57.6)
        self.assertAlmostEqual(x, 12.0, places=6)
        self.assertAlmostEqual(y, -7.0, places=6)


class CommandLineTests(unittest.TestCase):
    def run_main(self, points, *argv):
        import contextlib
        import io
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'fence.json')
            if points is not None:
                with open(path, 'w') as f:
                    json.dump({'points': [{'lat': a, 'lng': b} for a, b in points]}, f)
            out, err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                rc = dff.main(['--fence', path, *argv])
            return rc, out.getvalue(), err.getvalue()

    def test_imprime_el_comando_completo(self):
        rc, out, _ = self.run_main(rect(-25.3, -57.6, 24.0, 20.0), '--world', 'field')
        self.assertEqual(rc, 0)
        self.assertIn('sim_world.launch.py world:=field', out)
        line = next(l for l in out.splitlines() if 'nav.launch.py' in l)
        self.assertIn('geofence_conversion_mode:=datum', line)
        lat = float(line.split('datum_lat:=')[1].split()[0])
        lng = float(line.split('datum_lng:=')[1].split()[0])
        r = dff.compute(rect(-25.3, -57.6, 24.0, 20.0), 'field')
        self.assertAlmostEqual(lat, r['datum_lat'], places=8)
        self.assertAlmostEqual(lng, r['datum_lng'], places=8)
        bridge = next(l for l in out.splitlines() if 'bridge.py' in l)
        self.assertIn('--sim-position', bridge)
        self.assertAlmostEqual(float(bridge.split('--datum-lat ')[1].split()[0]), r['datum_lat'], places=8)
        self.assertAlmostEqual(float(bridge.split('--datum-lng ')[1].split()[0]), r['datum_lng'], places=8)

    def test_sin_archivo_de_cerco(self):
        rc, out, err = self.run_main(None, '--world', 'field')
        self.assertEqual(rc, 1)
        self.assertEqual(out, '')
        self.assertIn('error', err)

    def test_cerco_que_no_sirve_no_imprime_comando(self):
        rc, out, err = self.run_main(rect(-25.3, -57.6, 2.0, 2.0), '--world', 'room')
        self.assertEqual(rc, 2)
        self.assertNotIn('nav.launch.py', out)


if __name__ == '__main__':
    unittest.main()
