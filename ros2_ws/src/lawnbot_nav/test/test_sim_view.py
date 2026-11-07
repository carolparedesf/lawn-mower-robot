import math
import os
import unittest

from lawnbot_nav.sim_view import Trail, bumper_polylines, footprint
from lawnbot_nav.sim_world import BODY_X, BODY_Y, SimWorld, bumper_zones


class FootprintTests(unittest.TestCase):
    def test_sin_radio_es_el_rectangulo(self):
        pts = footprint(0.0)
        self.assertEqual(pts[0], pts[-1])
        self.assertEqual(set(pts), {(BODY_X[1], BODY_Y[0]), (BODY_X[1], BODY_Y[1]), (BODY_X[0], BODY_Y[1]), (BODY_X[0], BODY_Y[0])})

    def test_con_radio_las_esquinas_son_arcos_dentro_del_rectangulo(self):
        r = 0.1
        pts = footprint(r)
        self.assertEqual(pts[0], pts[-1])
        self.assertGreater(len(pts), 20)
        for x, y in pts:
            self.assertTrue(BODY_X[0] - 1e-9 <= x <= BODY_X[1] + 1e-9 and BODY_Y[0] - 1e-9 <= y <= BODY_Y[1] + 1e-9)
        # ningun punto cae en la esquina del rectangulo
        self.assertNotIn((BODY_X[1], BODY_Y[1]), [(round(x, 9), round(y, 9)) for x, y in pts])
        corner_dist = min(math.dist(p, (BODY_X[1], BODY_Y[1])) for p in pts)
        self.assertAlmostEqual(corner_dist, r * (math.sqrt(2) - 1), delta=0.005)

    def test_el_contorno_es_antihorario(self):
        pts = footprint(0.08)
        area2 = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:]))
        self.assertGreater(area2, 0.0)


class BumperPolylineTests(unittest.TestCase):
    def check(self, zones):
        lines = bumper_polylines(zones)
        self.assertEqual(set(lines), set(zones))
        for name, pts in lines.items():
            self.assertEqual(sorted(pts), sorted(zones[name]))
            # un tramo continuo del contorno: ningun salto grande entre puntos consecutivos
            jumps = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
            self.assertLess(max(jumps), 0.07, (name, max(jumps)))

    def test_cada_bumper_es_un_segmento_continuo_sin_radio(self):
        self.check(bumper_zones(0.0))

    def test_cada_bumper_es_un_segmento_continuo_con_esquinas_y_zona_muerta(self):
        self.check(bumper_zones(0.1, 45.0, 0.0))
        self.check(bumper_zones(0.1, 45.0, 20.0))

    def test_el_bumper_trasero_que_cruza_180_grados_no_se_corta(self):
        pts = bumper_polylines(bumper_zones(0.1, 45.0, 0.0))['bumper/B5_back']
        ys = [y for _, y in pts]
        self.assertTrue(ys == sorted(ys) or ys == sorted(ys, reverse=True))


class TrailTests(unittest.TestCase):
    def test_limitado_a_las_ultimas_poses(self):
        t = Trail(max_poses=5, min_step=0.01)
        for i in range(20):
            t.add(i * 0.1, 0.0, 0.0)
        self.assertEqual(len(t.poses), 5)
        self.assertAlmostEqual(t.poses[-1][0], 1.9)
        self.assertAlmostEqual(t.poses[0][0], 1.5)

    def test_quieto_no_agrega_poses_y_girando_si(self):
        t = Trail(max_poses=100, min_step=0.02, min_yaw=0.087)
        self.assertTrue(t.add(1.0, 1.0, 0.0))
        self.assertFalse(t.add(1.001, 1.0, 0.01))
        self.assertTrue(t.add(1.0, 1.0, 0.2))                     # giro en el lugar
        self.assertTrue(t.add(1.05, 1.0, 0.2))                    # avance
        self.assertEqual(len(t.poses), 3)

    def test_el_giro_cruza_pi(self):
        t = Trail(min_yaw=0.087)
        t.add(0.0, 0.0, math.pi - 0.01)
        self.assertFalse(t.add(0.0, 0.0, -math.pi + 0.01))        # 0.02 rad de diferencia real


class SimRoomRvizTests(unittest.TestCase):
    CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')

    def test_sim_rviz_tiene_los_displays_pedidos(self):
        import yaml
        cfg = yaml.safe_load(open(os.path.join(self.CFG, 'sim.rviz')))['Visualization Manager']
        self.assertEqual(cfg['Global Options']['Fixed Frame'], 'map')
        by_class = {}
        for d in cfg['Displays']:
            by_class.setdefault(d['Class'].split('/')[-1], []).append(d)
        self.assertIn('TF', by_class)
        topics = {d['Topic']['Value'] for d in cfg['Displays'] if 'Topic' in d}
        self.assertEqual(topics, {'/sim/markers', '/sim/path', '/geofence/polygon', '/coverage/markers'})
        for d in cfg['Displays']:
            if 'Topic' in d:                                       # los cuatro topics son latched (transient_local)
                self.assertEqual(d['Topic']['Durability Policy'], 'Transient Local', d['Name'])

    def test_sim_room_enciende_los_marcadores_y_el_launch_real_no(self):
        import yaml
        room = yaml.safe_load(open(os.path.join(self.CFG, 'sim_room.yaml')))['fake_robot']['ros__parameters']
        self.assertTrue(room['publish_markers'])
        # nada del robot real (nav.launch.py ni los YAML de los nodos reales) menciona los marcadores de simulacion
        for name in os.listdir(self.CFG):
            if name.endswith('.yaml') and name != 'sim_room.yaml':
                self.assertNotIn('publish_markers', open(os.path.join(self.CFG, name)).read(), name)
        launch = open(os.path.join(self.CFG, '..', 'launch', 'nav.launch.py')).read()
        self.assertNotIn('fake_robot', launch)
        self.assertNotIn('sim/markers', launch)


if __name__ == '__main__':
    unittest.main()
