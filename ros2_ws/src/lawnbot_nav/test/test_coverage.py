import math
import random
import unittest

from lawnbot_nav.coverage_logic import (CoverageParams, ERR_NOT_CONVEX, ERR_NO_POLYGON, format_status,
                                        generate_coverage, inset_convex, is_convex,
                                        line_interval, signed_area, swath_coverage)
from lawnbot_nav.geofence_logic import signed_distance
from lawnbot_nav.pure_pursuit import DONE, RUNNING, FollowerParams, Pose, WaypointFollowerCore
from lawnbot_nav.sim_robot import SimRobot

RECT = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
TRAPEZOID = [(0.0, 0.0), (12.0, 0.0), (9.0, 6.0), (2.0, 6.0)]
TRIANGLE = [(0.0, 0.0), (10.0, 0.0), (5.0, 8.0)]
L_SHAPE = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (4.0, 4.0), (4.0, 10.0), (0.0, 10.0)]


def rotate(poly, deg, about=(0.0, 0.0)):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(about[0] + c * (x - about[0]) - s * (y - about[1]),
             about[1] + s * (x - about[0]) + c * (y - about[1])) for x, y in poly]


class ConvexityTests(unittest.TestCase):
    def test_convexos(self):
        for poly in (RECT, TRAPEZOID, TRIANGLE, list(reversed(RECT))):
            self.assertTrue(is_convex(poly))

    def test_no_convexos(self):
        self.assertFalse(is_convex(L_SHAPE))
        self.assertFalse(is_convex([(0, 0), (10, 10), (10, 0), (0, 10)]))

    def test_vertice_colineal_permitido(self):
        self.assertTrue(is_convex([(0, 0), (5, 0), (10, 0), (10, 6), (0, 6)]))


class InsetTests(unittest.TestCase):
    def test_recorte_de_un_rectangulo(self):
        region = inset_convex(RECT, 1.0)
        xs = sorted({round(p[0], 6) for p in region})
        ys = sorted({round(p[1], 6) for p in region})
        self.assertEqual((xs, ys), ([1.0, 9.0], [1.0, 5.0]))

    def test_todos_los_puntos_a_la_distancia_del_margen(self):
        region = inset_convex(TRAPEZOID, 0.8)
        for p in region:
            self.assertGreaterEqual(signed_distance(p, TRAPEZOID), 0.8 - 1e-6)

    def test_region_vacia(self):
        self.assertEqual(inset_convex([(0, 0), (3, 0), (3, 2), (0, 2)], 1.0), [])
        self.assertEqual(inset_convex(RECT, 3.0), [])

    def test_interseccion_de_linea(self):
        iv = line_interval(RECT, (0.0, 3.0), (1.0, 0.0))
        self.assertAlmostEqual(iv[0], 0.0)
        self.assertAlmostEqual(iv[1], 10.0)
        self.assertIsNone(line_interval(RECT, (0.0, 7.0), (1.0, 0.0)))


class GenerationTests(unittest.TestCase):
    def test_rectangulo_con_los_valores_por_defecto(self):
        r = generate_coverage(RECT, CoverageParams())
        self.assertTrue(r.ok, r.error)
        self.assertAlmostEqual(r.sweep_deg, 0.0)               # paralelo al lado de 10 m
        self.assertEqual(r.n_lines, 16)                        # region de 4 m de ancho / 0.27
        self.assertLessEqual(r.spacing, 0.30 * 0.90 + 1e-9)
        self.assertAlmostEqual(r.spacing, 4.0 / 15)
        self.assertGreater(r.length, 16 * 8.0)
        self.assertGreater(r.est_time_s, r.length / 0.2)

    def test_la_ruta_alterna_el_sentido(self):
        r = generate_coverage(RECT, CoverageParams(start_nearest=False))
        for i, (s, e) in enumerate(r.lines):
            self.assertEqual(e[0] > s[0], i % 2 == 0)
        self.assertEqual(r.path, [pt for line in r.lines for pt in line])

    def test_barrido_paralelo_al_lado_mas_largo(self):
        for angle in (0.0, 30.0, 77.0, 135.0):
            r = generate_coverage(rotate(RECT, angle), CoverageParams())
            self.assertTrue(r.ok, r.error)
            direction = r.sweep_deg % 180.0
            self.assertTrue(abs(direction - angle % 180.0) < 1e-6, f'{angle}: {direction}')

    def test_todos_los_puntos_de_la_ruta_quedan_dentro_con_margen(self):
        p = CoverageParams()
        for poly in (RECT, TRAPEZOID, TRIANGLE, rotate(RECT, 37.0), rotate(TRAPEZOID, -20.0)):
            r = generate_coverage(poly, p)
            self.assertTrue(r.ok, r.error)
            oriented = poly if signed_area(poly) > 0 else list(reversed(poly))
            for pt in r.path:
                self.assertGreaterEqual(signed_distance(pt, oriented), p.margin - p.turn_extension - 1e-6)

    def test_los_extremos_se_prolongan_dentro_del_margen(self):
        plain = generate_coverage(RECT, CoverageParams(turn_extension=0.0, start_nearest=False))
        ext = generate_coverage(RECT, CoverageParams(turn_extension=0.15, start_nearest=False))
        self.assertAlmostEqual(plain.path[0][0], 1.0)
        self.assertAlmostEqual(ext.path[0][0], 0.85)
        self.assertAlmostEqual(ext.path[1][0], 9.15)

    def test_angulo_manual(self):
        r = generate_coverage(RECT, CoverageParams(sweep_angle_deg=90.0))
        self.assertTrue(r.ok)
        self.assertAlmostEqual(abs(math.sin(math.radians(r.sweep_deg))), 1.0)
        # lineas verticales: la dimension perpendicular es la de 10 m
        self.assertEqual(r.n_lines, math.ceil(8.0 / (0.3 * 0.9) - 1e-9) + 1)

    def test_empieza_por_el_extremo_mas_cercano_al_robot(self):
        corners = {'abajo-izq': (0.5, 0.5), 'abajo-der': (9.5, 0.5), 'arriba-izq': (0.5, 5.5), 'arriba-der': (9.5, 5.5)}
        for name, robot in corners.items():
            r = generate_coverage(RECT, CoverageParams(), robot_xy=robot)
            first = r.path[0]
            self.assertLess(math.dist(first, robot), 2.0, name)
            # ninguna otra esquina de la ruta esta mas cerca
            other = [r.lines[0][0], r.lines[0][1], r.lines[-1][0], r.lines[-1][1]]
            self.assertEqual(min(other, key=lambda q: math.dist(q, robot)), first)

    def test_sin_start_nearest_empieza_siempre_igual(self):
        a = generate_coverage(RECT, CoverageParams(start_nearest=False), robot_xy=(9.0, 5.0))
        b = generate_coverage(RECT, CoverageParams(start_nearest=False))
        self.assertEqual(a.path, b.path)

    def test_poligono_horario_se_acepta(self):
        r = generate_coverage(list(reversed(RECT)), CoverageParams())
        self.assertTrue(r.ok, r.error)

    def test_triangulo(self):
        r = generate_coverage(TRIANGLE, CoverageParams())
        self.assertTrue(r.ok, r.error)
        self.assertGreater(r.n_lines, 10)


class ErrorTests(unittest.TestCase):
    def err(self, poly, **kw):
        r = generate_coverage(poly, CoverageParams(**kw))
        self.assertFalse(r.ok)
        self.assertEqual(r.path, [])
        return r.error

    def test_concavo(self):
        self.assertEqual(self.err(L_SHAPE), ERR_NOT_CONVEX)

    def test_autointersectado(self):
        self.assertEqual(self.err([(0, 0), (10, 10), (10, 0), (0, 10)]), ERR_NOT_CONVEX)

    def test_sin_poligono(self):
        self.assertEqual(self.err([]), ERR_NO_POLYGON)

    def test_margen_que_no_deja_region(self):
        msg = self.err([(0, 0), (3, 0), (3, 2), (0, 2)], margin=1.0)
        self.assertIn('region vacia', msg)
        self.assertIn('2.0 m', msg)                    # indica el ancho del cerco

    def test_parametros_invalidos(self):
        self.assertIn('cutting_width', self.err(RECT, cutting_width=0.0))
        self.assertIn('overlap', self.err(RECT, overlap=1.0))
        self.assertIn('margin', self.err(RECT, margin=-1.0))
        self.assertIn('turn_extension', self.err(RECT, turn_extension=1.5, margin=1.0))

    def test_estado_de_texto(self):
        self.assertTrue(format_status(generate_coverage(RECT, CoverageParams())).startswith('OK lineas=16 '))
        self.assertTrue(format_status(generate_coverage(L_SHAPE, CoverageParams())).startswith('ERROR:'))


class SweepWithRobotTests(unittest.TestCase):
    """El seguidor recorre el barrido con el robot simulado (el motor de fake_robot)."""

    DT = 0.05

    def drive(self, polygon, params=None, sigma=0.0, seed=1, tau=0.1):
        params = params or CoverageParams()
        result = generate_coverage(polygon, params)
        self.assertTrue(result.ok, result.error)
        core = WaypointFollowerCore(FollowerParams(waypoints=[(0.0, 0.0)]))
        core.set_route(result.path)
        start = result.path[0]
        robot = SimRobot(start[0], start[1], 0.0, tau=tau)
        rng = random.Random(seed)
        core.enable(Pose(robot.x, robot.y, robot.yaw), 0.0)
        t, trajectory, jumps, last_k = 0.0, [], 0, 0
        while t < 2400.0 and core.state == RUNNING:
            meas = Pose(robot.x + rng.gauss(0, sigma), robot.y + rng.gauss(0, sigma), robot.yaw)
            v, w = core.update(meas, t)
            k = core.k if core.state == RUNNING else len(result.path)
            jumps += max(0, k - last_k - 1)
            last_k = k
            robot.step(v, w, self.DT)
            if int(t / self.DT) % 5 == 0:
                trajectory.append((robot.x, robot.y))
            t += self.DT
        return result, core, trajectory, jumps, t

    def test_cubre_casi_toda_la_region_de_trabajo(self):
        poly = [(0.0, 0.0), (6.0, 0.0), (6.0, 5.0), (0.0, 5.0)]
        params = CoverageParams()
        result, core, traj, jumps, t = self.drive(poly, params)
        self.assertEqual(core.state, DONE, core.reason)
        self.assertEqual(jumps, 0)
        ratio = swath_coverage(result.region, traj, params.cutting_width / 2.0)
        self.assertGreater(ratio, 0.97, f'cobertura {ratio:.3f}')
        # y nunca sale de la region ampliada por la prolongacion
        for pt in traj:
            self.assertGreaterEqual(signed_distance(pt, poly), params.margin - params.turn_extension - 0.25)

    def test_cobertura_con_ruido_de_posicion(self):
        poly = [(0.0, 0.0), (6.0, 0.0), (6.0, 5.0), (0.0, 5.0)]
        params = CoverageParams()
        result, core, traj, jumps, t = self.drive(poly, params, sigma=0.03)
        self.assertEqual(core.state, DONE, core.reason)
        self.assertGreater(swath_coverage(result.region, traj, params.cutting_width / 2.0), 0.93)

    def test_poligono_inclinado(self):
        poly = rotate([(0.0, 0.0), (7.0, 0.0), (7.0, 4.5), (0.0, 4.5)], 33.0, about=(3.0, 3.0))
        params = CoverageParams()
        result, core, traj, jumps, t = self.drive(poly, params)
        self.assertEqual(core.state, DONE, core.reason)
        self.assertGreater(swath_coverage(result.region, traj, params.cutting_width / 2.0), 0.96)


if __name__ == '__main__':
    unittest.main()
