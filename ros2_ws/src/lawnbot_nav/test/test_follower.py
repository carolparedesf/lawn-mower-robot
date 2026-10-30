import math
import random
import unittest

from lawnbot_nav.pure_pursuit import (DONE, ERROR, IDLE, PAUSED, RUNNING, FollowerParams,
                                      Pose, WaypointFollowerCore)
from lawnbot_nav.sim_robot import SimRobot

SQUARE = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
DT = 0.05   # 20 Hz


def simulate(params, start=(0.0, 0.0, 0.0), tmax=600.0, **sim_kwargs):
    core = WaypointFollowerCore(params)
    robot = SimRobot(*start, **sim_kwargs)
    t = 0.0
    core.enable(Pose(robot.x, robot.y, robot.yaw), t)
    log = []
    while t < tmax and core.state == RUNNING:
        v, w = core.update(Pose(robot.x, robot.y, robot.yaw), t)
        robot.step(v, w, DT)
        t += DT
        log.append((t, robot.x, robot.y, v, w, core.info.get('phase')))
    return core, robot, t, log


def dist_to_polyline(pt, poly):
    best = math.inf
    for (ax, ay), (bx, by) in zip(poly[:-1], poly[1:]):
        abx, aby = bx - ax, by - ay
        l2 = abx * abx + aby * aby
        s = 0.0 if l2 == 0 else max(0.0, min(1.0, ((pt[0] - ax) * abx + (pt[1] - ay) * aby) / l2))
        best = min(best, math.hypot(pt[0] - (ax + s * abx), pt[1] - (ay + s * aby)))
    return best


class SquareTests(unittest.TestCase):
    def test_cuadrado_relativo_se_completa_y_respeta_limites(self):
        p = FollowerParams(waypoints=SQUARE)
        start = (2.0, 1.0, 0.5)
        core, robot, t, log = simulate(p, start)
        self.assertEqual(core.state, DONE)
        self.assertLess(math.hypot(robot.x - start[0], robot.y - start[1]), p.goal_tolerance + 0.05)
        self.assertLessEqual(max(abs(r[3]) for r in log), p.max_linear + 1e-9)
        self.assertGreaterEqual(min(r[3] for r in log), 0.0)           # nunca retrocede
        self.assertLessEqual(max(abs(r[4]) for r in log), p.max_angular + 1e-9)
        # Pasa por las cuatro esquinas (en el marco de la pose inicial)
        c, s = math.cos(start[2]), math.sin(start[2])
        corners = [(start[0] + c * x - s * y, start[1] + s * x + c * y)
                   for x, y in SQUARE[1:4]]
        for corner in corners:
            self.assertLess(min(math.hypot(r[1] - corner[0], r[2] - corner[1]) for r in log),
                            p.goal_tolerance + 0.02)
        # Se mantiene cerca del contorno ideal
        poly = [(start[0], start[1])] + [(start[0] + c * x - s * y, start[1] + s * x + c * y)
                                         for x, y in SQUARE]
        self.assertLess(max(dist_to_polyline((r[1], r[2]), poly) for r in log), 0.35)

    def test_gira_en_el_lugar_en_las_esquinas(self):
        core, _, _, log = simulate(FollowerParams(waypoints=SQUARE))
        rotating = [r for r in log if r[5] == 'rotate']
        self.assertGreater(len(rotating), 20)
        self.assertTrue(all(r[3] == 0.0 and abs(r[4]) > 0 for r in rotating))
        # Sin oscilaciones entre giro y avance
        switches = sum(1 for a, b in zip(log[:-1], log[1:]) if a[5] != b[5])
        self.assertLessEqual(switches, 16)

    def test_waypoints_absolutos_en_map(self):
        p = FollowerParams(waypoints=SQUARE, relative_to_start=False)
        core, robot, _, log = simulate(p, start=(-0.5, -0.3, 0.2))
        self.assertEqual(core.state, DONE)
        self.assertLess(math.hypot(robot.x - 0.0, robot.y - 0.0), p.goal_tolerance + 0.05)
        self.assertLess(min(math.hypot(r[1] - 3.0, r[2] - 3.0) for r in log), 0.22)

    def test_con_lazo_de_velocidad_lento(self):
        core, _, _, _ = simulate(FollowerParams(waypoints=SQUARE), tau=0.3)
        self.assertEqual(core.state, DONE)

    def test_modo_loop_sigue_despues_de_la_primera_vuelta(self):
        p = FollowerParams(waypoints=SQUARE, loop=True)
        core, _, _, _ = simulate(p, tmax=100.0)
        self.assertEqual(core.state, RUNNING)
        self.assertGreaterEqual(core.laps, 1)

    def test_primer_waypoint_en_el_origen_se_salta(self):
        core = WaypointFollowerCore(FollowerParams(waypoints=SQUARE))
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        core.update(Pose(0.0, 0.0, 0.0), 0.0)
        self.assertEqual(core.k, 1)


class StateTests(unittest.TestCase):
    def running_core(self):
        core = WaypointFollowerCore(FollowerParams(waypoints=SQUARE))
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        core.update(Pose(0.0, 0.0, 0.0), 0.0)
        return core

    def test_arranca_en_idle_y_no_se_mueve(self):
        core = WaypointFollowerCore(FollowerParams(waypoints=SQUARE))
        self.assertEqual(core.state, IDLE)
        self.assertEqual(core.update(Pose(0.0, 0.0, 0.0), 0.0), (0.0, 0.0))

    def test_enable_repetido_es_idempotente(self):
        core = self.running_core()
        core.update(Pose(2.9, 0.0, 0.0), 1.0)
        k = core.k
        msg = core.enable(Pose(5.0, 5.0, 1.0), 2.0)
        self.assertIn('RUNNING', msg)
        self.assertEqual((core.state, core.k), (RUNNING, k))

    def test_pausa_externa_exige_false_y_luego_true(self):
        core = self.running_core()
        k = core.k
        core.pause('manual')
        self.assertEqual(core.state, PAUSED)
        self.assertEqual(core.update(Pose(0.0, 0.0, 0.0), 1.0), (0.0, 0.0))
        self.assertIn('ignorado', core.enable(Pose(0.0, 0.0, 0.0), 2.0))
        self.assertEqual(core.state, PAUSED)
        core.disable()
        self.assertIn('reanudado', core.enable(Pose(0.0, 0.0, 0.0), 3.0))
        self.assertEqual((core.state, core.k), (RUNNING, k))

    def test_enable_false_pausa_y_true_reanuda(self):
        core = self.running_core()
        core.disable()
        self.assertEqual(core.state, PAUSED)
        core.enable(Pose(0.0, 0.0, 0.0), 1.0)
        self.assertEqual(core.state, RUNNING)

    def test_pausa_en_idle_no_desarma_el_enable(self):
        core = WaypointFollowerCore(FollowerParams(waypoints=SQUARE))
        core.pause('manual')
        self.assertTrue(core.armed)

    def test_enable_despues_de_done_reinicia(self):
        core, robot, t, _ = simulate(FollowerParams(waypoints=SQUARE))
        self.assertEqual(core.state, DONE)
        core.enable(Pose(robot.x, robot.y, robot.yaw), t)
        self.assertEqual((core.state, core.k), (RUNNING, 0))

    def test_sin_waypoints_no_arranca(self):
        core = WaypointFollowerCore(FollowerParams(waypoints=[]))
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        self.assertEqual(core.state, IDLE)

    def test_perdida_corta_de_pose_reanuda(self):
        core = self.running_core()
        for i in range(10):                       # 1 s sin pose
            self.assertEqual(core.pose_unavailable(1.0 + i * 0.1), (0.0, 0.0))
        self.assertEqual(core.state, RUNNING)
        v, w = core.update(Pose(0.0, 0.0, 0.0), 2.1)
        self.assertEqual(core.state, RUNNING)
        self.assertGreater(v, 0.0)

    def test_perdida_larga_de_pose_da_error(self):
        core = self.running_core()
        t = 1.0
        while t < 6.0 and core.state == RUNNING:
            self.assertEqual(core.pose_unavailable(t), (0.0, 0.0))
            t += 0.05
        self.assertEqual((core.state, core.reason), (ERROR, 'tf_lost'))
        self.assertAlmostEqual(t, 1.0 + 3.0, delta=0.2)

    def test_perdida_de_pose_no_cuenta_como_falta_de_avance(self):
        p = FollowerParams(waypoints=SQUARE, progress_timeout=5.0, tf_abort_s=10.0)
        core = WaypointFollowerCore(p)
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        core.update(Pose(0.0, 0.0, 0.0), 0.0)
        for i in range(80):                       # 8 s sin pose (mas que progress_timeout)
            core.pose_unavailable(0.1 + i * 0.1)
        core.update(Pose(0.0, 0.0, 0.0), 8.2)
        self.assertEqual(core.state, RUNNING)


class ProgressTests(unittest.TestCase):
    def test_girar_en_el_lugar_reduciendo_el_error_cuenta_como_avance(self):
        # Rumbo 90 grados desviado y giro lento: tarda mas que progress_timeout, pero no es error
        p = FollowerParams(waypoints=[(5.0, 0.0)], relative_to_start=False,
                           max_angular=0.2, progress_timeout=3.0)
        core = WaypointFollowerCore(p)
        robot = SimRobot(0.0, 0.0, math.pi / 2)
        core.enable(Pose(robot.x, robot.y, robot.yaw), 0.0)
        t = 0.0
        while t < 8.0:   # girar 90 grados a 0.2 rad/s tarda ~6.1 s (> progress_timeout)
            v, w = core.update(Pose(robot.x, robot.y, robot.yaw), t)
            robot.step(v, w, DT)
            t += DT
            self.assertNotEqual(core.state, ERROR, f'error a t={t:.1f}')
        self.assertEqual(core.info['phase'], 'drive')

    def test_ruedas_que_no_giran_dan_error_por_falta_de_avance(self):
        p = FollowerParams(waypoints=SQUARE, progress_timeout=5.0)
        core, _, t, _ = simulate(p, wheel_deadzone=1.0)
        self.assertEqual((core.state, core.reason), (ERROR, 'no_progress'))
        self.assertAlmostEqual(t, 5.0, delta=0.3)

    def test_zona_muerta_sin_piso_de_velocidad_se_atasca_cerca_del_objetivo(self):
        p = FollowerParams(waypoints=[(3.0, 0.0)], relative_to_start=False,
                           progress_timeout=5.0)
        core, _, _, _ = simulate(p, wheel_deadzone=0.08)
        self.assertEqual((core.state, core.reason), (ERROR, 'no_progress'))

    def test_min_linear_y_min_turn_resuelven_la_zona_muerta(self):
        p = FollowerParams(waypoints=SQUARE, min_linear=0.1, min_turn=0.5, progress_timeout=5.0)
        core, _, _, _ = simulate(p, start=(1.0, 1.0, 0.7), wheel_deadzone=0.08)
        self.assertEqual(core.state, DONE)


class InvariantTests(unittest.TestCase):
    def test_limites_con_poses_y_rutas_aleatorias(self):
        rng = random.Random(7)
        for _ in range(60):
            n = rng.randint(1, 6)
            wps = [(rng.uniform(-6, 6), rng.uniform(-6, 6)) for _ in range(n)]
            p = FollowerParams(waypoints=wps, relative_to_start=rng.random() < 0.5,
                               loop=rng.random() < 0.3,
                               max_linear=rng.uniform(0.1, 0.5),
                               max_angular=rng.uniform(0.3, 1.2),
                               min_linear=rng.choice([0.0, 0.05]),
                               min_turn=rng.choice([0.0, 0.2]))
            start = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi))
            core = WaypointFollowerCore(p)
            robot = SimRobot(*start)
            core.enable(Pose(*start), 0.0)
            t = 0.0
            while t < 60.0 and core.state == RUNNING:
                v, w = core.update(Pose(robot.x, robot.y, robot.yaw), t)
                self.assertFalse(math.isnan(v) or math.isnan(w))
                self.assertGreaterEqual(v, 0.0)
                self.assertLessEqual(v, p.max_linear + 1e-9)
                self.assertLessEqual(abs(w), p.max_angular + 1e-9)
                robot.step(v, w, DT)
                t += DT


if __name__ == '__main__':
    unittest.main()
