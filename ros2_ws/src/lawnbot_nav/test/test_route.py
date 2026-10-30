import math
import random
import unittest

from lawnbot_nav.auto_start import (BLOCKED, IDLE, START, TIMEOUT, WAIT, AutoStart,
                                    AutoStartInputs)
from lawnbot_nav.pure_pursuit import (DONE, ERROR, IDLE as F_IDLE, PAUSED, RUNNING,
                                      FollowerParams, Pose, WaypointFollowerCore)
from lawnbot_nav.safety_logic import ALERT_NONE, merge_alerts, parse_alert_in
from lawnbot_nav.sim_robot import SimRobot

DT = 0.05


def zigzag(rows=6, length=8.0, spacing=0.27):
    """Lineas paralelas en zigzag: dos puntos (extremos) por linea."""
    pts = []
    for i in range(rows):
        y = i * spacing
        a, b = ((0.0, y), (length, y)) if i % 2 == 0 else ((length, y), (0.0, y))
        pts += [a, b]
    return pts


class RouteTests(unittest.TestCase):
    def make(self, **kw):
        return WaypointFollowerCore(FollowerParams(waypoints=[(0.0, 0.0)], **kw))

    def test_ruta_cargada_deja_en_idle_y_se_sigue_en_absolutas(self):
        core = self.make(relative_to_start=True)
        msg = core.set_route([(5.0, 5.0), (8.0, 5.0)])
        self.assertIn('2 puntos', msg)
        core.enable(Pose(1.0, 1.0, 0.0), 0.0)
        self.assertEqual(core.wps, [(5.0, 5.0), (8.0, 5.0)])      # no se reancla a la pose

    def test_ruta_rechazada_en_running(self):
        core = self.make()
        core.set_route([(5.0, 0.0)])
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        self.assertIn('rechazada', core.set_route([(1.0, 1.0)]))
        self.assertEqual(core.wps, [(5.0, 0.0)])

    def test_ruta_nueva_en_pausa_vuelve_a_idle(self):
        core = self.make()
        core.set_route([(5.0, 0.0)])
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        core.pause('modo')
        self.assertEqual(core.state, PAUSED)
        core.set_route([(2.0, 2.0)])
        self.assertEqual(core.state, F_IDLE)
        self.assertEqual(core.route, [(2.0, 2.0)])

    def test_ruta_vacia_la_borra(self):
        core = self.make()
        core.set_route([(5.0, 0.0)])
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        core.clear_route()
        self.assertEqual((core.state, core.route, core.wps), (F_IDLE, None, []))
        self.assertEqual(core.update(Pose(0.0, 0.0, 0.0), 1.0), (0.0, 0.0))

    def test_quita_puntos_consecutivos_repetidos(self):
        core = self.make()
        core.set_route([(1.0, 1.0), (1.0, 1.0), (3.0, 1.0)])
        self.assertEqual(core.route, [(1.0, 1.0), (3.0, 1.0)])

    def test_enable_despues_de_done_repite_la_misma_ruta(self):
        core = self.make()
        core.set_route([(1.0, 0.0)])
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        core.update(Pose(0.95, 0.0, 0.0), 1.0)
        self.assertEqual(core.state, DONE)
        core.enable(Pose(0.0, 0.0, 0.0), 2.0)
        self.assertEqual((core.state, core.k, core.wps), (RUNNING, 0, [(1.0, 0.0)]))

    def test_sin_ruta_ni_waypoints_no_arranca(self):
        core = WaypointFollowerCore(FollowerParams(waypoints=[]))
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        self.assertEqual(core.state, F_IDLE)


class ToleranceTests(unittest.TestCase):
    def tol(self, path, **kw):
        core = WaypointFollowerCore(FollowerParams(waypoints=[(0.0, 0.0)], **kw))
        core.set_route(path)
        return core.tol

    def test_lineas_separadas_027_dan_tolerancia_menor_que_la_fija(self):
        self.assertAlmostEqual(self.tol(zigzag(spacing=0.27)), 0.108)

    def test_ruta_larga_conserva_la_tolerancia_configurada(self):
        self.assertEqual(self.tol([(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)]), 0.2)

    def test_piso_minimo(self):
        self.assertEqual(self.tol([(0.0, 0.0), (0.05, 0.0), (3.0, 0.0)]), 0.05)

    def test_en_loop_cuenta_el_tramo_de_cierre(self):
        self.assertAlmostEqual(self.tol([(0.0, 0.0), (3.0, 0.0), (3.0, 0.25)], loop=True), 0.1)

    def test_las_rutas_del_yaml_no_cambian(self):
        core = WaypointFollowerCore(FollowerParams(
            waypoints=[(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0)]))
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        self.assertEqual(core.tol, 0.2)


class ZigzagTests(unittest.TestCase):
    """Lineas paralelas a 0.27 m con la tolerancia por defecto de 0.2 m."""

    def run_path(self, ratio, sigma, spacing=0.27, rows=6, length=8.0, seed=1):
        path = zigzag(rows, length, spacing)
        core = WaypointFollowerCore(FollowerParams(
            waypoints=[(0.0, 0.0)], route_tolerance_ratio=ratio))
        core.set_route(path)
        robot = SimRobot(0.0, 0.0, 0.0, tau=0.1)
        rng = random.Random(seed)
        core.enable(Pose(0.0, 0.0, 0.0), 0.0)
        t, jumps, last_k, trajectory = 0.0, 0, 0, []
        while t < 900.0 and core.state == RUNNING:
            meas = Pose(robot.x + rng.gauss(0, sigma), robot.y + rng.gauss(0, sigma), robot.yaw)
            v, w = core.update(meas, t)
            k = core.k if core.state == RUNNING else len(path)
            jumps += max(0, k - last_k - 1)
            last_k = k
            robot.step(v, w, DT)
            trajectory.append((robot.x, robot.y))
            t += DT
        misses = [min(math.dist(p, q) for q in trajectory) for p in path]
        return core, jumps, max(misses)

    def test_no_salta_ningun_waypoint_y_se_completa(self):
        for sigma in (0.0, 0.03):
            core, jumps, _ = self.run_path(0.4, sigma)
            self.assertEqual(core.state, DONE, f'sigma {sigma}')
            self.assertEqual(jumps, 0, f'sigma {sigma}')

    def test_llega_cerca_del_extremo_de_cada_linea(self):
        _, _, miss = self.run_path(0.4, 0.0)
        self.assertLess(miss, 0.108 + 0.01)

    def test_con_ruido_el_corte_de_esquina_sigue_acotado(self):
        _, _, miss = self.run_path(0.4, 0.03)
        self.assertLess(miss, 0.108 + 0.1)

    def test_la_tolerancia_derivada_corta_menos_la_esquina_que_la_fija(self):
        _, _, miss_fixed = self.run_path(1e9, 0.03)        # tolerancia fija 0.2
        _, _, miss_derived = self.run_path(0.4, 0.03)
        self.assertLess(miss_derived, miss_fixed)

    def test_espaciado_menor_que_la_tolerancia(self):
        core, jumps, miss = self.run_path(0.4, 0.0, spacing=0.15, length=6.0)
        self.assertEqual((core.state, jumps), (DONE, 0))
        self.assertLess(miss, 0.06 + 0.01)


class AutoStartTests(unittest.TestCase):
    def inputs(self, **kw):
        base = dict(follower_idle=True, has_route=True, mode_ok=True, estop=False, pose_ok=True,
                    geofence_required=True, geofence_status='OK', geofence_inside=True)
        base.update(kw)
        return AutoStartInputs(**base)

    def armed(self, now=0.0):
        auto = AutoStart(['auto_parallel'], timeout_s=30.0)
        auto.on_mode('idle', now)
        self.assertTrue(auto.on_mode('auto_parallel', now))
        return auto

    def test_el_primer_mensaje_solo_toma_la_referencia(self):
        auto = AutoStart(['auto_parallel'])
        self.assertFalse(auto.on_mode('auto_parallel', 0.0))
        self.assertEqual(auto.evaluate(1.0, self.inputs())[0], IDLE)

    def test_arranca_con_cerco_ok_y_dentro(self):
        self.assertEqual(self.armed().evaluate(1.0, self.inputs()), (START, None))

    def test_no_arranca_ni_con_cerco_fuera_ni_con_cerco_no_ok(self):
        for kw in (dict(geofence_inside=False), dict(geofence_status='NO_POLYGON'),
                   dict(geofence_status='WAITING_CONVERSION'), dict(geofence_status='NO_POSE'),
                   dict(geofence_status='INVALID:autointerseccion')):
            action, alert = self.armed().evaluate(1.0, self.inputs(**kw))
            self.assertEqual((action, alert), (BLOCKED, 'geofence_not_ready'), kw)

    def test_sin_cerco_requerido_arranca(self):
        i = self.inputs(geofence_required=False, geofence_status='NO_POLYGON', geofence_inside=False)
        self.assertEqual(self.armed().evaluate(1.0, i)[0], START)

    def test_espera_la_ruta_y_la_pose(self):
        self.assertEqual(self.armed().evaluate(1.0, self.inputs(has_route=False))[0], WAIT)
        self.assertEqual(self.armed().evaluate(1.0, self.inputs(follower_idle=False))[0], WAIT)
        self.assertEqual(self.armed().evaluate(1.0, self.inputs(pose_ok=False)), (BLOCKED, 'no_pose'))

    def test_estop_bloquea(self):
        self.assertEqual(self.armed().evaluate(1.0, self.inputs(estop=True)), (BLOCKED, 'estop'))

    def test_arranca_cuando_el_cerco_pasa_a_ok_dentro_del_plazo(self):
        auto = self.armed()
        self.assertEqual(auto.evaluate(1.0, self.inputs(geofence_status='WAITING_CONVERSION'))[0], BLOCKED)
        self.assertEqual(auto.evaluate(5.0, self.inputs())[0], START)

    def test_desiste_tras_el_plazo(self):
        auto = self.armed()
        self.assertEqual(auto.evaluate(31.0, self.inputs(geofence_inside=False)),
                         (TIMEOUT, 'auto_start_timeout'))
        self.assertEqual(auto.evaluate(32.0, self.inputs())[0], IDLE)       # ya no reintenta

    def test_salir_del_modo_cancela(self):
        auto = self.armed()
        auto.on_mode('stop', 2.0)
        self.assertEqual(auto.evaluate(3.0, self.inputs())[0], IDLE)

    def test_solo_arranca_una_vez_por_entrada(self):
        auto = self.armed()
        self.assertEqual(auto.evaluate(1.0, self.inputs())[0], START)
        self.assertEqual(auto.evaluate(2.0, self.inputs())[0], IDLE)

    def test_volver_a_elegir_el_modo_rearma(self):
        auto = self.armed()
        auto.evaluate(1.0, self.inputs())
        auto.on_mode('stop', 5.0)
        self.assertTrue(auto.on_mode('auto_parallel', 6.0))


class AlertMergeTests(unittest.TestCase):
    def test_parseo(self):
        self.assertEqual(parse_alert_in('follower:geofence_not_ready'), ('follower', 'geofence_not_ready'))
        self.assertEqual(parse_alert_in('solo_alerta'), ('desconocida', 'solo_alerta'))

    def test_sin_alertas(self):
        self.assertEqual(merge_alerts('none', {}), ALERT_NONE)

    def test_gana_la_de_mayor_prioridad(self):
        ext = {'follower': ('geofence_not_ready', 0.5)}
        self.assertEqual(merge_alerts('obstacle_near', ext), 'geofence_not_ready')
        self.assertEqual(merge_alerts('bumper_front', ext), 'bumper_front')

    def test_las_alertas_viejas_vencen(self):
        ext = {'follower': ('geofence_not_ready', 5.0)}
        self.assertEqual(merge_alerts('none', ext, ttl=3.0), ALERT_NONE)

    def test_none_externo_limpia(self):
        self.assertEqual(merge_alerts('none', {'follower': ('none', 0.1)}), ALERT_NONE)

    def test_fuentes_distintas_y_alerta_desconocida(self):
        ext = {'a': ('rara', 0.1), 'b': ('no_progress', 0.1)}
        self.assertEqual(merge_alerts('none', ext), 'no_progress')
        self.assertEqual(merge_alerts('none', {'a': ('rara', 0.1)}), 'rara')


if __name__ == '__main__':
    unittest.main()
