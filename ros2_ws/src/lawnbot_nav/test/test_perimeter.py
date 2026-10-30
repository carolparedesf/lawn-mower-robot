import math
import unittest

from lawnbot_nav.perimeter_logic import (CONTACT_BACK, CONTACT_TURN, CORNER, FAILED, FOLLOW, SEARCH_ARC,
                                         SEARCH_FORWARD,
                                         PerimeterCore, PerimeterInputs, PerimeterParams)
from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_world import SimWorld

DT = 0.05
SENSOR_DT = 0.2


def simulate(world, params, start, tmax=300.0, tau=0.1, side_sonar_yaw=None, use_sonar=True):
    """Lazo cerrado con el mundo simulado. Los sonares laterales miran a +-side_sonar_angle_deg."""
    core = PerimeterCore(params)
    robot = SimRobot(*start, tau=tau)
    yaw = params.side_sonar_angle_deg if side_sonar_yaw is None else side_sonar_yaw
    offsets = {'left': yaw, 'center': 0.0, 'right': -yaw}
    side = 'left' if params.side == 'left' else 'right'
    side_bumper = 'bumper/B3_left' if params.side == 'left' else 'bumper/B4_right'
    t, next_sensor = 0.0, 0.0
    ranges, bumpers = {}, {}
    log = {'xy': [], 'states': [], 'collisions': 0, 'lateral': [], 'v': [], 'w': [], 'alerts': set(),
           'failed_at': None}
    while t < tmax and core.state != FAILED:
        if t >= next_sensor:
            pose = (robot.x, robot.y, robot.yaw)
            ranges = world.sonars(pose, offsets)
            bumpers = world.bumpers(pose)
            next_sensor += SENSOR_DT
        inp = PerimeterInputs(
            sensors_ok=True, side_range=ranges[side] if use_sonar else None,
            center_range=ranges['center'],
            side_bumper=bumpers[side_bumper],
            front_bumper=bumpers['bumper/B1_front_left'] or bumpers['bumper/B2_front_right'],
            back_bumper=bumpers['bumper/B5_back'])
        v, w = core.update(inp, t)
        log['collisions'] += world.step_robot(robot, v, w, DT)
        log['xy'].append((robot.x, robot.y))
        log['states'].append(core.state)
        log['v'].append(v)
        log['w'].append(w)
        if core.lateral is not None:
            log['lateral'].append(core.lateral)
        if core.alert:
            log['alerts'].add(core.alert)
        t += DT
    log['failed_at'] = t if core.state == FAILED else None
    return core, robot, log


def wall_gap(world, xy):
    """Distancia del punto a la pared mas cercana."""
    return world.distance(xy)


class FollowTests(unittest.TestCase):
    def test_recorre_el_perimetro_de_un_cuarto_con_la_pared_a_la_derecha(self):
        world = SimWorld.room(10.0, 8.0)
        params = PerimeterParams(side='right')
        core, robot, log = simulate(world, params, start=(2.0, 1.0, 0.0), tmax=420.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['collisions'], 0)
        # da la vuelta completa: pasa cerca de las cuatro paredes y de las cuatro esquinas
        xy = log['xy']
        self.assertTrue(any(y > 7.0 for x, y in xy), 'no llego a la pared norte')
        self.assertTrue(any(x > 9.0 for x, y in xy), 'no llego a la pared este')
        self.assertTrue(any(x < 1.0 for x, y in xy), 'no llego a la pared oeste')
        self.assertGreaterEqual(sum(1 for a, b in zip(log['states'], log['states'][1:]) if b == CORNER and a != CORNER), 4)
        # mantiene la distancia a la pared la mayor parte del tiempo
        settled = [d for d in log['lateral'][200:]]
        near = [d for d in settled if abs(d - params.target_distance) < 0.15]
        self.assertGreater(len(near) / len(settled), 0.7)
        self.assertLessEqual(max(log['v']), params.linear + 1e-9)
        self.assertLessEqual(max(abs(w) for w in log['w']), params.max_angular + 0.11)

    def test_pared_a_la_izquierda_es_simetrica(self):
        world = SimWorld.room(10.0, 8.0)
        core, robot, log = simulate(world, PerimeterParams(side='left'), start=(2.0, 7.0, 0.0), tmax=420.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['collisions'], 0)
        xy = log['xy']
        self.assertTrue(any(y < 1.0 for x, y in xy) and any(x > 9.0 for x, y in xy) and any(x < 1.0 for x, y in xy))

    def test_esquina_exterior_rodeando_un_bloque(self):
        # un bloque de 3 x 2 m en el medio: el robot lo rodea con el bloque a su derecha (sentido horario)
        world = SimWorld(walls=SimWorld.box(4.0, 3.0, 7.0, 5.0))
        params = PerimeterParams(side='right')
        core, robot, log = simulate(world, params, start=(3.6, 4.0, math.pi / 2), tmax=300.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['collisions'], 0)
        self.assertIn(SEARCH_ARC, log['states'])                 # las esquinas exteriores usan la busqueda
        # recorrio los cuatro lados del bloque
        xy = log['xy']
        self.assertTrue(any(y > 5.2 for x, y in xy) and any(x > 7.2 for x, y in xy)
                        and any(y < 2.8 for x, y in xy) and any(x < 3.8 for x, y in xy))
        gaps = [wall_gap(world, p) for p in xy[400:]]
        self.assertGreater(min(gaps), 0.05)

    def test_solo_bumper_avanza_por_contacto_y_si_se_traba_avisa(self):
        # Respaldo sin sonar lateral: pegado a la pared, al alejarse la cola del robot gira contra ella
        # (un diferencial gira sobre su eje), asi que puede trabarse. Lo importante: avanza un buen
        # tramo por contacto y, si se traba, lo declara (perimeter_blocked) en vez de empujar para siempre.
        world = SimWorld.room(10.0, 8.0)
        params = PerimeterParams(side='right', use_side_sonar=False)
        core, robot, log = simulate(world, params, start=(2.0, 0.7, 0.0), tmax=240.0, use_sonar=False)
        self.assertGreater(max(x for x, y in log['xy']), 4.0)
        self.assertTrue(core.state != FAILED or core.fail_reason == 'perimeter_blocked')
        if core.state == FAILED:
            self.assertEqual((log['v'][-1], log['w'][-1]), (0.0, 0.0))
            self.assertIn('perimeter_blocked', log['alerts'])

    def test_pared_perdida_busca_de_forma_acotada_y_avisa(self):
        world = SimWorld()                                       # campo abierto: nunca encuentra pared
        params = PerimeterParams(side='right', search_timeout_s=12.0)
        core, robot, log = simulate(world, params, start=(0.0, 0.0, 0.0), tmax=60.0)
        self.assertEqual(core.state, FAILED)
        self.assertEqual(core.fail_reason, 'perimeter_lost')
        self.assertIn('perimeter_lost', log['alerts'])
        self.assertAlmostEqual(log['failed_at'], 12.0, delta=0.3)
        self.assertEqual((log['v'][-1], log['w'][-1]), (0.0, 0.0))
        # hubo tramos rectos y arcos hacia el lado de la pared
        self.assertIn(SEARCH_FORWARD, log['states'])
        self.assertIn(SEARCH_ARC, log['states'])
        self.assertTrue(all(w <= 1e-9 for w in log['w']))        # pared a la derecha: nunca gira a la izquierda

    def test_esquina_interior_sin_salida_falla_por_bloqueo(self):
        # un callejon ciego de 0.5 m: el robot no puede girar adentro, el frente nunca se libera
        world = SimWorld(walls=SimWorld.box(0.0, 0.0, 0.5, 4.0))
        params = PerimeterParams(side='right', corner_timeout_s=6.0)
        core, robot, log = simulate(world, params, start=(0.25, 1.0, math.pi / 2), tmax=60.0)
        self.assertEqual(core.state, FAILED)
        self.assertEqual(core.fail_reason, 'perimeter_blocked')

    def test_sonar_lateral_mal_orientado_se_rechaza(self):
        core = PerimeterCore(PerimeterParams(side_sonar_angle_deg=0.0))
        self.assertEqual(core.update(PerimeterInputs(sensors_ok=True, side_range=0.5), 0.0), (0.0, 0.0))
        self.assertEqual((core.state, core.alert), (FAILED, 'perimeter_misconfigured'))

    def test_sin_datos_de_sensores_no_se_mueve(self):
        core = PerimeterCore(PerimeterParams())
        self.assertEqual(core.update(PerimeterInputs(sensors_ok=False), 0.0), (0.0, 0.0))
        self.assertEqual(core.alert, 'sensor_timeout')
        self.assertEqual(core.update(PerimeterInputs(sensors_ok=True), 1.0)[0], 0.15)   # y sigue al volver

    def test_el_bumper_lateral_retrocede_y_luego_se_aleja(self):
        core = PerimeterCore(PerimeterParams(side='right'))
        core.update(PerimeterInputs(sensors_ok=True, side_range=0.5, center_range=1.5), 0.0)   # FOLLOW
        contact = PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5)
        v, w = core.update(contact, 0.1)
        self.assertEqual((core.state, w), (CONTACT_BACK, 0.0))
        self.assertLess(v, 0.0)                                  # retrocede derecho, sin girar contra la pared
        v, w = core.update(contact, 1.0)                         # terminado el retroceso
        self.assertEqual(core.state, CONTACT_TURN)
        self.assertGreater(w, 0.0)                               # pared a la derecha: se aleja girando a la izquierda
        core.update(PerimeterInputs(sensors_ok=True, side_range=0.5, center_range=1.5), 3.0)
        self.assertEqual(core.state, FOLLOW)

    def test_contacto_continuo_sin_progreso_se_declara_bloqueado(self):
        core = PerimeterCore(PerimeterParams(side='right', contact_timeout_s=3.0))
        t = 0.0
        while t < 10.0 and core.state != FAILED:
            core.update(PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), t)
            t += 0.05
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_blocked'))
        self.assertAlmostEqual(t, 3.0, delta=0.2)

    def test_reset_vuelve_a_empezar(self):
        core = PerimeterCore(PerimeterParams(search_timeout_s=1.0))
        for t in (0.0, 0.5, 2.0):
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), t)
        self.assertEqual(core.state, FAILED)
        core.reset(10.0)
        self.assertEqual(core.state, SEARCH_FORWARD)


if __name__ == '__main__':
    unittest.main()
