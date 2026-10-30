import math
import unittest

from lawnbot_nav.perimeter_logic import (CONTACT_BACK, CONTACT_TURN, CORNER, FAILED, FOLLOW, SEARCH_ARC,
                                         SEARCH_FORWARD, UNSTICK, PerimeterCore, PerimeterInputs, PerimeterParams)
from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_world import SimWorld

DT = 0.05
SENSOR_DT = 0.2        # el firmware publica sonares y bumpers a 5 Hz


def simulate(world, params, start, tmax=300.0, tau=0.1, side_sonar_yaw=None):
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
           'failed_at': None, 'contacts': 0}
    while t < tmax and core.state != FAILED:
        if t >= next_sensor:
            pose = (robot.x, robot.y, robot.yaw)
            ranges = world.sonars(pose, offsets)
            bumpers = world.bumpers(pose)
            log['contacts'] += any(bumpers.values())
            next_sensor += SENSOR_DT
        inp = PerimeterInputs(
            sensors_ok=True, side_range=ranges[side] if params.use_side_sonar else None,
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


def reached(xy, **kw):
    return (any(y > kw['n'] for x, y in xy) if 'n' in kw else True) and \
           (any(x > kw['e'] for x, y in xy) if 'e' in kw else True) and \
           (any(y < kw['s'] for x, y in xy) if 's' in kw else True) and \
           (any(x < kw['w'] for x, y in xy) if 'w' in kw else True)


class BumperModeTests(unittest.TestCase):
    """Modo base: solo contacto con los bumpers (use_side_sonar false por defecto)."""

    def test_el_modo_base_no_usa_el_sonar_lateral(self):
        self.assertFalse(PerimeterParams().use_side_sonar)
        self.assertFalse(PerimeterCore(PerimeterParams()).sonar_enabled)

    def test_recorre_el_perimetro_de_un_cuarto_con_la_pared_a_la_derecha(self):
        world = SimWorld.room(10.0, 8.0)
        core, robot, log = simulate(world, PerimeterParams(side='right'), start=(2.0, 0.7, 0.0), tmax=600.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertTrue(reached(log['xy'], n=7.0, e=9.0, w=1.0), 'no dio la vuelta completa')
        self.assertGreater(log['contacts'], 10)                       # sigue el borde tocandolo
        self.assertLessEqual(max(log['v']), 0.15 + 1e-9)

    def test_pared_a_la_izquierda_es_simetrica(self):
        world = SimWorld.room(10.0, 8.0)
        core, robot, log = simulate(world, PerimeterParams(side='left'), start=(2.0, 7.3, 0.0), tmax=600.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertTrue(reached(log['xy'], s=1.0, e=9.0, w=1.0))

    def test_rodea_un_bloque_por_contacto(self):
        world = SimWorld(walls=SimWorld.box(4.0, 3.0, 7.0, 5.0))
        core, robot, log = simulate(world, PerimeterParams(side='right'), start=(3.0, 3.5, math.pi / 2), tmax=900.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        xy = log['xy']
        self.assertTrue(any(y > 5.1 for x, y in xy) and any(x > 7.1 for x, y in xy)
                        and any(y < 2.9 for x, y in xy) and any(x < 3.9 for x, y in xy), 'no rodeo el bloque')

    def test_retrocede_derecho_antes_de_girar(self):
        p = PerimeterParams(side='right')
        core = PerimeterCore(p)
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        contact = PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5)
        v, w = core.update(contact, 1.0)
        self.assertEqual((core.state, w), (CONTACT_BACK, 0.0))
        self.assertLess(v, 0.0)                                   # primero retrocede recto, sin girar
        back_time = p.contact_back_distance / p.contact_back_speed
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 1.0 + back_time - 0.05)
        self.assertEqual((core.state, w), (CONTACT_BACK, 0.0))
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 1.0 + back_time + 0.01)
        self.assertEqual((core.state, v), (CONTACT_TURN, 0.0))    # despues gira en el lugar
        self.assertGreater(w, 0.0)                                # pared a la derecha: se aleja a la izquierda
        turn_time = math.radians(p.contact_turn_angle_deg) / p.contact_turn_speed
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 1.0 + back_time + 0.01 + turn_time + 0.05)
        self.assertEqual(core.state, FOLLOW)

    def test_el_giro_tras_un_contacto_frontal_es_mayor(self):
        p = PerimeterParams(side='right', contact_turn_angle_deg=20.0, front_contact_turn_angle_deg=70.0)
        for front, expected in ((False, 20.0), (True, 70.0)):
            core = PerimeterCore(p)
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
            core.update(PerimeterInputs(sensors_ok=True, side_bumper=not front, front_bumper=front, center_range=1.5), 1.0)
            t = 1.0 + p.contact_back_distance / p.contact_back_speed + 0.01
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), t)
            self.assertEqual(core.state, CONTACT_TURN)
            duration = math.radians(expected) / p.contact_turn_speed
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), t + duration - 0.05)
            self.assertEqual(core.state, CONTACT_TURN)
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), t + duration + 0.05)
            self.assertEqual(core.state, FOLLOW)

    def test_el_bumper_trasero_corta_el_retroceso(self):
        core = PerimeterCore(PerimeterParams())
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        core.update(PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), 1.0)
        v, w = core.update(PerimeterInputs(sensors_ok=True, back_bumper=True, center_range=1.5), 1.1)
        self.assertEqual((core.state, v), (CONTACT_TURN, 0.0))

    def test_sin_contacto_curva_hacia_la_pared(self):
        core = PerimeterCore(PerimeterParams(side='right'))
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        core.state = FOLLOW
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.1)
        self.assertEqual((v, w), (0.15, -0.3))                    # pared a la derecha: curva a la derecha
        core = PerimeterCore(PerimeterParams(side='left'))
        core.state = FOLLOW
        self.assertEqual(core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.1), (0.15, 0.3))

    def test_contacto_continuo_intenta_liberarse_y_luego_se_declara_bloqueado(self):
        p = PerimeterParams(side='right', contact_timeout_s=3.0, unstick_attempts=1)
        core = PerimeterCore(p)
        t, visited, first_unstick = 0.0, set(), None
        while t < 20.0 and core.state != FAILED:
            v, w = core.update(PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), t)
            visited.add(core.state)
            if core.state == UNSTICK and first_unstick is None:
                first_unstick = (t, v, w)
            t += 0.05
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_blocked'))
        self.assertIn(UNSTICK, visited)
        self.assertAlmostEqual(first_unstick[0], 3.0, delta=0.2)            # al vencer contact_timeout_s
        self.assertEqual(first_unstick[1], -p.unstick_speed)                # retrocede
        self.assertEqual(first_unstick[2], -p.unstick_turn)                 # pared a la derecha: arco hacia la izquierda... de espaldas
        self.assertLess(t, 14.0)
        self.assertEqual(core.unstick_count, 1)

    def test_sin_intentos_de_liberacion_falla_directo(self):
        core = PerimeterCore(PerimeterParams(side='right', contact_timeout_s=2.0, unstick_attempts=0))
        t = 0.0
        while t < 10.0 and core.state != FAILED:
            core.update(PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), t)
            t += 0.05
        self.assertEqual(core.fail_reason, 'perimeter_blocked')
        self.assertAlmostEqual(t, 2.0, delta=0.2)

    def test_el_arco_de_liberacion_se_invierte_con_el_lado(self):
        for side, expected in (('right', -0.4), ('left', 0.4)):
            core = PerimeterCore(PerimeterParams(side=side, contact_timeout_s=1.0))
            t, out = 0.0, None
            while t < 5.0 and out is None:
                v, w = core.update(PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), t)
                if core.state == UNSTICK:
                    out = (v, w)
                t += 0.05
            self.assertEqual(out, (-0.1, expected))

    def test_pared_perdida_busca_de_forma_acotada_y_avisa(self):
        world = SimWorld()                                       # campo abierto: nunca encuentra pared
        params = PerimeterParams(side='right', search_timeout_s=12.0)
        core, robot, log = simulate(world, params, start=(0.0, 0.0, 0.0), tmax=60.0)
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_lost'))
        self.assertIn('perimeter_lost', log['alerts'])
        self.assertAlmostEqual(log['failed_at'], 12.0, delta=0.3)
        self.assertEqual((log['v'][-1], log['w'][-1]), (0.0, 0.0))
        self.assertIn(SEARCH_FORWARD, log['states'])
        self.assertIn(SEARCH_ARC, log['states'])
        self.assertTrue(all(w <= 1e-9 for w in log['w']))        # pared a la derecha: nunca gira a la izquierda

    def test_callejon_ciego_falla_por_bloqueo(self):
        world = SimWorld(walls=SimWorld.box(0.0, 0.0, 0.5, 4.0))
        core, robot, log = simulate(world, PerimeterParams(side='right', corner_timeout_s=6.0),
                                    start=(0.25, 1.0, math.pi / 2), tmax=90.0)
        self.assertEqual(core.state, FAILED)
        self.assertEqual(core.fail_reason, 'perimeter_blocked')

    def test_sin_datos_de_sensores_no_se_mueve(self):
        core = PerimeterCore(PerimeterParams())
        self.assertEqual(core.update(PerimeterInputs(sensors_ok=False), 0.0), (0.0, 0.0))
        self.assertEqual(core.alert, 'sensor_timeout')
        self.assertEqual(core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 1.0)[0], 0.15)

    def test_esquina_interior_anticipada_con_el_sonar_central(self):
        core = PerimeterCore(PerimeterParams(side='right'))
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), 0.1)
        self.assertEqual((core.state, v), (CORNER, 0.0))
        self.assertGreater(w, 0.0)                                # pared a la derecha: gira a la izquierda

    def test_el_sonar_central_se_puede_desactivar(self):
        core = PerimeterCore(PerimeterParams(use_center_sonar=False))
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), 0.1)
        self.assertNotEqual(core.state, CORNER)

    def test_reset_vuelve_a_empezar(self):
        core = PerimeterCore(PerimeterParams(search_timeout_s=1.0))
        for t in (0.0, 0.5, 2.0):
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), t)
        self.assertEqual(core.state, FAILED)
        core.reset(10.0)
        self.assertEqual(core.state, SEARCH_FORWARD)


class SonarRefinementTests(unittest.TestCase):
    """Refinamiento opcional: el sonar lateral mantiene la distancia (use_side_sonar true, angulo valido)."""

    def params(self, **kw):
        return PerimeterParams(use_side_sonar=True, side_sonar_angle_deg=45.0, **kw)

    def test_angulo_valido_activa_el_sonar_y_uno_invalido_lo_ignora(self):
        self.assertTrue(PerimeterCore(self.params()).sonar_enabled)
        self.assertFalse(PerimeterCore(PerimeterParams(use_side_sonar=True, side_sonar_angle_deg=0.0)).sonar_enabled)
        self.assertFalse(PerimeterCore(PerimeterParams(use_side_sonar=True, side_sonar_angle_deg=19.9)).sonar_enabled)
        self.assertTrue(PerimeterCore(PerimeterParams(use_side_sonar=True, side_sonar_angle_deg=20.0)).sonar_enabled)

    def test_con_angulo_invalido_sigue_por_bumper_sin_fallar(self):
        world = SimWorld.room(10.0, 8.0)
        params = PerimeterParams(side='right', use_side_sonar=True, side_sonar_angle_deg=0.0)
        core, robot, log = simulate(world, params, start=(2.0, 0.7, 0.0), tmax=300.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['lateral'], [])                      # nunca uso el sonar lateral
        self.assertGreater(log['contacts'], 5)                    # avanza por contacto

    def test_mantiene_la_distancia_lateral_en_un_cuarto(self):
        world = SimWorld.room(10.0, 8.0)
        params = self.params(side='right')
        core, robot, log = simulate(world, params, start=(2.0, 1.0, 0.0), tmax=420.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['collisions'], 0)
        self.assertTrue(reached(log['xy'], n=7.0, e=9.0, w=1.0))
        settled = log['lateral'][200:]
        near = [d for d in settled if abs(d - params.target_distance) < 0.15]
        self.assertGreater(len(near) / len(settled), 0.7)
        self.assertLess(log['contacts'], 15)                      # casi sin tocar la pared

    def test_pared_a_la_izquierda_es_simetrica(self):
        world = SimWorld.room(10.0, 8.0)
        core, robot, log = simulate(world, self.params(side='left'), start=(2.0, 7.0, 0.0), tmax=420.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['collisions'], 0)
        self.assertTrue(reached(log['xy'], s=1.0, e=9.0, w=1.0))

    def test_esquina_exterior_rodeando_un_bloque(self):
        world = SimWorld(walls=SimWorld.box(4.0, 3.0, 7.0, 5.0))
        core, robot, log = simulate(world, self.params(side='right'), start=(3.6, 4.0, math.pi / 2), tmax=300.0)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertEqual(log['collisions'], 0)
        self.assertIn(SEARCH_ARC, log['states'])
        xy = log['xy']
        self.assertTrue(any(y > 5.2 for x, y in xy) and any(x > 7.2 for x, y in xy)
                        and any(y < 2.8 for x, y in xy) and any(x < 3.8 for x, y in xy))


if __name__ == '__main__':
    unittest.main()
