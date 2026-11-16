import math
import unittest

from lawnbot_nav.geofence_logic import polygon_centroid, signed_distance
from lawnbot_nav.perimeter_logic import (CONTACT_BACK, CONTACT_FWD, CONTACT_TURN, CORNER, CORNER_BACK, FAILED, FOLLOW, SEARCH_ARC,
                                         SEARCH_FORWARD, SEARCH_TURN, UNSTICK, PerimeterCore, PerimeterInputs,
                                         PerimeterParams)
from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_world import SimWorld

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


DT = 0.05
SENSOR_DT = 0.2        # el firmware publica sonares y bumpers a 5 Hz


def simulate(world, params, start, tmax=300.0, tau=0.1, side_sonar_yaw=None, use_pose=False, fence=None, seed=1):
    """Lazo cerrado con el mundo simulado. Los sonares laterales miran a +-side_sonar_angle_deg."""
    core = PerimeterCore(params, seed)
    centroid = polygon_centroid(fence) if fence else None
    robot = SimRobot(*start, tau=tau)
    yaw = params.side_sonar_angle_deg if side_sonar_yaw is None else side_sonar_yaw
    offsets = {'left': yaw, 'center': 0.0, 'right': -yaw}
    side = 'left' if params.side == 'left' else 'right'
    side_bumper = 'bumper/B3_left' if params.side == 'left' else 'bumper/B4_right'
    t, next_sensor = 0.0, 0.0
    ranges, bumpers = {}, {}
    log = {'xy': [], 'states': [], 'collisions': 0, 'lateral': [], 'v': [], 'w': [], 'alerts': set(),
           'failed_at': None, 'contacts': 0, 'edge_min': math.inf}
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
            back_bumper=bumpers['bumper/B5_back'],
            pose=(robot.x, robot.y) if use_pose else None)
        if fence:
            inp.edge_distance = signed_distance((robot.x, robot.y), fence)
            inp.interior_angle = wrap(math.atan2(centroid[1] - robot.y, centroid[0] - robot.x) - robot.yaw)
            log['edge_min'] = min(log['edge_min'], inp.edge_distance)
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


def _legacy_zones():
    """Zonas de bumper del sim_world de antes (96ad7fd): las esquinas del cuerpo tambien eran del bumper lateral Y del frontal/trasero
    (un contacto lateral en la esquina presionaba dos bumpers). Se conservan aqui para probar que el perimetral no depende de eso."""
    from lawnbot_nav.sim_world import BODY_X, BODY_Y
    x0, x1 = BODY_X
    y0, y1 = BODY_Y

    def span(a, b):
        n = max(1, int(round(abs(b - a) / 0.03)))
        return [a + (b - a) * i / n for i in range(n + 1)]
    return {
        'bumper/B1_front_left': [(x1, y) for y in span(0.0, y1)],
        'bumper/B2_front_right': [(x1, y) for y in span(y0, 0.0)],
        'bumper/B3_left': [(x, y1) for x in span(x0, x1)],
        'bumper/B4_right': [(x, y0) for x in span(x0, x1)],
        'bumper/B5_back': [(x0, y) for y in span(y0, y1)],
    }


class LegacyBumperWorld(SimWorld):
    """SimWorld con las zonas de bumper anteriores (esquinas compartidas)."""
    ZONES = _legacy_zones()

    def bumpers(self, pose):
        from lawnbot_nav.sim_world import BUMPER_CONTACT, _to_world
        return {name: any(self.distance(_to_world(pose, *pt)) <= BUMPER_CONTACT for pt in zone)
                for name, zone in self.ZONES.items()}


def sim_room_world(cls, **kw):
    """El cuarto de config/sim_room.yaml: 10 x 8 m con un bloque central y un obstaculo circular."""
    walls = list(SimWorld.room(10.0, 8.0).walls) + list(SimWorld.box(4.0, 3.0, 7.0, 5.0))
    return cls(walls=walls, circles=[(2.0, 6.0, 0.5)], **kw)


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
        core, robot, log = simulate(world, PerimeterParams(side='right'), start=(2.0, 4.0, 0.0), tmax=900.0)
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

    def test_sin_pared_a_la_vista_avanza_recto_y_se_rinde_a_los_search_timeout_s(self):
        world = SimWorld()                                       # campo abierto: nunca encuentra pared
        params = PerimeterParams(side='right', search_timeout_s=30.0, search_speed=0.2)
        core, robot, log = simulate(world, params, start=(0.0, 0.0, 0.0), tmax=60.0)
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_lost'))
        self.assertIn('perimeter_lost', log['alerts'])
        self.assertAlmostEqual(log['failed_at'], 30.0, delta=0.3)
        self.assertEqual((log['v'][-1], log['w'][-1]), (0.0, 0.0))
        self.assertEqual(set(log['states'][:-1]), {SEARCH_FORWARD})
        moving = log['v'][:-1]
        self.assertTrue(all(abs(v - 0.2) < 1e-9 for v in moving))   # velocidad de busqueda
        self.assertTrue(all(w == 0.0 for w in log['w']))            # sin giros: no hace circulos
        self.assertGreater(robot.x, 5.5)                            # ~0,2 m/s durante 30 s
        self.assertAlmostEqual(robot.y, 0.0, delta=1e-6)

    def test_el_tiempo_de_busqueda_por_defecto_es_120_s(self):
        self.assertEqual(PerimeterParams().search_timeout_s, 120.0)
        core = PerimeterCore(PerimeterParams())
        free = PerimeterInputs(sensors_ok=True, center_range=1.5)
        self.assertEqual(core.update(free, 0.0), (0.15, 0.0))
        self.assertEqual(core.update(free, 119.0), (0.15, 0.0))
        self.assertEqual(core.update(free, 121.0), (0.0, 0.0))
        self.assertEqual((core.state, core.alert), (FAILED, 'perimeter_lost'))

    def test_la_busqueda_recta_termina_al_primer_contacto_o_sonar(self):
        for inp, kind in ((PerimeterInputs(sensors_ok=True, center_range=0.3), 'sonar'),
                          (PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), 'contacto')):
            core = PerimeterCore(PerimeterParams())
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
            core.update(inp, 50.0)
            self.assertNotEqual(core.state, SEARCH_FORWARD, kind)
            self.assertIsNone(core.alert, kind)

    def test_pared_perdida_ya_encontrada_busca_con_arcos_y_avisa(self):
        """Un borde ya hallado que se pierde sigue con la busqueda acotada de siempre (lost_search_timeout_s)."""
        params = PerimeterParams(side='right', search_timeout_s=120.0, lost_search_timeout_s=12.0,
                                 bump_lost_timeout_s=3.0)
        core = PerimeterCore(params)
        free = PerimeterInputs(sensors_ok=True, center_range=1.5)
        core.update(free, 0.0)
        core.update(PerimeterInputs(sensors_ok=True, side_bumper=True, center_range=1.5), 1.0)   # halla la pared
        self.assertEqual(core.state, CONTACT_BACK)
        t, states, alert_at = 1.05, set(), None
        while t < 60.0 and core.state != FAILED:
            core.update(free, t)
            states.add(core.state)
            t += 0.05
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_lost'))
        self.assertIn(SEARCH_ARC, states)
        self.assertIn(SEARCH_FORWARD, states)
        self.assertLess(t, 40.0)                                  # mucho antes de los 120 s de la busqueda inicial

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



class EsquinasTests(unittest.TestCase):
    """Que el perimetral no dependa de que bumpers toque el chasis en la esquina (se prueban las dos formas del sim)."""

    def lap(self, cls, side, start, room=None, use_pose=False):
        world = room or cls(walls=SimWorld.room(10.0, 8.0).walls)
        core, robot, log = simulate(world, PerimeterParams(side=side), start=start, tmax=900.0, use_pose=use_pose)
        self.assertNotEqual(core.state, FAILED, f'{cls.__name__} {side}: {core.fail_reason} en ({robot.x:.2f}, {robot.y:.2f})')
        self.assertNotIn('perimeter_blocked', log['alerts'])
        return log

    def test_vuelta_completa_con_las_dos_formas_de_bumpers(self):
        for cls in (SimWorld, LegacyBumperWorld):
            for side, start in (('right', (2.0, 0.7, 0.0)), ('left', (2.0, 7.3, 0.0))):
                with self.subTest(sim=cls.__name__, side=side):
                    log = self.lap(cls, side, start)
                    self.assertTrue(reached(log['xy'], n=7.0, e=9.0, w=1.0, s=1.0) or reached(log['xy'], n=7.0, e=9.0, w=1.0),
                                    'no dio la vuelta completa')

    def test_vuelta_al_cuarto_de_sim_room_con_las_dos_formas_de_bumpers(self):
        for cls in (SimWorld, LegacyBumperWorld):
            for use_pose in (False, True):                       # respaldo v*dt y pose real
                with self.subTest(sim=cls.__name__, pose=use_pose):
                    log = self.lap(cls, 'right', (2.0, 1.0, 0.0), room=sim_room_world(cls), use_pose=use_pose)
                    self.assertTrue(reached(log['xy'], n=7.0, e=9.0, w=1.0), 'no dio la vuelta completa')

    def test_vuelta_con_esquinas_redondeadas_y_zona_muerta_entre_bumpers(self):
        # Los valores de config/sim_room.yaml (radio 0.05, split 45) y uno mas redondeado, con y sin zona muerta, con la pose real
        for radius in (0.05, 0.10):
            for dead in (0.0, 20.0):
                with self.subTest(radius=radius, dead=dead):
                    world = sim_room_world(SimWorld, corner_radius=radius, corner_split_deg=45.0, corner_dead_deg=dead)
                    log = self.lap(SimWorld, 'right', (2.0, 1.0, 0.0), room=world, use_pose=True)
                    self.assertTrue(reached(log['xy'], n=7.0, e=9.0, w=1.0), 'no dio la vuelta completa')
                    # vuelve al sur tras pasar por las otras tres paredes
                    self.assertTrue(any(x < 1.2 and y < 1.2 for x, y in log['xy'][len(log['xy']) // 3:]), 'no cerro la vuelta')

    # ── lógica pura ──

    def test_en_la_esquina_un_contacto_lateral_retrocede_antes_de_seguir_girando(self):
        p = PerimeterParams(side='right')
        core = PerimeterCore(p)
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), 0.1)
        self.assertEqual(core.state, CORNER)
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=0.3, side_bumper=True), 0.2)
        self.assertEqual((core.state, v, w), (CORNER_BACK, -p.contact_back_speed, 0.0))
        t = 0.2 + p.corner_back_distance / p.contact_back_speed
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), t - 0.05)
        self.assertEqual(core.state, CORNER_BACK)
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), t + 0.01)
        self.assertEqual((core.state, v), (CORNER, 0.0))
        self.assertGreater(w, 0.0)                               # sigue girando alejandose de la pared

    def test_si_el_retroceso_no_libera_la_esquina_avanza_en_vez_de_repetirlo(self):
        p = PerimeterParams(side='right')
        core = PerimeterCore(p)
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), 0.1)
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3, side_bumper=True), 0.2)
        t = 0.2 + p.corner_back_distance / p.contact_back_speed + 0.01
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3, side_bumper=True), t)
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=0.3, side_bumper=True), t + 0.05)
        self.assertEqual((core.state, v), (CONTACT_FWD, p.contact_forward_speed))
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), t + 0.1 + p.contact_forward_time_s)
        self.assertEqual(core.state, CORNER)                     # y vuelve a la esquina, no a FOLLOW

    def test_el_bumper_trasero_en_la_esquina_avanza(self):
        p = PerimeterParams(side='right')
        core = PerimeterCore(p)
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5), 0.0)
        core.update(PerimeterInputs(sensors_ok=True, center_range=0.3), 0.1)
        v, w = core.update(PerimeterInputs(sensors_ok=True, center_range=0.3, side_bumper=True, back_bumper=True), 0.2)
        self.assertEqual((core.state, v), (CONTACT_FWD, p.contact_forward_speed))

    def _ciclo(self, core, t, avance, kind_front=False):
        """Un ciclo: avanza `avance` m en FOLLOW y toca el bumper lateral. Devuelve el instante final y el estado tras el toque."""
        free = PerimeterInputs(sensors_ok=True, center_range=1.5)
        end = t + avance / 0.15
        while t < end:
            core.update(free, t)
            t += 0.05
        core.update(PerimeterInputs(sensors_ok=True, center_range=1.5, side_bumper=True), t)
        back_time = core.p.contact_back_distance / core.p.contact_back_speed
        t += back_time + 0.05
        core.update(free, t)                                     # termina el retroceso
        return t, core.contact_kind

    def test_tres_contactos_laterales_sin_avance_se_tratan_como_esquina(self):
        p = PerimeterParams(side='right')
        core = PerimeterCore(p)
        core.state = FOLLOW
        t, kinds = 0.0, []
        for _ in range(3):
            t, kind = self._ciclo(core, t, avance=0.30)          # 0.30 de ida y 0.10 de vuelta: neto 0.20 (suficiente)
            kinds.append(kind)
            core.state = FOLLOW
        self.assertEqual(kinds, ['side'] * 3)
        core = PerimeterCore(p)
        core.state = FOLLOW
        t, kinds = 0.0, []
        for _ in range(3):
            t, kind = self._ciclo(core, t, avance=0.12)          # 0.12 de ida y 0.10 de vuelta: neto 0.02 m
            kinds.append(kind)
            core.state = FOLLOW
        self.assertEqual(kinds, ['side', 'side', 'front'])       # el tercero es esquina: giro de 45 grados
        self.assertEqual(core.stalled, 0)

    def test_contactos_con_avance_suficiente_no_son_esquina(self):
        core = PerimeterCore(PerimeterParams(side='right'))
        core.state = FOLLOW
        t = 0.0
        for _ in range(6):
            t, kind = self._ciclo(core, t, avance=0.60)
            self.assertEqual(kind, 'side')
            core.state = FOLLOW

    # ── avance medido con la pose real ──

    def _ciclos_con_pose(self, poses, avance=0.30, **kw):
        """Ciclos de FOLLOW (se ordena `avance` m) y contacto lateral; el robot reporta `poses[i]` en el contacto i.
        Devuelve (tipos de contacto, nucleo)."""
        core = PerimeterCore(PerimeterParams(side='right', **kw))
        core.state = FOLLOW
        t, kinds = 0.0, []
        free = PerimeterInputs(sensors_ok=True, center_range=1.5)
        back_time = core.p.contact_back_distance / core.p.contact_back_speed
        for pose in poses:
            end = t + avance / 0.15
            while t < end:
                core.update(PerimeterInputs(sensors_ok=True, center_range=1.5, pose=pose), t)
                t += 0.05
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5, side_bumper=True, pose=pose), t)
            kinds.append(core.contact_kind)
            t += back_time + 0.05
            core.update(PerimeterInputs(sensors_ok=True, center_range=1.5, pose=pose), t)
            core.state = FOLLOW
        return kinds, core

    def test_robot_trabado_que_recibe_v_positiva_se_detecta_como_falta_de_progreso(self):
        # Se le ordena avanzar 0.30 m por ciclo (v*dt integrado: neto 0.20, "suficiente") pero la pose no cambia.
        stuck = [(5.0, 0.3)] * 4
        kinds, core = self._ciclos_con_pose(stuck)
        self.assertEqual(kinds, ['side', 'side', 'side', 'front'])    # 1er contacto sin referencia; 2, 3 y 4 sin avance
        self.assertEqual(core.progress_source, 'pose')
        self.assertAlmostEqual(core.last_progress, 0.0)
        # lo mismo sin pose (respaldo): cree que avanzo y NO lo detecta
        kinds, core = self._ciclos_con_pose([None] * 6)
        self.assertEqual(kinds, ['side'] * 6)
        self.assertEqual(core.progress_source, 'integrated')

    def test_la_pose_mide_el_avance_real_entre_contactos(self):
        moving = [(5.0 + 0.30 * i, 0.3) for i in range(6)]            # 0.30 m reales por ciclo
        kinds, core = self._ciclos_con_pose(moving)
        self.assertEqual(kinds, ['side'] * 6)
        self.assertAlmostEqual(core.last_progress, 0.30)
        slow = [(5.0 + 0.05 * i, 0.3) for i in range(5)]              # 5 cm por ciclo, aunque se ordenen 0.30 m
        kinds, _ = self._ciclos_con_pose(slow)
        self.assertEqual(kinds, ['side', 'side', 'side', 'front', 'side'])

    def test_si_falta_la_pose_en_un_contacto_usa_v_dt_en_ese_ciclo(self):
        poses = [(5.0, 0.3), None, (5.0, 0.3), (5.0, 0.3), (5.0, 0.3)]
        kinds, core = self._ciclos_con_pose(poses)
        # contactos 2 y 3 comparan contra una pose faltante: v*dt (0.20 neto, no cuenta); 4 y 5 con pose: sin avance
        self.assertEqual(kinds[:3], ['side'] * 3)
        self.assertEqual(core.progress_source, 'pose')

    def test_stalled_contacts_cero_lo_apaga(self):
        core = PerimeterCore(PerimeterParams(side='right', stalled_contacts=0))
        core.state = FOLLOW
        t = 0.0
        for _ in range(6):
            t, kind = self._ciclo(core, t, avance=0.12)
            self.assertEqual(kind, 'side')
            core.state = FOLLOW


class EdgeSearchTests(unittest.TestCase):
    """Busqueda inicial: recto; cerca del borde del cerco y mirando hacia afuera, gira hacia adentro y sigue buscando."""

    def inputs(self, edge=None, angle=None, **kw):
        return PerimeterInputs(sensors_ok=True, center_range=1.5, edge_distance=edge, interior_angle=angle, **kw)

    def test_mirando_hacia_afuera_cerca_del_borde_gira_hacia_el_interior(self):
        core = PerimeterCore(PerimeterParams(search_edge_spread_deg=0.0), seed=1)
        core.update(self.inputs(), 0.0)
        v, w = core.update(self.inputs(edge=0.8, angle=math.pi / 2 + 0.5), 0.1)    # el centroide esta a la izquierda
        self.assertEqual((core.state, v), (SEARCH_TURN, 0.0))
        self.assertAlmostEqual(w, 0.5)
        v, w = core.update(self.inputs(edge=0.8, angle=-math.pi / 2 - 0.5), 100.0)  # (giro terminado) ahora a la derecha
        self.assertEqual(core.state, SEARCH_TURN)

    def test_el_giro_dura_lo_que_pide_el_angulo_y_despues_sigue_recto(self):
        core = PerimeterCore(PerimeterParams(search_edge_spread_deg=0.0, search_turn_speed=0.5), seed=1)
        core.update(self.inputs(), 0.0)
        core.update(self.inputs(edge=0.5, angle=math.pi), 1.0)                    # 180 grados a 0,5 rad/s = ~6,3 s
        self.assertEqual(core.state, SEARCH_TURN)
        _, w = core.update(self.inputs(edge=0.5, angle=math.pi), 6.0)
        self.assertEqual(w, 0.5)
        v, w = core.update(self.inputs(edge=1.2, angle=0.0), 7.5)
        self.assertEqual((core.state, v, w), (SEARCH_FORWARD, 0.15, 0.0))
        self.assertEqual(core.search_edge_turns, 1)

    def test_mirando_hacia_adentro_o_lejos_del_borde_no_gira(self):
        for edge, angle in ((0.8, 0.3), (0.8, -1.0), (2.0, math.pi), (1.0, math.pi)):
            core = PerimeterCore(PerimeterParams(), seed=1)
            v, w = core.update(self.inputs(edge=edge, angle=angle), 0.0)
            self.assertEqual((core.state, v, w), (SEARCH_FORWARD, 0.15, 0.0), (edge, angle))

    def test_sin_cerco_nunca_gira(self):
        core = PerimeterCore(PerimeterParams(), seed=1)
        self.assertEqual(core.update(self.inputs(edge=None, angle=None), 0.0), (0.15, 0.0))

    def test_fuera_del_cerco_o_sin_rumbo_al_interior_gira_igual(self):
        core = PerimeterCore(PerimeterParams(), seed=1)
        core.update(self.inputs(edge=-0.3, angle=0.1), 0.0)
        self.assertEqual(core.state, SEARCH_TURN)
        core = PerimeterCore(PerimeterParams(), seed=1)
        core.update(self.inputs(edge=0.4, angle=None), 0.0)
        self.assertEqual(core.state, SEARCH_TURN)

    def test_girar_hacia_adentro_no_reinicia_el_plazo_de_busqueda(self):
        core = PerimeterCore(PerimeterParams(search_timeout_s=20.0), seed=1)
        t = 0.0
        while t < 40.0 and core.state != FAILED:
            core.update(self.inputs(edge=0.4, angle=math.pi), t)
            t += 0.5
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_lost'))
        self.assertAlmostEqual(t, 21.0, delta=1.0)

    def test_un_contacto_mientras_gira_cuenta_como_borde_encontrado(self):
        core = PerimeterCore(PerimeterParams(), seed=1)
        core.update(self.inputs(edge=0.4, angle=math.pi), 0.0)
        self.assertEqual(core.state, SEARCH_TURN)
        core.update(self.inputs(edge=0.4, angle=math.pi, side_bumper=True), 1.0)
        self.assertEqual(core.state, CONTACT_BACK)

    def test_el_cerco_cerrado_mantiene_al_robot_dentro_y_encuentra_la_pared(self):
        fence = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        world = SimWorld(walls=[((15.0, 2.0), (15.0, 18.0))])
        for seed in (1, 2, 3):
            core, robot, log = simulate(world, PerimeterParams(side='right'), start=(3.0, 10.0, math.pi),
                                        tmax=400.0, fence=fence, seed=seed)
            self.assertNotEqual(core.state, FAILED, f'seed {seed}: {core.fail_reason}')
            self.assertGreater(log['edge_min'], 0.0, f'seed {seed}: salio del cerco')
            self.assertIn(SEARCH_TURN, log['states'], f'seed {seed}: no giro en el borde')
            self.assertGreater(log['contacts'], 3, f'seed {seed}: no llego a la pared')
            self.assertTrue(any(x > 13.0 for x, y in log['xy']), f'seed {seed}')

    def test_sin_pared_en_un_cerco_chico_recorre_dentro_hasta_rendirse(self):
        fence = [(0.0, 0.0), (12.0, 0.0), (12.0, 12.0), (0.0, 12.0)]
        core, robot, log = simulate(SimWorld(), PerimeterParams(side='right', search_timeout_s=150.0),
                                    start=(6.0, 6.0, 0.0), tmax=200.0, fence=fence)
        self.assertEqual((core.state, core.fail_reason), (FAILED, 'perimeter_lost'))
        self.assertAlmostEqual(log['failed_at'], 150.0, delta=0.5)
        self.assertGreater(log['edge_min'], 0.0)
        self.assertGreaterEqual(core.search_edge_turns, 2)


def sim_field_world(cls=SimWorld):
    """El campo de config/sim_field.yaml (paredes, barreras y obstaculos tal cual estan en el YAML)."""
    import os
    import yaml
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'sim_field.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)['fake_robot']['ros__parameters']
    w, o = cfg['walls'], cfg['obstacles']
    walls = [((w[i], w[i + 1]), (w[i + 2], w[i + 3])) for i in range(0, len(w), 4)]
    circles = [tuple(o[i:i + 3]) for i in range(0, len(o), 3)]
    return cls(walls=walls, circles=circles, corner_radius=cfg['corner_radius'],
               corner_split_deg=cfg['corner_split_deg'], corner_dead_deg=cfg['corner_dead_deg']), cfg


class FieldWorldTests(unittest.TestCase):
    """world:=field: pared recta de 12 m al este del arranque y barrera en L de 3 x 3 m al oeste."""

    def test_el_yaml_trae_la_pared_recta_la_l_y_los_obstaculos_de_antes(self):
        world, cfg = sim_field_world()
        segments = {tuple(sorted(s)) for s in world.walls}
        self.assertIn(((19.0, 9.0), (19.0, 21.0)), segments)                       # recta de 12 m, 4 m al este
        self.assertIn(((9.0, 13.0), (9.0, 16.0)), segments)                        # brazos de la L, 3 m cada uno
        self.assertIn(((9.0, 13.0), (12.0, 13.0)), segments)
        self.assertEqual(len(world.walls), 4 + 3)
        self.assertEqual(world.circles, [(11.0, 19.0, 0.3), (20.0, 11.0, 0.25), (19.0, 21.0, 0.4)])
        self.assertEqual((cfg['x'], cfg['y'], cfg['yaw']), (15.0, 15.0, 0.0))

    def test_desde_el_arranque_encuentra_la_pared_recta_y_la_sigue(self):
        world, cfg = sim_field_world()
        core, robot, log = simulate(world, PerimeterParams(side='right'), start=(cfg['x'], cfg['y'], cfg['yaw']),
                                    tmax=400.0, use_pose=True)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        xy = log['xy']
        first = next(i for i, c in enumerate(log['states']) if c not in (SEARCH_FORWARD,))
        self.assertAlmostEqual(xy[first][0], 19.0 - 0.45, delta=0.6)                # primer contacto en la pared
        self.assertTrue(all(w == 0.0 for w in log['w'][:first]))                    # hasta ahi, recto
        self.assertTrue(any(y > 20.0 for x, y in xy), 'no siguio la pared hasta el extremo norte')
        self.assertTrue(all(x > 17.0 for x, y in xy[first:first + 200] if y > 9.5), 'se aparto de la pared')
        self.assertGreater(log['contacts'], 10)

    def test_con_la_pared_a_la_izquierda_tambien_la_encuentra(self):
        world, cfg = sim_field_world()
        core, robot, log = simulate(world, PerimeterParams(side='left'), start=(cfg['x'], cfg['y'], cfg['yaw']),
                                    tmax=400.0, use_pose=True)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertTrue(any(y < 10.0 for x, y in log['xy']), 'no siguio la pared hacia el sur')

    def test_rodea_la_l(self):
        world, cfg = sim_field_world()
        core, robot, log = simulate(world, PerimeterParams(side='right'), start=(15.0, 15.0, math.pi),
                                    tmax=1500.0, use_pose=True)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        xy = log['xy']
        # rodea las dos puntas y pasa por el lado de afuera de los dos brazos
        self.assertTrue(any(y > 16.3 for x, y in xy), 'no rodeo la punta del brazo vertical')
        self.assertTrue(any(x < 8.7 and 13.5 < y < 15.5 for x, y in xy), 'no paso por el lado de afuera del brazo vertical')
        self.assertTrue(any(x > 12.3 for x, y in xy if 12.0 < y < 13.6), 'no rodeo la punta del brazo horizontal')
        self.assertTrue(any(y < 12.7 and 9.5 < x < 11.5 for x, y in xy), 'no paso por debajo del brazo horizontal')

    def test_con_el_cerco_del_campo_no_sale_y_lo_encuentra(self):
        world, cfg = sim_field_world()
        fence = [(1.0, 1.0), (29.0, 1.0), (29.0, 29.0), (1.0, 29.0)]
        core, robot, log = simulate(world, PerimeterParams(side='right'), start=(cfg['x'], cfg['y'], cfg['yaw']),
                                    tmax=300.0, use_pose=True, fence=fence)
        self.assertNotEqual(core.state, FAILED, core.fail_reason)
        self.assertGreater(log['edge_min'], 0.0)
        self.assertGreater(log['contacts'], 10)


if __name__ == '__main__':
    unittest.main()
