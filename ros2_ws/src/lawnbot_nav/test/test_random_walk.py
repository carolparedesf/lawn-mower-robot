import math
import unittest

from lawnbot_nav.geofence_logic import polygon_centroid, signed_distance
from lawnbot_nav.random_walk_logic import (BACKUP, BLOCKED, FORWARD, TURN, RandomWalkCore,
                                           RandomWalkInputs, RandomWalkParams)
from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_world import SimWorld

DT = 0.05
SENSOR_DT = 0.2        # el firmware publica sonares y bumpers a 5 Hz


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def simulate(world, params, seed, tmax=600.0, start=(5.0, 4.0, 0.0), fence=None,
             use_sonar=True, tau=0.1, use_centroid=False):
    core = RandomWalkCore(params, seed)
    centroid = polygon_centroid(fence) if fence and use_centroid else None
    robot = SimRobot(*start, tau=tau)
    t, next_sensor = 0.0, 0.0
    sonar_min, bumpers = 1.5, {}
    log = {'xy': [], 'collisions': 0, 'bumper_presses': 0, 'v': [], 'w': [], 'states': set(),
           'reasons': set(), 'alerts': set(), 'edge_min': math.inf, 'edge_turn_errors': [],
           'edge_trigger_times': []}
    prev_state, prev_reason = FORWARD, ''
    while t < tmax:
        if t >= next_sensor:
            pose = (robot.x, robot.y, robot.yaw)
            sonar_min = min(world.sonars(pose).values()) if use_sonar else 1.5
            bumpers = world.bumpers(pose)
            log['bumper_presses'] += any(bumpers.values())
            next_sensor += SENSOR_DT
        edge = signed_distance((robot.x, robot.y), fence) if fence else None
        inp = RandomWalkInputs(
            sonar_min=sonar_min, sensors_ok=True,
            front_bumper=bumpers.get('bumper/B1_front_left', False) or bumpers.get('bumper/B2_front_right', False),
            side_bumper=bumpers.get('bumper/B3_left', False) or bumpers.get('bumper/B4_right', False),
            back_bumper=bumpers.get('bumper/B5_back', False),
            edge_distance=edge, geofence_ok=fence is not None,
            interior_angle=(wrap(math.atan2(centroid[1] - robot.y, centroid[0] - robot.x) - robot.yaw)
                            if centroid else None))
        v, w = core.update(inp, t)
        if core.state == BACKUP and prev_state == FORWARD and core.reason == 'edge':
            log['edge_trigger_times'].append(t)
        if prev_state == TURN and core.state == FORWARD and prev_reason == 'edge' and centroid:
            # al terminar un giro por borde: error de rumbo respecto del centroide
            log['edge_turn_errors'].append(abs(wrap(math.atan2(centroid[1] - robot.y, centroid[0] - robot.x) - robot.yaw)))
        prev_state, prev_reason = core.state, core.reason
        log['collisions'] += world.step_robot(robot, v, w, DT)
        log['xy'].append((robot.x, robot.y))
        log['v'].append(v)
        log['w'].append(w)
        log['states'].add(core.state)
        if core.reason:
            log['reasons'].add(core.reason)
        if core.alert:
            log['alerts'].add(core.alert)
        if edge is not None:
            log['edge_min'] = min(log['edge_min'], edge)
        t += DT
    return core, robot, log


def room_with_obstacles():
    world = SimWorld.room(10.0, 8.0)
    world.circles = [(3.0, 5.5, 0.5), (7.0, 2.5, 0.4)]
    return world


def cells_visited(xy, cell=1.0):
    return {(int(x // cell), int(y // cell)) for x, y in xy}


class WalkTests(unittest.TestCase):
    def test_recorre_el_cuarto_sin_chocar_y_respeta_los_limites(self):
        params = RandomWalkParams(require_geofence=False)
        core, robot, log = simulate(room_with_obstacles(), params, seed=1)
        self.assertEqual(log['collisions'], 0)
        self.assertEqual(log['bumper_presses'], 0)             # los sonares frenan antes del contacto
        self.assertLessEqual(max(log['v']), params.linear + 1e-9)
        self.assertGreaterEqual(min(log['v']), -params.backup_speed - 1e-9)
        self.assertLessEqual(max(abs(w) for w in log['w']), params.turn_speed + 1e-9)
        self.assertEqual(log['states'], {FORWARD, BACKUP, TURN})
        self.assertIn('obstacle', log['reasons'])
        self.assertGreaterEqual(len(cells_visited(log['xy'])), 40)       # de 80 celdas de 1 m
        for x, y in log['xy']:
            self.assertTrue(0.0 < x < 10.0 and 0.0 < y < 8.0)

    def test_misma_semilla_misma_trayectoria(self):
        params = RandomWalkParams(require_geofence=False)
        a = simulate(room_with_obstacles(), params, seed=7, tmax=120.0)[2]['xy']
        b = simulate(room_with_obstacles(), params, seed=7, tmax=120.0)[2]['xy']
        c = simulate(room_with_obstacles(), params, seed=8, tmax=120.0)[2]['xy']
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)

    def test_varias_semillas_sin_choques(self):
        params = RandomWalkParams(require_geofence=False)
        for seed in range(2, 8):
            log = simulate(room_with_obstacles(), params, seed=seed, tmax=300.0)[2]
            self.assertEqual(log['collisions'], 0, f'semilla {seed}')

    def test_el_borde_del_cerco_dispara_retroceso_y_se_queda_adentro(self):
        # cerco de 6 x 4 m dentro de un cuarto grande: sin paredes cerca, solo el cerco lo detiene
        fence = [(2.0, 2.0), (8.0, 2.0), (8.0, 6.0), (2.0, 6.0)]
        world = SimWorld.room(10.0, 8.0)
        core, robot, log = simulate(world, RandomWalkParams(edge_distance=0.6, obstacle_distance=0.3),
                                    seed=3, tmax=500.0, start=(5.0, 4.0, 0.0), fence=fence)
        self.assertIn('edge', log['reasons'])
        self.assertGreater(log['edge_min'], 0.0)               # nunca sale del cerco
        self.assertGreaterEqual(len(cells_visited(log['xy'])), 12)

    def test_puede_arrancar_dentro_de_la_franja_del_borde_y_sale_hacia_adentro(self):
        # arranca a 0.6 m del borde (franja de 1.5 m): no debe quedar retrocediendo para siempre
        fence = [(0.5, 0.5), (9.5, 0.5), (9.5, 7.5), (0.5, 7.5)]
        world = SimWorld.room(10.0, 8.0)
        params = RandomWalkParams(edge_distance=1.5, edge_hard_distance=0.4)
        core, robot, log = simulate(world, params, seed=4, tmax=300.0, start=(1.1, 4.0, 0.0), fence=fence)
        self.assertGreater(log['edge_min'], 0.0)                        # nunca sale del cerco
        self.assertGreaterEqual(len(cells_visited(log['xy'])), 20)      # y recorre bastante

    def test_dentro_de_la_franja_avanzar_hacia_adentro_no_dispara(self):
        core = RandomWalkCore(RandomWalkParams(edge_distance=1.5, edge_hard_distance=0.5), seed=1)
        t, edge = 0.0, 0.8
        for _ in range(60):                                       # se aleja del borde: 0.8 -> 1.4 m
            core.update(RandomWalkInputs(sonar_min=1.5, sensors_ok=True, edge_distance=edge, geofence_ok=True), t)
            self.assertEqual(core.state, FORWARD, f'edge {edge:.2f}')
            t += 0.1
            edge += 0.01
        t, edge = 0.0, 1.4                                        # acercandose: 1.4 -> dispara antes de 0.5
        core = RandomWalkCore(RandomWalkParams(edge_distance=1.5, edge_hard_distance=0.5), seed=1)
        fired = None
        for _ in range(200):
            core.update(RandomWalkInputs(sonar_min=1.5, sensors_ok=True, edge_distance=edge, geofence_ok=True), t)
            if core.state != FORWARD:
                fired = edge
                break
            t += 0.1
            edge -= 0.01
        self.assertIsNotNone(fired)
        self.assertGreater(fired, 0.5)

    def test_giro_por_borde_apunta_al_interior_del_cerco(self):
        fence = [(0.5, 0.5), (9.5, 0.5), (9.5, 7.5), (0.5, 7.5)]
        world = SimWorld.room(10.0, 8.0)
        params = RandomWalkParams(edge_distance=1.5, edge_hard_distance=0.5, edge_turn_spread_deg=30.0)
        errors = []
        for seed in range(1, 6):
            log = simulate(world, params, seed, tmax=500.0, start=(5.0, 4.0, 0.0), fence=fence,
                           use_centroid=True)[2]
            errors += log['edge_turn_errors']
            self.assertGreater(log['edge_min'], 0.0)
        self.assertGreater(len(errors), 15)
        # tras el giro mira al centroide dentro de la variacion pedida (mas lo que gira durante el giro por tiempo)
        self.assertLess(max(errors), math.radians(30.0 + 12.0))
        self.assertGreater(max(errors), math.radians(5.0))               # y hay variacion, no apunta siempre exacto

    def test_apuntar_al_centroide_reduce_los_disparos_por_borde(self):
        # Mismo cerco, mismas semillas: girar hacia el interior evita volver a chocar con el borde
        fence = [(0.5, 0.5), (9.5, 0.5), (9.5, 7.5), (0.5, 7.5)]
        world = SimWorld.room(10.0, 8.0)
        params = RandomWalkParams(edge_distance=1.5, edge_hard_distance=0.4, obstacle_distance=0.3)

        def edge_triggers(use_centroid):
            return sum(len(simulate(world, params, seed, tmax=500.0, fence=fence,
                                    use_centroid=use_centroid)[2]['edge_trigger_times'])
                       for seed in range(1, 7))

        with_centroid, at_random = edge_triggers(True), edge_triggers(False)
        self.assertLess(with_centroid, at_random * 0.85, (with_centroid, at_random))

    def test_solo_bumpers_cuando_los_sonares_no_ven(self):
        params = RandomWalkParams(require_geofence=False)
        core, robot, log = simulate(room_with_obstacles(), params, seed=2, tmax=300.0, use_sonar=False)
        self.assertIn('bumper', log['reasons'])
        self.assertGreater(core.n_backups, 5)
        self.assertGreaterEqual(len(cells_visited(log['xy'])), 20)       # no se queda pegado

    def test_sin_dato_de_sensores_no_se_mueve(self):
        core = RandomWalkCore(RandomWalkParams(require_geofence=False), seed=1)
        v, w = core.update(RandomWalkInputs(sonar_min=None, sensors_ok=False), 0.0)
        self.assertEqual((v, w, core.state, core.alert), (0.0, 0.0, BLOCKED, 'sensor_timeout'))
        v, w = core.update(RandomWalkInputs(sonar_min=1.5, sensors_ok=True), 1.0)
        self.assertEqual((v, core.state), (0.2, FORWARD))               # y retoma al volver el dato

    def test_exige_cerco_ok(self):
        core = RandomWalkCore(RandomWalkParams(require_geofence=True), seed=1)
        ok = dict(sonar_min=1.5, sensors_ok=True)
        for kw in (dict(geofence_ok=False, edge_distance=5.0), dict(geofence_ok=True, edge_distance=None)):
            v, w = core.update(RandomWalkInputs(**ok, **kw), 0.0)
            self.assertEqual((v, w, core.alert), (0.0, 0.0, 'geofence_not_ready'))
        v, w = core.update(RandomWalkInputs(**ok, geofence_ok=True, edge_distance=5.0), 1.0)
        self.assertEqual(v, 0.2)

    def test_atrapado_avisa(self):
        params = RandomWalkParams(require_geofence=False, stuck_triggers=4, stuck_window_s=60.0)
        core = RandomWalkCore(params, seed=1)
        t = 0.0
        for _ in range(3000):                                  # siempre hay un obstaculo enfrente
            core.update(RandomWalkInputs(sonar_min=0.2, sensors_ok=True), t)
            t += DT
            if core.alert == 'random_blocked':
                break
        self.assertEqual(core.alert, 'random_blocked')

    def test_bumper_trasero_corta_el_retroceso(self):
        core = RandomWalkCore(RandomWalkParams(require_geofence=False), seed=1)
        core.update(RandomWalkInputs(sonar_min=0.2, sensors_ok=True), 0.0)
        self.assertEqual(core.state, BACKUP)
        v, w = core.update(RandomWalkInputs(sonar_min=0.2, sensors_ok=True, back_bumper=True), 0.1)
        self.assertEqual((core.state, v), (TURN, 0.0))

    def edge_turn(self, interior, seed=1, spread=30.0):
        params = RandomWalkParams(require_geofence=False, edge_turn_spread_deg=spread, edge_distance=1.5,
                                  edge_hard_distance=0.5)
        core = RandomWalkCore(params, seed)
        inp = RandomWalkInputs(sonar_min=1.5, sensors_ok=True, edge_distance=0.2, geofence_ok=True,
                               interior_angle=interior)
        core.update(inp, 0.0)                                       # FORWARD -> BACKUP por el borde
        self.assertEqual((core.state, core.reason), (BACKUP, 'edge'))
        v, w = core.update(inp, 10.0)                               # termina el retroceso -> TURN
        self.assertEqual(core.state, TURN)
        return core, w

    def test_giro_por_borde_hacia_el_angulo_interior_con_variacion(self):
        for interior in (1.2, -2.5, 0.1, 3.0):
            for seed in range(25):
                core, w = self.edge_turn(interior, seed)
                spread = math.radians(30.0)
                self.assertTrue(interior - spread - 1e-9 <= core.last_turn_angle <= interior + spread + 1e-9,
                                (interior, core.last_turn_angle))
                self.assertEqual(w > 0, core.last_turn_angle >= 0)             # el sentido sigue al angulo
                duration = abs(core.last_turn_angle) / 0.6
                self.assertAlmostEqual(core.until - 10.0, duration, places=6)

    def test_sin_variacion_apunta_exacto(self):
        core, w = self.edge_turn(1.0, spread=0.0)
        self.assertAlmostEqual(core.last_turn_angle, 1.0)
        self.assertGreater(w, 0.0)

    def test_la_variacion_depende_de_la_semilla(self):
        angles = {round(self.edge_turn(1.0, seed)[0].last_turn_angle, 6) for seed in range(20)}
        self.assertGreater(len(angles), 10)
        self.assertEqual(self.edge_turn(1.0, 3)[0].last_turn_angle, self.edge_turn(1.0, 3)[0].last_turn_angle)

    def test_sin_angulo_interior_gira_como_antes(self):
        for seed in range(20):
            core, _ = self.edge_turn(None, seed)
            self.assertTrue(math.radians(120.0) - 1e-9 <= abs(core.last_turn_angle) <= math.radians(180.0) + 1e-9)

    def test_los_otros_disparos_no_usan_el_angulo_interior(self):
        params = RandomWalkParams(require_geofence=False, turn_min_deg=60.0, turn_max_deg=90.0)
        core = RandomWalkCore(params, seed=2)
        inp = RandomWalkInputs(sonar_min=0.2, sensors_ok=True, interior_angle=0.0)      # obstaculo, no borde
        core.update(inp, 0.0)
        core.update(inp, 10.0)
        self.assertTrue(math.radians(60.0) <= abs(core.last_turn_angle) <= math.radians(90.0))

    def test_giros_dentro_del_rango_configurado(self):
        params = RandomWalkParams(require_geofence=False, turn_min_deg=90.0, turn_max_deg=135.0)
        for seed in range(20):
            core = RandomWalkCore(params, seed)
            core.update(RandomWalkInputs(sonar_min=0.2, sensors_ok=True), 0.0)       # -> BACKUP
            core.update(RandomWalkInputs(sonar_min=0.2, sensors_ok=True), 10.0)      # -> TURN
            angle = math.degrees((core.until - 10.0) * params.turn_speed)
            self.assertTrue(90.0 - 1e-6 <= angle <= 135.0 + 1e-6, angle)


if __name__ == '__main__':
    unittest.main()
