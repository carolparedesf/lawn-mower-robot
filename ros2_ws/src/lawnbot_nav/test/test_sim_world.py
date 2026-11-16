import math
import unittest

from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_world import SimWorld


class SimWorldTests(unittest.TestCase):
    def setUp(self):
        self.room = SimWorld.room(10.0, 8.0)

    def test_sonar_ve_la_pared_de_enfrente(self):
        # robot mirando al este; el sonar central esta 0.198 m adelante de base_footprint
        r = self.room.sonar_range((8.6, 4.0, 0.0), 'center')
        self.assertAlmostEqual(r, 10.0 - 8.6 - 0.198, places=2)

    def test_sin_eco_devuelve_el_maximo(self):
        self.assertEqual(self.room.sonar_range((5.0, 4.0, 0.0), 'center'), 1.5)

    def test_sonar_lateral_con_yaw(self):
        # sonar izquierdo a 45 grados; pared norte a 1.0 m de su posicion. El cono de +-7.5 grados
        # devuelve el rayo mas corto: 1 / sin(52.5 grados)
        pose = (5.0, 7.0 - 0.136, 0.0)
        r = self.room.sonar_range(pose, 'left', 45.0)
        self.assertAlmostEqual(r, 1.0 / math.sin(math.radians(52.5)), delta=0.01)

    def test_obstaculo_circular(self):
        world = SimWorld(circles=[(1.5, 0.0, 0.5)])
        self.assertAlmostEqual(world.sonar_range((0.0, 0.0, 0.0), 'center'), 1.0 - 0.198, delta=0.02)

    def test_bumper_frontal(self):
        pose = (9.78, 4.0, 0.0)                       # bumpers frontales a ~0.01 m de la pared este
        pressed = self.room.bumpers(pose)
        self.assertTrue(pressed['bumper/B1_front_left'] and pressed['bumper/B2_front_right'])
        self.assertFalse(pressed['bumper/B5_back'])

    def test_bumper_lateral_derecho(self):
        pose = (5.0, 0.2127 + 0.01, 0.0)              # pared sur pegada al costado derecho
        pressed = self.room.bumpers(pose)
        self.assertTrue(pressed['bumper/B4_right'])
        self.assertFalse(pressed['bumper/B3_left'])

    def test_las_esquinas_son_solo_de_los_laterales(self):
        from lawnbot_nav.sim_world import _BUMPER_ZONES
        owners = {}
        for name, pts in _BUMPER_ZONES.items():
            for p in pts:
                owners.setdefault((round(p[0], 6), round(p[1], 6)), []).append(name)
        # (el centro del frente es de B1 y B2 a la vez, a proposito; el resto no comparte nada)
        shared = {p: n for p, n in owners.items() if len(n) > 1}
        self.assertEqual(shared, {(0.2137, 0.0): ['bumper/B1_front_left', 'bumper/B2_front_right']})

    def test_lateral_derecho_en_las_esquinas_no_presiona_frontal_ni_trasero(self):
        from lawnbot_nav.safety_logic import SafetyParams, SafetyState, apply_safety
        # una pared larga (sur) pegada al costado derecho: la esquina trasera y la delantera tambien quedan a 1 cm de ella
        # (solo la pared sur: contra una pared al fondo o al frente si corresponde presionar B5 o B1/B2)
        room = SimWorld(walls=[((-20.0, 0.0), (20.0, 0.0))])
        for x in (0.2463 + 0.01, 5.0, 10.0 - 0.2137 - 0.01):
            pressed = room.bumpers((x, 0.2127 + 0.01, 0.0))
            self.assertTrue(pressed['bumper/B4_right'], x)
            others = [n for n, on in pressed.items() if on and n != 'bumper/B4_right']
            self.assertEqual(others, [], x)
            # lo que haria el safety_gate con estos bumpers: ninguna alerta de bumper
            state = SafetyState(sonar_ranges={},
                                front_pressed=pressed['bumper/B1_front_left'] or pressed['bumper/B2_front_right'],
                                back_pressed=pressed['bumper/B5_back'])
            self.assertEqual(apply_safety(0.3, 0.0, state, SafetyParams())[2], 'none')

    def test_el_robot_no_atraviesa_la_pared(self):
        robot = SimRobot(8.0, 4.0, 0.0)
        collided = False
        for _ in range(400):
            collided |= self.room.step_robot(robot, 0.2, 0.0, 0.05)
        self.assertTrue(collided)
        self.assertLess(robot.x + 0.2137, 10.0 + 1e-6)
        self.assertGreater(robot.x, 9.5)

    def test_puede_alejarse_despues_de_chocar(self):
        robot = SimRobot(8.0, 4.0, 0.0)
        for _ in range(400):
            self.room.step_robot(robot, 0.2, 0.0, 0.05)
        x_hit = robot.x
        for _ in range(60):
            self.room.step_robot(robot, -0.15, 0.0, 0.05)
        self.assertLess(robot.x, x_hit - 0.3)

    def test_no_hay_colision_en_el_centro(self):
        self.assertFalse(self.room.collides((5.0, 4.0, 1.0)))
        self.assertTrue(self.room.collides((9.9, 4.0, 0.0)))

    def test_colision_de_costado_y_con_obstaculos(self):
        self.assertTrue(self.room.collides((5.0, 0.1, 0.0)))               # un lado contra la pared sur
        self.assertTrue(self.room.collides((5.0, 0.1, math.pi / 2)))
        world = SimWorld(circles=[(2.0, 0.0, 0.3)])
        self.assertTrue(world.collides((1.6, 0.0, 0.0)))
        self.assertFalse(world.collides((1.2, 0.0, 0.0)))

    def test_no_se_cuela_por_una_esquina_al_girar(self):
        robot = SimRobot(9.6, 7.6, 0.0)
        for _ in range(200):
            self.room.step_robot(robot, 0.0, 0.6, 0.05)
            self.assertFalse(self.room.collides((robot.x, robot.y, robot.yaw)))



class EsquinasRedondeadasTests(unittest.TestCase):
    R = 0.15
    ARCS = (  # (centro, signo x, signo y, bumper frontal/trasero, bumper lateral)
        ('fl', +1, +1, 'bumper/B1_front_left', 'bumper/B3_left'),
        ('fr', +1, -1, 'bumper/B2_front_right', 'bumper/B4_right'),
        ('bl', -1, +1, 'bumper/B5_back', 'bumper/B3_left'),
        ('br', -1, -1, 'bumper/B5_back', 'bumper/B4_right'),
    )

    def angle(self, pt, sx, sy):
        """Angulo del punto en su arco, desde la cara frontal/trasera (0) hacia la lateral (90)."""
        from lawnbot_nav.sim_world import BODY_X, BODY_Y
        cx = (BODY_X[1] - self.R) if sx > 0 else (BODY_X[0] + self.R)
        cy = (BODY_Y[1] - self.R) if sy > 0 else (BODY_Y[0] + self.R)
        return math.degrees(math.atan2(abs(pt[1] - cy), abs(pt[0] - cx)))

    def arc_points(self, zones, sx, sy):
        from lawnbot_nav.sim_world import BODY_X, BODY_Y
        out = []
        for name, pts in zones.items():
            for p in pts:
                in_x = (p[0] > BODY_X[1] - self.R) if sx > 0 else (p[0] < BODY_X[0] + self.R)
                in_y = (p[1] > BODY_Y[1] - self.R) if sy > 0 else (p[1] < BODY_Y[0] + self.R)
                if in_x and in_y:
                    out.append((name, self.angle(p, sx, sy)))
        return out

    def test_radio_cero_es_el_cuerpo_rectangular_de_siempre(self):
        from lawnbot_nav.sim_world import _BUMPER_ZONES, bumper_zones
        self.assertEqual(bumper_zones(0.0), _BUMPER_ZONES)
        self.assertEqual(SimWorld().zones, _BUMPER_ZONES)

    def test_el_arco_se_reparte_en_el_split_sin_zona_muerta(self):
        from lawnbot_nav.sim_world import bumper_zones
        for split in (30.0, 45.0, 60.0):
            zones = bumper_zones(self.R, split, 0.0)
            for _, sx, sy, face, side in self.ARCS:
                pts = self.arc_points(zones, sx, sy)
                self.assertTrue(pts)
                for name, a in pts:
                    self.assertEqual(name, face if a < split - 1e-6 else side, (split, a, name))
                self.assertTrue(any(n == face for n, _ in pts) and any(n == side for n, _ in pts))

    def test_cada_punto_del_arco_es_de_un_solo_bumper(self):
        from lawnbot_nav.sim_world import bumper_zones
        zones = bumper_zones(self.R, 45.0, 0.0)
        owners = {}
        for name, pts in zones.items():
            for p in pts:
                owners.setdefault((round(p[0], 6), round(p[1], 6)), set()).add(name)
        shared = {p: n for p, n in owners.items() if len(n) > 1}
        self.assertEqual(list(shared.values()), [{'bumper/B1_front_left', 'bumper/B2_front_right'}])   # solo el centro del frente

    def test_la_zona_muerta_queda_sin_bumper(self):
        from lawnbot_nav.sim_world import bumper_zones
        zones = bumper_zones(self.R, 45.0, 20.0)
        full = bumper_zones(self.R, 45.0, 0.0)
        for _, sx, sy, face, side in self.ARCS:
            angles = [a for _, a in self.arc_points(zones, sx, sy)]
            self.assertFalse([a for a in angles if 35.0 + 1e-6 < a < 55.0 - 1e-6], 'hay puntos en la zona muerta')
            self.assertTrue(any(a <= 35.0 + 1e-6 for a in angles) and any(a >= 55.0 - 1e-6 for a in angles))
            self.assertLess(len(self.arc_points(zones, sx, sy)), len(self.arc_points(full, sx, sy)))

    def test_pared_pegada_al_costado_solo_presiona_el_lateral_si_el_split_es_45(self):
        # el costado derecho a 1 cm de la pared sur: con radio 0.15 el contacto de 2 cm toca el arco desde ~69 grados
        for split, expected in ((45.0, {'bumper/B4_right'}), (80.0, {'bumper/B4_right', 'bumper/B2_front_right', 'bumper/B5_back'})):
            room = SimWorld.room(10.0, 8.0, corner_radius=self.R, corner_split_deg=split)
            pressed = {n for n, on in room.bumpers((5.0, 0.2127 + 0.01, 0.0)).items() if on}
            self.assertEqual(pressed, expected, split)

    def test_un_obstaculo_en_la_diagonal_de_la_esquina_cae_en_la_zona_muerta(self):
        from lawnbot_nav.sim_world import BODY_X, BODY_Y
        c = math.sqrt(0.5)
        # punto del arco frontal derecho a 45 grados y un obstaculo diminuto a 0.5 cm hacia afuera
        px = BODY_X[1] - self.R + self.R * c
        py = BODY_Y[0] + self.R - self.R * c
        ox, oy = px + 0.005 * c, py - 0.005 * c
        for dead, expected in ((0.0, {'bumper/B4_right', 'bumper/B2_front_right'}), (40.0, set())):
            world = SimWorld(circles=[(5.0 + ox, 5.0 + oy, 0.001)], corner_radius=self.R, corner_dead_deg=dead)
            pressed = {n for n, on in world.bumpers((5.0, 5.0, 0.0)).items() if on}
            if dead == 0.0:
                self.assertTrue(pressed <= expected and pressed, pressed)      # el contacto de 2 cm abarca ambos lados del split
            else:
                self.assertEqual(pressed, expected)                            # zona muerta de 40 grados: ninguno

    def test_el_cuerpo_redondeado_pasa_donde_el_rectangular_choca(self):
        from lawnbot_nav.sim_world import BODY_X, BODY_Y
        ox, oy = BODY_X[1] + 0.012, BODY_Y[0] - 0.012            # junto a la esquina delantera derecha
        circle = [(5.0 + ox, 5.0 + oy, 0.02)]
        self.assertTrue(SimWorld(circles=circle).collides((5.0, 5.0, 0.0)))
        self.assertFalse(SimWorld(circles=circle, corner_radius=self.R).collides((5.0, 5.0, 0.0)))

    def test_el_cuerpo_redondeado_sigue_chocando_de_frente_y_de_lado(self):
        room = SimWorld.room(10.0, 8.0, corner_radius=self.R)
        robot = SimRobot(8.0, 4.0, 0.0)
        collided = False
        for _ in range(400):
            collided |= room.step_robot(robot, 0.2, 0.0, 0.05)
        self.assertTrue(collided)
        self.assertLess(robot.x + 0.2137, 10.0 + 1e-6)
        self.assertTrue(room.collides((5.0, 0.2127 - 0.01, 0.0)))
        self.assertFalse(room.collides((5.0, 0.2127 + 0.01, 0.0)))

    def test_sim_room_yaml_tiene_los_parametros_de_las_esquinas(self):
        import os
        import yaml
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'sim_room.yaml')
        cfg = yaml.safe_load(open(path))['fake_robot']['ros__parameters']
        self.assertEqual((cfg['corner_radius'], cfg['corner_split_deg'], cfg['corner_dead_deg']), (0.06, 41.0, 55.0))
        self.assertEqual(cfg['body_shape'], 'chassis')


class HuellaDelChasisTests(unittest.TestCase):
    """Huella del plano CAD: frente de 400 x 226 mm, trasera de 250 mm, largo 436 mm, esquinas delanteras de 60 mm."""

    def setUp(self):
        from lawnbot_nav.sim_world import Footprint
        self.fp = Footprint(corner_radius=0.06, split_deg=41.0, dead_deg=55.0)
        self.xf = self.fp.front_x

    def test_medidas(self):
        x0, x1, y0, y1 = self.fp.bbox
        self.assertAlmostEqual(x1 - x0, 0.436, places=6)
        self.assertAlmostEqual(y1 - y0, 0.400, places=6)
        self.assertAlmostEqual(self.xf - self.fp.x_step, 0.226, places=6)
        ys_rear = {round(p[1], 6) for p in self.fp.outline if p[0] < self.fp.x_step - 1e-9}
        self.assertEqual(ys_rear, {-0.125, 0.125})                              # trasera de 250 mm

    def test_no_es_un_rectangulo(self):
        """Una pared junto a la trasera (que un rectangulo de 400 mm tocaria) no choca; junto al frente si."""
        world = SimWorld(walls=[((4.9, 0.16), (4.78, 0.16))], footprint=self.fp)
        self.assertFalse(world.collides((5.0, 0.0, 0.0)))                      # x de la pared: detras del escalon
        legacy = SimWorld(walls=[((4.9, 0.16), (4.78, 0.16))])
        self.assertTrue(legacy.collides((5.0, 0.0, 0.0)))
        front = SimWorld(walls=[((5.1, 0.17), (5.2, 0.17))], footprint=self.fp)
        self.assertTrue(front.collides((5.0, 0.0, 0.0)))                        # junto al bloque delantero (ancho 400)

    def test_los_bumpers_frontales_entran_en_el_arco_hasta_13_grados(self):
        r, yf = self.fp.corner_radius, 0.2
        b1 = self.fp.zones['bumper/B1_front_left']
        far = max(p[1] for p in b1)
        self.assertAlmostEqual(far, (yf - r) + r * math.sin(math.radians(41.0 - 27.5)), delta=0.002)
        self.assertAlmostEqual(self.xf - min(p[0] for p in b1 if p[1] == far), r * (1 - math.cos(math.radians(13.5))), delta=0.002)

    def test_los_laterales_empiezan_a_38_mm_del_frente(self):
        for name in ('bumper/B3_left', 'bumper/B4_right'):
            depth = self.xf - max(p[0] for p in self.fp.zones[name])
            self.assertAlmostEqual(depth, 0.038, delta=0.004)                   # ~68 grados del arco
            self.assertGreaterEqual(min(p[0] for p in self.fp.zones[name]), self.fp.x_step - 1e-9)   # hasta el escalon

    def test_zona_sin_bumper_de_55_grados_centrada_en_41(self):
        dead = self.fp.unprotected['dead']
        self.assertGreater(len(dead), 5)
        r = self.fp.corner_radius
        for x, y in dead:
            alpha = math.degrees(math.atan2(abs(y) - (0.2 - r), x - (self.xf - r)))
            self.assertTrue(13.5 - 1e-6 <= alpha <= 68.5 + 1e-6, alpha)
        alphas = [math.degrees(math.atan2(abs(y) - (0.2 - r), x - (self.xf - r))) for x, y in dead]
        self.assertAlmostEqual((min(alphas) + max(alphas)) / 2.0, 41.0, delta=3.0)
        self.assertAlmostEqual(max(alphas) - min(alphas), 55.0, delta=5.0)

    def test_trasero_centrado_y_sin_bumper_en_los_costados_traseros(self):
        b5 = self.fp.zones['bumper/B5_back']
        self.assertAlmostEqual(sum(p[1] for p in b5) / len(b5), 0.0, places=9)
        self.assertTrue(all(abs(p[0] - self.fp.x_rear) < 1e-9 for p in b5))
        self.assertGreater(len(self.fp.unprotected['rear_side']), 0)
        for name, pts in self.fp.zones.items():                                 # ningun punto en dos bumpers
            for other, pts2 in self.fp.zones.items():
                if name < other:
                    self.assertFalse(set(pts) & set(pts2))

    def test_un_contacto_en_la_zona_muerta_no_presiona_ningun_bumper(self):
        r = self.fp.corner_radius
        a = math.radians(41.0)
        tip = (self.xf - r + r * math.cos(a), 0.2 - r + r * math.sin(a))        # punto del arco a 41 grados
        world = SimWorld(circles=[(5.0 + tip[0] + 0.012, tip[1], 0.001)], footprint=self.fp)
        pose = (5.0, 0.0, 0.0)
        self.assertFalse(any(world.bumpers(pose).values()))
        self.assertTrue(world.unprotected_contacts(pose)['dead'])
        far = SimWorld(circles=[(8.0, 5.0, 0.2)], footprint=self.fp)
        self.assertFalse(any(far.unprotected_contacts(pose).values()))

    def test_un_contacto_de_frente_presiona_los_dos_frontales(self):
        world = SimWorld(walls=[((5.22, -2.0), (5.22, 2.0))], footprint=self.fp)
        pressed = {k for k, v in world.bumpers((5.0, 0.0, 0.0)).items() if v}
        self.assertEqual(pressed, {'bumper/B1_front_left', 'bumper/B2_front_right'})

    def test_el_mundo_por_defecto_sigue_siendo_el_rectangulo(self):
        self.assertIsNone(SimWorld().footprint)

    def test_los_yaml_cargan_la_huella_del_chasis(self):
        import os
        import yaml
        for name in ('room', 'field'):
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', f'sim_{name}.yaml')
            with open(path) as f:
                cfg = yaml.safe_load(f)['fake_robot']['ros__parameters']
            self.assertEqual((cfg['body_shape'], cfg['corner_radius'], cfg['corner_split_deg'], cfg['corner_dead_deg']),
                             ('chassis', 0.06, 41.0, 55.0), name)
            world = SimWorld.from_config(cfg)
            self.assertIsNotNone(world.footprint, name)
            self.assertFalse(world.collides((cfg['x'], cfg['y'], cfg['yaw'])), name)

    def test_el_angulo_de_los_sonares_de_las_esquinas_sale_de_un_parametro(self):
        from lawnbot_nav.robot_geometry import CORNER_SONAR_ANGLE_DEG, sonar_yaws
        self.assertEqual(CORNER_SONAR_ANGLE_DEG, 30.0)
        self.assertEqual(sonar_yaws(25.0), {'left': 25.0, 'center': 0.0, 'right': -25.0})
        world = SimWorld(walls=[((1.0, -2.0), (1.0, 2.0))], footprint=self.fp)
        straight = world.sonars((0.0, 0.0, 0.0), sonar_yaws(0.0))['left']
        angled = world.sonars((0.0, 0.0, 0.0), sonar_yaws(30.0))['left']
        self.assertGreater(angled, straight)                                    # mirando de costado ve la pared mas lejos


if __name__ == '__main__':
    unittest.main()