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
        # pared sur pegada al costado derecho, con el robot al fondo (esquina trasera) y al frente (esquina delantera)
        room = SimWorld.room(10.0, 8.0)
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


if __name__ == '__main__':
    unittest.main()
