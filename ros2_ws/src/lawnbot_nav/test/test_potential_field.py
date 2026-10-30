import math
import unittest

from lawnbot_nav.potential_field_logic import (ALERT_LOCAL_MINIMUM, PFCore, PFParams, box_distance, repulsion,
                                               sonar_point)

CENTER = (0.198, 0.0, 0.0)
LEFT = (0.194, 0.136, math.radians(45))
RIGHT = (0.193, -0.1435, math.radians(-45))


class Geometry(unittest.TestCase):
    def test_distancia_al_cuerpo(self):
        self.assertEqual(box_distance((0.0, 0.0)), 0.0)
        self.assertAlmostEqual(box_distance((0.2137 + 0.3, 0.0)), 0.3)
        self.assertAlmostEqual(box_distance((0.2137 + 0.3, 0.4 + 0.2113)), math.hypot(0.3, 0.4))

    def test_punto_de_sonar(self):
        x, y = sonar_point(CENTER, 0.5)
        self.assertAlmostEqual(x, 0.698)
        x, y = sonar_point(LEFT, 1.0)
        self.assertGreater(y, 0.136)

    def test_sin_obstaculos_no_hay_repulsion(self):
        self.assertEqual(repulsion([], PFParams())[:2], (0.0, 0.0))

    def test_lejos_no_repele(self):
        rx, ry, _ = repulsion([sonar_point(CENTER, 0.9)], PFParams())
        self.assertEqual((rx, ry), (0.0, 0.0))

    def test_repulsion_apunta_lejos_del_obstaculo(self):
        rx, ry, _ = repulsion([sonar_point(LEFT, 0.3)], PFParams())
        self.assertLess(rx, 0.0)
        self.assertLess(ry, 0.0, 'obstaculo a la izquierda: empuja a la derecha')

    def test_norma_acotada(self):
        obs = [sonar_point(CENTER, 0.16), sonar_point(LEFT, 0.16), sonar_point(RIGHT, 0.16)]
        rx, ry, _ = repulsion(obs, PFParams())
        self.assertLessEqual(math.hypot(rx, ry), 1.0 + 1e-9)


class Correction(unittest.TestCase):
    def setUp(self):
        self.core = PFCore(PFParams())

    def out(self, nominal, obstacles, now=0.0, pose=(0.0, 0.0)):
        return self.core.update(now, nominal, obstacles, pose)

    def test_sin_obstaculos_pasa_igual(self):
        self.assertEqual(self.out((0.3, 0.1), []), (0.3, 0.1))

    def test_nunca_crea_movimiento(self):
        self.assertEqual(self.out((0.0, 0.0), [sonar_point(CENTER, 0.2)]), (0.0, 0.0))

    def test_sin_nominal_no_publica(self):
        self.assertIsNone(self.out(None, [sonar_point(CENTER, 0.2)]))

    def test_nunca_aumenta_la_velocidad_lineal(self):
        for r in (0.15, 0.2, 0.3, 0.45, 0.59):
            for pt in (CENTER, LEFT, RIGHT):
                v, _ = self.out((0.3, 0.0), [sonar_point(pt, r)])
                self.assertLessEqual(v, 0.3 + 1e-9)
                self.assertGreaterEqual(v, 0.0)

    def test_frena_mas_cuanto_mas_cerca_de_frente(self):
        far = self.out((0.3, 0.0), [sonar_point(CENTER, 0.5)])[0]
        near = self.out((0.3, 0.0), [sonar_point(CENTER, 0.2)])[0]
        self.assertGreater(far, near)

    def test_gira_alejandose(self):
        _, w = self.out((0.3, 0.0), [sonar_point(LEFT, 0.3)])
        self.assertLess(w, 0.0)
        _, w = self.out((0.3, 0.0), [sonar_point(RIGHT, 0.3)])
        self.assertGreater(w, 0.0)

    def test_de_frente_desempata_hacia_tie_turn(self):
        _, w = PFCore(PFParams(tie_turn=1.0)).update(0.0, (0.3, 0.0), [sonar_point(CENTER, 0.25)], (0, 0))
        self.assertGreater(w, 0.0)
        _, w = PFCore(PFParams(tie_turn=-1.0)).update(0.0, (0.3, 0.0), [sonar_point(CENTER, 0.25)], (0, 0))
        self.assertLess(w, 0.0)

    def test_giro_agregado_acotado(self):
        obs = [sonar_point(LEFT, 0.15), sonar_point(CENTER, 0.15)]
        _, w = self.out((0.3, 0.0), obs)
        self.assertLessEqual(abs(w), 0.6 + 1e-9)

    def test_marcha_atras_no_se_toca_en_lineal(self):
        v, _ = self.out((-0.1, 0.0), [sonar_point(CENTER, 0.2)])
        self.assertEqual(v, -0.1)


class LocalMinimum(unittest.TestCase):
    def run_stuck(self, core, seconds, obstacles, pose=(1.0, 1.0), nominal=(0.3, 0.0), dt=0.1):
        t, outs = 0.0, []
        while t < seconds:
            outs.append(core.update(t, nominal, obstacles, pose))
            t += dt
        return t, outs

    def test_trabado_avisa_y_cede(self):
        core = PFCore(PFParams(stall_window_s=6.0, yield_s=4.0))
        _, outs = self.run_stuck(core, 7.0, [sonar_point(CENTER, 0.25)])
        self.assertEqual(core.alert, ALERT_LOCAL_MINIMUM)
        self.assertIsNone(outs[-1], 'cede: no publica mientras dura yield_s')

    def test_avanzando_no_es_minimo_local(self):
        core = PFCore(PFParams(stall_window_s=6.0))
        t = 0.0
        while t < 20.0:
            out = core.update(t, (0.3, 0.0), [sonar_point(CENTER, 0.3)], (0.1 * t, 0.0))
            self.assertIsNotNone(out)
            t += 0.1
        self.assertIsNone(core.alert)

    def test_sin_obstaculos_no_es_minimo_local(self):
        core = PFCore(PFParams())
        self.run_stuck(core, 30.0, [])
        self.assertIsNone(core.alert)

    def test_sin_pose_no_detecta(self):
        core = PFCore(PFParams())
        self.run_stuck(core, 30.0, [sonar_point(CENTER, 0.25)], pose=None)
        self.assertIsNone(core.alert)

    def test_retoma_tras_ceder(self):
        core = PFCore(PFParams(stall_window_s=6.0, yield_s=4.0))
        t, _ = self.run_stuck(core, 7.0, [sonar_point(CENTER, 0.25)])
        out = core.update(t + 5.0, (0.3, 0.0), [], (1.0, 1.0))
        self.assertIsNotNone(out)
        self.assertIsNone(core.alert)


if __name__ == '__main__':
    unittest.main()
