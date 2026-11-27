import math
import random
import unittest

from lawnbot_nav.mode_logic import resolve_mode
from lawnbot_nav.safety_logic import (SafetyParams, SafetyState, apply_safety,
                                      sonar_scale)

P = SafetyParams()  # slow 0.30, stop 0.15, max_linear 0.5, max_angular 1.5


def gate(v, w, **kw):
    return apply_safety(v, w, SafetyState(**kw), P)


class SafetyTests(unittest.TestCase):
    def test_sin_obstaculos_pasa_igual(self):
        self.assertEqual(gate(0.3, 0.5, sonar_ranges=[1.5, 1.5, 1.5]), (0.3, 0.5, 'none'))

    def test_bumper_frontal_prohibe_avance_permite_retroceso_y_giro(self):
        self.assertEqual(gate(0.3, 0.0, front_pressed=True)[0], 0.0)
        self.assertEqual(gate(-0.2, 0.0, front_pressed=True)[0], -0.2)
        self.assertEqual(gate(0.0, 1.0, front_pressed=True)[1], 1.0)
        self.assertEqual(gate(0.3, 0.0, front_pressed=True)[2], 'bumper_front')

    def test_bumper_trasero_prohibe_retroceso(self):
        self.assertEqual(gate(-0.3, 0.0, back_pressed=True)[0], 0.0)
        self.assertEqual(gate(0.3, 0.0, back_pressed=True)[0], 0.3)
        self.assertEqual(gate(-0.3, 0.0, back_pressed=True)[2], 'bumper_back')

    def test_sonar_usa_el_minimo_de_los_tres(self):
        v, _, alert = gate(0.3, 0.0, sonar_ranges=[0.225, 1.5, 1.5])
        self.assertAlmostEqual(v, 0.3 * 0.5)
        self.assertEqual(alert, 'obstacle_near')

    def test_sonar_zona_de_parada(self):
        v, _, alert = gate(0.3, 0.4, sonar_ranges=[1.5, 0.15, 1.5])
        self.assertEqual(v, 0.0)
        self.assertEqual(alert, 'obstacle_stop')
        self.assertEqual(gate(0.3, 0.4, sonar_ranges=[0.10])[1], 0.4)  # giro permitido

    def test_sonar_cerca_no_limita_retroceso(self):
        self.assertEqual(gate(-0.2, 0.0, sonar_ranges=[0.05])[0], -0.2)

    def test_lectura_nan_se_ignora(self):
        self.assertEqual(gate(0.3, 0.0, sonar_ranges=[math.nan, 1.5])[0], 0.3)

    def test_sensor_viejo_bloquea_solo_avance(self):
        self.assertEqual(gate(0.3, 0.5, stale=True)[:2], (0.0, 0.5))
        self.assertEqual(gate(-0.3, 0.0, stale=True)[0], -0.3)
        self.assertEqual(gate(0.3, 0.0, stale=True)[2], 'sensor_timeout')

    def test_limites_maximos_solo_reducen(self):
        v, w, _ = gate(2.0, -9.0)
        self.assertEqual((v, w), (0.5, -1.5))

    def test_nunca_amplifica(self):
        rng = random.Random(1)
        for _ in range(5000):
            v = rng.uniform(-3, 3)
            w = rng.uniform(-3, 3)
            st = SafetyState(
                sonar_ranges=[rng.choice([0.0, 0.1, 0.2, 0.3, 1.5, math.nan])
                              for _ in range(rng.randint(0, 3))],
                front_pressed=rng.random() < 0.3,
                back_pressed=rng.random() < 0.3,
                stale=rng.random() < 0.2)
            vo, wo, _ = apply_safety(v, w, st, P)
            self.assertLessEqual(abs(vo), abs(v) + 1e-12)
            self.assertLessEqual(abs(wo), abs(w) + 1e-12)
            self.assertTrue(vo == 0.0 or (vo > 0) == (v > 0))
            self.assertTrue(wo == 0.0 or (wo > 0) == (w > 0))

    def test_escala_de_sonar(self):
        self.assertEqual(sonar_scale(0.5, 0.30, 0.15), 1.0)
        self.assertEqual(sonar_scale(0.15, 0.30, 0.15), 0.0)
        self.assertAlmostEqual(sonar_scale(0.225, 0.30, 0.15), 0.5)


class ModeTests(unittest.TestCase):
    def test_sin_heartbeat_es_idle(self):
        self.assertEqual(resolve_mode('manual', None, 3.0), 'idle')

    def test_heartbeat_vigente(self):
        self.assertEqual(resolve_mode('auto_parallel', 1.0, 3.0), 'auto_parallel')

    def test_heartbeat_vencido(self):
        self.assertEqual(resolve_mode('auto_parallel', 3.5, 3.0), 'idle')

    def test_modo_desconocido(self):
        self.assertEqual(resolve_mode('volar', 0.5, 3.0), 'idle')


if __name__ == '__main__':
    unittest.main()
