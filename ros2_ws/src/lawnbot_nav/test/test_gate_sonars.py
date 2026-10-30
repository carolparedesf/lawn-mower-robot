import math
import random
import unittest

from lawnbot_nav.safety_logic import (SafetyParams, SafetyState, apply_safety, ignored_sonars,
                                      sonar_scale)


def gate(v, w, params=None, **kw):
    return apply_safety(v, w, SafetyState(**kw), params or SafetyParams())


class PerSonarThresholdTests(unittest.TestCase):
    def setUp(self):
        # laterales mas permisivos (0.20 / 0.08), central con los globales (0.30 / 0.15)
        self.p = SafetyParams(sonar_thresholds={'left': (0.20, 0.08), 'right': (0.20, 0.08)})

    def test_el_lateral_usa_su_propio_umbral(self):
        # 0.12 m: el lateral solo frena (0.08 < 0.12 < 0.20), el central ya habria parado (< 0.15)
        v, _, alert = gate(0.3, 0.0, self.p, sonar_ranges={'left': 0.12, 'center': 1.5, 'right': 1.5})
        self.assertAlmostEqual(v, 0.3 * (0.12 - 0.08) / (0.20 - 0.08))
        self.assertEqual(alert, 'obstacle_near')
        v, _, alert = gate(0.3, 0.0, self.p, sonar_ranges={'left': 1.5, 'center': 0.12, 'right': 1.5})
        self.assertEqual((v, alert), (0.0, 'obstacle_stop'))

    def test_el_lateral_para_en_su_propio_umbral_de_parada(self):
        v, _, alert = gate(0.3, 0.0, self.p, sonar_ranges={'left': 0.08, 'center': 1.5, 'right': 1.5})
        self.assertEqual((v, alert), (0.0, 'obstacle_stop'))

    def test_gana_el_sonar_mas_restrictivo(self):
        v, _, _ = gate(0.3, 0.0, self.p, sonar_ranges={'left': 0.14, 'center': 0.225, 'right': 1.5})
        self.assertAlmostEqual(v, 0.3 * 0.5)                      # el central (0.5) es mas restrictivo que el lateral (0.5)

    def test_lista_sigue_usando_los_umbrales_globales(self):
        v, _, _ = gate(0.3, 0.0, self.p, sonar_ranges=[0.225])
        self.assertAlmostEqual(v, 0.3 * 0.5)

    def test_sonar_sin_umbral_propio_hereda_los_globales(self):
        v, _, _ = gate(0.3, 0.0, SafetyParams(sonar_thresholds={'left': (0.5, 0.2)}),
                       sonar_ranges={'left': 1.5, 'center': 0.225, 'right': 1.5})
        self.assertAlmostEqual(v, 0.3 * 0.5)

    def test_retroceso_y_giro_no_dependen_de_los_sonares(self):
        v, w, _ = gate(-0.2, 0.6, self.p, sonar_ranges={'left': 0.01, 'center': 0.01, 'right': 0.01})
        self.assertEqual((v, w), (-0.2, 0.6))


class IgnoreSideSonarsTests(unittest.TestCase):
    def test_solo_los_laterales_y_solo_en_auto_perimeter(self):
        self.assertEqual(ignored_sonars(True, 'auto_perimeter', True), ('left', 'right'))
        for mode in ('manual', 'idle', 'auto_random', 'auto_parallel', 'stop', None):
            self.assertEqual(ignored_sonars(True, mode, True), (), mode)

    def test_nunca_incluye_al_sonar_central(self):
        for ignore in (True, False):
            for mode in ('auto_perimeter', 'manual', None):
                for fresh in (True, False):
                    self.assertNotIn('center', ignored_sonars(ignore, mode, fresh))

    def test_sin_modo_vigente_o_con_el_parametro_apagado_se_consideran_todos(self):
        self.assertEqual(ignored_sonars(True, 'auto_perimeter', False), ())
        self.assertEqual(ignored_sonars(False, 'auto_perimeter', True), ())

    def test_el_central_sigue_frenando_aunque_los_laterales_se_ignoren(self):
        # el nodo quita los laterales del estado; con solo el central, la proteccion se mantiene
        v, _, alert = gate(0.2, 0.0, sonar_ranges={'center': 0.10})
        self.assertEqual((v, alert), (0.0, 'obstacle_stop'))
        v, _, alert = gate(0.2, 0.0, sonar_ranges={'center': 0.225})
        self.assertAlmostEqual(v, 0.1)

    def test_los_bumpers_siguen_protegiendo(self):
        self.assertEqual(gate(0.2, 0.0, sonar_ranges={'center': 1.5}, front_pressed=True)[0], 0.0)
        self.assertEqual(gate(-0.2, 0.0, sonar_ranges={'center': 1.5}, back_pressed=True)[0], 0.0)

    def test_un_sensor_requerido_sin_datos_sigue_bloqueando_el_avance(self):
        self.assertEqual(gate(0.2, 0.0, sonar_ranges={'center': 1.5}, stale=True)[0], 0.0)


class NeverAmplifiesTests(unittest.TestCase):
    def test_nunca_amplifica_con_umbrales_por_sonar_aleatorios(self):
        rng = random.Random(11)
        for _ in range(8000):
            thresholds = {name: (rng.uniform(-0.5, 2.0), rng.uniform(-0.5, 2.0))
                          for name in ('left', 'center', 'right') if rng.random() < 0.7}
            params = SafetyParams(sonar_slow_distance=rng.uniform(0.0, 1.0),
                                  sonar_stop_distance=rng.uniform(0.0, 1.0),
                                  max_linear=rng.uniform(0.05, 1.0), max_angular=rng.uniform(0.1, 3.0),
                                  sonar_thresholds=thresholds)
            ranges = {name: rng.choice([0.0, 0.05, 0.1, 0.2, 0.3, 1.5, math.nan, math.inf])
                      for name in ('left', 'center', 'right') if rng.random() < 0.8}
            v, w = rng.uniform(-3, 3), rng.uniform(-3, 3)
            vo, wo, _ = apply_safety(v, w, SafetyState(sonar_ranges=ranges, front_pressed=rng.random() < 0.2,
                                                      back_pressed=rng.random() < 0.2, stale=rng.random() < 0.1),
                                     params)
            self.assertLessEqual(abs(vo), abs(v) + 1e-12)
            self.assertLessEqual(abs(wo), abs(w) + 1e-12)
            self.assertTrue(vo == 0.0 or (vo > 0) == (v > 0))
            self.assertTrue(wo == 0.0 or (wo > 0) == (w > 0))

    def test_los_umbrales_degenerados_no_dan_factores_fuera_de_0_1(self):
        for slow, stop in ((0.1, 0.5), (0.3, 0.3), (-1.0, -1.0), (0.0, 0.0), (5.0, 4.0)):
            for d in (0.0, 0.05, 0.3, 1.0, 5.0):
                self.assertTrue(0.0 <= sonar_scale(d, slow, max(stop, 0.0)) <= 1.0, (slow, stop, d))

    def test_un_umbral_laxo_en_un_lateral_no_afloja_al_central(self):
        params = SafetyParams(sonar_thresholds={'left': (0.01, 0.0), 'right': (0.01, 0.0)})
        v, _, _ = gate(0.2, 0.0, params, sonar_ranges={'left': 1.5, 'center': 0.10, 'right': 1.5})
        self.assertEqual(v, 0.0)                                   # el central conserva 0.30 / 0.15


if __name__ == '__main__':
    unittest.main()
