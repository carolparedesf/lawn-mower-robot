import random
import unittest

from lawnbot_nav.sonar_filter_logic import SonarFilter, SonarFilterParams, median


def feed(f, values):
    return [f.update(v) for v in values]


class Basics(unittest.TestCase):
    def test_mediana(self):
        self.assertEqual(median([3, 1, 2]), 2)
        self.assertEqual(median([4, 1, 2, 3]), 2.5)

    def test_lecturas_constantes_pasan_iguales(self):
        out = feed(SonarFilter(), [0.8] * 8)
        self.assertTrue(all(abs(o - 0.8) < 1e-12 for o in out))

    def test_ruido_chico_se_suaviza(self):
        rng = random.Random(1)
        f = SonarFilter()
        out = feed(f, [0.8 + rng.uniform(-0.03, 0.03) for _ in range(60)])
        self.assertLess(max(abs(o - 0.8) for o in out[5:]), 0.031)


class Invalid(unittest.TestCase):
    def test_invalidas_no_se_publican_ni_se_mezclan(self):
        f = SonarFilter()
        feed(f, [0.6] * 5)
        for bad in (float('nan'), float('inf'), -1.0, 0.0, 0.01, None):
            self.assertIsNone(f.update(bad), repr(bad))
        self.assertAlmostEqual(f.update(0.6), 0.6)


class Spikes(unittest.TestCase):
    def test_pico_hacia_arriba_se_rechaza(self):
        f = SonarFilter()
        feed(f, [0.6] * 5)
        self.assertAlmostEqual(f.update(1.3), 0.6)          # salto de 0.7 > 0.5
        self.assertAlmostEqual(f.update(0.6), 0.6)

    def test_pico_hacia_abajo_se_rechaza(self):
        f = SonarFilter()
        feed(f, [0.8] * 5)
        self.assertAlmostEqual(f.update(0.1), 0.8)
        self.assertAlmostEqual(f.update(0.8), 0.8)

    def test_un_pico_chico_no_mueve_la_mediana(self):
        f = SonarFilter()
        feed(f, [0.8] * 5)
        self.assertAlmostEqual(f.update(0.45), 0.8)          # dentro del umbral pero aislado

    def test_dos_lecturas_seguidas_en_otro_nivel_se_aceptan(self):
        f = SonarFilter()
        feed(f, [1.0] * 5)
        self.assertAlmostEqual(f.update(0.3), 1.0)           # primera: sospechosa
        self.assertAlmostEqual(f.update(0.3), 0.3)           # segunda: confirmada, obstaculo real

    def test_dos_picos_distintos_no_se_confirman_entre_si(self):
        f = SonarFilter()
        feed(f, [0.8] * 5)
        self.assertAlmostEqual(f.update(0.1), 0.8)
        self.assertAlmostEqual(f.update(1.4), 0.8)           # no coincide con el pico anterior: tambien se rechaza
        self.assertAlmostEqual(f.update(0.8), 0.8)

    def test_aproximacion_sostenida_tiene_una_lectura_de_retraso(self):
        f = SonarFilter()
        feed(f, [1.0] * 5)
        true, out = 1.0, []
        for _ in range(16):
            true -= 0.05
            out.append((true, f.update(true)))
        for t, o in out:
            self.assertGreaterEqual(o, t - 1e-9, 'nunca informa mas cerca de lo real')
            self.assertLessEqual(o - t, 0.0501 * 1 + 1e-9, 'a lo sumo una lectura de retraso')
        vals = [o for _, o in out]
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(vals, vals[1:])), 'baja sin oscilar')


class MaxRange(unittest.TestCase):
    def test_max_range_aislado_no_se_mezcla(self):
        f = SonarFilter()
        feed(f, [0.6] * 5)
        out = feed(f, [1.5, 0.6, 1.5, 0.6, 1.5, 0.6])
        self.assertTrue(all(abs(o - 0.6) < 1e-9 for o in out))

    def test_dos_max_range_seguidos_todavia_no(self):
        f = SonarFilter()
        feed(f, [0.6] * 5)
        out = feed(f, [1.5, 1.5])
        self.assertTrue(all(abs(o - 0.6) < 1e-9 for o in out))

    def test_tres_seguidos_publican_max_range(self):
        f = SonarFilter()
        feed(f, [0.6] * 5)
        out = feed(f, [1.5, 1.5, 1.5])
        self.assertEqual(out[-1], 1.5)

    def test_tras_tres_max_range_una_lectura_baja_aislada_no_pasa(self):
        f = SonarFilter()
        feed(f, [0.6] * 5)
        feed(f, [1.5, 1.5, 1.5])
        self.assertEqual(f.update(0.2), 1.5)                  # sospechosa, sigue libre
        self.assertEqual(f.update(1.5), 1.5)

    def test_tras_tres_max_range_un_obstaculo_real_se_confirma_a_la_segunda(self):
        f = SonarFilter()
        feed(f, [1.5, 1.5, 1.5])
        self.assertEqual(f.update(0.4), 1.5)
        self.assertAlmostEqual(f.update(0.4), 0.4)

    def test_los_max_range_no_contaminan_la_mediana(self):
        f = SonarFilter()
        feed(f, [0.5, 0.5, 0.5])
        feed(f, [1.5, 1.5])
        self.assertAlmostEqual(f.update(0.5), 0.5)

    def test_arranque_sin_historia(self):
        f = SonarFilter()
        self.assertIsNone(f.update(1.5))
        self.assertIsNone(f.update(1.5))
        self.assertEqual(f.update(1.5), 1.5)
        self.assertAlmostEqual(SonarFilter().update(0.7), 0.7)

    def test_parametros(self):
        f = SonarFilter(SonarFilterParams(window=3, max_range=2.0, max_range_run=2))
        feed(f, [0.5] * 3)
        self.assertAlmostEqual(f.update(2.0), 0.5)
        self.assertEqual(f.update(2.0), 2.0)


if __name__ == '__main__':
    unittest.main()
