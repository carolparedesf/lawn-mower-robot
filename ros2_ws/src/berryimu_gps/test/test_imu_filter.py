"""Ganancias del filtro complementario independientes de la frecuencia (imu_filter.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from berryimu_gps.imu_filter import rate_scaled_gain  # noqa: E402


def residual(gain, rate_hz, seconds=1.0):
    """Fraccion del error que queda tras `seconds` corrigiendo a rate_hz."""
    left = 1.0
    for _ in range(int(round(seconds * rate_hz))):
        left *= 1.0 - rate_scaled_gain(gain, 1.0 / rate_hz)
    return left


class GananciaPorFrecuencia(unittest.TestCase):
    def test_a_50_hz_es_la_ganancia_configurada(self):
        self.assertAlmostEqual(rate_scaled_gain(0.02, 0.02), 0.02)

    def test_a_25_hz_corrige_lo_mismo_por_segundo(self):
        for gain in (0.02, 0.01):
            self.assertAlmostEqual(residual(gain, 25.0), residual(gain, 50.0), places=12)
        self.assertGreater(rate_scaled_gain(0.02, 0.04), 0.02)          # mas por muestra

    def test_constante_de_tiempo_del_acelerometro_cerca_de_1_s(self):
        self.assertAlmostEqual(residual(0.02, 25.0, seconds=1.0), 0.98 ** 50, places=12)

    def test_bordes(self):
        self.assertEqual(rate_scaled_gain(0.02, 0.0), 0.0)                # primera muestra
        self.assertEqual(rate_scaled_gain(0.0, 0.04), 0.0)
        self.assertEqual(rate_scaled_gain(1.0, 0.04), 1.0)
        self.assertAlmostEqual(rate_scaled_gain(0.02, 0.04, ref_hz=25.0), 0.02)


if __name__ == '__main__':
    unittest.main()
