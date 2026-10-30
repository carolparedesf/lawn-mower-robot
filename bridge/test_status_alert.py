import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from status_alert import pick_alert


class PickAlert(unittest.TestCase):
    def test_sin_nada(self):
        self.assertEqual(pick_alert('none', None, None), 'none')

    def test_evento_reciente_se_muestra(self):
        self.assertEqual(pick_alert('none', 'low_battery', 1.0), 'low_battery')

    def test_evento_vence(self):
        self.assertEqual(pick_alert('none', 'low_battery', 5.1), 'none')

    def test_la_seguridad_gana(self):
        self.assertEqual(pick_alert('geofence_breach', 'lifted', 0.5), 'geofence_breach')

    def test_vuelve_al_evento_si_la_seguridad_se_apaga(self):
        self.assertEqual(pick_alert('none', 'lifted', 2.0), 'lifted')


if __name__ == '__main__':
    unittest.main()
