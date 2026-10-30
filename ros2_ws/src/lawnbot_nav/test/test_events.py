import unittest

from lawnbot_nav.events_logic import (CUTTING_STARTED, LIFTED, LOW_BATTERY, NAVIGATION_STARTED,
                                      PERIMETER_STARTED, EventCore, EventParams)


class Run:
    """Avanza el nucleo a 1 Hz y junta los eventos."""
    def __init__(self, **kw):
        self.core, self.t, self.events = EventCore(EventParams(**kw)), 0.0, []
        self.mode, self.cutting, self.lifted, self.batt = 'idle', False, False, 80.0

    def step(self, n=1):
        got = []
        for _ in range(n):
            self.t += 1.0
            got += self.core.update(self.t, self.mode, self.cutting, self.lifted, self.batt)
        self.events += got
        return got


class ModeEvents(unittest.TestCase):
    def test_primer_dato_no_genera_eventos(self):
        r = Run()
        r.mode, r.cutting = 'auto_perimeter', True
        self.assertEqual(r.step(3), [])

    def test_navegacion_y_perimetro_una_sola_vez(self):
        r = Run()
        r.step()
        r.mode = 'auto_parallel'
        self.assertEqual(r.step(5), [NAVIGATION_STARTED])
        r.mode = 'auto_random'
        self.assertEqual(r.step(), [], 'parallel -> random sigue siendo navegacion')
        r.mode = 'auto_perimeter'
        self.assertEqual(r.step(3), [PERIMETER_STARTED])
        r.mode = 'idle'
        self.assertEqual(r.step(), [])
        r.mode = 'auto_random'
        self.assertEqual(r.step(), [NAVIGATION_STARTED])

    def test_manual_no_es_navegacion(self):
        r = Run()
        r.step()
        r.mode = 'manual'
        self.assertEqual(r.step(3), [])

    def test_cutting_started_en_el_flanco(self):
        r = Run()
        r.step()
        r.cutting = True
        self.assertEqual(r.step(4), [CUTTING_STARTED])
        r.cutting = False
        r.step()
        r.cutting = True
        self.assertEqual(r.step(), [CUTTING_STARTED])


class LiftedEvents(unittest.TestCase):
    def test_un_levantado_un_evento_aunque_rebote(self):
        r = Run(lift_hold_s=3.0)
        r.step()
        for pressed in (True, False, True, False, False):
            r.lifted = pressed
            r.step()
        self.assertEqual(r.events.count(LIFTED), 1)

    def test_vuelve_a_avisar_tras_soltarlo(self):
        r = Run(lift_hold_s=3.0)
        r.step()
        r.lifted = True
        r.step()
        r.lifted = False
        r.step(5)
        r.lifted = True
        r.step()
        self.assertEqual(r.events.count(LIFTED), 2)


class BatteryEvents(unittest.TestCase):
    def test_avisa_una_vez_y_no_repite(self):
        r = Run(low_battery_threshold=20.0, low_battery_samples=3)
        r.step()
        r.batt = 19.0
        r.step(100)
        self.assertEqual(r.events.count(LOW_BATTERY), 1)

    def test_ruido_no_dispara(self):
        r = Run(low_battery_samples=3)
        for b in (19, 21, 19, 21, 19, 21):
            r.batt = b
            r.step()
        self.assertNotIn(LOW_BATTERY, r.events)

    def test_histeresis_dentro_de_la_banda_no_rearma(self):
        r = Run(low_battery_threshold=20.0, low_battery_margin=5.0, low_battery_samples=1)
        r.batt = 19.0
        r.step()
        for b in (22.0, 18.0, 23.0, 19.0):      # oscila sin llegar a 25
            r.batt = b
            r.step()
        self.assertEqual(r.events.count(LOW_BATTERY), 1)

    def test_rearma_al_superar_umbral_mas_margen(self):
        r = Run(low_battery_threshold=20.0, low_battery_margin=5.0, low_battery_samples=1)
        r.batt = 19.0
        r.step()
        r.batt = 26.0
        r.step()
        r.batt = 19.0
        r.step()
        self.assertEqual(r.events.count(LOW_BATTERY), 2)

    def test_sin_dato_de_bateria_no_avisa_ni_rearma(self):
        r = Run(low_battery_samples=1)
        r.batt = None
        r.step(10)
        self.assertEqual(r.events, [])

    def test_recordatorio_opcional(self):
        r = Run(low_battery_samples=1, low_battery_repeat_s=30.0)
        r.batt = 10.0
        r.step(65)
        self.assertEqual(r.events.count(LOW_BATTERY), 3)


if __name__ == '__main__':
    unittest.main()
