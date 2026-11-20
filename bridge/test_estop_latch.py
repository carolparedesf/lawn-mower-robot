import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from estop_latch import EstopLatch, is_release_command


class Latch(unittest.TestCase):
    def test_arranca_sin_estop(self):
        self.assertFalse(EstopLatch().active)

    def test_lifted_se_recuerda(self):
        latch = EstopLatch()
        latch.on_event('lifted')
        self.assertTrue(latch.active)

    def test_otros_eventos_no_activan(self):
        latch = EstopLatch()
        for event in ('low_battery', 'cutting_started', 'navigation_started', None):
            latch.on_event(event)
        self.assertFalse(latch.active)

    def test_un_evento_posterior_no_libera(self):
        latch = EstopLatch()
        latch.on_event('lifted')
        latch.on_event('low_battery')
        self.assertTrue(latch.active)

    def test_estop_true_se_recuerda_hasta_estop_false(self):
        latch = EstopLatch()
        latch.on_estop(True)
        self.assertTrue(latch.active)
        latch.on_estop(True)
        self.assertTrue(latch.active)
        latch.on_estop(False)
        self.assertFalse(latch.active)

    def test_estop_false_libera_un_lifted(self):
        latch = EstopLatch()
        latch.on_event('lifted')
        latch.on_estop(False)
        self.assertFalse(latch.active)

    def test_release(self):
        latch = EstopLatch()
        latch.on_event('lifted')
        latch.release()
        self.assertFalse(latch.active)


class ReleaseCommand(unittest.TestCase):
    def test_active_false_libera(self):
        self.assertTrue(is_release_command({'active': False}))

    def test_active_true_se_ignora(self):
        self.assertFalse(is_release_command({'active': True}))

    def test_sin_clave_se_ignora(self):
        self.assertFalse(is_release_command({}))

    def test_valores_que_no_son_false_se_ignoran(self):
        for value in (0, None, 'false', 'False', [], ''):
            self.assertFalse(is_release_command({'active': value}), repr(value))

    def test_payload_que_no_es_objeto(self):
        for payload in (False, None, [], 'x', 0):
            self.assertFalse(is_release_command(payload), repr(payload))


if __name__ == '__main__':
    unittest.main()
