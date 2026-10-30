import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from actuator_relay import ActuatorRelay, mode_allows_actuators  # noqa: E402


class RelayTests(unittest.TestCase):
    def test_arranca_apagado(self):
        r = ActuatorRelay()
        self.assertEqual(r.outputs(True), (False, False, False))
        self.assertEqual(r.outputs(False), (False, False, False))

    def test_payload_real_de_la_app(self):
        r = ActuatorRelay()
        self.assertTrue(r.on_command({'blade': True, 'trimmer': False, 'charging': False}, True))
        self.assertEqual(r.outputs(True), (True, False, False))
        r.on_command({'blade': True, 'trimmer': True, 'charging': False}, True)
        self.assertEqual(r.outputs(True), (True, True, False))

    def test_cada_payload_reemplaza_al_anterior(self):
        r = ActuatorRelay()
        r.on_command({'blade': True, 'trimmer': True}, True)
        r.on_command({'blade': False, 'trimmer': True}, True)
        self.assertEqual(r.outputs(True), (False, True, False))
        r.on_command({'blade': False, 'trimmer': False, 'charging': False}, True)     # emergencyStop de la app
        self.assertEqual(r.outputs(True), (False, False, False))

    def test_solo_el_booleano_true_enciende(self):
        for bad in ('true', 'True', 1, 'yes', None, [True], {'x': 1}, 0, ''):
            r = ActuatorRelay()
            r.on_command({'blade': bad, 'trimmer': bad, 'force_cut': bad}, True)
            self.assertEqual(r.outputs(True), (False, False, False), repr(bad))

    def test_claves_ausentes_son_false(self):
        r = ActuatorRelay()
        r.on_command({'blade': True}, True)
        self.assertEqual(r.outputs(True), (True, False, False))
        r.on_command({}, True)
        self.assertEqual(r.outputs(True), (False, False, False))

    def test_payload_que_no_es_un_objeto_se_ignora(self):
        r = ActuatorRelay()
        r.on_command({'blade': True}, True)
        for junk in ([], 'blade', 3, None):
            self.assertFalse(r.on_command(junk, True))
        self.assertEqual(r.outputs(True), (True, False, False))        # no cambia el estado vigente

    def test_force_cut_es_opcional_y_se_pasa_tal_cual(self):
        r = ActuatorRelay()
        r.on_command({'blade': True, 'trimmer': False, 'force_cut': True}, True)
        self.assertEqual(r.outputs(True), (True, False, True))
        r.on_command({'blade': True, 'trimmer': False}, True)          # la app actual no lo manda
        self.assertEqual(r.outputs(True), (True, False, False))

    def test_charging_no_se_atiende(self):
        r = ActuatorRelay()
        r.on_command({'charging': True}, True)
        self.assertEqual(r.outputs(True), (False, False, False))

    def test_sin_modo_vigente_se_ignora_y_queda_apagado(self):
        r = ActuatorRelay()
        self.assertFalse(r.on_command({'blade': True, 'trimmer': True, 'force_cut': True}, False))
        self.assertEqual(r.outputs(True), (False, False, False))

    def test_al_perder_el_modo_se_borra_y_no_vuelve_solo(self):
        r = ActuatorRelay()
        r.on_command({'blade': True, 'trimmer': True, 'force_cut': True}, True)
        self.assertEqual(r.outputs(True), (True, True, True))
        self.assertEqual(r.outputs(False), (False, False, False))      # idle / stop / sin heartbeat
        self.assertEqual(r.outputs(True), (False, False, False))       # vuelve el modo: sigue apagado
        r.on_command({'blade': True}, True)                            # la app tiene que pedir de nuevo
        self.assertEqual(r.outputs(True), (True, False, False))

    def test_un_pedido_en_pleno_corte_con_el_modo_perdido_apaga(self):
        r = ActuatorRelay()
        r.on_command({'blade': True}, True)
        r.on_command({'blade': True}, False)
        self.assertEqual(r.outputs(True), (False, False, False))

    def test_modos_que_permiten_actuadores(self):
        for m in ('manual', 'auto_parallel', 'auto_random', 'auto_perimeter'):
            self.assertTrue(mode_allows_actuators(m), m)
        for m in ('idle', 'stop', '', 'MANUAL', 'automatic', 'manual2'):
            self.assertFalse(mode_allows_actuators(m), m)


if __name__ == '__main__':
    unittest.main()
