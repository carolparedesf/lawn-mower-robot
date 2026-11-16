import unittest

from lawnbot_nav.safety_logic import (ALERT_IMU_LOST, ALERT_PRIORITY, AUTOMATIC_MODES, SafetyParams, SafetyState,
                                      apply_safety, imu_blocks_motion, imu_is_lost, merge_alerts)


class VigilanteTests(unittest.TestCase):
    def test_a_los_05_s_sin_mensajes_el_imu_esta_perdido(self):
        self.assertFalse(imu_is_lost(10.0, 9.8, 0.0, 0.5))
        self.assertFalse(imu_is_lost(10.0, 9.5, 0.0, 0.5))            # justo en el limite no
        self.assertTrue(imu_is_lost(10.0, 9.49, 0.0, 0.5))

    def test_el_limite_es_un_parametro(self):
        self.assertFalse(imu_is_lost(10.0, 9.0, 0.0, 2.0))
        self.assertTrue(imu_is_lost(10.0, 9.0, 0.0, 0.8))

    def test_un_imu_que_nunca_arranca_tambien_se_detecta(self):
        self.assertFalse(imu_is_lost(100.3, None, 100.0, 0.5))        # recien arranco el vigilante: da margen
        self.assertTrue(imu_is_lost(100.6, None, 100.0, 0.5))

    def test_timeout_cero_o_negativo_apaga_el_vigilante(self):
        self.assertFalse(imu_is_lost(1e6, None, 0.0, 0.0))
        self.assertFalse(imu_is_lost(1e6, 0.0, 0.0, -1.0))


class FrenoDeLosModosAutomaticosTests(unittest.TestCase):
    def test_solo_los_modos_automaticos_vigentes_se_frenan(self):
        alerts = {ALERT_IMU_LOST}
        for mode in AUTOMATIC_MODES:
            self.assertTrue(imu_blocks_motion(alerts, mode, True), mode)
        for mode in ('manual', 'idle', 'stop', None, 'raro'):
            self.assertFalse(imu_blocks_motion(alerts, mode, True), mode)
        self.assertFalse(imu_blocks_motion(alerts, 'auto_random', False))   # modo viejo: no se sabe cual es
        self.assertFalse(imu_blocks_motion({'obstacle_near', 'tf_lost'}, 'auto_random', True))
        self.assertFalse(imu_blocks_motion(set(), 'auto_random', True))

    def test_con_imu_perdido_en_automatico_no_avanza_ni_gira(self):
        v, w, _ = apply_safety(0.3, 0.8, SafetyState(sonar_ranges=[1.5], imu_lost_auto=True), SafetyParams())
        self.assertEqual((v, w), (0.0, 0.0))
        v, w, _ = apply_safety(-0.2, -0.5, SafetyState(sonar_ranges=[1.5], imu_lost_auto=True), SafetyParams())
        self.assertEqual((v, w), (0.0, 0.0))

    def test_el_manual_queda_libre(self):
        v, w, alert = apply_safety(0.3, 0.8, SafetyState(sonar_ranges=[1.5], imu_lost_auto=False), SafetyParams())
        self.assertEqual((v, w, alert), (0.3, 0.8, 'none'))

    def test_nunca_amplifica(self):
        for flag in (False, True):
            v, w, _ = apply_safety(0.2, -0.4, SafetyState(sonar_ranges=[1.5], imu_lost_auto=flag), SafetyParams())
            self.assertLessEqual(abs(v), 0.2)
            self.assertLessEqual(abs(w), 0.4)


class PrioridadTests(unittest.TestCase):
    def test_imu_lost_esta_en_la_lista_de_alertas_y_gana_a_las_de_navegacion(self):
        self.assertIn(ALERT_IMU_LOST, ALERT_PRIORITY)
        self.assertLess(ALERT_PRIORITY.index(ALERT_IMU_LOST), ALERT_PRIORITY.index('geofence_breach'))
        self.assertLess(ALERT_PRIORITY.index('estop'), ALERT_PRIORITY.index(ALERT_IMU_LOST))
        self.assertEqual(merge_alerts('none', {'imu': (ALERT_IMU_LOST, 0.1), 'fence': ('outside_fence', 0.1)}), ALERT_IMU_LOST)
        self.assertEqual(merge_alerts('obstacle_near', {'imu': (ALERT_IMU_LOST, 0.1)}), ALERT_IMU_LOST)
        self.assertEqual(merge_alerts('none', {'imu': (ALERT_IMU_LOST, 5.0)}), 'none')       # vencida (ttl 3 s)


if __name__ == '__main__':
    unittest.main()
