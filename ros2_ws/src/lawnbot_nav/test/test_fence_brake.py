"""Freno por cerco del safety_gate: en modos automaticos /geofence/inside false frena y avisa outside_fence; en manual no."""
import unittest

from lawnbot_nav.safety_logic import (ALERT_NEAR_FENCE_EDGE, ALERT_OBSTACLE_NEAR, ALERT_OUTSIDE_FENCE, ALERT_PRIORITY,
                                      AUTOMATIC_MODES, SafetyParams, SafetyState, apply_safety, fence_alert,
                                      fence_blocks_motion, merge_alerts)

T = 1.0     # geofence_timeout


class FrenoTests(unittest.TestCase):
    def test_afuera_en_modos_automaticos_frena(self):
        self.assertEqual(set(AUTOMATIC_MODES), {'auto_random', 'auto_parallel', 'auto_perimeter'})
        for mode in AUTOMATIC_MODES:
            self.assertTrue(fence_blocks_motion(False, 0.1, 'OK', mode, True, T), mode)
            self.assertFalse(fence_blocks_motion(True, 0.1, 'OK', mode, True, T), mode)

    def test_en_manual_idle_stop_no_frena(self):
        for mode in ('manual', 'idle', 'stop', None):
            self.assertFalse(fence_blocks_motion(False, 0.1, 'OK', mode, True, T), mode)

    def test_modo_viejo_no_frena(self):
        self.assertFalse(fence_blocks_motion(False, 0.1, 'OK', 'auto_random', False, T))

    def test_sin_cerco_o_sin_geofence_node_no_frena(self):
        self.assertFalse(fence_blocks_motion(False, 0.1, 'NO_POLYGON', 'auto_perimeter', True, T))
        self.assertFalse(fence_blocks_motion(None, 99.0, None, 'auto_perimeter', True, T))

    def test_con_cerco_y_sin_dato_de_inside_frena(self):
        # falla segura: el cerco esta cargado pero inside no llega (o dejo de llegar)
        self.assertTrue(fence_blocks_motion(None, 99.0, 'OK', 'auto_parallel', True, T))
        self.assertTrue(fence_blocks_motion(True, 1.5, 'OK', 'auto_parallel', True, T))
        # sin pose o esperando conversion geofence_node publica inside false: tambien frena
        self.assertTrue(fence_blocks_motion(False, 0.1, 'NO_POSE', 'auto_parallel', True, T))

    def test_apply_safety_anula_avance_y_giro(self):
        state = SafetyState(sonar_ranges=[1.5], outside_fence_auto=True)
        for v, w in ((0.3, 0.8), (-0.2, -0.5), (0.0, 1.0)):
            out = apply_safety(v, w, state, SafetyParams())
            self.assertEqual(out[:2], (0.0, 0.0))
        self.assertEqual(apply_safety(0.3, 0.8, SafetyState(sonar_ranges=[1.5]), SafetyParams())[:2], (0.3, 0.8))


class AlertaTests(unittest.TestCase):
    def test_alerta_al_frenar(self):
        self.assertTrue(fence_alert(True, None, 99.0, 'OK', T))

    def test_en_manual_afuera_avisa_sin_frenar(self):
        self.assertTrue(fence_alert(False, False, 0.1, 'OK', T))

    def test_sin_pose_no_dice_que_salio(self):
        self.assertFalse(fence_alert(False, False, 0.1, 'NO_POSE', T))
        self.assertFalse(fence_alert(False, False, 5.0, 'OK', T))      # dato viejo
        self.assertFalse(fence_alert(False, True, 0.1, 'OK', T))

    def test_outside_fence_manda_sobre_el_aviso_de_borde(self):
        ext = {'geofence': (ALERT_NEAR_FENCE_EDGE, 0.0), 'safety_gate:geofence': (ALERT_OUTSIDE_FENCE, 0.0)}
        self.assertEqual(merge_alerts('none', ext), ALERT_OUTSIDE_FENCE)

    def test_el_aviso_de_borde_es_el_de_menor_prioridad(self):
        self.assertEqual(ALERT_PRIORITY[-1], ALERT_NEAR_FENCE_EDGE)
        self.assertEqual(merge_alerts(ALERT_OBSTACLE_NEAR, {'geofence': (ALERT_NEAR_FENCE_EDGE, 0.0)}),
                         ALERT_OBSTACLE_NEAR)


if __name__ == '__main__':
    unittest.main()
