"""ActiveResources: suscripciones, TF y timer solo mientras el algoritmo esta activo (CPU de la Pi)."""
import sys
import time
import types
import unittest

try:
    import tf2_ros  # noqa: F401
except ImportError:                                   # sin ROS: un TransformListener minimo
    class _Listener:
        def __init__(self, buffer, node, **kw):
            self.registered = True

        def unregister(self):
            self.registered = False
    sys.modules['tf2_ros'] = types.SimpleNamespace(TransformListener=_Listener)

import lawnbot_nav.active_resources as ar             # noqa: E402
from lawnbot_nav.active_resources import ActiveResources, Sub  # noqa: E402


class Listener:
    instances = []

    def __init__(self, buffer, node, **kw):
        self.registered = True
        Listener.instances.append(self)

    def unregister(self):
        self.registered = False


class Timer:
    def __init__(self):
        self.canceled = False

    def cancel(self):
        self.canceled = True

    def reset(self):
        self.canceled = False


class Node:
    def __init__(self):
        self.subs, self.timers, self.destroyed = {}, [], []

    def create_timer(self, period, cb):
        t = Timer()
        self.timers.append(t)
        return t

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb
        return topic

    def destroy_subscription(self, handle):
        self.destroyed.append(handle)
        del self.subs[handle]


class ActiveResourcesTests(unittest.TestCase):
    def setUp(self):
        self.now = [100.0]
        self._mono, time.monotonic = time.monotonic, lambda: self.now[0]
        self._listener, ar.TransformListener = ar.TransformListener, Listener
        Listener.instances.clear()
        self.node = Node()
        self.got = []
        subs = [Sub(object, 'sonar/left', self.got.append, None),
                Sub(object, 'bumper/B1', self.got.append, None),
                Sub(object, 'cmd_vel_random', self.got.append, None, required=False)]
        self.tf_ok = [True]
        self.res = ActiveResources(self.node, 0.05, lambda: None, subs, tf_buffer=object(),
                                   tf_ready=lambda: self.tf_ok[0], max_warmup_s=1.0)

    def tearDown(self):
        time.monotonic = self._mono
        ar.TransformListener = self._listener

    def test_arranca_inactivo_sin_suscripciones_ni_timer(self):
        self.assertFalse(self.res.running)
        self.assertEqual(self.node.subs, {})
        self.assertTrue(self.node.timers[0].canceled)
        self.assertEqual(Listener.instances, [])
        self.assertFalse(self.res.ready(self.now[0]))

    def test_start_suscribe_crea_la_tf_y_arranca_el_timer(self):
        self.res.start()
        self.assertTrue(self.res.running)
        self.assertEqual(set(self.node.subs), {'sonar/left', 'bumper/B1', 'cmd_vel_random'})
        self.assertFalse(self.node.timers[0].canceled)
        self.assertEqual(len(Listener.instances), 1)
        self.res.start()                                       # idempotente
        self.assertEqual(len(Listener.instances), 1)

    def test_los_mensajes_llegan_al_callback(self):
        self.res.start()
        self.node.subs['sonar/left']('m1')
        self.assertEqual(self.got, ['m1'])

    def test_listo_cuando_llegan_todas_las_entradas_requeridas(self):
        self.res.start()
        self.assertFalse(self.res.ready(self.now[0]))
        self.node.subs['sonar/left']('s')
        self.assertFalse(self.res.ready(self.now[0]))          # falta el bumper
        self.node.subs['bumper/B1']('b')
        self.assertTrue(self.res.ready(self.now[0]))           # cmd_vel_random no es requerido

    def test_tambien_espera_la_tf_si_se_pide(self):
        self.tf_ok[0] = False
        self.res.start()
        self.node.subs['sonar/left']('s')
        self.node.subs['bumper/B1']('b')
        self.assertFalse(self.res.ready(self.now[0]))
        self.tf_ok[0] = True
        self.assertTrue(self.res.ready(self.now[0]))

    def test_la_espera_esta_acotada(self):
        self.res.start()
        self.now[0] += 0.9
        self.assertFalse(self.res.ready(self.now[0]))
        self.now[0] += 0.1
        self.assertTrue(self.res.ready(self.now[0]))           # 1 s sin datos: arranca igual (el nucleo falla seguro)

    def test_una_vez_listo_no_vuelve_atras(self):
        self.res.start()
        self.node.subs['sonar/left']('s')
        self.node.subs['bumper/B1']('b')
        self.assertTrue(self.res.ready(self.now[0]))
        self.tf_ok[0] = False
        self.assertTrue(self.res.ready(self.now[0]))

    def test_stop_se_desuscribe_suelta_la_tf_y_cancela_el_timer(self):
        self.res.start()
        listener = Listener.instances[0]
        self.res.stop()
        self.assertFalse(self.res.running)
        self.assertEqual(self.node.subs, {})
        self.assertEqual(len(self.node.destroyed), 3)
        self.assertFalse(listener.registered)
        self.assertTrue(self.node.timers[0].canceled)
        self.res.stop()                                        # idempotente
        self.assertEqual(len(self.node.destroyed), 3)

    def test_al_reactivar_vuelve_a_esperar_datos_nuevos(self):
        self.res.start()
        self.node.subs['sonar/left']('s')
        self.node.subs['bumper/B1']('b')
        self.assertTrue(self.res.ready(self.now[0]))
        self.res.stop()
        self.now[0] += 10.0
        self.res.start()
        self.assertFalse(self.res.ready(self.now[0]))          # lo recibido antes no cuenta


if __name__ == '__main__':
    unittest.main()
