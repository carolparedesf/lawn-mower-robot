"""Flujo del e-stop en el bridge (robot/cmd/estop, memoria del e-stop, robot/status.estop) sin nodo ROS ni broker:
se llaman los metodos reales de RobotBridge sobre un objeto de mentira. Necesita rclpy y paho (no corre sin ellos)."""
import importlib.util
import json
import os
import sys
import time
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    # Por ruta: "import bridge" podria resolver a la carpeta bridge/ si se corre desde la raiz del repo
    _spec = importlib.util.spec_from_file_location('bridge_node', os.path.join(HERE, 'bridge.py'))
    _module = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_module)
    RobotBridge = _module.RobotBridge
    from actuator_relay import ActuatorRelay
    from estop_latch import EstopLatch
    HAVE_ROS = True
except (ImportError, SystemExit):
    HAVE_ROS = False


class _Log:
    def info(self, *_):
        pass


def _msg(topic, payload):
    return SimpleNamespace(topic=topic, payload=json.dumps(payload).encode())


@unittest.skipUnless(HAVE_ROS, 'rclpy/paho no disponibles')
class EstopFlow(unittest.TestCase):
    def make(self, requested_mode='manual'):
        calls = []

        class Fake:
            on_message = RobotBridge.on_message
            release_estop = RobotBridge.release_estop
            on_robot_event = RobotBridge.on_robot_event
            on_estop = RobotBridge.on_estop
            publish_status = RobotBridge.publish_status
            effective_mode = RobotBridge.effective_mode
            actuator_mode_ok = RobotBridge.actuator_mode_ok
            motor_running = RobotBridge.motor_running
            send_stop = lambda s: calls.append('send_stop')
            publish_mode = lambda s: calls.append(('publish_mode', s.requested_mode))
            send_estop = lambda s, active: calls.append(('estop', active))
            send_twist_from_dir = lambda s, d, v: calls.append(('twist', d, v))
            get_logger = lambda s: _Log()

        fake = Fake()
        fake.requested_mode = requested_mode
        fake.estop_latch = EstopLatch()
        fake.event, fake.event_time = None, 0.0
        fake.alert = 'none'
        fake.active_mode, fake.active_mode_time = requested_mode, time.monotonic()
        fake.battery_pct, fake.battery_time = None, 0.0
        fake.blade_status, fake.blade_status_time = None, 0.0
        fake.relay = ActuatorRelay()
        fake.motor_cmd = {}
        published = []
        fake.mqtt = SimpleNamespace(publish=lambda topic, payload, **kw: published.append((topic, json.loads(payload))))
        fake.calls, fake.published = calls, published
        return fake

    def last_status(self, fake):
        return [p for t, p in fake.published if t == 'robot/status'][-1]

    def test_liberar_deja_el_modo_en_stop_y_publica_estop_false(self):
        fake = self.make('manual')
        fake.estop_latch.on_event('lifted')
        fake.on_message(None, None, _msg('robot/cmd/estop', {'active': False}))
        self.assertEqual(fake.requested_mode, 'stop')
        self.assertFalse(fake.estop_latch.active)
        # el modo (stop) se publica ANTES de /estop false
        self.assertEqual(fake.calls, ['send_stop', ('publish_mode', 'stop'), ('estop', False)])
        self.assertIs(self.last_status(fake)['estop'], False)

    def test_despues_de_liberar_el_joystick_no_mueve_el_robot(self):
        fake = self.make('manual')
        fake.on_message(None, None, _msg('robot/cmd/estop', {'active': False}))
        fake.calls.clear()
        fake.on_message(None, None, _msg('robot/cmd/move', {'dir': 'fwd', 'speed': 0.5}))
        self.assertEqual(fake.calls, [])          # el modo es stop, no manual

    def test_active_true_o_sin_clave_se_ignoran(self):
        for payload in ({'active': True}, {}, {'active': 'false'}):
            fake = self.make('manual')
            fake.estop_latch.on_event('lifted')
            fake.on_message(None, None, _msg('robot/cmd/estop', payload))
            self.assertEqual(fake.calls, [], payload)
            self.assertEqual(fake.requested_mode, 'manual')
            self.assertTrue(fake.estop_latch.active)

    def test_lifted_queda_en_status_aunque_el_evento_venza(self):
        fake = self.make()
        fake.on_robot_event(SimpleNamespace(data='lifted'))
        self.assertIs(self.last_status(fake)['estop'], True)
        fake.event_time -= 60.0                    # el evento ya vencio (5 s)
        fake.publish_status()
        status = self.last_status(fake)
        self.assertEqual(status['alert'], 'none')
        self.assertIs(status['estop'], True)       # pero el e-stop se sigue informando

    def test_estop_true_se_recuerda_y_false_lo_libera(self):
        fake = self.make()
        fake.on_estop(SimpleNamespace(data=True))
        self.assertIs(self.last_status(fake)['estop'], True)
        fake.on_estop(SimpleNamespace(data=False))
        self.assertIs(self.last_status(fake)['estop'], False)

    def test_al_liberar_el_lifted_viejo_no_queda_como_alerta(self):
        fake = self.make()
        fake.on_robot_event(SimpleNamespace(data='lifted'))
        fake.on_message(None, None, _msg('robot/cmd/estop', {'active': False}))
        status = self.last_status(fake)
        self.assertEqual(status['alert'], 'none')
        self.assertIs(status['estop'], False)

    def test_otros_eventos_no_activan_el_estop(self):
        fake = self.make()
        fake.on_robot_event(SimpleNamespace(data='low_battery'))
        self.assertIs(self.last_status(fake)['estop'], False)


if __name__ == '__main__':
    unittest.main()
