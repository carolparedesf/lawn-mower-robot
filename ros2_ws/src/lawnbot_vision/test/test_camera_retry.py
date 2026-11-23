"""Camara ausente o desconectada: error una sola vez por caida, status sin_camara, grass_present=false y reintento
hasta que vuelve (con una camara y un detector falsos, el nodo real sobre rclpy)."""
import threading
import time
import unittest

try:
    import rclpy
    from rclpy.parameter import Parameter
    from lawnbot_vision.vision_node import VisionNode
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


class World:
    present = False
    opens = 0


class FakeCamera:
    def __init__(self, source, *args):
        World.opens += 1
        if not World.present:
            raise RuntimeError(f'no existe {source}')

    def describe(self):
        return 'falsa'

    def read(self):
        return (True, 'frame') if World.present else (False, None)

    def release(self):
        pass


class FakeDetector:
    last_infer_s = 0.0

    def detect(self, frame):
        return 0.9, [], 640                      # mucho pasto


class Logger:
    def __init__(self):
        self.errors, self.infos = [], []

    def error(self, m, **kw):
        self.errors.append(m)

    def info(self, m, **kw):
        self.infos.append(m)

    def warn(self, m, **kw):
        pass

    warning = warn


@unittest.skipUnless(HAVE_ROS, 'requiere rclpy')
class CameraRetry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        World.present, World.opens = False, 0
        overrides = [Parameter('start_capture', value=False), Parameter('process_rate_hz', value=2.0),
                     Parameter('grass_window', value=1), Parameter('camera_retry_s', value=0.5)]
        self.node = VisionNode(parameter_overrides=overrides)
        self.log = Logger()
        self.node.get_logger = lambda: self.log
        self.status, self.grass = [], []
        self.node.status_pub.publish = lambda m: self.status.append(m.data)
        self.node.grass_pub.publish = lambda m: self.grass.append(m.data)
        self.node.obstacles_pub.publish = lambda m: None
        self.thread = threading.Thread(target=self.node.run_capture, daemon=True,
                                       args=(self.node.cfg, FakeDetector(), FakeCamera))

    def tearDown(self):
        self.node.stop.set()
        if self.thread.is_alive():
            self.thread.join(2.0)
        self.node.destroy_node()

    def tick(self):
        self.node.last_status = None                  # que publique el status en cada tick
        self.node.tick()
        return self.status[-1], self.grass[-1]

    def test_sin_camara_al_arrancar_y_vuelve_sola(self):
        self.thread.start()
        time.sleep(1.3)                               # ~3 intentos
        self.assertGreaterEqual(World.opens, 2)
        self.assertEqual(len(self.log.errors), 1)     # un solo aviso aunque reintente
        self.assertIn('no disponible', self.log.errors[0])
        self.assertEqual(self.tick(), ('sin_camara', False))

        World.present = True
        time.sleep(1.0)
        status, grass = self.tick()
        self.assertTrue(status.startswith('ok'), status)
        self.assertTrue(grass)
        self.assertTrue(any('reconectada' in m for m in self.log.infos))

    def test_desconexion_en_marcha(self):
        World.present = True
        self.thread.start()
        time.sleep(0.8)
        self.assertTrue(self.tick()[0].startswith('ok'))
        self.assertEqual(self.log.errors, [])

        World.present = False                         # se desenchufa
        time.sleep(1.5)
        self.assertEqual(self.tick(), ('sin_camara', False))   # sin esperar a que venza stale_s
        self.assertEqual(len(self.log.errors), 1)
        self.assertIn('desconectada', self.log.errors[0])

        World.present = True
        time.sleep(1.0)
        self.assertTrue(self.tick()[0].startswith('ok'))
        self.assertEqual(len(self.log.errors), 1)


if __name__ == '__main__':
    unittest.main()
