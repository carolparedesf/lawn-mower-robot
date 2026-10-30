#!/usr/bin/env python3
"""Nodo de vision: publica /vision/grass_present (Bool) y /vision/obstacles (PoseArray en base_footprint).

Un hilo captura y segmenta (modelo YOLO ncnn); un timer publica el ultimo resultado a `publish_rate_hz`. Falla seguro:
sin camara, con el modelo caido o con un resultado de mas de `stale_s`, se publica grass_present=false y obstaculos vacios
(blade_controller ya apaga la cuchilla con el pasto en false). Los obstaculos solo salen con `calibrated: true`.

No publica velocidades ni mueve nada. No correr junto con vision_stub (publican los mismos topics).
"""
import threading
import time
from dataclasses import fields

from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool, String

from lawnbot_vision.vision_logic import VisionCore, VisionParams, validate_calibration


def node_utils_spin(make_node, args=None):
    """Arranque con apagado limpio (mismo patron que lawnbot_nav.node_utils)."""
    import signal

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    def _stop(signum, frame):
        raise KeyboardInterrupt

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _stop)
    node = None
    try:
        node = make_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.shutdown_cleanup()
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        d = self.declare_parameter
        d('model', 'ncnn_640')
        d('source', '0')                      # indice de camara o ruta de un video
        d('imgsz', 640)                       # debe coincidir con el export del modelo
        d('conf', 0.4)
        d('max_fps', 5.0)                     # tope de inferencias por segundo (la Pi 4 comparte CPU con ROS)
        d('publish_rate_hz', 5.0)
        d('stale_s', 1.0)                     # un resultado mas viejo que esto se publica como "sin pasto, sin obstaculos"
        d('frame_id', 'base_footprint')
        d('grass_class', 'pasto')
        d('use_test_alias', False)            # nombres de COCO -> clases del TP, solo para probar con yolov8n-seg
        d('start_capture', True)              # false: sin hilo de captura (pruebas)
        defaults = VisionParams()
        names = [f.name for f in fields(VisionParams)]
        for n in names:
            v = getattr(defaults, n)
            d(n, list(v) if isinstance(v, tuple) else v)

        gp = self.get_parameter
        params = VisionParams(**{n: (list(gp(n).value) if isinstance(getattr(defaults, n), tuple) else gp(n).value)
                                 for n in names})
        err = validate_calibration(params.calib_y, params.calib_d)
        if err:
            raise ValueError(f'calibracion invalida: {err}')
        self.core = VisionCore(params)
        self.frame_id = gp('frame_id').value
        self.stale_s = gp('stale_s').value
        self.min_period = 1.0 / max(gp('max_fps').value, 0.1)
        self.cfg = {k: gp(k).value for k in ('model', 'source', 'imgsz', 'conf', 'grass_class', 'use_test_alias')}

        self.lock = threading.Lock()
        self.latest = None                    # (VisionResult, instante)
        self.last_status = None
        self.last_status_pub = 0.0
        self.stop = threading.Event()
        self.thread = None

        self.grass_pub = self.create_publisher(Bool, 'vision/grass_present', 10)
        self.obstacles_pub = self.create_publisher(PoseArray, 'vision/obstacles', 10)
        self.status_pub = self.create_publisher(String, 'vision/status', 10)
        self.create_timer(1.0 / gp('publish_rate_hz').value, self.tick)

        if not params.calibrated:
            self.get_logger().warn(
                'calibrated=false: /vision/obstacles sale VACIO (las distancias no son confiables). Correr calibrar.py, '
                'cargar calib_y/calib_d y camara en el YAML y poner calibrated: true')
        if gp('start_capture').value:
            self.thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.thread.start()
        self.get_logger().info(f'vision_node listo (modelo {self.cfg["model"]}, fuente {self.cfg["source"]})')

    # ── Resultado de cada cuadro ─────────────────────────────

    def on_detection(self, grass_fraction, detections, width):
        result = self.core.process(grass_fraction, detections, width)
        with self.lock:
            self.latest = (result, time.monotonic())

    def on_failure(self):
        """Camara o modelo con problemas: se descarta el resultado y la historia (no se arrastra un 'hay pasto' viejo)."""
        self.core.reset()
        with self.lock:
            self.latest = None

    # ── Hilo de captura ──────────────────────────────────────

    def capture_loop(self):
        try:
            from lawnbot_vision.detector import Camera, YoloDetector
            detector = YoloDetector(self.cfg['model'], self.cfg['imgsz'], self.cfg['conf'], self.cfg['grass_class'],
                                    self.cfg['use_test_alias'])
            self.get_logger().info(f'clases del modelo: {detector.names}')
            camera = Camera(self.cfg['source'])
        except Exception as exc:
            self.get_logger().error(f'vision no arranca: {exc}. Se publica grass_present=false y sin obstaculos')
            return
        last = 0.0
        failures = 0
        while not self.stop.is_set():
            wait = self.min_period - (time.monotonic() - last)
            if wait > 0:
                time.sleep(wait)
            last = time.monotonic()
            try:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError('la camara no entrego imagen')
                grass, dets, width = detector.detect(frame)
                self.on_detection(grass, dets, width)
                failures = 0
            except Exception as exc:
                failures += 1
                self.on_failure()
                if failures in (1, 30) or failures % 300 == 0:
                    self.get_logger().error(f'vision: {exc} ({failures} fallos seguidos)')
                time.sleep(0.1)
        camera.release()

    # ── Publicacion ──────────────────────────────────────────

    def tick(self):
        now = time.monotonic()
        with self.lock:
            latest = self.latest
        fresh = latest is not None and now - latest[1] <= self.stale_s
        grass = bool(fresh and latest[0].grass)
        obstacles = latest[0].obstacles if fresh else []
        self.publish(grass, obstacles)
        status = 'ok' if fresh else 'sin_datos'
        if fresh:
            status += f' pasto={latest[0].grass_fraction:.2f} obstaculos={len(obstacles)}'
            if latest[0].skipped_far:
                status += f' fuera_de_rango={latest[0].skipped_far}'
            if not self.core.p.calibrated:
                status += ' SIN_CALIBRAR'
        if status != self.last_status or now - self.last_status_pub >= 5.0:
            self.last_status, self.last_status_pub = status, now
            self.status_pub.publish(String(data=status))

    def publish(self, grass, obstacles):
        self.grass_pub.publish(Bool(data=bool(grass)))
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in obstacles:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(x), float(y), 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.obstacles_pub.publish(msg)

    def shutdown_cleanup(self):
        """Sin vision no hay pasto ni obstaculos: la cuchilla se apaga."""
        self.stop.set()
        for _ in range(3):
            try:
                self.publish(False, [])
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    node_utils_spin(VisionNode, args)


if __name__ == '__main__':
    main()
