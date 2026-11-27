#!/usr/bin/env python3
"""Nodo de vision: publica /vision/grass_present (Bool) y /vision/obstacles (PoseArray en base_footprint).

Un hilo captura y segmenta (YOLOv8-seg en NCNN, 1 hilo) a `process_rate_hz` (1-2 Hz: la Pi 4 comparte CPU con ROS).
- /vision/grass_present: un timer lo publica a `publish_rate_hz` con el ultimo resultado, para que blade_controller
  (input_timeout 1 s) no lo de por faltante entre dos inferencias.
- /vision/obstacles: UNA vez por inferencia (coverage_map suma evidencia por mensaje: repetirlo la multiplicaria).
Falla seguro: sin camara, con el modelo caido o con un resultado de mas de `stale_s`, grass_present=false y obstaculos
vacios (blade_controller apaga la cuchilla con el pasto en false). Los obstaculos solo salen con `calibrated: true`.

No publica velocidades ni mueve nada. No correr junto con vision_stub (publican los mismos topics).
"""
import os

# Un hilo para numpy/BLAS y OpenCV: la inferencia ya es de 1 hilo y la Pi tiene la CPU justa (antes de importar numpy)
for _var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

import threading  # noqa: E402
import time  # noqa: E402
from dataclasses import fields  # noqa: E402

from geometry_msgs.msg import Pose, PoseArray  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from lawnbot_vision.vision_logic import VisionCore, VisionParams, validate_calibration  # noqa: E402

MAX_PROCESS_RATE_HZ = 2.0                     # tope de inferencias por segundo, aunque el YAML pida mas


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
    def __init__(self, **kwargs):
        super().__init__('vision_node', **kwargs)        # kwargs: parameter_overrides en los tests
        d = self.declare_parameter
        d('model', '')                        # carpeta del export NCNN (model.ncnn.param/.bin y metadata.yaml)
        d('source', '0')                      # indice de camara, /dev/videoN o ruta de un video
        d('imgsz', 320)                       # debe coincidir con el export del modelo (se verifica)
        d('conf', 0.4)
        d('iou', 0.7)                         # NMS
        d('threads', 1)                       # hilos de ncnn
        d('process_rate_hz', 1.0)             # inferencias por segundo (maximo MAX_PROCESS_RATE_HZ)
        d('publish_rate_hz', 2.0)             # grass_present con el ultimo resultado
        d('stale_s', 2.5)                     # resultado mas viejo que esto = "sin pasto, sin obstaculos"
        d('frame_id', 'base_footprint')
        d('grass_class', 'pasto')
        d('camera_width', 640)
        d('camera_height', 480)
        d('camera_fps', 15.0)
        d('camera_fourcc', 'MJPG')            # '' = el que elija el driver
        d('camera_retry_s', 2.5)              # sin camara: reintenta abrirla cada este tiempo
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
        rate = float(gp('process_rate_hz').value)
        if rate > MAX_PROCESS_RATE_HZ:
            self.get_logger().warn(f'process_rate_hz={rate} supera el tope: se usa {MAX_PROCESS_RATE_HZ}')
            rate = MAX_PROCESS_RATE_HZ
        self.min_period = 1.0 / max(rate, 0.05)
        if self.stale_s <= self.min_period:
            self.get_logger().warn(f'stale_s={self.stale_s} no supera el periodo de inferencia '
                                   f'({self.min_period:.2f} s): grass_present va a caer a false entre cuadros')
        self.cfg = {k: gp(k).value for k in ('model', 'source', 'imgsz', 'conf', 'iou', 'threads', 'grass_class',
                                             'camera_width', 'camera_height', 'camera_fps', 'camera_fourcc',
                                             'camera_retry_s')}
        self.stats = {'frames': 0, 'infer_s': 0.0, 'since': time.monotonic()}

        self.lock = threading.Lock()
        self.latest = None                    # (VisionResult, instante)
        self.camera_ok = None                 # None: todavia no se intento; False: no esta / se desconecto
        self._camera = None
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
                'calibrated=false: /vision/obstacles sale VACIO (las distancias no son confiables). Correr '
                '`ros2 run lawnbot_vision calibrar` y pegar en el YAML lo que imprime (calibrated: true, calib_y, calib_d)')
        if gp('start_capture').value:
            self.thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.thread.start()
        self.get_logger().info(f'vision_node listo (modelo {self.cfg["model"]}, fuente {self.cfg["source"]})')

    # ── Resultado de cada cuadro ─────────────────────────────

    def on_detection(self, grass_fraction, detections, width, stamp=None):
        result = self.core.process(grass_fraction, detections, width)
        with self.lock:
            self.latest = (result, time.monotonic())
        self.publish_obstacles(result.obstacles, stamp)

    def on_failure(self):
        """Camara o modelo con problemas: se descarta el resultado y la historia (no se arrastra un 'hay pasto' viejo)."""
        self.core.reset()
        with self.lock:
            had = self.latest is not None
            self.latest = None
        if had:
            self.publish_obstacles([])

    # ── Hilo de captura ──────────────────────────────────────

    def capture_loop(self):
        c = self.cfg
        try:
            import cv2
            cv2.setNumThreads(1)
            from lawnbot_vision.detector import Camera, NcnnSegDetector
            if not c['model']:
                raise ValueError('falta el parametro model (carpeta del export NCNN)')
            detector = NcnnSegDetector(c['model'], c['imgsz'], c['conf'], c['iou'], c['threads'], c['grass_class'])
            self.get_logger().info(f'modelo {c["model"]} ({c["imgsz"]} px, {c["threads"]} hilo/s): {detector.names}')
        except Exception as exc:
            self.get_logger().error(f'vision no arranca: {exc}. Se publica grass_present=false y sin obstaculos')
            return
        try:
            self.run_capture(c, detector, Camera)
        finally:
            if self._camera is not None:
                self._camera.release()

    def run_capture(self, c, detector, camera_cls):
        """Lazo de captura. Sin camara (no esta, no abre o se desconecto): error en el log UNA vez por caida,
        status sin_camara, grass_present=false y reintento cada camera_retry_s hasta que vuelva."""
        retry = max(0.5, float(c['camera_retry_s']))
        self._camera = None
        next_try = 0.0
        down = False                          # ya se aviso la caida actual
        last = 0.0
        failures = 0
        while not self.stop.is_set():
            if self._camera is None:
                wait = next_try - time.monotonic()
                if wait > 0:
                    self.stop.wait(min(wait, 0.2))
                    continue
                try:
                    self._camera = camera_cls(c['source'], c['camera_width'], c['camera_height'], c['camera_fps'],
                                              c['camera_fourcc'])
                except Exception as exc:
                    self.camera_down(f'camara {c["source"]} no disponible: {exc}', not down, retry)
                    down = True
                    next_try = time.monotonic() + retry
                    continue
                self.camera_ok = True
                self.get_logger().info(f'camara {"reconectada" if down else "abierta"}: {c["source"]} '
                                       f'{self._camera.describe()}')
                down = False
                last = 0.0

            wait = self.min_period - (time.monotonic() - last)
            if wait > 0 and self.stop.wait(wait):
                break
            last = time.monotonic()
            ok, frame = self._camera.read()
            if not ok:
                self._camera.release()
                self._camera = None
                self.camera_down(f'camara {c["source"]} desconectada o sin imagen', not down, retry)
                down = True
                next_try = time.monotonic() + retry
                continue
            try:
                stamp = self.get_clock().now().to_msg()
                grass, dets, width = detector.detect(frame)
                self.on_detection(grass, dets, width, stamp)
                self.stats['frames'] += 1
                self.stats['infer_s'] += detector.last_infer_s
                failures = 0
            except Exception as exc:
                failures += 1
                self.on_failure()
                if failures in (1, 30) or failures % 300 == 0:
                    self.get_logger().error(f'vision: {exc} ({failures} fallos seguidos)')
                self.stop.wait(0.1)

    def camera_down(self, msg, log, retry):
        self.camera_ok = False
        self.on_failure()
        if log:
            self.get_logger().error(f'{msg}. grass_present=false y sin obstaculos; se reintenta cada {retry:.1f} s '
                                    '(no se vuelve a avisar hasta que vuelva)')

    # ── Publicacion ──────────────────────────────────────────

    def tick(self):
        now = time.monotonic()
        with self.lock:
            latest = self.latest
        fresh = latest is not None and now - latest[1] <= self.stale_s
        if latest is not None and not fresh:
            with self.lock:                   # vencio: se descarta y se avisa una vez con obstaculos vacios
                if self.latest is latest:
                    self.latest = None
            self.publish_obstacles([])
        no_camera = self.camera_ok is False
        if no_camera:
            fresh = False
        self.grass_pub.publish(Bool(data=bool(fresh and latest[0].grass)))
        status = 'ok' if fresh else ('sin_camara' if no_camera else 'sin_datos')
        if fresh:
            status += f' pasto={latest[0].grass_fraction:.2f} obstaculos={len(latest[0].obstacles)}'
            if latest[0].skipped_far:
                status += f' fuera_de_rango={latest[0].skipped_far}'
            if not self.core.p.calibrated:
                status += ' SIN_CALIBRAR'
        if status != self.last_status or now - self.last_status_pub >= 5.0:
            self.last_status, self.last_status_pub = status, now
            self.status_pub.publish(String(data=status))
        st = self.stats
        if st['frames'] and now - st['since'] >= 60.0:
            self.get_logger().info(f'{st["frames"] / (now - st["since"]):.2f} inferencias/s, '
                                   f'{1000.0 * st["infer_s"] / st["frames"]:.0f} ms cada una')
            self.stats = {'frames': 0, 'infer_s': 0.0, 'since': now}

    def publish(self, grass, obstacles):
        self.grass_pub.publish(Bool(data=bool(grass)))
        self.publish_obstacles(obstacles)

    def publish_obstacles(self, obstacles, stamp=None):
        msg = PoseArray()
        msg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
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
        # Esperar al hilo de captura (suelta la camara): si Python termina con el hilo dentro de OpenCV/ncnn, el
        # proceso aborta ("terminate called without an active exception"). Una inferencia tarda < 0,3 s en la Pi.
        if self.thread is not None:
            self.thread.join(3.0)
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
