"""Modelo YOLOv8-seg en NCNN (paquete pip `ncnn`) y camara con OpenCV. Se importa solo desde el hilo de captura del
nodo, para que el resto del paquete se pueda probar sin esas librerias.

No usa ultralytics ni torch: en la Pi pesan cientos de MB, no estan en el python de ROS y usan todos los nucleos.
"""
import os
import time
from typing import Dict, List, Tuple

from lawnbot_vision.vision_logic import Detection


def load_metadata(model_dir: str) -> Tuple[Dict[int, str], int]:
    """(clases, tamano de entrada) desde metadata.yaml del export de ultralytics."""
    import yaml
    path = os.path.join(model_dir, 'metadata.yaml')
    with open(path) as f:
        meta = yaml.safe_load(f)
    names = {int(k): str(v) for k, v in meta['names'].items()}
    imgsz = meta.get('imgsz', [0])
    if meta.get('task') not in (None, 'segment'):
        raise ValueError(f'{path}: task={meta.get("task")!r}, se necesita un modelo de segmentacion')
    return names, int(imgsz[0] if isinstance(imgsz, (list, tuple)) else imgsz)


class NcnnSegDetector:
    def __init__(self, model_dir: str, imgsz: int, conf: float, iou: float = 0.7, threads: int = 1,
                 grass_class: str = 'pasto'):
        import ncnn
        import numpy as np
        from lawnbot_vision import seg_postprocess as sp
        self.np, self.sp, self.ncnn = np, sp, ncnn
        model_dir = os.path.expanduser(model_dir)
        param, binf = os.path.join(model_dir, 'model.ncnn.param'), os.path.join(model_dir, 'model.ncnn.bin')
        for p in (param, binf):
            if not os.path.isfile(p):
                raise FileNotFoundError(f'no existe {p}')
        self.names, meta_imgsz = load_metadata(model_dir)
        if meta_imgsz and imgsz != meta_imgsz:
            raise ValueError(f'imgsz={imgsz} pero el modelo se exporto a {meta_imgsz} (metadata.yaml)')
        grass = [k for k, v in self.names.items() if v == grass_class]
        if not grass:
            raise ValueError(f'el modelo no tiene la clase {grass_class!r}: {self.names}')
        self.grass_cls = grass[0]
        self.imgsz, self.conf, self.iou = imgsz, conf, iou

        self.net = ncnn.Net()
        self.net.opt.num_threads = max(1, int(threads))
        self.net.opt.use_vulkan_compute = False
        if self.net.load_param(param) != 0 or self.net.load_model(binf) != 0:
            raise RuntimeError(f'ncnn no pudo cargar el modelo de {model_dir}')
        self.last_infer_s = 0.0

    def detect(self, frame) -> Tuple[float, List[Detection], int]:
        """(fraccion de pasto, obstaculos candidatos con su punto base, ancho de la imagen)."""
        np, sp = self.np, self.sp
        t0 = time.monotonic()
        blob, lb = sp.letterbox(frame, self.imgsz)
        ex = self.net.create_extractor()
        ex.input('in0', self.ncnn.Mat(blob))
        r0, out0 = ex.extract('out0')
        r1, out1 = ex.extract('out1')
        if r0 != 0 or r1 != 0:
            raise RuntimeError(f'ncnn extract fallo ({r0}, {r1})')
        dets = sp.decode(np.array(out0), np.array(out1), len(self.names), self.conf, self.iou)
        grass = sp.grass_fraction(dets, self.grass_cls, lb)
        obstacles = []
        for d in dets:
            if d.cls == self.grass_cls:
                continue
            bp = sp.base_point(d.mask, lb)
            if bp is not None:
                obstacles.append(Detection(self.names[d.cls], bp[0], bp[1]))
        self.last_infer_s = time.monotonic() - t0
        return grass, obstacles, lb.orig_w


class Camera:
    """Camara (indice, /dev/videoN o ruta de video) con cv2.VideoCapture.

    Entre cuadro y cuadro pasa ~1 s: el driver guarda cuadros viejos en su cola. Con un solo buffer y un grab() de
    descarte antes del bueno, la imagen procesada es de ese momento y no de hace segundos."""

    def __init__(self, source: str, width: int = 640, height: int = 480, fps: float = 15.0, fourcc: str = 'MJPG'):
        import cv2
        self.cv2 = cv2
        src = int(source) if str(source).isdigit() else source
        self.is_file = not isinstance(src, int) and not str(src).startswith('/dev/')
        self.cap = cv2.VideoCapture(src, cv2.CAP_ANY if self.is_file else cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'no se pudo abrir la camara/video {source!r}')
        if not self.is_file:
            if fourcc:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def describe(self) -> str:
        c = int(self.cap.get(self.cv2.CAP_PROP_FOURCC))
        fourcc = ''.join(chr((c >> 8 * i) & 0xFF) for i in range(4)) if c else '?'
        return f'{self.width}x{self.height} {fourcc} {self.cap.get(self.cv2.CAP_PROP_FPS):.0f} fps'

    def read(self):
        if self.is_file:
            ok, frame = self.cap.read()
            if not ok:                                        # video: vuelve a empezar
                self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            return ok, frame
        self.cap.grab()                                       # descarta el cuadro que quedo en la cola
        return self.cap.read()

    def release(self):
        self.cap.release()
