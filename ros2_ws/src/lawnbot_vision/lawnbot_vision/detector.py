"""Envoltorio de YOLO (ultralytics) + OpenCV. Se importa solo desde el nodo, en el hilo de captura, para que el resto
del paquete se pueda probar sin esas librerias."""
from typing import List, Tuple

from lawnbot_vision.vision_logic import Detection, base_point

# Nombres de clases de COCO que se usan para probar con el modelo generico yolov8n-seg (igual que vision_core.py)
TEST_ALIAS = {'potted plant': 'planta', 'chair': 'barrera', 'person': 'otro', 'couch': 'barrera'}


class YoloDetector:
    def __init__(self, model_path: str, imgsz: int, conf: float, grass_class: str = 'pasto', use_test_alias: bool = False):
        import cv2
        import numpy as np
        from ultralytics import YOLO
        self.cv2, self.np = cv2, np
        self.model = YOLO(model_path, task='segment')
        self.imgsz, self.conf = imgsz, conf
        self.grass_class = grass_class
        self.alias = TEST_ALIAS if use_test_alias else {}
        self.names = self.model.names

    def detect(self, frame) -> Tuple[float, List[Detection], int]:
        """(fraccion de pasto, obstaculos candidatos con su punto base, ancho de la imagen)."""
        h, w = frame.shape[:2]
        r = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        grass_area = 0.0
        dets: List[Detection] = []
        if r.masks is not None:
            for i, pts in enumerate(r.masks.xy):
                if len(pts) < 3:
                    continue
                pts = pts.astype(self.np.int32)
                raw = self.names[int(r.boxes.cls[i])]
                name = self.alias.get(raw, raw)
                if name == self.grass_class:
                    grass_area += self.cv2.contourArea(pts.reshape(-1, 1, 2))
                else:
                    bx, by = base_point([(float(x), float(y)) for x, y in pts])
                    dets.append(Detection(name, bx, by))
        return grass_area / (w * h), dets, w


class Camera:
    """Camara (indice o ruta de video) con cv2.VideoCapture."""

    def __init__(self, source: str):
        import cv2
        src = int(source) if str(source).isdigit() else source
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f'no se pudo abrir la camara/video {source!r}')

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()
