"""Pre y post proceso de YOLOv8-seg exportado a NCNN, con numpy y OpenCV (sin ultralytics ni torch).

Hace lo mismo que ultralytics para un modelo de tamano fijo (imgsz x imgsz):
  - letterbox: escala manteniendo la proporcion y rellena con gris 114, centrado; RGB, 0..1, CHW
  - salida 0 (4 + clases + 32, N): caja cx, cy, w, h en pixeles de la entrada, puntaje por clase (ya con sigmoide)
    y 32 coeficientes de mascara; NMS por clase
  - salida 1 (32, imgsz/4, imgsz/4): prototipos; mascara = coeficientes @ prototipos, se lleva al tamano de la
    entrada (bilineal), se recorta a la caja y se umbraliza en 0 (= sigmoide > 0,5)
De cada mascara se saca lo que usa la vision: la fraccion de pasto (union de las mascaras de pasto sobre el area
de la imagen) y el punto de apoyo de los obstaculos (fila mas baja y promedio de las columnas a 2 px de ella),
ambos en pixeles de la imagen original.
"""
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class Letterbox:
    gain: float          # pixeles de la entrada por pixel de la imagen original
    left: int
    top: int
    new_w: int           # tamano de la imagen escalada dentro de la entrada
    new_h: int
    orig_w: int
    orig_h: int


@dataclass
class SegDetection:
    cls: int
    score: float
    box: Tuple[float, float, float, float]    # x1, y1, x2, y2 en pixeles de la entrada
    mask: np.ndarray                          # bool (imgsz, imgsz)


def letterbox(frame: np.ndarray, size: int) -> Tuple[np.ndarray, Letterbox]:
    """BGR uint8 (h, w, 3) -> entrada float32 (3, size, size) RGB en 0..1, y la transformacion usada."""
    h, w = frame.shape[:2]
    gain = min(size / h, size / w)
    new_w, new_h = int(round(w * gain)), int(round(h * gain))
    dw, dh = (size - new_w) / 2, (size - new_h) / 2
    top, left = int(round(dh - 0.1)), int(round(dw - 0.1))
    img = frame if (new_w, new_h) == (w, h) else cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    img = cv2.copyMakeBorder(img, top, size - new_h - top, left, size - new_w - left,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    blob = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32) / 255.0
    return blob, Letterbox(gain, left, top, new_w, new_h, w, h)


def nms_per_class(xywh: np.ndarray, scores: np.ndarray, cls: np.ndarray, conf: float, iou: float):
    """NMS separado por clase. OpenCV < 4.7 (el de Ubuntu 22.04 en la Pi es 4.5.4) no tiene NMSBoxesBatched: se
    desplazan las cajas de cada clase para que nunca se superpongan con las de otra (como hace ultralytics)."""
    if hasattr(cv2.dnn, 'NMSBoxesBatched'):
        return cv2.dnn.NMSBoxesBatched(xywh.tolist(), scores.tolist(), cls.tolist(), conf, iou)
    shifted = xywh.copy()
    offset = cls.astype(np.float32) * 4096.0
    shifted[:, 0] += offset
    shifted[:, 1] += offset
    return cv2.dnn.NMSBoxes(shifted.tolist(), scores.tolist(), conf, iou)


def decode(out0: np.ndarray, protos: np.ndarray, num_classes: int, conf: float, iou: float = 0.7,
           max_det: int = 100) -> List[SegDetection]:
    """Detecciones con su mascara a partir de las dos salidas del modelo."""
    nm = protos.shape[0]
    if out0.shape[0] != 4 + num_classes + nm:
        raise ValueError(f'salida 0 de forma {out0.shape}: se esperaban {4 + num_classes + nm} filas '
                         f'({num_classes} clases, {nm} coeficientes de mascara)')
    pred = out0.T                                            # (N, 4 + nc + nm)
    scores_all = pred[:, 4:4 + num_classes]
    cls = scores_all.argmax(axis=1)
    scores = scores_all[np.arange(len(cls)), cls]
    keep = scores > conf
    if not keep.any():
        return []
    pred, cls, scores = pred[keep], cls[keep], scores[keep]
    cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
    xywh = np.stack([cx - bw / 2, cy - bh / 2, bw, bh], axis=1)
    idx = np.array(nms_per_class(xywh, scores, cls, conf, iou), dtype=int).reshape(-1)
    idx = idx[np.argsort(-scores[idx])][:max_det]
    if len(idx) == 0:
        return []

    size = protos.shape[1] * 4                               # la entrada es 4 veces los prototipos
    ph, pw = protos.shape[1:]
    logits = (pred[idx, 4 + num_classes:] @ protos.reshape(nm, -1)).reshape(-1, ph, pw)
    dets = []
    for k, i in enumerate(idx):
        x1, y1, w, h = xywh[i]
        box = (max(0.0, x1), max(0.0, y1), min(float(size), x1 + w), min(float(size), y1 + h))
        up = cv2.resize(logits[k], (size, size), interpolation=cv2.INTER_LINEAR)
        mask = np.zeros((size, size), dtype=bool)
        bx1, by1 = int(np.floor(box[0])), int(np.floor(box[1]))
        bx2, by2 = int(np.ceil(box[2])), int(np.ceil(box[3]))
        # recorte como ultralytics: se conserva el pixel si su indice cae en [x1, x2) e [y1, y2)
        xs = np.arange(bx1, bx2)
        ys = np.arange(by1, by2)
        sub = up[by1:by2, bx1:bx2] > 0.0
        sub &= (xs >= box[0])[None, :] & (xs < box[2])[None, :]
        sub &= (ys >= box[1])[:, None] & (ys < box[3])[:, None]
        mask[by1:by2, bx1:bx2] = sub
        dets.append(SegDetection(int(cls[i]), float(scores[i]), box, mask))
    return dets


def content_region(lb: Letterbox) -> Tuple[slice, slice]:
    """Filas y columnas de la entrada que son imagen (no relleno)."""
    return slice(lb.top, lb.top + lb.new_h), slice(lb.left, lb.left + lb.new_w)


def grass_fraction(dets: Sequence[SegDetection], grass_cls: int, lb: Letterbox) -> float:
    """Fraccion de la imagen cubierta por pasto (union de las mascaras, sin contar el relleno)."""
    rows, cols = content_region(lb)
    union = None
    for d in dets:
        if d.cls == grass_cls:
            union = d.mask.copy() if union is None else (union | d.mask)
    if union is None:
        return 0.0
    return float(union[rows, cols].sum()) / float(lb.new_h * lb.new_w)


def base_point(mask: np.ndarray, lb: Letterbox):
    """Punto de apoyo (x, y) en pixeles de la imagen original, o None si la mascara esta vacia.

    Igual que los scripts: fila mas baja de la mascara y promedio de las columnas a 2 px (de la imagen original)
    de esa fila."""
    rows, cols = content_region(lb)
    m = mask[rows, cols]
    ys, xs = np.nonzero(m)
    if len(ys) == 0:
        return None
    ymax = ys.max()
    near = ys >= ymax - 2.0 * lb.gain
    # pixel de la entrada -> pixel de la imagen original (sin medio pixel: asi lo hace ultralytics con masks.xy,
    # y el calibrar.py viejo midio calib_y con eso)
    x = xs[near].mean() / lb.gain
    y = ymax / lb.gain
    return float(min(x, lb.orig_w - 1)), float(min(y, lb.orig_h - 1))
