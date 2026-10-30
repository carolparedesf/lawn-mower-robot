"""Logica de vision sin dependencias (ni ROS, ni OpenCV, ni numpy): del resultado de la segmentacion a
"hay pasto" y a obstaculos en base_footprint.

Las formulas son las de los scripts de la Pi (vision/pi_scripts/vision_core.py y potential_field.py): distancia por
la fila de pixel de la base del objeto (interpolacion), angulo por la columna y el campo de vision horizontal.
Cambia SOLO que la calibracion y el montaje de la camara son parametros y que se valida que tengan sentido.
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass
class VisionParams:
    min_grass: float = 0.10                  # fraccion de la imagen que debe ser pasto
    grass_window: int = 7                    # cuadros: "hay pasto" si la mayoria lo vio
    fov_h_deg: float = 62.0                  # campo de vision horizontal [grados] (SUPUESTO: medir)
    calib_y: Sequence[float] = (300.0, 420.0)    # fila de la base del objeto [px], de lejos a cerca (crece)
    calib_d: Sequence[float] = (1.5, 0.5)        # distancia real [m] para esas filas (decrece)
    cam_x: float = 0.0                       # posicion de la camara en base_footprint [m]
    cam_y: float = 0.0
    cam_yaw_deg: float = 0.0                 # giro de la camara respecto al frente del robot
    obstacle_classes: Sequence[str] = ('arbol', 'planta', 'barrera', 'otro')
    min_rho: float = 0.05                    # m: mas cerca que esto no es una medicion creible
    calibrated: bool = False                 # las distancias solo se publican con la calibracion hecha


def validate_calibration(calib_y: Sequence[float], calib_d: Sequence[float]) -> Optional[str]:
    """Mensaje de error si la tabla no sirve, None si esta bien."""
    if len(calib_y) != len(calib_d) or len(calib_y) < 2:
        return 'calib_y y calib_d deben tener la misma cantidad de puntos (al menos 2)'
    if any(b <= a for a, b in zip(calib_y, calib_y[1:])):
        return 'calib_y debe crecer estrictamente (de lejos a cerca)'
    if any(b >= a for a, b in zip(calib_d, calib_d[1:])):
        return 'calib_d debe decrecer estrictamente (mas abajo en la imagen = mas cerca)'
    if any(d <= 0 for d in calib_d):
        return 'calib_d debe ser positiva'
    return None


def distance_from_y(y_base: float, calib_y: Sequence[float], calib_d: Sequence[float]) -> Optional[float]:
    """Distancia [m] segun la fila de la base del objeto. None si esta mas lejos que el punto mas lejano calibrado
    (no se sabe cuanto); si esta mas cerca que el mas cercano, se usa la distancia mas cercana (conservador)."""
    if y_base < calib_y[0]:
        return None
    if y_base >= calib_y[-1]:
        return float(calib_d[-1])
    for (y0, y1), (d0, d1) in zip(zip(calib_y, calib_y[1:]), zip(calib_d, calib_d[1:])):
        if y0 <= y_base <= y1:
            return float(d0 + (d1 - d0) * (y_base - y0) / (y1 - y0))
    return None


def bearing_from_x(x_base: float, width: float, fov_h_deg: float) -> float:
    """Angulo [rad] respecto al eje de la camara. Positivo = a la derecha."""
    return math.radians((x_base / width - 0.5) * fov_h_deg)


def to_base_footprint(theta: float, rho: float, cam_x: float, cam_y: float, cam_yaw: float) -> Point:
    """(x adelante, y izquierda) en base_footprint de un obstaculo visto a (theta derecha+, rho)."""
    x_c, y_c = rho * math.cos(theta), -rho * math.sin(theta)       # frame de la camara: x adelante, y izquierda
    c, s = math.cos(cam_yaw), math.sin(cam_yaw)
    return cam_x + c * x_c - s * y_c, cam_y + s * x_c + c * y_c


def base_point(points: Sequence[Point]) -> Tuple[int, int]:
    """Punto de apoyo de una mascara: la fila mas baja y el promedio de las columnas a 2 px de ella."""
    ymax = max(p[1] for p in points)
    xs = [p[0] for p in points if p[1] >= ymax - 2]
    return int(sum(xs) / len(xs)), int(ymax)


@dataclass
class Detection:
    name: str
    base_x: float
    base_y: float


@dataclass
class VisionResult:
    grass: bool
    grass_fraction: float
    obstacles: List[Point] = field(default_factory=list)
    skipped_far: int = 0                    # obstaculos mas lejos que el rango calibrado


class VisionCore:
    def __init__(self, params: Optional[VisionParams] = None):
        self.p = params or VisionParams()
        err = validate_calibration(self.p.calib_y, self.p.calib_d)
        if err:
            raise ValueError(err)
        self.hist = deque(maxlen=max(1, self.p.grass_window))

    def process(self, grass_fraction: float, detections: Sequence[Detection], width: float) -> VisionResult:
        """grass_fraction: area de pasto / area de la imagen. detections: obstaculos candidatos ya con su punto base."""
        p = self.p
        self.hist.append(grass_fraction >= p.min_grass)
        grass = sum(self.hist) > len(self.hist) / 2
        result = VisionResult(grass=grass, grass_fraction=grass_fraction)
        if not p.calibrated:
            return result                    # sin calibrar las distancias no valen: no se publican obstaculos
        yaw = math.radians(p.cam_yaw_deg)
        for d in detections:
            if d.name not in p.obstacle_classes:
                continue
            rho = distance_from_y(d.base_y, p.calib_y, p.calib_d)
            if rho is None:
                result.skipped_far += 1
                continue
            rho = max(rho, p.min_rho)
            theta = bearing_from_x(d.base_x, width, p.fov_h_deg)
            result.obstacles.append(to_base_footprint(theta, rho, p.cam_x, p.cam_y, yaw))
        return result

    def reset(self):
        self.hist.clear()
