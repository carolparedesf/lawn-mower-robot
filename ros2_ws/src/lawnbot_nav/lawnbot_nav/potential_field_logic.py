"""Evasion de obstaculos por campo de potencial (sin dependencias de ROS).

No es un planificador: toma la orden del algoritmo activo (v, w nominal) y la corrige con una repulsion de los
obstaculos cercanos. Reglas de seguridad:
  - nunca crea movimiento: con orden nominal (0, 0) devuelve (0, 0);
  - nunca aumenta la velocidad lineal (v_out <= v_nom) ni retrocede por su cuenta (marcha atras: se deja pasar igual);
  - el giro agregado se limita a `max_turn` (o al nominal si era mayor).
Detecta el minimo local por falta de progreso: obstaculos cerca y el robot no avanza `stall_min_progress` en
`stall_window_s`. Entonces avisa y CEDE el control `yield_s` (deja de publicar; manda el algoritmo de abajo).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

Point = Tuple[float, float]

# Caja del cuerpo respecto de base_footprint (del URDF), para medir distancias al borde y no al eje
BODY_X = (-0.2463, 0.2137)
BODY_Y = (-0.2127, 0.2113)

ALERT_LOCAL_MINIMUM = 'local_minimum'


@dataclass
class PFParams:
    influence_distance: float = 0.60     # m desde el borde del cuerpo: mas lejos no repele (el gate frena a 0,30 / 0,15)
    min_distance: float = 0.15           # m: a esta distancia la repulsion es maxima
    slow_gain: float = 1.0               # cuanto frena de frente (1 = frena del todo con repulsion maxima de frente)
    turn_gain: float = 1.0               # rad/s de giro por unidad de repulsion lateral
    max_turn: float = 0.6                # rad/s: tope del giro agregado
    tie_turn: float = 1.0                # +1 gira a la izquierda si el obstaculo esta justo de frente; -1 a la derecha
    tie_band: float = 0.08               # |repulsion lateral| menor que esto cuenta como "justo de frente"
    min_command: float = 0.02            # m/s: nominal con |v| menor que esto no cuenta como "queriendo avanzar"
    stall_window_s: float = 6.0
    stall_min_progress: float = 0.20     # m recorridos en la ventana para no considerarse trabado
    yield_s: float = 4.0                 # s sin publicar tras detectar el minimo local


def box_distance(p: Point) -> float:
    """Distancia de un punto (base_footprint) al borde del cuerpo; 0 si esta dentro."""
    dx = max(BODY_X[0] - p[0], 0.0, p[0] - BODY_X[1])
    dy = max(BODY_Y[0] - p[1], 0.0, p[1] - BODY_Y[1])
    return math.hypot(dx, dy)


def sonar_point(mount: Tuple[float, float, float], rng: float) -> Point:
    """Punto detectado por un sonar montado en (x, y, yaw) a distancia rng."""
    x, y, yaw = mount
    return x + rng * math.cos(yaw), y + rng * math.sin(yaw)


def repulsion(obstacles: List[Point], p: PFParams) -> Tuple[float, float, float]:
    """(rx, ry, cercano): repulsion total en el cuerpo (norma <= 1) y la distancia al obstaculo mas cercano."""
    rx = ry = 0.0
    nearest = math.inf
    span = max(p.influence_distance - p.min_distance, 1e-6)
    for ox, oy in obstacles:
        d = box_distance((ox, oy))
        nearest = min(nearest, d)
        if d >= p.influence_distance:
            continue
        w = min(1.0, (p.influence_distance - d) / span)
        n = math.hypot(ox, oy)
        if n < 1e-6:
            continue
        rx -= w * ox / n
        ry -= w * oy / n
    norm = math.hypot(rx, ry)
    if norm > 1.0:
        rx, ry = rx / norm, ry / norm
    return rx, ry, nearest


class PFCore:
    def __init__(self, params: Optional[PFParams] = None):
        self.p = params or PFParams()
        self.reset()

    def reset(self):
        self.anchor = None            # (instante, x, y) al comenzar a estar influenciado
        self.yield_until = -math.inf
        self.alert: Optional[str] = None
        self.influenced = False

    def update(self, now: float, nominal: Optional[Tuple[float, float]], obstacles: List[Point],
               pose: Optional[Point]) -> Optional[Tuple[float, float]]:
        """Orden corregida (v, w), o None si no hay que publicar (sin nominal o cediendo el control)."""
        p = self.p
        if now < self.yield_until:
            self.influenced = False
            return None
        if self.alert and now >= self.yield_until:
            self.alert = None
        if nominal is None:
            self.anchor, self.influenced = None, False
            return None
        v, w = nominal
        rx, ry, nearest = repulsion(obstacles, p)
        self.influenced = nearest < p.influence_distance

        self._detect_stall(now, v, pose)
        if self.alert:                       # recien detectado: ceder desde ya
            return None

        if abs(v) < 1e-9 and abs(w) < 1e-9:
            return 0.0, 0.0
        v_out = v
        if v > 0.0:
            v_out = v * max(0.0, 1.0 + p.slow_gain * min(0.0, rx))
        lateral = ry
        if self.influenced and rx < -0.2 and abs(ry) < p.tie_band:
            lateral = math.copysign(abs(rx) * 0.5, p.tie_turn)
        w_out = w + p.turn_gain * lateral
        limit = max(abs(w), p.max_turn)
        return v_out, max(-limit, min(limit, w_out))

    def _detect_stall(self, now, v, pose):
        p = self.p
        wants_forward = v >= p.min_command
        if not (self.influenced and wants_forward and pose is not None):
            self.anchor = None
            return
        if self.anchor is None:
            self.anchor = (now, pose[0], pose[1])
            return
        t0, x0, y0 = self.anchor
        if math.hypot(pose[0] - x0, pose[1] - y0) >= p.stall_min_progress:
            self.anchor = (now, pose[0], pose[1])      # hubo progreso: nueva ventana
        elif now - t0 >= p.stall_window_s:
            self.alert = ALERT_LOCAL_MINIMUM
            self.yield_until = now + p.yield_s
            self.anchor = None
