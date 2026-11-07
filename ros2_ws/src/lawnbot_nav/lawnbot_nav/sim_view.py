"""Geometria para ver la simulacion en RViz (sin ROS): contorno del cuerpo, segmentos de bumper y rastro del robot.

Lo usa fake_robot para /sim/markers y /sim/path. Solo simulacion.
"""
import math
from collections import deque
from typing import Dict, List, Tuple

from lawnbot_nav.sim_world import BODY_X, BODY_Y

Point = Tuple[float, float]


def footprint(corner_radius: float = 0.0, steps: int = 8) -> List[Point]:
    """Contorno cerrado del cuerpo en el marco del robot (sentido antihorario). Esquinas redondeadas de radio `corner_radius`."""
    x0, x1 = BODY_X
    y0, y1 = BODY_Y
    r = max(0.0, min(corner_radius, (x1 - x0) / 2.0, (y1 - y0) / 2.0))
    if r == 0.0:
        pts = [(x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    else:
        pts = []
        for cx, cy, a0 in ((x1 - r, y0 + r, -90.0), (x1 - r, y1 - r, 0.0), (x0 + r, y1 - r, 90.0), (x0 + r, y0 + r, 180.0)):
            for i in range(steps + 1):
                a = math.radians(a0 + 90.0 * i / steps)
                pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts + pts[:1]


def bumper_polylines(zones: Dict[str, List[Point]]) -> Dict[str, List[Point]]:
    """Cada zona de bumper (puntos sueltos del contorno) ordenada a lo largo del contorno, para dibujarla como un segmento.

    Una zona es un tramo continuo del contorno convexo, asi que basta ordenar por angulo alrededor del centro del cuerpo,
    medido desde la direccion media de la zona (para que la zona trasera, que cruza +-180 grados, no se corte).
    """
    cx, cy = (BODY_X[0] + BODY_X[1]) / 2.0, (BODY_Y[0] + BODY_Y[1]) / 2.0
    out = {}
    for name, pts in zones.items():
        if not pts:
            out[name] = []
            continue
        angles = [math.atan2(p[1] - cy, p[0] - cx) for p in pts]
        mean = math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))
        keyed = sorted(zip(((a - mean + math.pi) % (2 * math.pi) - math.pi for a in angles), pts))
        out[name] = [p for _, p in keyed]
    return out


class Trail:
    """Rastro de poses limitado a las ultimas `max_poses`; solo agrega una pose si se movio o roto lo suficiente."""

    def __init__(self, max_poses: int = 2000, min_step: float = 0.02, min_yaw: float = 0.087):
        self.poses = deque(maxlen=max(1, int(max_poses)))
        self.min_step, self.min_yaw = min_step, min_yaw

    def add(self, x: float, y: float, yaw: float) -> bool:
        if self.poses:
            lx, ly, lyaw = self.poses[-1]
            dyaw = abs((yaw - lyaw + math.pi) % (2 * math.pi) - math.pi)
            if math.hypot(x - lx, y - ly) < self.min_step and dyaw < self.min_yaw:
                return False
        self.poses.append((x, y, yaw))
        return True
