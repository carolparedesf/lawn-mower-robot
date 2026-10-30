"""Grilla de cobertura: pasadas por celda y obstaculos con certeza por sonar y por camara (sin ROS).

Una *pasada* es una visita completa a la celda (se cuenta al SALIR de ella) durante la cual el robot estuvo
cortando. `update_pose` devuelve las pasadas ya completadas de la celda actual, sin contar la visita en curso.
"""
import base64
import math
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Cell = Tuple[int, int]

SONAR_HIT_GAIN = 0.4         # certeza que suma un eco: c += (1 - c) * gain
SONAR_FREE_DECAY = 0.15      # el espacio libre por delante de un eco resta: c *= (1 - decay)
CAMERA_HIT_GAIN = 0.5
SONAR_NO_ECHO_FRACTION = 0.98


class CoverageGrid:
    def __init__(self, x0: float, y0: float, width: int, height: int, resolution: float, hysteresis: float = 0.15):
        self.x0, self.y0 = x0, y0
        self.w, self.h = width, height
        self.res = resolution
        self.hysteresis = hysteresis            # fraccion de celda que hay que pasarse del borde para contar el cambio
        n = width * height
        self.visits = [0] * n
        self.sonar = [0.0] * n
        self.camera = [0.0] * n
        self.cur: Optional[Cell] = None
        self.cur_cut = False

    @staticmethod
    def from_polygon(poly: Sequence[Point], resolution: float, pad: float = 1.0, **kw) -> 'CoverageGrid':
        xs, ys = [p[0] for p in poly], [p[1] for p in poly]
        x0, y0 = min(xs) - pad, min(ys) - pad
        width = max(1, math.ceil((max(xs) + pad - x0) / resolution))
        height = max(1, math.ceil((max(ys) + pad - y0) / resolution))
        return CoverageGrid(x0, y0, width, height, resolution, **kw)

    # ── Indices ──────────────────────────────────────────────

    def cell_of(self, x: float, y: float) -> Optional[Cell]:
        i = math.floor((x - self.x0) / self.res)
        j = math.floor((y - self.y0) / self.res)
        return (i, j) if 0 <= i < self.w and 0 <= j < self.h else None

    def idx(self, cell: Cell) -> int:
        return cell[1] * self.w + cell[0]

    def center(self, cell: Cell) -> Point:
        return (self.x0 + (cell[0] + 0.5) * self.res, self.y0 + (cell[1] + 0.5) * self.res)

    # ── Pasadas ──────────────────────────────────────────────

    def update_pose(self, x: float, y: float, cutting: bool) -> Optional[int]:
        """Registra la pose. Devuelve las pasadas completadas de la celda actual, o None si esta fuera de la grilla."""
        cell = self.cell_of(x, y)
        if cell is None:
            self._finish_visit()
            return None
        if self.cur is None:
            self.cur, self.cur_cut = cell, cutting
        elif cell != self.cur:
            cx, cy = self.center(self.cur)
            # histeresis: solo se cuenta el cambio de celda si se paso del borde por mas de `hysteresis` celdas
            if max(abs(x - cx), abs(y - cy)) > self.res * (0.5 + self.hysteresis):
                self._finish_visit()
                self.cur, self.cur_cut = cell, cutting
        if self.cur is not None:
            self.cur_cut = self.cur_cut or cutting
        return self.visits[self.idx(self.cur)]

    def _finish_visit(self) -> None:
        if self.cur is not None and self.cur_cut:
            k = self.idx(self.cur)
            self.visits[k] = min(255, self.visits[k] + 1)
        self.cur, self.cur_cut = None, False

    # ── Obstaculos ───────────────────────────────────────────

    def add_sonar(self, sx: float, sy: float, yaw: float, rng: float, max_range: float) -> None:
        """Un rayo de sonar desde (sx, sy) con rumbo `yaw` y lectura `rng` (>= max_range*0.98 = sin eco)."""
        hit = rng < max_range * SONAR_NO_ECHO_FRACTION
        end = rng if hit else max_range * SONAR_NO_ECHO_FRACTION
        dx, dy = math.cos(yaw), math.sin(yaw)
        seen = set()
        d = 0.0
        step = self.res / 2.0
        while d < end:
            cell = self.cell_of(sx + dx * d, sy + dy * d)
            if cell is not None and cell not in seen:
                seen.add(cell)
                k = self.idx(cell)
                self.sonar[k] *= (1.0 - SONAR_FREE_DECAY)
            d += step
        if hit:
            cell = self.cell_of(sx + dx * end, sy + dy * end)
            if cell is not None:
                k = self.idx(cell)
                self.sonar[k] += (1.0 - self.sonar[k]) * SONAR_HIT_GAIN

    def add_camera(self, points: Iterable[Point]) -> None:
        for x, y in points:
            cell = self.cell_of(x, y)
            if cell is not None:
                k = self.idx(cell)
                self.camera[k] += (1.0 - self.camera[k]) * CAMERA_HIT_GAIN

    # ── Salida ───────────────────────────────────────────────

    def to_dict(self, robot_pose: Optional[Tuple[float, float, float]] = None, stamp: float = 0.0) -> dict:
        def pack(values, scale):
            return base64.b64encode(bytes(min(255, int(round(v * scale))) for v in values)).decode('ascii')
        return {
            'frame': 'map', 'resolution': self.res, 'origin': [self.x0, self.y0],
            'width': self.w, 'height': self.h, 'encoding': 'base64-u8-row-major',
            'passes': pack(self.visits, 1.0),            # pasadas completadas, 0..255
            'obstacles_sonar': pack(self.sonar, 100.0),  # certeza 0..100
            'obstacles_camera': pack(self.camera, 100.0),
            'robot': list(robot_pose) if robot_pose else None, 'stamp': stamp,
        }


def decode_layer(encoded: str) -> List[int]:
    return list(base64.b64decode(encoded))
