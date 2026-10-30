"""Geometria del cerco virtual sin dependencias de ROS.

El cerco llega de la app en lat/lon y se guarda siempre asi; se convierte a metros del frame
`map` cada vez que hace falta (el origen de `map` cambia entre arranques).
"""
import json
import math
import os
import tempfile
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]

STATUS_NO_POLYGON = 'NO_POLYGON'
STATUS_WAITING = 'WAITING_CONVERSION'
STATUS_NO_POSE = 'NO_POSE'
STATUS_OK = 'OK'
STATUS_INVALID = 'INVALID'   # se publica como 'INVALID:<motivo>'

_EPS = 1e-9


# ── Entrada de la app ────────────────────────────────────────

def parse_geofence_json(text: str) -> List[Tuple[float, float]]:
    """{"points":[{"lat":..,"lng":..}, ...]} -> [(lat, lng)]. ValueError si es invalido."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'json invalido ({exc})')
    points = data.get('points') if isinstance(data, dict) else None
    if not isinstance(points, list):
        raise ValueError("falta la lista 'points'")
    out = []
    for p in points:
        if not isinstance(p, dict):
            raise ValueError('punto con formato invalido')
        lat = p.get('lat')
        lng = p.get('lng', p.get('lon'))
        for v in (lat, lng):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError('coordenada no numerica')
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lng <= 180.0):
            raise ValueError('coordenada fuera de rango')
        out.append((float(lat), float(lng)))
    return out


def drop_closing_vertex(points: Sequence[Point], eps: float = 1e-9) -> List[Point]:
    """Quita el ultimo vertice si repite al primero."""
    pts = list(points)
    if len(pts) > 1 and abs(pts[0][0] - pts[-1][0]) <= eps and abs(pts[0][1] - pts[-1][1]) <= eps:
        pts.pop()
    return pts


# ── Conversion de respaldo (plano tangente local, x = este, y = norte) ──

def lat_lon_to_local_xy(lat: float, lng: float, lat0: float, lng0: float) -> Point:
    phi = math.radians(lat0)
    m_per_deg_lat = (111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
                     - 0.0023 * math.cos(6 * phi))
    m_per_deg_lng = (111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
                     + 0.118 * math.cos(5 * phi))
    return ((lng - lng0) * m_per_deg_lng, (lat - lat0) * m_per_deg_lat)


# ── Geometria ────────────────────────────────────────────────

def _cross(o: Point, a: Point, b: Point) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _on_segment(p: Point, q: Point, r: Point) -> bool:
    """r (colineal con p-q) esta dentro del segmento p-q."""
    return (min(p[0], q[0]) - _EPS <= r[0] <= max(p[0], q[0]) + _EPS and
            min(p[1], q[1]) - _EPS <= r[1] <= max(p[1], q[1]) + _EPS)


def segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    """Interseccion de segmentos, incluyendo roce en un extremo y solapamiento colineal."""
    d1, d2 = _cross(p3, p4, p1), _cross(p3, p4, p2)
    d3, d4 = _cross(p1, p2, p3), _cross(p1, p2, p4)
    if ((d1 > _EPS and d2 < -_EPS) or (d1 < -_EPS and d2 > _EPS)) and \
       ((d3 > _EPS and d4 < -_EPS) or (d3 < -_EPS and d4 > _EPS)):
        return True
    if abs(d1) <= _EPS and _on_segment(p3, p4, p1):
        return True
    if abs(d2) <= _EPS and _on_segment(p3, p4, p2):
        return True
    if abs(d3) <= _EPS and _on_segment(p1, p2, p3):
        return True
    if abs(d4) <= _EPS and _on_segment(p1, p2, p4):
        return True
    return False


def signed_area(poly: Sequence[Point]) -> float:
    s = 0.0
    for (x1, y1), (x2, y2) in zip(poly, list(poly[1:]) + [poly[0]]):
        s += x1 * y2 - x2 * y1
    return s / 2.0


def validate_polygon(points: Sequence[Point], min_vertices: int = 4, min_edge: float = 0.5,
                     min_area: float = 2.0):
    """Valida un poligono en metros. Devuelve (poligono antihorario | None, motivo | None)."""
    pts = drop_closing_vertex(points, 1e-6)
    n = len(pts)
    if n < min_vertices:
        return None, f'menos de {min_vertices} vertices ({n})'

    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        d = math.dist(a, b)
        if d < min_edge:
            return None, f'arista {i + 1} de {d:.2f} m (minimo {min_edge} m)'

    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        # Aristas adyacentes: no pueden volver sobre si mismas
        c = pts[(i + 2) % n]
        if abs(_cross(a, b, c)) <= _EPS and \
                ((b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1])) < 0:
            return None, f'arista {i + 1} se superpone con la siguiente'
        # Aristas no adyacentes: no pueden cruzarse ni tocarse
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if segments_intersect(a, b, pts[j], pts[(j + 1) % n]):
                return None, f'autointerseccion entre las aristas {i + 1} y {j + 1}'

    area = signed_area(pts)
    if abs(area) < min_area:
        return None, f'area de {abs(area):.2f} m2 (minimo {min_area} m2)'
    if area < 0:
        pts.reverse()
    return pts, None


def point_in_polygon(pt: Point, poly: Sequence[Point]) -> bool:
    """Trazado de rayo; un punto sobre el borde cuenta como adentro."""
    if distance_to_boundary(pt, poly) <= _EPS:
        return True
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def _dist_point_segment(p: Point, a: Point, b: Point) -> float:
    abx, aby = b[0] - a[0], b[1] - a[1]
    l2 = abx * abx + aby * aby
    if l2 == 0.0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2))
    return math.hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby))


def distance_to_boundary(pt: Point, poly: Sequence[Point]) -> float:
    n = len(poly)
    return min(_dist_point_segment(pt, poly[i], poly[(i + 1) % n]) for i in range(n))


def signed_distance(pt: Point, poly: Sequence[Point]) -> float:
    """Distancia al borde: positiva dentro, negativa fuera."""
    d = distance_to_boundary(pt, poly)
    return d if point_in_polygon(pt, poly) else -d


# ── Histeresis ───────────────────────────────────────────────

class Hysteresis:
    """Cambia de estado solo tras N lecturas consecutivas del valor contrario.

    Un valor desconocido (None) fuerza False de inmediato (falla segura).
    """

    def __init__(self, to_true: int = 5, to_false: int = 2, initial: bool = False):
        self.to_true = max(1, int(to_true))
        self.to_false = max(1, int(to_false))
        self.state = initial
        self._count = 0

    def update(self, raw: Optional[bool]) -> bool:
        if raw is None:
            self.state = False
            self._count = 0
            return False
        if raw == self.state:
            self._count = 0
            return self.state
        self._count += 1
        if self._count >= (self.to_true if raw else self.to_false):
            self.state = raw
            self._count = 0
        return self.state


# ── Persistencia (siempre en lat/lon) ────────────────────────

def save_geofence(path: str, points_lat_lng: Sequence[Tuple[float, float]]) -> None:
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    payload = {'points': [{'lat': lat, 'lng': lng} for lat, lng in points_lat_lng]}
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(payload, f)
        os.replace(tmp, path)
    except OSError:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def load_geofence(path: str) -> Optional[List[Tuple[float, float]]]:
    path = os.path.expanduser(path)
    try:
        with open(path) as f:
            return parse_geofence_json(f.read())
    except (OSError, ValueError):
        return None
