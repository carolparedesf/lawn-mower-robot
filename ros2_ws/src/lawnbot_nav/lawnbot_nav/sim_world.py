"""Mundo simulado con paredes y obstaculos circulares: sonares, bumpers y colisiones (sin ROS).

Lo usan fake_robot y los tests. Las posiciones de sensores salen del URDF (relativas a base_footprint);
en el URDF los tres sonares miran al frente, por eso el yaw de montaje de los laterales es un parametro.
"""
import math
from typing import Dict, Sequence, Tuple

Point = Tuple[float, float]

# Montaje (x, y) respecto de base_footprint
SONAR_MOUNTS = {'left': (0.194, 0.136), 'center': (0.198, 0.0), 'right': (0.193, -0.1435)}
BUMPER_MOUNTS = {
    'bumper/B1_front_left': (0.2137, 0.0813),
    'bumper/B2_front_right': (0.2137, -0.0827),
    'bumper/B3_left': (0.0957, 0.2113),
    'bumper/B4_right': (0.0957, -0.2127),
    'bumper/B5_back': (-0.2463, 0.0),
}
BODY_X = (-0.2463, 0.2137)      # extension longitudinal del cuerpo
BODY_Y = (-0.2127, 0.2113)      # extension lateral del cuerpo
SONAR_MAX_RANGE = 1.5
SONAR_MIN_RANGE = 0.02
SONAR_FOV = math.radians(15.0)
BUMPER_CONTACT = 0.02           # un bumper se considera presionado a menos de esta distancia
_BUMPER_ZONES = None            # se arma debajo de las funciones auxiliares


def _edge_samples():
    """Puntos sobre el borde del cuerpo, cada ~3 cm, agrupados por la zona que cubre cada bumper."""
    x0, x1 = BODY_X
    y0, y1 = BODY_Y
    step = 0.03
    def span(a, b):
        n = max(1, int(round(abs(b - a) / step)))
        return [a + (b - a) * i / n for i in range(n + 1)]
    def inner(a, b):                  # sin los extremos: las esquinas son de los bumpers laterales (si tambien fueran
        return span(a, b)[1:-1]       # del frontal/trasero, una pared paralela a un costado los presionaria a ellos)
    return {
        'bumper/B1_front_left': [(x1, y) for y in span(0.0, y1)[:-1]],
        'bumper/B2_front_right': [(x1, y) for y in span(y0, 0.0)[1:]],
        'bumper/B3_left': [(x, y1) for x in span(x0, x1)],
        'bumper/B4_right': [(x, y0) for x in span(x0, x1)],
        'bumper/B5_back': [(x0, y) for y in inner(y0, y1)],
    }


def _to_world(pose, dx, dy):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * dx - s * dy, pose[1] + s * dx + c * dy


def _dist_point_segment(p, a, b):
    abx, aby = b[0] - a[0], b[1] - a[1]
    l2 = abx * abx + aby * aby
    if l2 == 0.0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2))
    return math.hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby))


def _segment_hits_rect(p0, p1, xmin, xmax, ymin, ymax) -> bool:
    """Liang-Barsky: el segmento p0-p1 toca el rectangulo alineado con los ejes."""
    t0, t1 = 0.0, 1.0
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    for p, q in ((-dx, p0[0] - xmin), (dx, xmax - p0[0]), (-dy, p0[1] - ymin), (dy, ymax - p0[1])):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        r = q / p
        if p < 0.0:
            if r > t1:
                return False
            t0 = max(t0, r)
        else:
            if r < t0:
                return False
            t1 = min(t1, r)
    return True


def _segment_rect_distance(p0, p1, xmin, xmax, ymin, ymax) -> float:
    """Distancia minima entre el segmento p0-p1 y el rectangulo (0 si se tocan)."""
    if _segment_hits_rect(p0, p1, xmin, xmax, ymin, ymax):
        return 0.0
    best = math.inf
    for p in (p0, p1):
        nearest = (min(max(p[0], xmin), xmax), min(max(p[1], ymin), ymax))
        best = min(best, math.dist(p, nearest))
    for corner in ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)):
        best = min(best, _dist_point_segment(corner, p0, p1))
    return best


def bumper_zones(corner_radius: float = 0.0, split_deg: float = 45.0, dead_deg: float = 0.0):
    """Puntos del contorno del cuerpo que cubre cada bumper.

    Con `corner_radius` 0 el cuerpo es un rectangulo y las esquinas son de los bumpers laterales (ver _edge_samples).
    Con radio > 0 las cuatro esquinas son arcos de ese radio. En cada arco el angulo se mide desde la cara
    frontal/trasera (0 grados) hacia la lateral (90 grados): hasta `split_deg` - `dead_deg`/2 lo cubre el bumper
    frontal/trasero, desde `split_deg` + `dead_deg`/2 el lateral, y en medio (zona muerta, centrada en el split)
    ningun bumper. Con `dead_deg` 0 el punto exacto del split es del lateral.
    """
    r = float(corner_radius)
    if r <= 0.0:
        return _edge_samples()
    x0, x1 = BODY_X
    y0, y1 = BODY_Y
    r = min(r, (x1 - x0) / 2.0, (y1 - y0) / 2.0)
    step = 0.03

    def span(a, b):
        n = max(1, int(round(abs(b - a) / step)))
        return [a + (b - a) * i / n for i in range(n + 1)]

    lo, hi = split_deg - dead_deg / 2.0, split_deg + dead_deg / 2.0
    zones = {name: [] for name in BUMPER_MOUNTS}

    def arc(cx, cy, sx, sy, face, side):
        """Arco de centro (cx, cy) en el cuadrante (sx, sy); `face` = bumper frontal/trasero, `side` = lateral."""
        n = max(6, int(math.ceil(r * math.pi / 2.0 / 0.01)))
        for i in range(n + 1):
            alpha = 90.0 * i / n                       # 0: cara frontal/trasera, 90: cara lateral
            a = math.radians(alpha)
            pt = (cx + sx * r * math.cos(a), cy + sy * r * math.sin(a))
            if dead_deg <= 0.0:
                owner = face if alpha < split_deg else side
            else:
                owner = face if alpha <= lo else side if alpha >= hi else None
            if owner:
                zones[owner].append(pt)

    zones['bumper/B1_front_left'] += [(x1, y) for y in span(0.0, y1 - r)]
    zones['bumper/B2_front_right'] += [(x1, y) for y in span(y0 + r, 0.0)]
    zones['bumper/B3_left'] += [(x, y1) for x in span(x0 + r, x1 - r)]
    zones['bumper/B4_right'] += [(x, y0) for x in span(x0 + r, x1 - r)]
    zones['bumper/B5_back'] += [(x0, y) for y in span(y0 + r, y1 - r)]
    arc(x1 - r, y1 - r, 1, 1, 'bumper/B1_front_left', 'bumper/B3_left')
    arc(x1 - r, y0 + r, 1, -1, 'bumper/B2_front_right', 'bumper/B4_right')
    arc(x0 + r, y1 - r, -1, 1, 'bumper/B5_back', 'bumper/B3_left')
    arc(x0 + r, y0 + r, -1, -1, 'bumper/B5_back', 'bumper/B4_right')
    return zones


_BUMPER_ZONES = _edge_samples()


class SimWorld:
    def __init__(self, walls: Sequence[Tuple[Point, Point]] = (),
                 circles: Sequence[Tuple[float, float, float]] = (),
                 corner_radius: float = 0.0, corner_split_deg: float = 45.0, corner_dead_deg: float = 0.0):
        """`corner_radius` (m) redondea las esquinas del cuerpo (0 = rectangulo). El arco se reparte entre el bumper
        frontal/trasero y el lateral en `corner_split_deg` (medido desde la cara frontal/trasera) y, si
        `corner_dead_deg` > 0, queda una zona sin bumper de ese ancho centrada en el split. Ver bumper_zones()."""
        self.walls = [(tuple(a), tuple(b)) for a, b in walls]
        self.circles = [tuple(c) for c in circles if c[2] > 0.0]
        self.corner_radius = max(0.0, float(corner_radius))
        self.zones = bumper_zones(self.corner_radius, corner_split_deg, corner_dead_deg)

    @staticmethod
    def room(width: float, height: float, x0: float = 0.0, y0: float = 0.0, walls_only=True, **kw):
        a, b = (x0, y0), (x0 + width, y0)
        c, d = (x0 + width, y0 + height), (x0, y0 + height)
        return SimWorld(walls=[(a, b), (b, c), (c, d), (d, a)], **kw)

    @staticmethod
    def box(x1: float, y1: float, x2: float, y2: float):
        a, b, c, d = (x1, y1), (x2, y1), (x2, y2), (x1, y2)
        return [(a, b), (b, c), (c, d), (d, a)]

    # ── Distancias y rayos ───────────────────────────────────

    def distance(self, p: Point) -> float:
        d = math.inf
        for a, b in self.walls:
            d = min(d, _dist_point_segment(p, a, b))
        for cx, cy, r in self.circles:
            d = min(d, max(0.0, math.hypot(p[0] - cx, p[1] - cy) - r))
        return d

    def ray(self, origin: Point, angle: float, max_range: float) -> float:
        """Distancia al primer obstaculo a lo largo del rayo (max_range si no hay)."""
        dx, dy = math.cos(angle), math.sin(angle)
        best = max_range
        for a, b in self.walls:
            ex, ey = b[0] - a[0], b[1] - a[1]
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((a[0] - origin[0]) * ey - (a[1] - origin[1]) * ex) / den
            u = ((a[0] - origin[0]) * dy - (a[1] - origin[1]) * dx) / den
            if t >= 0.0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        for cx, cy, r in self.circles:
            fx, fy = origin[0] - cx, origin[1] - cy
            bq = fx * dx + fy * dy
            disc = bq * bq - (fx * fx + fy * fy - r * r)
            if disc >= 0.0:
                t = -bq - math.sqrt(disc)
                if t >= 0.0:
                    best = min(best, t)
        return best

    # ── Sensores ─────────────────────────────────────────────

    def sonar_range(self, pose, name: str, yaw_offset_deg: float = 0.0) -> float:
        """Rango como lo entrega el firmware: max_range si no hay eco."""
        dx, dy = SONAR_MOUNTS[name]
        origin = _to_world(pose, dx, dy)
        base = pose[2] + math.radians(yaw_offset_deg)
        hits = [self.ray(origin, base + k * SONAR_FOV / 2.0, SONAR_MAX_RANGE) for k in (-1, 0, 1)]
        return max(SONAR_MIN_RANGE, min(hits))

    def sonars(self, pose, yaw_offsets: Dict[str, float] = None) -> Dict[str, float]:
        yaw_offsets = yaw_offsets or {}
        return {n: self.sonar_range(pose, n, yaw_offsets.get(n, 0.0)) for n in SONAR_MOUNTS}

    def bumpers(self, pose) -> Dict[str, bool]:
        """Cada bumper cubre una zona del borde del cuerpo (no un solo punto)."""
        out = {}
        for name, zone in self.zones.items():
            out[name] = any(self.distance(_to_world(pose, *pt)) <= BUMPER_CONTACT for pt in zone)
        return out

    # ── Colisiones ───────────────────────────────────────────

    def collides(self, pose) -> bool:
        """El cuerpo (rectangulo) toca una pared o un obstaculo."""
        c, s = math.cos(pose[2]), math.sin(pose[2])

        def body(p):                                    # mundo -> marco del cuerpo
            dx, dy = p[0] - pose[0], p[1] - pose[1]
            return c * dx + s * dy, -s * dx + c * dy

        r = min(self.corner_radius, (BODY_X[1] - BODY_X[0]) / 2.0, (BODY_Y[1] - BODY_Y[0]) / 2.0)
        x0, x1 = BODY_X[0] + r, BODY_X[1] - r              # rectangulo interior; el cuerpo es ese rectangulo + r
        y0, y1 = BODY_Y[0] + r, BODY_Y[1] - r
        for a, b in self.walls:
            if _segment_rect_distance(body(a), body(b), x0, x1, y0, y1) <= r:
                return True
        for cx, cy, rad in self.circles:
            bx, by = body((cx, cy))
            nearest = (min(max(bx, x0), x1), min(max(by, y0), y1))
            if math.dist((bx, by), nearest) <= rad + r:
                return True
        return False

    def step_robot(self, robot, v: float, w: float, dt: float) -> bool:
        """Avanza al SimRobot; si el cuerpo chocaria, prueba solo trasladar, luego solo girar.

        Asi el robot puede deslizarse junto a una pared. Devuelve True si hubo choque.
        """
        before = (robot.x, robot.y, robot.yaw)
        robot.step(v, w, dt)
        after = (robot.x, robot.y, robot.yaw)
        if not self.collides(after):
            return False
        if not self.collides((after[0], after[1], before[2])):      # solo la traslacion
            robot.yaw = before[2]
        elif not self.collides((before[0], before[1], after[2])):   # solo el giro
            robot.x, robot.y = before[0], before[1]
        else:
            robot.x, robot.y, robot.yaw = before
        robot.v = 0.0
        return True
