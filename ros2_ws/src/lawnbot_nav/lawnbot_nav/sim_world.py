"""Mundo simulado con paredes y obstaculos circulares: sonares, bumpers y colisiones (sin ROS).

Lo usan fake_robot y los tests. Las posiciones de sensores salen del URDF (relativas a base_footprint); el yaw de montaje
de los sonares de las esquinas es el parametro compartido `corner_sonar_angle_deg` (ver robot_geometry.py).

Cuerpo: por defecto un rectangulo (BODY_X x BODY_Y, esquinas redondeadas segun `corner_radius`). Con `footprint=Footprint.chassis()`
es la huella real del chasis (frente ancho, trasera angosta; ver robot_geometry.py).
"""
import math
from typing import Dict, List, Optional, Sequence, Tuple

from lawnbot_nav import robot_geometry as geo
from lawnbot_nav.robot_geometry import SONAR_MOUNTS  # noqa: F401  (se reexporta)

Point = Tuple[float, float]
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


def _point_in_polygon(pt, poly) -> bool:
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _segments_cross(p1, p2, p3, p4) -> bool:
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = side(p3, p4, p1), side(p3, p4, p2)
    d3, d4 = side(p1, p2, p3), side(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


class Footprint:
    """Huella del chasis: contorno (poligono) y zonas de bumper. Marco del robot (x adelante, y a la izquierda).

    Un bloque delantero ancho (`front_width` x `front_depth`, esquinas delanteras de radio `corner_radius`) y un bloque trasero
    mas angosto (`rear_width`) hasta completar `length`. Bumpers: B1/B2 cubren el frente recto y entran en cada arco solo hasta
    `split_deg - dead_deg/2`; B3/B4 cubren el costado del bloque delantero, desde `split_deg + dead_deg/2` del arco hasta el escalon
    donde empieza el bloque trasero; B5 cubre la cara trasera. Sin bumper (`unprotected`): la zona muerta de cada arco
    delantero, los costados del bloque trasero y los escalones. Los angulos del arco se miden desde la cara frontal (0 grados)
    hacia la lateral (90).
    """

    def __init__(self, front_x: float = geo.BODY_FRONT_X, length: float = geo.BODY_LENGTH,
                 front_depth: float = geo.BODY_FRONT_DEPTH, front_width: float = geo.BODY_FRONT_WIDTH,
                 rear_width: float = geo.BODY_REAR_WIDTH, corner_radius: float = geo.CORNER_RADIUS,
                 split_deg: float = 41.0, dead_deg: float = 55.0):
        self.front_x, self.length = front_x, length
        self.front_depth, self.front_width, self.rear_width = front_depth, front_width, rear_width
        yf, yr = front_width / 2.0, rear_width / 2.0
        self.corner_radius = r = max(0.0, min(float(corner_radius), front_depth, yf))
        self.split_deg, self.dead_deg = split_deg, dead_deg
        xr, xs = front_x - length, front_x - front_depth        # cara trasera y escalon entre los dos bloques
        self.x_rear, self.x_step = xr, xs
        lo, hi = split_deg - dead_deg / 2.0, split_deg + dead_deg / 2.0

        def owner(alpha, face, side, dead='dead'):
            if dead_deg <= 0.0:
                return face if alpha < split_deg else side
            return face if alpha <= lo else side if alpha >= hi else dead

        def span(a, b, step=0.01):
            n = max(1, int(round(abs(b - a) / step)))
            return [a + (b - a) * i / n for i in range(n + 1)]

        n_arc = max(6, int(math.ceil(r * math.pi / 2.0 / 0.01))) if r > 0 else 0
        # --- armado del contorno antihorario empezando en la esquina trasera derecha ---
        out: List[Point] = [(xr, -yr), (xs, -yr), (xs, -yf)]
        if r > 0:
            out.append((front_x - r, -yf))
            for i in range(n_arc + 1):                                  # arco delantero derecho: alpha 90 -> 0
                a = math.radians(90.0 - 90.0 * i / n_arc)
                out.append((front_x - r + r * math.cos(a), -yf + r - r * math.sin(a)))
            for i in range(n_arc + 1):                                  # arco delantero izquierdo: alpha 0 -> 90
                a = math.radians(90.0 * i / n_arc)
                out.append((front_x - r + r * math.cos(a), yf - r + r * math.sin(a)))
            out.append((front_x - r, yf))
        else:
            out += [(front_x, -yf), (front_x, yf)]
        out += [(xs, yf), (xs, yr), (xr, yr)]
        dedup: List[Point] = []
        for q in out:
            if not dedup or math.dist(q, dedup[-1]) > 1e-9:
                dedup.append(q)
        if math.dist(dedup[0], dedup[-1]) < 1e-9:
            dedup.pop()
        self.outline = dedup
        self._edges = [(dedup[i], dedup[(i + 1) % len(dedup)]) for i in range(len(dedup))]
        self.bbox = (min(p[0] for p in dedup), max(p[0] for p in dedup), min(p[1] for p in dedup), max(p[1] for p in dedup))

        # --- zonas: puntos del contorno con su duenio. El arco se muestrea cada ~2,25 grados e incluye los limites exactos
        # de las zonas (split -/+ dead/2), para que la zona muerta mida lo que dice ---
        alphas = sorted({90.0 * i / 40 for i in range(41)} | ({lo, hi} if dead_deg > 0.0 else {split_deg}), reverse=True)
        alphas = [a_ for a_ in alphas if 0.0 <= a_ <= 90.0] if r > 0 else []
        samples: List[Tuple[Point, str]] = []
        for y in span(-yr, yr)[1:-1]:
            samples.append(((xr, y), 'bumper/B5_back'))
        for sgn, face, side in ((1.0, 'bumper/B1_front_left', 'bumper/B3_left'), (-1.0, 'bumper/B2_front_right', 'bumper/B4_right')):
            for x in span(xr, xs)[:-1]:
                samples.append(((x, sgn * yr), 'rear_side'))
            for y in span(yr, yf)[:-1]:
                samples.append(((xs, sgn * y), 'step'))
            for x in span(xs, front_x - r)[:-1]:
                samples.append(((x, sgn * yf), side))
            for alpha in alphas:                                        # arco: alpha 90 (lateral) -> 0 (frontal)
                a = math.radians(alpha)
                samples.append(((front_x - r + r * math.cos(a), sgn * (yf - r + r * math.sin(a))),
                                owner(alpha, face, side)))
            for y in span(yf - r, 0.0)[1:]:
                if sgn > 0 and y <= 1e-12:
                    continue                                            # y = 0 es de B2
                samples.append(((front_x, sgn * y), face))
        self.zones: Dict[str, List[Point]] = {name: [] for name in BUMPER_MOUNTS}
        self.unprotected: Dict[str, List[Point]] = {'dead': [], 'rear_side': [], 'step': []}
        for pt, who in samples:
            if who in self.zones:
                self.zones[who].append(pt)
            else:
                self.unprotected[who].append(pt)

    # -- Colision (marco del robot) --
    def hits_segment(self, p0: Point, p1: Point) -> bool:
        """El segmento toca o entra en la huella."""
        x0, x1, y0, y1 = self.bbox
        if max(p0[0], p1[0]) < x0 or min(p0[0], p1[0]) > x1 or max(p0[1], p1[1]) < y0 or min(p0[1], p1[1]) > y1:
            return False
        if _point_in_polygon(p0, self.outline) or _point_in_polygon(p1, self.outline):
            return True
        return any(_segments_cross(p0, p1, a, b) for a, b in self._edges)

    def hits_circle(self, c: Point, radius: float) -> bool:
        x0, x1, y0, y1 = self.bbox
        if c[0] + radius < x0 or c[0] - radius > x1 or c[1] + radius < y0 or c[1] - radius > y1:
            return False
        if _point_in_polygon(c, self.outline):
            return True
        return any(_dist_point_segment(c, a, b) <= radius for a, b in self._edges)


class SimWorld:
    def __init__(self, walls: Sequence[Tuple[Point, Point]] = (),
                 circles: Sequence[Tuple[float, float, float]] = (),
                 corner_radius: float = 0.0, corner_split_deg: float = 45.0, corner_dead_deg: float = 0.0,
                 footprint: Optional[Footprint] = None):
        """Con `footprint` (Footprint.chassis-like) el cuerpo es esa huella y los demas parametros de esquina se ignoran
        (los trae el Footprint). Sin el, `corner_radius` (m) redondea las esquinas del cuerpo (0 = rectangulo). El arco se reparte entre el bumper
        frontal/trasero y el lateral en `corner_split_deg` (medido desde la cara frontal/trasera) y, si
        `corner_dead_deg` > 0, queda una zona sin bumper de ese ancho centrada en el split. Ver bumper_zones()."""
        self.walls = [(tuple(a), tuple(b)) for a, b in walls]
        self.circles = [tuple(c) for c in circles if c[2] > 0.0]
        self.footprint = footprint
        if footprint is not None:
            self.corner_radius = footprint.corner_radius
            self.zones = footprint.zones
        else:
            self.corner_radius = max(0.0, float(corner_radius))
            self.zones = bumper_zones(self.corner_radius, corner_split_deg, corner_dead_deg)

    @staticmethod
    def from_config(cfg: dict) -> 'SimWorld':
        """Mundo a partir de los parametros de fake_robot (sim_room.yaml / sim_field.yaml): walls, obstacles, body_shape y esquinas."""
        w, o = cfg.get('walls', []), cfg.get('obstacles', [])
        walls = [((w[i], w[i + 1]), (w[i + 2], w[i + 3])) for i in range(0, len(w) - 3, 4)]
        circles = [(o[i], o[i + 1], o[i + 2]) for i in range(0, len(o) - 2, 3)]
        r, split, dead = cfg.get('corner_radius', 0.0), cfg.get('corner_split_deg', 45.0), cfg.get('corner_dead_deg', 0.0)
        footprint = None
        if cfg.get('body_shape', 'rectangle') == 'chassis':
            footprint = Footprint(length=cfg.get('body_length', geo.BODY_LENGTH),
                                  front_depth=cfg.get('body_front_depth', geo.BODY_FRONT_DEPTH),
                                  front_width=cfg.get('body_front_width', geo.BODY_FRONT_WIDTH),
                                  rear_width=cfg.get('body_rear_width', geo.BODY_REAR_WIDTH),
                                  corner_radius=r, split_deg=split, dead_deg=dead)
        return SimWorld(walls, circles, corner_radius=r, corner_split_deg=split, corner_dead_deg=dead, footprint=footprint)

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

    def unprotected_contacts(self, pose) -> Dict[str, bool]:
        """Zonas del contorno SIN bumper ('dead': zona muerta de las esquinas delanteras, 'rear_side', 'step') que estan a menos
        de BUMPER_CONTACT de una pared u obstaculo. Vacio sin Footprint (en el cuerpo rectangular todo el contorno es de un bumper)."""
        if self.footprint is None:
            return {}
        return {name: any(self.distance(_to_world(pose, *pt)) <= BUMPER_CONTACT for pt in pts)
                for name, pts in self.footprint.unprotected.items()}

    def collides(self, pose) -> bool:
        """El cuerpo toca una pared o un obstaculo."""
        c, s = math.cos(pose[2]), math.sin(pose[2])

        def body(p):                                    # mundo -> marco del cuerpo
            dx, dy = p[0] - pose[0], p[1] - pose[1]
            return c * dx + s * dy, -s * dx + c * dy

        if self.footprint is not None:
            fp = self.footprint
            return (any(fp.hits_segment(body(a), body(b)) for a, b in self.walls)
                    or any(fp.hits_circle(body((cx, cy)), rad) for cx, cy, rad in self.circles))

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
