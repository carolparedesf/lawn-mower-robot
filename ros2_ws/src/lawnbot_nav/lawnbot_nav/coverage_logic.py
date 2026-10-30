"""Barrido en lineas paralelas (boustrophedon) sobre un poligono convexo, sin ROS.

La region de trabajo es el poligono recortado `margin` metros hacia adentro; las lineas se trazan
paralelas al lado mas largo (o a un angulo dado), separadas `cutting_width * (1 - overlap)`.
Solo se aceptan poligonos convexos: asi cualquier tramo de union entre dos lineas queda dentro del
cerco. La ruta lleva solo los extremos de cada linea.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Point = Tuple[float, float]

ERR_NO_POLYGON = 'sin cerco'
ERR_NOT_CONVEX = 'poligono no convexo (solo se aceptan cercos convexos)'


@dataclass
class CoverageParams:
    cutting_width: float = 0.30
    overlap: float = 0.10
    margin: float = 1.0
    sweep_angle_deg: Optional[float] = None    # None = paralelo al lado mas largo
    start_nearest: bool = True
    turn_extension: float = 0.15               # prolonga cada extremo (sin pasar del margen)
    min_line_length: float = 0.05
    est_linear: float = 0.2                    # m/s, solo para estimar el tiempo
    est_angular: float = 0.6                   # rad/s


@dataclass
class CoverageResult:
    ok: bool
    error: str = ''
    path: List[Point] = field(default_factory=list)
    lines: List[Tuple[Point, Point]] = field(default_factory=list)   # (inicio, fin) en orden de recorrido
    region: List[Point] = field(default_factory=list)                # poligono recortado
    sweep_deg: float = 0.0
    spacing: float = 0.0
    length: float = 0.0
    est_time_s: float = 0.0

    @property
    def n_lines(self) -> int:
        return len(self.lines)


# ── Geometria ────────────────────────────────────────────────

def _cross(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def signed_area(poly) -> float:
    return sum(_cross(p, q) for p, q in zip(poly, list(poly[1:]) + [poly[0]])) / 2.0


def is_convex(poly, collinear_tol: float = 1e-3) -> bool:
    """Todos los giros hacia el mismo lado (vertices casi colineales permitidos)."""
    n = len(poly)
    if n < 3:
        return False
    sign = 0
    for i in range(n):
        a, b, c = poly[i], poly[(i + 1) % n], poly[(i + 2) % n]
        e1 = (b[0] - a[0], b[1] - a[1])
        e2 = (c[0] - b[0], c[1] - b[1])
        n1, n2 = math.hypot(*e1), math.hypot(*e2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cr = _cross(e1, e2)
        if abs(cr) <= collinear_tol * n1 * n2:
            continue
        s = 1 if cr > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return sign != 0


def _clip_halfplane(poly, a: Point, b: Point, offset: float):
    """Se queda con lo que esta a mas de `offset` a la izquierda de la arista a->b."""
    ex, ey = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ex, ey)

    def dist(p):
        return (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / length - offset

    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        dp, dq = dist(p), dist(q)
        if dp >= 0:
            out.append(p)
        if (dp >= 0) != (dq >= 0):
            t = dp / (dp - dq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def inset_convex(poly_ccw, margin: float):
    """Poligono convexo (antihorario) recortado `margin` hacia adentro; [] si no queda region."""
    if margin <= 0.0:
        return list(poly_ccw)
    out = list(poly_ccw)
    n = len(poly_ccw)
    for i in range(n):
        a, b = poly_ccw[i], poly_ccw[(i + 1) % n]
        if math.dist(a, b) < 1e-9:
            continue
        out = _clip_halfplane(out, a, b, margin)
        if len(out) < 3:
            return []
    dedup = []
    for p in out:
        if not dedup or math.dist(dedup[-1], p) > 1e-9:
            dedup.append(p)
    if len(dedup) > 1 and math.dist(dedup[0], dedup[-1]) <= 1e-9:
        dedup.pop()
    return dedup if len(dedup) >= 3 and abs(signed_area(dedup)) > 1e-6 else []


def line_interval(poly_ccw, origin: Point, direction: Point):
    """Intervalo [t_lo, t_hi] de origin + t*direction dentro del poligono convexo, o None."""
    t_lo, t_hi = -math.inf, math.inf
    n = len(poly_ccw)
    for i in range(n):
        a, b = poly_ccw[i], poly_ccw[(i + 1) % n]
        e = (b[0] - a[0], b[1] - a[1])
        length = math.hypot(*e)
        if length < 1e-12:
            continue
        c0 = _cross(e, (origin[0] - a[0], origin[1] - a[1])) / length
        c1 = _cross(e, direction) / length
        if abs(c1) < 1e-12:
            if c0 < -1e-9:
                return None
            continue
        t0 = -c0 / c1
        if c1 > 0:
            t_lo = max(t_lo, t0)
        else:
            t_hi = min(t_hi, t0)
    if not (math.isfinite(t_lo) and math.isfinite(t_hi)) or t_hi < t_lo:
        return None
    return t_lo, t_hi


# ── Generacion ───────────────────────────────────────────────

def _validate(polygon, p: CoverageParams) -> str:
    if not polygon:
        return ERR_NO_POLYGON
    if len(polygon) < 3:
        return 'poligono con menos de 3 vertices'
    if p.cutting_width <= 0.0:
        return 'cutting_width debe ser mayor que 0'
    if not (0.0 <= p.overlap < 1.0):
        return 'overlap debe estar entre 0 y 1'
    if p.margin < 0.0:
        return 'margin no puede ser negativo'
    if p.turn_extension < 0.0:
        return 'turn_extension no puede ser negativo'
    if p.turn_extension > p.margin:
        return f'turn_extension ({p.turn_extension} m) mayor que margin ({p.margin} m)'
    return ''


def generate_coverage(polygon, params: CoverageParams, robot_xy: Optional[Point] = None) -> CoverageResult:
    err = _validate(polygon, params)
    if err:
        return CoverageResult(False, err)

    poly = list(polygon)
    if signed_area(poly) < 0:
        poly.reverse()
    if not is_convex(poly):
        return CoverageResult(False, ERR_NOT_CONVEX)

    region = inset_convex(poly, params.margin)
    if not region:
        width = _min_width(poly)
        return CoverageResult(False, (
            f'region vacia: un margen de {params.margin} m no deja espacio '
            f'(el cerco mide {width:.1f} m en su lado mas angosto; reducir margin)'))

    # Direccion de barrido: paralela al lado mas largo (o la pedida)
    if params.sweep_angle_deg is None:
        longest = max(range(len(poly)), key=lambda i: math.dist(poly[i], poly[(i + 1) % len(poly)]))
        a, b = poly[longest], poly[(longest + 1) % len(poly)]
        theta = math.atan2(b[1] - a[1], b[0] - a[0])
    else:
        theta = math.radians(params.sweep_angle_deg)
    d = (math.cos(theta), math.sin(theta))
    nrm = (-d[1], d[0])

    offsets = [nrm[0] * v[0] + nrm[1] * v[1] for v in region]
    p_min, p_max = min(offsets), max(offsets)
    extent = p_max - p_min
    nominal = params.cutting_width * (1.0 - params.overlap)
    if extent < 1e-6:
        positions, spacing = [(p_min + p_max) / 2.0], 0.0
    else:
        n_lines = math.ceil(extent / nominal - 1e-9) + 1
        spacing = extent / (n_lines - 1)
        positions = [p_min + k * spacing for k in range(n_lines)]

    # Region ampliada solo para prolongar los extremos de las lineas
    outer = inset_convex(poly, params.margin - params.turn_extension) if params.turn_extension > 0 else region
    if not outer:
        outer = region

    segments = []                                  # (S, E) con S en t bajo y E en t alto
    for pos in positions:
        origin = (nrm[0] * pos, nrm[1] * pos)
        inner = line_interval(region, origin, d)
        if inner is None or inner[1] - inner[0] < params.min_line_length:
            continue
        t_lo, t_hi = inner
        if params.turn_extension > 0:
            outer_iv = line_interval(outer, origin, d)
            if outer_iv is not None:
                t_lo = max(t_lo - params.turn_extension, outer_iv[0])
                t_hi = min(t_hi + params.turn_extension, outer_iv[1])
        segments.append(((origin[0] + t_lo * d[0], origin[1] + t_lo * d[1]),
                         (origin[0] + t_hi * d[0], origin[1] + t_hi * d[1])))
    if not segments:
        return CoverageResult(False, 'ninguna linea de barrido cabe en la region de trabajo')

    # Cuatro formas de recorrerlo; con start_nearest se elige la que empieza mas cerca del robot
    variants = [(False, False), (False, True), (True, False), (True, True)]   # (orden inverso, primera linea al reves)
    def build(reverse, flip):
        ordered = list(reversed(segments)) if reverse else list(segments)
        out, forward = [], not flip
        for s, e in ordered:
            out.append((s, e) if forward else (e, s))
            forward = not forward
        return out

    candidates = [build(r, f) for r, f in variants]
    if params.start_nearest and robot_xy is not None:
        lines = min(candidates, key=lambda c: math.dist(robot_xy, c[0][0]))
    else:
        lines = candidates[0]

    path = [pt for line in lines for pt in line]
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    turns = max(0, len(path) - 2)
    est_time = length / params.est_linear + turns * (math.pi / 2.0) / params.est_angular
    return CoverageResult(True, '', path, lines, region, math.degrees(theta), spacing, length, est_time)


def _min_width(poly) -> float:
    """Ancho minimo del poligono convexo (menor extension entre direcciones de sus lados)."""
    best = math.inf
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        length = math.dist(a, b)
        if length < 1e-9:
            continue
        nx, ny = -(b[1] - a[1]) / length, (b[0] - a[0]) / length
        proj = [nx * p[0] + ny * p[1] for p in poly]
        best = min(best, max(proj) - min(proj))
    return best if math.isfinite(best) else 0.0


def format_status(result: CoverageResult) -> str:
    if not result.ok:
        return f'ERROR:{result.error}'
    return (f'OK lineas={result.n_lines} longitud={result.length:.1f} m '
            f'tiempo_estimado={result.est_time_s / 60.0:.1f} min angulo={result.sweep_deg:.1f} deg '
            f'espaciado={result.spacing:.3f} m')


# ── Medicion (pruebas) ───────────────────────────────────────

def swath_coverage(region, trajectory, half_width: float, step: float = 0.15) -> float:
    """Fraccion de `region` (poligono convexo) a menos de `half_width` de algun punto de `trajectory`."""
    xs = [p[0] for p in region]
    ys = [p[1] for p in region]
    traj = list(trajectory)
    total = covered = 0
    x = min(xs)
    while x <= max(xs):
        y = min(ys)
        while y <= max(ys):
            if line_interval(region, (x, y), (1.0, 0.0)) is not None and _inside(region, (x, y)):
                total += 1
                if any(math.hypot(x - q[0], y - q[1]) <= half_width for q in traj):
                    covered += 1
            y += step
        x += step
    return covered / total if total else 0.0


def _inside(poly_ccw, pt) -> bool:
    n = len(poly_ccw)
    for i in range(n):
        a, b = poly_ccw[i], poly_ccw[(i + 1) % n]
        if _cross((b[0] - a[0], b[1] - a[1]), (pt[0] - a[0], pt[1] - a[1])) < -1e-9:
            return False
    return True
