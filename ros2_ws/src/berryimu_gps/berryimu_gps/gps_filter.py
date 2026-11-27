"""
Calidad de los fixes de gpsd antes de mandarlos a navsat_transform (sin dependencias de ROS).

- Covarianza: epx/epy/eph/epv de gpsd son errores al 95 %; sigma = ep * ep_to_sigma (0,5 ~ 1/1,96).
- Se descartan los fixes 2D, los de eph > max_eph_m y los de HDOP > max_hdop (el HDOP llega en
  los reportes SKY).
- Saltos: se rechaza un fix que queda mas lejos del ultimo aceptado que
  max_speed * dt + jump_sigmas * sigma. Despues de jump_accept_after rechazos seguidos se acepta
  igual (si no, un primer fix malo trabaria el GPS para siempre).
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

EARTH_RADIUS_M = 6371008.8

ACCEPT = 'ok'
REJECT_NO_FIX = 'sin_fix'
REJECT_2D = 'fix_2d'
REJECT_EPH = 'eph_alto'
REJECT_HDOP = 'hdop_alto'
REJECT_JUMP = 'salto'


@dataclass
class FilterParams:
    ep_to_sigma: float = 0.5            # epx/epy/eph/epv de gpsd (95 %) -> sigma
    min_sigma_m: float = 0.5            # piso del sigma horizontal y vertical
    fallback_sigma_m: float = 10.0      # sin epx/epy ni eph
    fallback_vertical_variance: float = 1.0e4
    require_3d: bool = True             # descartar fixes 2D (mode 2)
    max_eph_m: float = 8.0              # <= 0: sin limite
    max_hdop: float = 2.5               # <= 0: sin limite
    max_speed: float = 0.8              # m/s
    jump_sigmas: float = 3.0
    # rechazos por salto seguidos; el siguiente se acepta igual. <= 0: sin rechazo de saltos
    jump_accept_after: int = 5


@dataclass
class Decision:
    reason: str
    sigma_x: float = math.nan
    sigma_y: float = math.nan
    vertical_variance: float = math.nan
    known: bool = False                 # epx y epy reales (covarianza DIAGONAL_KNOWN)
    detail: str = ''

    @property
    def accepted(self):
        return self.reason == ACCEPT


def distance_m(lat1, lon1, lat2, lon2):
    """Distancia horizontal aproximada (equirectangular; sobra para saltos de metros)."""
    phi = math.radians((lat1 + lat2) / 2.0)
    dx = math.radians(lon2 - lon1) * math.cos(phi) * EARTH_RADIUS_M
    dy = math.radians(lat2 - lat1) * EARTH_RADIUS_M
    return math.hypot(dx, dy)


def _num(value) -> Optional[float]:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def sigmas(tpv, p: FilterParams) -> Tuple[float, float, float, bool]:
    """
    (sigma_x, sigma_y, varianza vertical, conocida) a partir del TPV de gpsd.

    x = este (epx), y = norte (epy).
    """
    epx, epy = _num(tpv.get('epx')), _num(tpv.get('epy'))
    known = epx is not None and epy is not None
    if not known:
        eph = _num(tpv.get('eph'))
        if eph is not None:
            epx = epy = eph
            scale = p.ep_to_sigma
        else:
            epx = epy = p.fallback_sigma_m          # ya es un sigma
            scale = 1.0
    else:
        scale = p.ep_to_sigma
    sx = max(epx * scale, p.min_sigma_m)
    sy = max(epy * scale, p.min_sigma_m)
    epv = _num(tpv.get('epv'))
    if epv is not None:
        vvar = max(epv * p.ep_to_sigma, p.min_sigma_m) ** 2
    else:
        vvar = p.fallback_vertical_variance
    return sx, sy, vvar, known


class FixFilter:
    def __init__(self, params: FilterParams = None):
        self.p = params or FilterParams()
        self.hdop: Optional[float] = None       # ultimo HDOP de un reporte SKY
        # (lat, lon, t) del ultimo fix aceptado
        self.last: Optional[Tuple[float, float, float]] = None
        self.jump_rejects = 0

    def on_sky(self, sky):
        hdop = _num(sky.get('hdop'))
        if hdop is not None:
            self.hdop = hdop

    def check(self, tpv, now) -> Decision:
        """
        Decide si se publica el TPV recibido en `now` [s].

        Solo un fix aceptado pasa a ser la referencia de saltos.
        """
        p = self.p
        mode = tpv.get('mode', 0)
        lat, lon = _num(tpv.get('lat')), _num(tpv.get('lon'))
        if mode is None or mode < 2 or lat is None or lon is None:
            return Decision(REJECT_NO_FIX)
        if p.require_3d and mode < 3:
            return Decision(REJECT_2D)
        eph = _num(tpv.get('eph'))
        if p.max_eph_m > 0.0 and eph is not None and eph > p.max_eph_m:
            return Decision(REJECT_EPH, detail=f'eph {eph:.1f} m > {p.max_eph_m:.1f} m')
        if p.max_hdop > 0.0 and self.hdop is not None and self.hdop > p.max_hdop:
            return Decision(REJECT_HDOP, detail=f'HDOP {self.hdop:.2f} > {p.max_hdop:.2f}')

        sx, sy, vvar, known = sigmas(tpv, p)
        if self.last is not None and p.jump_accept_after > 0:
            lat0, lon0, t0 = self.last
            jump = distance_m(lat0, lon0, lat, lon)
            limit = p.max_speed * max(now - t0, 0.0) + p.jump_sigmas * max(sx, sy)
            if jump > limit:
                if self.jump_rejects < p.jump_accept_after:
                    self.jump_rejects += 1
                    return Decision(REJECT_JUMP, sx, sy, vvar, known,
                                    detail=f'salto de {jump:.1f} m > {limit:.1f} m '
                                           f'({self.jump_rejects}/{p.jump_accept_after})')
                detail = (f'salto de {jump:.1f} m aceptado tras {self.jump_rejects} '
                          'rechazos seguidos')
                self.jump_rejects = 0
                self.last = (lat, lon, now)
                return Decision(ACCEPT, sx, sy, vvar, known, detail=detail)
        self.jump_rejects = 0
        self.last = (lat, lon, now)
        return Decision(ACCEPT, sx, sy, vvar, known)
