"""Filtro de lecturas de sonar (sin dependencias de ROS).

Los sonares saltan (rebotes, lecturas espurias, perdidas de eco) y el safety_gate escala la velocidad con la
distancia, asi que el robot tiembla. Este filtro, por sonar:
  - descarta lecturas invalidas (NaN, inf, <= 0 o menores que min_range): no se mezclan ni se publican;
  - separa las lecturas de max_range ("sin eco"): no se mezclan con las validas. Solo tras `max_range_run`
    seguidas (3) se publica max_range (y se borra la historia: el camino esta libre); antes se sigue
    publicando la mediana vigente;
  - rechaza saltos de mas de `max_jump` respecto de la mediana, salvo que dos lecturas seguidas coincidan en el
    nuevo nivel (un obstaculo real que aparece de golpe se acepta a la segunda lectura);
  - publica la mediana de las ultimas `window` (5) lecturas validas. Para no frenar tarde ante un obstaculo
    real, si las dos lecturas mas recientes estan por debajo de la mediana en mas de `approach_margin` (acercamiento
    sostenido, no ruido) se publica la mayor de las dos: un pico aislado hacia abajo no pasa, y una aproximacion real
    solo tiene una lectura de retraso.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional


@dataclass
class SonarFilterParams:
    window: int = 5
    max_jump: float = 0.5            # m: salto respecto de la mediana que se considera sospechoso
    max_range: float = 1.5           # m: el sonar reporta esto (o mas) cuando no hay eco
    max_range_tolerance: float = 0.02
    max_range_run: int = 3           # lecturas de max_range seguidas para creerle
    min_range: float = 0.02          # m: menos que esto no es una medicion creible
    approach_margin: float = 0.03    # m: las dos ultimas lecturas deben estar al menos esto bajo la mediana para adelantarla


def median(values) -> float:
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class SonarFilter:
    def __init__(self, params: Optional[SonarFilterParams] = None):
        self.p = params or SonarFilterParams()
        self.hist: Deque[float] = deque(maxlen=max(1, self.p.window))
        self.max_run = 0
        self.pending: Optional[float] = None       # lectura valida rechazada por salto, a la espera de confirmacion
        self.ref: Optional[float] = None           # referencia (max_range) tras borrar la historia por 'camino libre'

    def reset(self):
        self.hist.clear()
        self.max_run = 0
        self.pending = None
        self.ref = None

    def is_valid(self, r: float) -> bool:
        return r is not None and math.isfinite(r) and r > 0.0 and r >= self.p.min_range

    def is_max_range(self, r: float) -> bool:
        return r >= self.p.max_range - self.p.max_range_tolerance

    def current(self) -> Optional[float]:
        """Valor a publicar con la historia actual, o None si no hay."""
        if not self.hist:
            return None
        med = median(self.hist)
        if len(self.hist) >= 3:
            a, b = self.hist[-1], self.hist[-2]
            if a < med - self.p.approach_margin and b < med - self.p.approach_margin:
                return max(a, b)
        return med

    def update(self, r: float) -> Optional[float]:
        """Procesa una lectura. Devuelve el valor a publicar, o None si no hay que publicar nada."""
        p = self.p
        if not self.is_valid(r):
            return None                              # invalida: ni se mezcla ni se publica (el consumidor la vera vieja)
        if self.is_max_range(r):
            self.max_run += 1
            self.pending = None
            if self.max_run >= p.max_range_run:
                self.hist.clear()                    # tres "sin eco" seguidos: el camino esta libre
                self.ref = p.max_range               # y una lectura baja aislada ya no se acepta sin confirmacion
                return p.max_range
            cur = self.current()                     # todavia no se le cree: se mantiene lo ultimo bueno
            return cur if cur is not None else self.ref
        self.max_run = 0

        base = median(self.hist) if self.hist else self.ref
        if base is not None:
            if abs(r - base) > p.max_jump:
                if self.pending is not None and abs(r - self.pending) <= p.max_jump:
                    self.hist.clear()                # dos lecturas seguidas en el nuevo nivel: es real
                    self.hist.append(self.pending)
                    self.pending = None
                else:
                    self.pending = r                 # posible espuria: se espera confirmacion
                    return self.current() if self.hist else self.ref
            else:
                self.pending = None
        self.ref = None
        self.hist.append(r)
        return self.current()
