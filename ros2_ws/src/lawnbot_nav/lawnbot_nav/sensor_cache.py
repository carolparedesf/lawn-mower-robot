"""Ultimas lecturas de sonares y bumpers con su antiguedad (sin ROS)."""
import math
from typing import Dict, Iterable, Optional, Tuple

BUMPER_FRONT = ('bumper/B1_front_left', 'bumper/B2_front_right')
BUMPER_SIDE_LEFT = 'bumper/B3_left'
BUMPER_SIDE_RIGHT = 'bumper/B4_right'
BUMPER_BACK = 'bumper/B5_back'
SONARS = ('left', 'center', 'right')


class SensorCache:
    def __init__(self):
        self._data: Dict[str, Tuple[float, float]] = {}

    def set(self, name: str, value: float, now: float) -> None:
        self._data[name] = (float(value), now)

    def get(self, name: str, now: float, timeout: float) -> Optional[float]:
        """Ultimo valor, o None si nunca llego o es mas viejo que `timeout`."""
        entry = self._data.get(name)
        if entry is None or now - entry[1] > timeout:
            return None
        value = entry[0]
        return None if math.isnan(value) else value

    def fresh(self, names: Iterable[str], now: float, timeout: float) -> bool:
        return all(self.get(n, now, timeout) is not None for n in names)

    def pressed(self, names, now: float, timeout: float) -> bool:
        """Alguno de los bumpers dados esta presionado y con dato vigente."""
        names = [names] if isinstance(names, str) else list(names)
        return any((self.get(n, now, timeout) or 0.0) > 0.5 for n in names)
