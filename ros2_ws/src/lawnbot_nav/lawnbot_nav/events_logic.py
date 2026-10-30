"""Eventos del robot para el buzzer y la app (sin dependencias de ROS).

Los eventos son flancos: se emiten UNA vez al producirse el cambio, no mientras la condicion se mantiene.
"""
from dataclasses import dataclass
from typing import List, Optional

LIFTED = 'lifted'
CUTTING_STARTED = 'cutting_started'
NAVIGATION_STARTED = 'navigation_started'
PERIMETER_STARTED = 'perimeter_started'
LOW_BATTERY = 'low_battery'
EVENTS = (LIFTED, CUTTING_STARTED, NAVIGATION_STARTED, PERIMETER_STARTED, LOW_BATTERY)

NAVIGATION_MODES = ('auto_parallel', 'auto_random')
PERIMETER_MODE = 'auto_perimeter'


@dataclass
class EventParams:
    low_battery_threshold: float = 20.0     # % : por debajo se avisa
    low_battery_margin: float = 5.0         # % : hay que subir hasta umbral + margen para rearmar el aviso
    low_battery_samples: int = 3            # lecturas consecutivas bajo el umbral antes de avisar (ruido)
    low_battery_repeat_s: float = 0.0       # 0 = no repite; >0 = recordatorio cada tanto mientras siga baja
    lift_hold_s: float = 3.0                # un levantado cuenta como uno solo hasta pasar este tiempo sin detectarlo


class EventCore:
    def __init__(self, params: Optional[EventParams] = None):
        self.p = params or EventParams()
        self.prev_mode = None
        self.prev_cutting = None
        self.lifted_until = -float('inf')
        self.prev_lifted = None
        self.low_count = 0
        self.low_armed = True
        self.last_low = -float('inf')

    def update(self, now: float, mode: str, cutting: bool, lifted_now: bool,
               battery_pct: Optional[float]) -> List[str]:
        """mode: modo activo ('idle' si no es vigente). battery_pct: None si no hay dato valido."""
        p = self.p
        out: List[str] = []

        # El primer dato solo fija el estado de partida: reiniciar el nodo no genera eventos falsos
        if self.prev_mode is not None:
            if mode != self.prev_mode:
                if mode == PERIMETER_MODE:
                    out.append(PERIMETER_STARTED)
                elif mode in NAVIGATION_MODES and self.prev_mode not in NAVIGATION_MODES:
                    out.append(NAVIGATION_STARTED)
            if cutting and not self.prev_cutting:
                out.append(CUTTING_STARTED)
        self.prev_mode, self.prev_cutting = mode, cutting

        if lifted_now:
            self.lifted_until = now + p.lift_hold_s
        lifted = now < self.lifted_until
        if lifted and self.prev_lifted is False:
            out.append(LIFTED)
        self.prev_lifted = lifted

        # Bateria con histeresis: avisa una vez, rearma recien al pasar umbral + margen
        if battery_pct is not None:
            if battery_pct < p.low_battery_threshold:
                self.low_count += 1
                if self.low_count >= p.low_battery_samples:
                    if self.low_armed:
                        out.append(LOW_BATTERY)
                        self.low_armed, self.last_low = False, now
                    elif p.low_battery_repeat_s > 0 and now - self.last_low >= p.low_battery_repeat_s:
                        out.append(LOW_BATTERY)
                        self.last_low = now
            else:
                self.low_count = 0
                if battery_pct >= p.low_battery_threshold + p.low_battery_margin:
                    self.low_armed = True
        return out
