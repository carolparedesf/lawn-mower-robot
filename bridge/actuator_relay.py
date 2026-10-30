"""Pedidos de la app para los motores de corte (sin ROS ni MQTT, para poder probarlos aparte).

Payload real de la app (robot/cmd/actuator, QoS 1, uno por cada cambio; no se refresca):
  {"blade": bool, "trimmer": bool, "charging": bool}
`force_cut` (bool) es opcional: la app todavia no lo envia. Falta o no-bool = false. `charging` no se atiende.

Reglas: arranca siempre apagado; solo se aceptan pedidos con un modo automatico vigente; al salir de uno
(idle, stop, manual, o sin heartbeat del modo) se borra todo y la app tiene que volver a pedir.
"""
from typing import Tuple


class ActuatorRelay:
    def __init__(self):
        self.blade = False
        self.trimmer = False
        self.force_cut = False

    def clear(self) -> None:
        self.blade = self.trimmer = self.force_cut = False

    @staticmethod
    def _flag(payload: dict, key: str) -> bool:
        return payload.get(key) is True          # solo el booleano true; "true", 1 o null no cuentan

    def on_command(self, payload, mode_allowed: bool) -> bool:
        """Aplica el payload de la app. Devuelve True si se acepto (False: ignorado y todo apagado)."""
        if not isinstance(payload, dict):
            return False
        if not mode_allowed:
            self.clear()
            return False
        self.blade = self._flag(payload, 'blade')
        self.trimmer = self._flag(payload, 'trimmer')
        self.force_cut = self._flag(payload, 'force_cut')
        return True

    def outputs(self, mode_allowed: bool) -> Tuple[bool, bool, bool]:
        """(cutter_enable, trimmer_enable, force_cut) a publicar; sin modo automatico vigente, todo false."""
        if not mode_allowed:
            self.clear()
        return self.blade, self.trimmer, self.force_cut
