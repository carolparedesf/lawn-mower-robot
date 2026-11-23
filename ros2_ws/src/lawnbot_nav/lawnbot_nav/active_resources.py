"""Suscripciones rapidas, TF y timer que existen solo mientras un nodo de algoritmo esta activo.

Por que: en la Raspberry Pi 4 (Humble) cada mensaje que recibe un nodo rclpy cuesta ~2-3 ms de CPU aunque el
callback no haga nada (medido: 5 bumpers a 5 Hz = +4,6 % de un nucleo, /tf a 25 Hz = +10 %, un timer de 20 Hz
= +2,7 %). Que el callback "retorne enseguida" casi no ahorra; lo que ahorra es no recibir. Un algoritmo inactivo
no necesita sonares, bumpers, distancia al cerco ni TF: se suscribe al activarse y se desuscribe al terminar.

Al activarse espera a que llegue al menos un mensaje de cada entrada requerida (y la TF, si se pide) antes de
dar el nodo por listo, como mucho max_warmup_s. Asi el algoritmo arranca con los mismos datos que tenia cuando
estaba suscripto siempre, en vez de arrancar sin sensores (alerta sensor_timeout) o sin pose.
"""
import time
from typing import Callable, List, NamedTuple, Optional

from tf2_ros import TransformListener


class Sub(NamedTuple):
    msg_type: type
    topic: str
    callback: Callable
    qos: object
    required: bool = True          # el nodo no se da por listo hasta recibir uno (o vencer max_warmup_s)


class ActiveResources:
    def __init__(self, node, period: float, tick: Callable, subs: List[Sub], tf_buffer=None,
                 tf_ready: Optional[Callable[[], bool]] = None, max_warmup_s: float = 1.0):
        self._node = node
        self._specs = list(subs)
        self._tf_buffer = tf_buffer
        self._tf_ready = tf_ready
        self._max_warmup = max_warmup_s
        self._handles = []
        self._listener = None
        self._seen = set()
        self._started_at = None
        self._warm = False
        self._timer = node.create_timer(period, tick)
        self._timer.cancel()                      # arranca inactivo

    @property
    def running(self) -> bool:
        return self._started_at is not None

    def start(self) -> None:
        """Se suscribe, crea la TF y arranca el timer. Idempotente."""
        if self.running:
            return
        self._started_at = time.monotonic()
        self._seen = set()
        self._warm = False
        for i, spec in enumerate(self._specs):
            self._handles.append(self._node.create_subscription(
                spec.msg_type, spec.topic, self._mark(i, spec.callback), spec.qos))
        if self._tf_buffer is not None:
            self._listener = TransformListener(self._tf_buffer, self._node)
        self._timer.reset()

    def stop(self) -> None:
        """Cancela el timer, se desuscribe y suelta la TF. Idempotente."""
        if not self.running:
            return
        self._timer.cancel()
        for handle in self._handles:
            self._node.destroy_subscription(handle)
        self._handles = []
        if self._listener is not None:
            self._listener.unregister()
            self._listener = None
        self._started_at = None

    def ready(self, now: float) -> bool:
        """True cuando ya llegaron las entradas requeridas (y la TF), o paso max_warmup_s desde start()."""
        if not self.running:
            return False
        if not self._warm:
            required = all(i in self._seen for i, s in enumerate(self._specs) if s.required)
            tf_ok = self._tf_ready is None or self._tf_ready()
            self._warm = (required and tf_ok) or now - self._started_at >= self._max_warmup
        return self._warm

    def _mark(self, index, callback):
        def on_msg(msg):
            self._seen.add(index)
            callback(msg)
        return on_msg
