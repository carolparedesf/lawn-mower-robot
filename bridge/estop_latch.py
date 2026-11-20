"""Memoria del e-stop para robot/status (sin ROS).

El firmware se frena solo al levantar el robot y el evento 'lifted' (/robot/events) dura ~5 s en
robot/status.alert; sin esta memoria la app no sabria que el robot sigue frenado. Se recuerda un 'lifted' o un
/estop true hasta que se publique /estop false (la liberacion desde la app o un `ros2 topic pub /estop ...`).
"""


class EstopLatch:
    def __init__(self):
        self.active = False

    def on_event(self, event):
        """Ultimo /robot/events. Solo 'lifted' activa el e-stop."""
        if event == 'lifted':
            self.active = True

    def on_estop(self, data):
        """/estop (Bool): true lo activa, false lo libera."""
        self.active = bool(data)

    def release(self):
        self.active = False


def is_release_command(payload):
    """robot/cmd/estop: solo se acepta la liberacion, {"active": false}. Cualquier otra cosa (active true, sin la
    clave, otro tipo, payload que no es un objeto) se ignora: la app no activa el e-stop por aqui (para eso esta STOP)."""
    return isinstance(payload, dict) and payload.get('active') is False
