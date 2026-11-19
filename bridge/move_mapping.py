"""robot/cmd/move {"dir","speed"} -> (lineal m/s, angular rad/s). Sin ROS, para poder probarlo solo.

El formato MQTT no cambia: speed sigue siendo 0..1. Lo que cambia es la escala:
  fwd/back  : lineal  = speed * MAX_LINEAR_MPS   (0,5 m/s a speed 1, igual que antes)
  left/right: angular = speed * MAX_ANGULAR_RADS (1,5 rad/s a speed 1; antes era 1,0)
La app decide el speed de cada giro (minimo 0,8 rad/s en cualquier nivel de velocidad), asi que
si cambia MAX_ANGULAR_RADS hay que cambiar turnMaxRadS en la app (joystick_widget.dart).
"""

MAX_LINEAR_MPS = 0.5
MAX_ANGULAR_RADS = 1.5


def twist_from_dir(direction, speed, max_linear=MAX_LINEAR_MPS, max_angular=MAX_ANGULAR_RADS):
    """Devuelve (linear_x, angular_z). Direccion desconocida o speed invalido -> (0, 0)."""
    try:
        speed = max(0.0, min(1.0, float(speed)))
    except (TypeError, ValueError):
        return 0.0, 0.0
    v = speed * max_linear
    w = speed * max_angular
    if direction == "fwd":
        return v, 0.0
    if direction == "back":
        return -v, 0.0
    if direction == "left":
        return 0.0, w
    if direction == "right":
        return 0.0, -w
    return 0.0, 0.0
