"""Resolucion del modo activo (sin dependencias de ROS)."""

DEFAULT_VALID_MODES = (
    'idle', 'stop', 'manual', 'auto_parallel', 'auto_random', 'auto_perimeter')


def resolve_mode(requested, heartbeat_age, timeout, valid_modes=DEFAULT_VALID_MODES):
    """Modo efectivo. Cae a 'idle' si el modo es desconocido o no hay heartbeat reciente.

    heartbeat_age es None si nunca llego un /robot_mode.
    """
    if heartbeat_age is None or heartbeat_age > timeout:
        return 'idle'
    if requested not in valid_modes:
        return 'idle'
    return requested
