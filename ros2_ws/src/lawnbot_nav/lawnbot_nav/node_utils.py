"""Arranque y apagado comunes: los nodos que mueven el robot publican cero al cerrar."""
import signal

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

# Cada cuanto despierta el lazo aunque no llegue nada. Hace falta porque el manejador de senales es de Python y
# solo corre cuando la espera de rclpy (en C) devuelve: un nodo sin timers activos (sonar_filter, o random_walk /
# perimeter inactivos) se quedaba dormido para siempre y no se enteraba de Ctrl+C ni de SIGTERM.
WAKE_S = 0.5


def _stop(signum, frame):
    raise KeyboardInterrupt


def spin_with_cleanup(make_node, args=None):
    """Crea el nodo, lo hace girar y, al recibir Ctrl+C o SIGTERM, llama a node.shutdown_cleanup().

    Se desactiva el manejador de senales de rclpy para poder publicar antes de cerrar el contexto.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _stop)
    node = None
    try:
        node = make_node()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=WAKE_S)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            cleanup = getattr(node, 'shutdown_cleanup', None)
            if cleanup is not None:
                try:
                    cleanup()
                except Exception:
                    pass
            node.destroy_node()
        rclpy.try_shutdown()
