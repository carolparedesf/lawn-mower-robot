#!/usr/bin/env python3
"""
Nodo ROS2 para el TF-mini Plus 
Publica sensor_msgs/Range en el topic 'tfmini/range'.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
import serial
import math


class TFMiniNode(Node):
    def __init__(self):
        super().__init__('tfmini_node')

        # --- Parámetros configurables ---
        self.declare_parameter('port', '/dev/tfmini')  # regla udev: docs/setup_usb.md
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'tf_mini_link')
        self.declare_parameter('min_range', 0.1)   # metros
        self.declare_parameter('max_range', 12.0)  # metros
        self.declare_parameter('field_of_view', 0.04)  # rad (~2.3°)
        self.declare_parameter('min_strength', 100)  # descarta lecturas de señal débil
        self.declare_parameter('publish_rate_hz', 5.0)  #(5Hz)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.fov = self.get_parameter('field_of_view').value
        self.min_strength = self.get_parameter('min_strength').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher_ = self.create_publisher(Range, 'tfmini/range', qos)

        # --- Apertura del puerto serie ---
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"TF-mini Plus conectado en {port} @ {baud} baudios")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir {port}: {e}")
            raise

        # Buffer para reconstruir frames si llegan bytes parciales
        self._buf = bytearray()

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self._last_valid_count = 0
        self._last_invalid_count = 0

    def timer_callback(self):
        # Leer todo lo disponible en el buffer serie
        n = self.ser.in_waiting
        if n > 0:
            self._buf.extend(self.ser.read(n))

        # El sensor manda frames a ~100Hz pero publicamos a 5Hz (consistente con sonars),
        # así que en cada ciclo puede haber varios frames acumulados en el buffer.
        # Nos quedamos solo con el más reciente para minimizar latencia; los anteriores
        # se descartan (ya están "viejos" para cuando llega el próximo ciclo del timer).
        latest_frame = None
        while True:
            frame = self._extract_frame()
            if frame is None:
                break
            latest_frame = frame

        if latest_frame is not None:
            self._publish_from_frame(latest_frame)

    def _extract_frame(self):
        """Busca y extrae un frame válido de 9 bytes (header 0x59 0x59) del buffer."""
        buf = self._buf

        # Buscar el header
        idx = buf.find(b'\x59\x59')
        if idx == -1:
            # No hay header en el buffer; descartar todo salvo el último byte
            # (por si es un 0x59 suelto que podría ser el primer byte del header siguiente)
            if len(buf) > 1:
                del buf[:-1]
            return None

        # Descartar basura antes del header
        if idx > 0:
            del buf[:idx]

        # Verificar si ya tenemos el frame completo (9 bytes)
        if len(buf) < 9:
            return None

        frame = bytes(buf[:9])
        del buf[:9]

        # Validar checksum
        checksum_calc = sum(frame[0:8]) & 0xFF
        if checksum_calc != frame[8]:
            self._last_invalid_count += 1
            return None

        return frame

    def _publish_from_frame(self, frame):
        dist_cm = frame[2] | (frame[3] << 8)
        strength = frame[4] | (frame[5] << 8)

        # Descartar lecturas de señal débil o fuera de rango físico del sensor
        if strength < self.min_strength:
            self.get_logger().debug(f"Lectura descartada, señal débil: {strength}")
            return

        dist_m = dist_cm / 100.0

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = dist_m

        self.publisher_.publish(msg)
        self._last_valid_count += 1

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TFMiniNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
