#!/usr/bin/env python3
"""Traductor MQTT <-> ROS 2 (contrato con la app sin cambios).

MQTT -> ROS:
  robot/cmd/mode     -> /robot_mode      (String, transient_local; se republica cada 1 s: heartbeat)
  robot/cmd/move     -> /cmd_vel_manual  (Twist, solo en modo manual)
  robot/cmd/geofence -> /geofence_raw    (String con el JSON recibido, transient_local)
  robot/cmd/actuator -> /cutter/enable, /trimmer/enable, /blade/force_cut (Bool, 4 Hz mientras se mantenga;
                        payload {"blade","trimmer","charging"} + "force_cut" opcional; ver actuator_relay.py)
ROS -> MQTT:
  robot/status       <- modo (/nav/active_mode), bateria (/battery/state), alerta (/nav/safety_alert),
                        blade/trimmer/force_cut pedidos y blade_running/trimmer_running reales
  robot/position     <- gpsd (hilo propio, sin cambios)
  robot/map          <- /coverage/map (pasadas por celda y obstaculos; JSON, retenido)

La navegacion automatica ya no vive aqui: la resuelven mode_manager / twist_mux / safety_gate
(paquete lawnbot_nav). El bridge no publica en /cmd_vel.
"""
import json
import os
import sys
import threading
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # actuator_relay.py va junto a este archivo
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
import paho.mqtt.client as mqtt
import gpsd

from actuator_relay import ActuatorRelay

BROKER = "localhost"

STATUS_PERIOD_S = 1.0       # robot/status se publica cada 1 s y ante cambios
HEARTBEAT_PERIOD_S = 1.0    # /robot_mode; mode_manager pasa a idle si pasan 3 s sin recibirlo
ACTIVE_MODE_STALE_S = 3.0   # sin /nav/active_mode reciente se informa 'idle'
BATTERY_STALE_S = 5.0
BATTERY_MIN_VALID_V = 1.0   # el firmware publica 0 V si el INA228 no esta conectado
ENABLE_PERIOD_S = 0.25      # los pedidos de encendido se republican a 4 Hz; blade_controller los vence a los 2 s
MOTOR_CMD_STALE_S = 1.0     # /cutter/cmd viejo = el motor no esta corriendo

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # Publishers ROS2
        self.cmd_vel_manual_pub = self.create_publisher(Twist, 'cmd_vel_manual', 10)
        self.estop_pub   = self.create_publisher(Bool, 'estop', 10)
        self.mode_pub    = self.create_publisher(String, 'robot_mode', LATCHED)
        self.geofence_pub = self.create_publisher(String, 'geofence_raw', LATCHED)
        # Pedidos de encendido: volatiles a proposito (un reinicio nunca deja nada encendido)
        self.cutter_enable_pub = self.create_publisher(Bool, 'cutter/enable', 10)
        self.trimmer_enable_pub = self.create_publisher(Bool, 'trimmer/enable', 10)
        self.force_cut_pub = self.create_publisher(Bool, 'blade/force_cut', 10)

        # Subscribers ROS2
        self.create_subscription(String, 'nav/active_mode', self.on_active_mode, LATCHED)
        self.create_subscription(String, 'nav/safety_alert', self.on_safety_alert, LATCHED)
        self.create_subscription(BatteryState, 'battery/state', self.on_battery,
                                 qos_profile_sensor_data)
        self.create_subscription(String, 'coverage/map', self.on_coverage_map, 10)
        self.create_subscription(Float32, 'cutter/cmd', lambda m: self.on_motor_cmd('cutter', m), 10)
        self.create_subscription(Float32, 'trimmer/cmd', lambda m: self.on_motor_cmd('trimmer', m), 10)

        # Estado
        self.requested_mode = "idle"
        self.active_mode = None
        self.active_mode_time = 0.0
        self.alert = "none"
        self.battery_pct = None
        self.battery_time = 0.0
        self.geofence_points = []
        self.relay = ActuatorRelay()            # arranca apagado
        self.motor_cmd = {}                     # motor -> (duty publicado, instante)

        # MQTT
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="robot_pi")
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.connect(BROKER, 1883, keepalive=60)

        # Heartbeat del modo y estado periodico
        self.create_timer(HEARTBEAT_PERIOD_S, self.publish_mode)
        self.create_timer(STATUS_PERIOD_S, self.publish_status)
        self.create_timer(ENABLE_PERIOD_S, self.publish_enables)
        self.publish_mode()

        # GPS thread
        self.gps_thread = threading.Thread(target=self.gps_loop, daemon=True)
        self.gps_thread.start()

        # MQTT loop en hilo separado
        self.mqtt_thread = threading.Thread(target=self.mqtt.loop_forever, daemon=True)
        self.mqtt_thread.start()

        self.get_logger().info("Robot bridge iniciado")

    # ── MQTT ─────────────────────────────────────────────────

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().info(f"MQTT conectado ({reason_code})")
        client.subscribe("robot/cmd/move", qos=0)
        client.subscribe("robot/cmd/mode", qos=1)
        client.subscribe("robot/cmd/geofence", qos=1)
        client.subscribe("robot/cmd/actuator", qos=1)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except Exception:
            return

        if msg.topic == "robot/cmd/move" and self.requested_mode == "manual":
            self.send_twist_from_dir(
                payload.get("dir", "stop"),
                payload.get("speed", 0.0)
            )

        elif msg.topic == "robot/cmd/mode":
            self.requested_mode = payload.get("mode", "idle")
            self.get_logger().info(f"Modo pedido: {self.requested_mode}")
            self.send_stop()
            self.publish_mode()

        elif msg.topic == "robot/cmd/actuator":
            accepted = self.relay.on_command(payload, self.actuator_mode_ok())
            if not accepted:
                self.get_logger().info("Pedido de actuadores ignorado (sin modo automatico vigente): todo apagado")
            self.publish_enables()
            self.publish_status()

        elif msg.topic == "robot/cmd/geofence":
            self.geofence_points = payload.get("points", [])
            self.get_logger().info(f"Geofence: {len(self.geofence_points)} puntos")
            self.geofence_pub.publish(String(data=msg.payload.decode()))

    # ── ROS2 publishers ──────────────────────────────────────

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.requested_mode))

    def send_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_manual_pub.publish(msg)

    def send_stop(self):
        self.send_twist(0.0, 0.0)

    def send_twist_from_dir(self, direction, speed):
        speed = max(0.0, min(1.0, float(speed)))
        # Convertir speed (0-1) a m/s (max 0.5 m/s)
        v = speed * 0.5
        if direction == "fwd":
            self.send_twist(v, 0.0)
        elif direction == "back":
            self.send_twist(-v, 0.0)
        elif direction == "left":
            self.send_twist(0.0, v * 2)
        elif direction == "right":
            self.send_twist(0.0, -v * 2)
        else:
            self.send_stop()

    def send_estop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)

    # ── ROS2 subscribers -> robot/status ──────────────────────

    def effective_mode(self):
        """Modo que aplico mode_manager; 'idle' si no hay mensaje reciente (heartbeat perdido)."""
        if self.active_mode is None or time.monotonic() - self.active_mode_time > ACTIVE_MODE_STALE_S:
            return "idle"
        return self.active_mode

    def actuator_mode_ok(self):
        return self.effective_mode().startswith("auto_")

    def publish_enables(self):
        """Republica los pedidos a 4 Hz mientras la app los mantenga y haya un modo automatico vigente."""
        blade, trimmer, force_cut = self.relay.outputs(self.actuator_mode_ok())
        self.cutter_enable_pub.publish(Bool(data=blade))
        self.trimmer_enable_pub.publish(Bool(data=trimmer))
        self.force_cut_pub.publish(Bool(data=force_cut))

    def on_motor_cmd(self, name, msg):
        self.motor_cmd[name] = (float(msg.data), time.monotonic())

    def motor_running(self, name):
        duty, t = self.motor_cmd.get(name, (0.0, 0.0))
        return duty > 0.0 and time.monotonic() - t <= MOTOR_CMD_STALE_S

    def on_active_mode(self, msg):
        changed = msg.data != self.active_mode
        self.active_mode = msg.data
        self.active_mode_time = time.monotonic()
        if changed:
            self.publish_status()

    def on_safety_alert(self, msg):
        changed = msg.data != self.alert
        self.alert = msg.data
        if changed:
            self.publish_status()

    def on_battery(self, msg):
        pct = msg.percentage * 100.0
        if msg.voltage > BATTERY_MIN_VALID_V and 0.0 <= pct <= 100.0:
            self.battery_pct = pct
            self.battery_time = time.monotonic()

    def on_coverage_map(self, msg):
        # Mapa de cobertura para la app. Retenido: quien se suscriba despues recibe el ultimo de inmediato.
        self.mqtt.publish("robot/map", msg.data, qos=0, retain=True)

    def publish_status(self):
        now = time.monotonic()
        # El modo informado es el que aplico mode_manager, no solo el pedido por la app
        if self.active_mode is None or now - self.active_mode_time > ACTIVE_MODE_STALE_S:
            mode = "idle"
        else:
            mode = self.active_mode

        status = {"mode": mode}
        # Sin dato de bateria valido no se envia la clave (la app muestra 0)
        if self.battery_pct is not None and now - self.battery_time <= BATTERY_STALE_S:
            status["battery"] = int(round(self.battery_pct))
        status["alert"] = self.alert
        # Estado de los actuadores: "blade"/"trimmer" son lo pedido y aceptado (la app arma su proximo comando a partir
        # de ellos); "*_running" es si el motor esta corriendo de verdad (puede estar apagado por una condicion)
        blade, trimmer, force_cut = self.relay.outputs(self.actuator_mode_ok())
        status["blade"] = blade
        status["trimmer"] = trimmer
        status["force_cut"] = force_cut
        status["blade_running"] = self.motor_running("cutter")
        status["trimmer_running"] = self.motor_running("trimmer")
        self.mqtt.publish("robot/status", json.dumps(status), qos=1)

    # ── GPS ───────────────────────────────────────────────────

    def gps_loop(self):
        gpsd.connect()
        while True:
            try:
                packet = gpsd.get_current()
                if packet.mode >= 2:
                    position = {
                        "lat": packet.lat,
                        "lng": packet.lon,
                        "hdg": getattr(packet, 'track', 0.0) or 0.0
                    }
                    self.mqtt.publish("robot/position", json.dumps(position), qos=0)
            except Exception:
                pass
            time.sleep(1)


def main():
    rclpy.init()
    bridge = RobotBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.requested_mode = "idle"
        bridge.relay.clear()
        bridge.publish_enables()
        bridge.send_stop()
        bridge.publish_mode()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
