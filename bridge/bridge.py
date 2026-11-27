#!/usr/bin/env python3
"""Traductor MQTT <-> ROS 2 (contrato con la app sin cambios).

MQTT -> ROS:
  robot/cmd/mode     -> /robot_mode      (String, transient_local; se republica cada 1 s: heartbeat)
  robot/cmd/move     -> /cmd_vel_manual  (Twist, solo en modo manual)
  robot/cmd/geofence -> /geofence_raw    (String con el JSON recibido, transient_local)
  robot/cmd/estop    -> /estop (Bool false; SOLO liberacion, payload {"active": false}; ver estop_latch.py). Al liberar el modo queda
                        en "stop": el usuario vuelve a elegir Manual
  robot/cmd/actuator -> /cutter/enable, /trimmer/enable, /blade/force_cut (Bool, 4 Hz mientras se mantenga;
                        payload {"blade","trimmer","charging"} + "force_cut" opcional; ver actuator_relay.py)
ROS -> MQTT:
  robot/status       <- modo (/nav/active_mode), bateria (/battery/state), alerta (/nav/safety_alert),
                        blade/trimmer/force_cut pedidos, blade_running/trimmer_running reales y blade_status (texto de
                        /blade/status: por que la cuchilla corre o no; se omite si esta viejo)
  robot/position     <- posicion FUSIONADA a 1 Hz: TF map->base_footprint (ekf_global) pasada a lat/lng con el servicio
                        toLL de navsat_transform, y hdg desde el yaw de esa TF (fused_position.py). Sin TF o sin toLL:
                        el fix crudo de gpsd (hilo propio, se reconecta cada 2 s) y robot/status.position_source lo avisa
                        ("fused", "gps_raw" o "none"). Solo en simulacion, con --sim-position o LAWNBOT_SIM_POSITION=1:
                        la TF convertida con el datum del geofence_node (sim_position.py); con esa opcion no usa gpsd
  robot/status.estop <- true mientras se recuerde un e-stop: un 'lifted' (/robot/events) o un /estop true, hasta que se
                        publique /estop false (estop_latch.py)
  robot/status.alert <- tambien /robot/events (lifted, low_battery...) por unos segundos si no hay alerta de seguridad
  robot/map          <- /coverage/map (pasadas por celda y obstaculos; JSON, retenido)

La navegacion automatica ya no vive aqui: la resuelven mode_manager / twist_mux / safety_gate
(paquete lawnbot_nav). El bridge no publica en /cmd_vel.
"""
import json
import os
import signal
import sys
import threading
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # actuator_relay.py va junto a este archivo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
import paho.mqtt.client as mqtt
try:
    import gpsd
except ImportError:                                 # en simulacion (portatil) puede no estar instalado
    gpsd = None

try:
    import fused_position
    from actuator_relay import ActuatorRelay, mode_allows_actuators
    from estop_latch import EstopLatch, is_release_command
    from move_mapping import twist_from_dir
    from status_alert import pick_alert
except ImportError as exc:
    sys.exit(f"bridge.py: falta un modulo auxiliar ({exc}). bridge.py corre desde ~/lawn-mower-robot/bridge/ junto a "
             "actuator_relay.py, estop_latch.py, fused_position.py, move_mapping.py y status_alert.py: actualizar el repo "
             "en la Pi (subirpi desde la laptop) y reiniciar bridge.service.")

BROKER = "localhost"

STATUS_PERIOD_S = 1.0       # robot/status se publica cada 1 s y ante cambios
HEARTBEAT_PERIOD_S = 1.0    # /robot_mode; mode_manager pasa a idle si pasan 3 s sin recibirlo
ACTIVE_MODE_STALE_S = 3.0   # sin /nav/active_mode reciente se informa 'idle'
BATTERY_STALE_S = 5.0
BATTERY_MIN_VALID_V = 1.0   # el firmware publica 0 V si el INA228 no esta conectado
ENABLE_PERIOD_S = 0.25      # los pedidos de encendido se republican a 4 Hz; blade_controller los vence a los 2 s
BLADE_STATUS_STALE_S = 3.0  # /blade/status se publica a >= 1 Hz
MOTOR_CMD_STALE_S = 1.0     # /cutter/cmd viejo = el motor no esta corriendo

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RobotBridge(Node):
    def __init__(self, sim_position=None):
        """sim_position: None (robot real, posicion de gpsd) o (datum_lat, datum_lng) para tomarla de la TF de la simulacion."""
        super().__init__('robot_bridge')
        self.sim_datum = sim_position

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
        self.create_subscription(String, 'blade/status', self.on_blade_status, 10)
        self.create_subscription(String, 'robot/events', self.on_robot_event, 10)
        self.create_subscription(Bool, 'estop', self.on_estop, 10)
        self.create_subscription(String, 'coverage/map', self.on_coverage_map, 10)
        self.create_subscription(Float32, 'cutter/cmd', lambda m: self.on_motor_cmd('cutter', m), 10)
        self.create_subscription(Float32, 'trimmer/cmd', lambda m: self.on_motor_cmd('trimmer', m), 10)

        # Estado
        self.requested_mode = "idle"
        self.active_mode = None
        self.active_mode_time = 0.0
        self.alert = "none"
        self.event = None
        self.estop_latch = EstopLatch()         # e-stop recordado hasta /estop false (arranca sin e-stop)
        self.blade_status = None
        self.blade_status_time = 0.0
        self.event_time = 0.0
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

        self.position = fused_position.PositionSelector()
        self.position_source = None             # ultima fuente informada (para loguear solo los cambios)
        self.toll_pending = False               # hay una llamada a toLL sin respuesta
        self.toll_sent = 0.0
        self.toll_client = None
        if self.sim_datum is None:
            self.setup_fused_position()
            # GPS crudo de respaldo (hilo propio)
            self.gps_thread = threading.Thread(target=self.gps_loop, daemon=True)
            self.gps_thread.start()
            self.create_timer(fused_position.POSITION_PERIOD_S, self.publish_position)
        else:
            # Simulacion: sin gpsd; la pose sale de la TF map -> base_footprint
            import sim_position
            from tf2_ros import Buffer, TransformListener
            from lawnbot_nav.tf_utils import lookup_pose
            self.sim_position = sim_position
            self.lookup_pose = lookup_pose
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(sim_position.SIM_PERIOD_S, self.publish_sim_position)

        # MQTT loop en hilo separado
        self.mqtt_thread = threading.Thread(target=self.mqtt.loop_forever, daemon=True)
        self.mqtt_thread.start()

        if self.sim_datum is None:
            self.get_logger().info("Robot bridge iniciado")
        else:
            self.get_logger().info(f"Robot bridge iniciado (posicion SIMULADA desde la TF, datum {self.sim_datum}; sin gpsd)")

    # ── MQTT ─────────────────────────────────────────────────

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().info(f"MQTT conectado ({reason_code})")
        client.subscribe("robot/cmd/move", qos=0)
        client.subscribe("robot/cmd/mode", qos=1)
        client.subscribe("robot/cmd/geofence", qos=1)
        client.subscribe("robot/cmd/actuator", qos=1)
        client.subscribe("robot/cmd/estop", qos=1)

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
                self.get_logger().info("Pedido de actuadores ignorado (modo idle/stop o sin heartbeat): todo apagado")
            self.publish_enables()
            self.publish_status()

        elif msg.topic == "robot/cmd/estop":
            if is_release_command(payload):
                self.release_estop()
            else:
                self.get_logger().info("robot/cmd/estop ignorado: solo se acepta {\"active\": false}")

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
        # speed 0..1 -> m/s y rad/s (escalas en move_mapping.py; el giro lo dimensiona la app)
        self.send_twist(*twist_from_dir(direction, speed))

    def send_estop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)

    def release_estop(self):
        """Libera el e-stop (robot/cmd/estop). El modo queda en "stop": el robot NO vuelve solo a manual, el usuario
        elige Manual despues. El modo se publica antes que /estop false para que nunca haya un instante con el e-stop
        liberado y el modo anterior vigente."""
        self.get_logger().info("Liberacion de e-stop pedida: modo stop y /estop false")
        self.requested_mode = "stop"
        self.send_stop()
        self.publish_mode()
        self.send_estop(False)
        self.estop_latch.release()
        self.event = None                       # el 'lifted' viejo no debe seguir mostrandose como alerta
        self.publish_status()

    # ── ROS2 subscribers -> robot/status ──────────────────────

    def effective_mode(self):
        """Modo que aplico mode_manager; 'idle' si no hay mensaje reciente (heartbeat perdido)."""
        if self.active_mode is None or time.monotonic() - self.active_mode_time > ACTIVE_MODE_STALE_S:
            return "idle"
        return self.active_mode

    def actuator_mode_ok(self):
        return mode_allows_actuators(self.effective_mode())

    def publish_enables(self):
        """Republica los pedidos a 4 Hz mientras la app los mantenga y haya un modo manual o automatico vigente."""
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

    def on_blade_status(self, msg):
        self.blade_status, self.blade_status_time = msg.data, time.monotonic()

    def on_robot_event(self, msg):
        self.event, self.event_time = msg.data, time.monotonic()
        self.estop_latch.on_event(msg.data)
        self.publish_status()

    def on_estop(self, msg):
        """/estop de cualquiera (tambien el nuestro al liberar): true se recuerda, false lo libera."""
        changed = bool(msg.data) != self.estop_latch.active
        self.estop_latch.on_estop(msg.data)
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
        status["alert"] = pick_alert(self.alert, self.event, now - self.event_time)
        status["estop"] = self.estop_latch.active
        if self.sim_datum is None:
            # De donde sale robot/position: "fused" (EKF + GPS), "gps_raw" (respaldo, sin TF/toLL) o "none"
            status["position_source"] = self.position.source(now)
        # Estado de los actuadores: "blade"/"trimmer" son lo pedido y aceptado (la app arma su proximo comando a partir
        # de ellos); "*_running" es si el motor esta corriendo de verdad (puede estar apagado por una condicion)
        blade, trimmer, force_cut = self.relay.outputs(self.actuator_mode_ok())
        status["blade"] = blade
        status["trimmer"] = trimmer
        status["force_cut"] = force_cut
        status["blade_running"] = self.motor_running("cutter")
        status["trimmer_running"] = self.motor_running("trimmer")
        if self.blade_status is not None and now - self.blade_status_time <= BLADE_STATUS_STALE_S:
            status["blade_status"] = self.blade_status
        self.mqtt.publish("robot/status", json.dumps(status), qos=1)

    # ── Posicion ──────────────────────────────────────────────

    def setup_fused_position(self):
        """TF map->base_footprint y cliente de toLL (navsat_transform). Si faltan los modulos, solo GPS crudo."""
        try:
            from tf2_ros import Buffer, TransformListener
            from robot_localization.srv import ToLL
        except ImportError as exc:
            self.get_logger().warn(f"Sin posicion fusionada ({exc}): robot/position usa el GPS crudo de gpsd")
            return
        self.ToLL = ToLL
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.toll_client = self.create_client(ToLL, fused_position.TOLL_SERVICE)

    def lookup_fused_pose(self):
        """(x, y, yaw) de base_footprint en map, o None si falta la TF o esta vieja."""
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(fused_position.MAP_FRAME, fused_position.BASE_FRAME, Time())
        except Exception:
            return None
        stamp = tf.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
            if age > fused_position.TF_TIMEOUT_S:
                return None
        t = tf.transform
        return t.translation.x, t.translation.y, fused_position.yaw_from_quaternion(t.rotation)

    def publish_position(self):
        """1 Hz: pide la conversion de la pose fusionada (la respuesta llega a on_toll) y publica la mejor fuente vigente."""
        if self.toll_pending and time.monotonic() - self.toll_sent > fused_position.FUSED_STALE_S:
            self.toll_pending = False           # navsat_transform no respondio (se cayo a mitad de la llamada)
        if self.toll_client is not None and not self.toll_pending and self.toll_client.service_is_ready():
            pose = self.lookup_fused_pose()
            if pose is not None:
                x, y, yaw = pose
                req = self.ToLL.Request()
                req.map_point.x = float(x)
                req.map_point.y = float(y)
                req.map_point.z = 0.0
                self.toll_pending = True
                self.toll_sent = time.monotonic()
                self.toll_client.call_async(req).add_done_callback(lambda fut, yaw=yaw: self.on_toll(fut, yaw))
                return                          # se publica al llegar la respuesta
        self.send_position()

    def on_toll(self, future, yaw):
        self.toll_pending = False
        try:
            ll = future.result().ll_point
            self.position.on_fused(ll.latitude, ll.longitude, yaw, time.monotonic())
        except Exception:
            pass
        self.send_position()

    def send_position(self):
        source, position = self.position.payload(time.monotonic())
        msg = fused_position.source_change_message(self.position_source, source)
        if msg is not None:
            (self.get_logger().info if source == fused_position.SOURCE_FUSED else self.get_logger().warn)(msg)
            self.position_source = source
            self.publish_status()
        if position is not None:
            self.mqtt.publish("robot/position", json.dumps(position), qos=0)

    def publish_sim_position(self):
        sp = self.sim_position
        pose = self.lookup_pose(self, self.tf_buffer, sp.SIM_FRAME, sp.SIM_BASE_FRAME, sp.SIM_POSE_TIMEOUT_S)
        if pose is None:
            return
        position = sp.position_payload(pose.x, pose.y, pose.yaw, *self.sim_datum)
        self.mqtt.publish("robot/position", json.dumps(position), qos=0)

    def gps_loop(self):
        """Fix crudo de gpsd (respaldo). Si gpsd no esta (arranco antes) o se cae, reintenta cada GPSD_RETRY_S."""
        warned = False
        while True:
            try:
                gpsd.connect()
            except Exception as exc:
                if not warned:
                    self.get_logger().warn(f"gpsd no disponible ({exc}); reintento cada {fused_position.GPSD_RETRY_S:.0f} s")
                    warned = True
                time.sleep(fused_position.GPSD_RETRY_S)
                continue
            self.get_logger().info("Conectado a gpsd")
            warned = False
            while True:
                try:
                    packet = gpsd.get_current()
                except (OSError, ValueError) as exc:     # gpsd se cayo (socket roto o respuesta vacia): reconectar
                    self.get_logger().warn(f"Se perdio gpsd ({exc}); reintento cada {fused_position.GPSD_RETRY_S:.0f} s")
                    warned = True
                    break
                except Exception:                       # sin GPS activo o sin datos todavia: sigue conectado
                    packet = None
                if packet is not None and packet.mode >= 2:
                    self.position.on_raw(packet.lat, packet.lon, getattr(packet, 'track', None), time.monotonic())
                time.sleep(1)
            time.sleep(fused_position.GPSD_RETRY_S)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Sin el manejador de senales de rclpy: asi, al recibir SIGTERM (systemd) o Ctrl+C, el contexto sigue valido
    # y se alcanzan a publicar las ordenes de apagado antes de cerrar.
    sim_position = None
    if '--sim-position' in sys.argv or os.environ.get('LAWNBOT_SIM_POSITION', '').strip().lower() in ('1', 'true', 'yes', 'on'):
        try:
            import sim_position as sim
            sim_position = sim.parse_options(sys.argv, os.environ)
        except (ImportError, ValueError) as exc:
            sys.exit(f"bridge.py: {exc}")
    elif gpsd is None:
        sys.exit("bridge.py: falta el modulo gpsd (pip install gpsd-py3). Para simulacion sin GPS use --sim-position.")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    bridge = RobotBridge(sim_position)
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            bridge.requested_mode = "idle"
            bridge.relay.clear()
            for _ in range(3):                  # cuchilla/trimmer apagados, parada y modo idle
                bridge.publish_enables()
                bridge.send_stop()
                bridge.publish_mode()
                time.sleep(0.02)
        except Exception:
            pass
        bridge.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
