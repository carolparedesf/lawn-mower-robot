"""Capa minima de ROS 2 falsa para ejercitar los nodos de lawnbot_nav SIN ROS instalado.

Solo sirve para detectar errores de nombres y de flujo antes de llegar al robot: no reemplaza
la prueba con ROS real. Reemplaza modulos en sys.modules, asi que NO usarla donde ROS 2 este
instalado ni importarla desde los tests que corre colcon.
"""
import math
import sys
import time as _time
import types

CLOCK = {'ns': 0}          # reloj de ROS falso
MONO = {'t': 1000.0}       # time.monotonic falso
TF = {'poses': {}}         # (frame, hijo) -> (x, y, yaw, stamp_ns) o ausente


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


# ── mensajes ─────────────────────────────────────────────────
class Vec:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Quat:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Header:
    def __init__(self):
        self.stamp = Msg(sec=0, nanosec=0)
        self.frame_id = ''
        self.stamp.to_msg = None


class Twist:
    def __init__(self):
        self.linear = Vec()
        self.angular = Vec()


class Bool:
    def __init__(self, data=False):
        self.data = data


class Float32:
    def __init__(self, data=0.0):
        self.data = data


class String:
    def __init__(self, data=''):
        self.data = data


class Point32:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Polygon:
    def __init__(self):
        self.points = []


class PolygonStamped:
    def __init__(self):
        self.header = Header()
        self.polygon = Polygon()


class Pose:
    def __init__(self):
        self.position = Vec()
        self.orientation = Quat()


class PoseStamped:
    def __init__(self, x=0.0, y=0.0):
        self.header = Header()
        self.pose = Pose()
        self.pose.position.x, self.pose.position.y = x, y


class Path:
    def __init__(self, frame_id='map', points=()):
        self.header = Header()
        self.header.frame_id = frame_id
        self.poses = [PoseStamped(x, y) for x, y in points]


class Marker:
    ADD, DELETEALL, LINE_STRIP, SPHERE_LIST = 0, 3, 4, 7
    CYLINDER, LINE_LIST = 3, 5

    def __init__(self):
        self.header = Header()
        self.ns, self.id, self.type, self.action = '', 0, 0, 0
        self.pose = Pose()
        self.scale = Vec()
        self.color = Msg(r=0.0, g=0.0, b=0.0, a=0.0)
        self.points = []


class MarkerArray:
    def __init__(self):
        self.markers = []


class BatteryState:
    def __init__(self):
        self.header = Header()
        self.percentage = 0.0
        self.voltage = 0.0


class Int32:
    def __init__(self, data=0):
        self.data = data


class PoseArray:
    def __init__(self):
        self.header = Header()
        self.poses = []


class Imu:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.header = Header()
        self.angular_velocity = Vec()
        self.orientation = Quat()
        self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w = x, y, z, w


class Range:
    ULTRASOUND = 0

    def __init__(self, range_m=1.5):
        self.header = Header()
        self.range = range_m
        self.radiation_type, self.field_of_view, self.min_range, self.max_range = 0, 0.0, 0.0, 0.0


class Transform:
    def __init__(self):
        self.translation = Vec()
        self.rotation = Quat()


class TransformStamped:
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ''
        self.transform = Transform()


# ── tiempo ───────────────────────────────────────────────────
class Duration:
    def __init__(self, ns):
        self.nanoseconds = ns


class Time:
    def __init__(self, ns=0):
        self.ns = ns

    @staticmethod
    def from_msg(m):
        return Time(m.sec * 10**9 + m.nanosec)

    def __sub__(self, other):
        return Duration(self.ns - other.ns)

    def to_msg(self):
        return Msg(sec=self.ns // 10**9, nanosec=self.ns % 10**9)


class Clock:
    def now(self):
        return Time(CLOCK['ns'])


# ── nodo, publicadores, servicios ────────────────────────────
class Logger:
    def __init__(self):
        self.lines = []

    def info(self, s, **kw):
        self.lines.append(('INFO', s))

    def warn(self, s, **kw):
        self.lines.append(('WARN', s))

    def error(self, s, **kw):
        self.lines.append(('ERROR', s))

    def has(self, text, level=None):
        return any(text in s and (level is None or lv == level) for lv, s in self.lines)


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)

    @property
    def last(self):
        return self.sent[-1] if self.sent else None


class Future:
    def __init__(self):
        self._result, self._callbacks, self.done = None, [], False

    def add_done_callback(self, cb):
        if self.done:
            cb(self)
        else:
            self._callbacks.append(cb)

    def result(self):
        if isinstance(self._result, Exception):
            raise self._result
        return self._result

    def resolve(self, value):
        self._result, self.done = value, True
        for cb in self._callbacks:
            cb(self)


class Client:
    """Cliente de servicio falso: las llamadas quedan pendientes hasta resolverlas a mano."""

    def __init__(self):
        self.ready = False
        self.calls = []        # [(request, future)]

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        fut = Future()
        self.calls.append((request, fut))
        return fut


class Subs(dict):
    """Callbacks por topic. Un topic al que el nodo no esta suscripto (p. ej. un algoritmo inactivo, que se
    desuscribe para ahorrar CPU) descarta el mensaje, como en ROS."""
    def __missing__(self, topic):
        return lambda msg: None


class FakeTimer:
    def __init__(self, period, cb):
        self.period, self.cb, self.canceled = period, cb, False

    def cancel(self):
        self.canceled = True

    def reset(self):
        self.canceled = False

    def fire(self):
        if not self.canceled:          # un timer cancelado no dispara (rclpy)
            self.cb()


class NodeBase:
    PARAM_OVERRIDES = {}

    def __init__(self, name):
        self.name = name
        self._params, self.pubs, self.subs, self.timers, self.clients = {}, {}, Subs(), [], {}
        self._timer_objs = []
        self.services = {}
        self._logger, self._clock = Logger(), Clock()

    def declare_parameter(self, name, default=None, *a):
        self._params[name] = self.PARAM_OVERRIDES.get(name, default)
        return Msg(value=self._params[name])

    def get_parameter(self, name):
        return Msg(value=self._params[name])

    def create_publisher(self, typ, topic, qos):
        pub = Pub()
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, typ, topic, cb, qos, raw=False):
        self.subs[topic] = cb
        return (topic, cb)

    def destroy_subscription(self, handle):
        topic, cb = handle
        if self.subs.get(topic) is cb:
            del self.subs[topic]

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        t = FakeTimer(period, cb)
        self._timer_objs.append(t)
        return t

    def create_service(self, typ, name, cb):
        self.services[name] = cb

    def create_client(self, typ, name):
        client = Client()
        self.clients[name] = client
        return client

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass

    def timer(self, period):
        """Callback del primer timer con ese periodo; no hace nada mientras el timer este cancelado."""
        t = [t for t in self._timer_objs if abs(t.period - period) < 1e-9][0]
        return t.fire


# ── tf2 ──────────────────────────────────────────────────────
class TransformException(Exception):
    pass


class Buffer:
    def lookup_transform(self, target, source, t):
        key = (target, source)
        if key not in TF['poses']:
            raise TransformException('sin tf')
        x, y, yaw, stamp = TF['poses'][key]
        tf = TransformStamped()
        tf.header.stamp = Msg(sec=stamp // 10**9, nanosec=stamp % 10**9)
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return tf


class TransformListener:
    def __init__(self, buffer, node, **kw):
        self.registered = True

    def unregister(self):
        self.registered = False


class Broadcaster:
    def __init__(self, node):
        pass

    def sendTransform(self, tf):
        TF['poses'][(tf.header.frame_id, tf.child_frame_id)] = (
            tf.transform.translation.x, tf.transform.translation.y,
            2 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w), CLOCK['ns'])


def set_pose(frame, child, x, y, yaw=0.0, stamp_ns=None):
    TF['poses'][(frame, child)] = (x, y, yaw, CLOCK['ns'] if stamp_ns is None else stamp_ns)


def drop_pose(frame, child):
    TF['poses'].pop((frame, child), None)


class SetBool:
    class Request:
        def __init__(self, data=False):
            self.data = data

    class Response:
        def __init__(self):
            self.success, self.message = False, ''


# ── servicio fromLL falso ────────────────────────────────────
class _Req:
    def __init__(self):
        self.ll_point = Msg(latitude=0.0, longitude=0.0, altitude=0.0)


class FromLL:
    Request = _Req


class _ToReq:
    def __init__(self):
        self.map_point = Msg(x=0.0, y=0.0, z=0.0)


class ToLL:
    Request = _ToReq


def _module(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


class FakeExecutor:
    """node_utils.spin_with_cleanup gira con spin_once; aca el primer giro equivale a Ctrl+C (sale enseguida y corre
    el apagado, como hacia el spin falso)."""

    def add_node(self, node):
        self.node = node

    def spin_once(self, timeout_sec=None):
        raise KeyboardInterrupt


def install():
    """Instala los modulos falsos y un time.monotonic/sleep controlados."""
    _module('rclpy', init=lambda *a, **k: None, spin=lambda n: None, try_shutdown=lambda: None,
            ok=lambda: True)
    _module('rclpy.node', Node=NodeBase)
    _module('rclpy.serialization', deserialize_message=lambda data, typ: data)   # raw=True entrega el objeto tal cual
    _module('rclpy.executors', ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}),
            SingleThreadedExecutor=FakeExecutor)
    _module('rclpy.qos', DurabilityPolicy=Msg(TRANSIENT_LOCAL=1), HistoryPolicy=Msg(KEEP_LAST=1),
            QoSProfile=lambda **k: None, ReliabilityPolicy=Msg(RELIABLE=1),
            qos_profile_sensor_data=None)
    _module('rclpy.signals', SignalHandlerOptions=Msg(NO=0))
    _module('rclpy.time', Time=Time)
    _module('geometry_msgs')
    _module('geometry_msgs.msg', Twist=Twist, TransformStamped=TransformStamped,
            PolygonStamped=PolygonStamped, Point32=Point32, PoseStamped=PoseStamped, Point=Point32,
            PoseArray=PoseArray, Pose=Pose)
    _module('visualization_msgs')
    _module('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    _module('nav_msgs')
    _module('nav_msgs.msg', Path=Path)
    _module('sensor_msgs')
    _module('sensor_msgs.msg', Range=Range, Imu=Imu, BatteryState=BatteryState)
    _module('std_msgs')
    _module('std_msgs.msg', Bool=Bool, String=String, Float32=Float32, Int32=Int32)
    _module('tf2_ros', Buffer=Buffer, TransformException=TransformException,
            TransformListener=TransformListener, TransformBroadcaster=Broadcaster)
    _module('std_srvs')
    _module('std_srvs.srv', SetBool=SetBool)
    _module('robot_localization')
    _module('robot_localization.srv', FromLL=FromLL, ToLL=ToLL)
    _time.monotonic = lambda: MONO['t']
    _time.sleep = lambda s: None


def advance(seconds):
    CLOCK['ns'] += int(seconds * 1e9)
    MONO['t'] += seconds
