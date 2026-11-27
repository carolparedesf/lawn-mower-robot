"""Pose del robot desde un nav_msgs/Odometry (por defecto /odometry/global de ekf_global), sin TransformListener.

Por que: en la Raspberry Pi 4 (Humble) cada mensaje que recibe un nodo rclpy cuesta ~3 ms de CPU. Un
TransformListener recibe todo /tf (ekf_local a 15 Hz + ekf_global a 10 Hz = 25 Hz) aunque el nodo use la pose a
10 Hz; /odometry/global llega a 10 Hz y es la misma pose que la TF map -> base_footprint (la calcula ekf_global).

En simulacion fake_robot publica /odometry/global con la pose simulada (frame `map`).
"""
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from lawnbot_nav.pure_pursuit import Pose
from lawnbot_nav.tf_utils import yaw_from_quaternion

DEFAULT_TOPIC = 'odometry/global'


class OdomPose:
    """Guarda el ultimo Odometry; get() lo devuelve como Pose si esta en los frames esperados y no es viejo."""

    def __init__(self, node, topic: str, frame_id: str, base_frame: str):
        self._node = node
        self._frames = (frame_id, base_frame)
        self._msg = None
        self._warned_frames = None
        self.topic = topic
        node.create_subscription(Odometry, topic, self._on_msg, qos_profile_sensor_data)

    def _on_msg(self, msg):
        self._msg = msg

    def get(self, pose_timeout: float):
        """Pose de base_frame en frame_id, o None si no llego, viene en otros frames o es mas vieja que pose_timeout.

        Un stamp 0 se considera vigente (igual que lookup_pose con una TF estatica).
        """
        msg = self._msg
        if msg is None:
            return None
        frames = (msg.header.frame_id.lstrip('/'), msg.child_frame_id.lstrip('/'))
        if frames != self._frames:
            if frames != self._warned_frames:
                self._warned_frames = frames
                self._node.get_logger().warn(
                    f'/{self.topic} viene en {frames[0]} -> {frames[1]}, se esperaba '
                    f'{self._frames[0]} -> {self._frames[1]}: sin pose')
            return None
        stamp = msg.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            age = (self._node.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
            if age > pose_timeout:
                return None
        p = msg.pose.pose
        return Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
