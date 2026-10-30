"""Utilidades de TF compartidas por los nodos de navegacion."""
import math

from rclpy.time import Time
from tf2_ros import TransformException

from lawnbot_nav.pure_pursuit import Pose


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lookup_pose(node, tf_buffer, frame_id: str, base_frame: str, pose_timeout: float):
    """Pose de base_frame en frame_id, o None si falta la TF o es mas vieja que pose_timeout.

    Una transformada con stamp 0 se considera estatica (siempre vigente).
    """
    try:
        tf = tf_buffer.lookup_transform(frame_id, base_frame, Time())
    except TransformException:
        return None
    stamp = tf.header.stamp
    if stamp.sec != 0 or stamp.nanosec != 0:
        age = (node.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
        if age > pose_timeout:
            return None
    t = tf.transform
    return Pose(t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
