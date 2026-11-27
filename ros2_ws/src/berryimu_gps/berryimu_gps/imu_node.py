#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import smbus2

from berryimu_gps.imu_filter import GAIN_REF_HZ, rate_scaled_gain

AG_ADDR = 0x6A
M_ADDR = 0x1C

WHO_AM_I_AG = 0x0F
CTRL_REG1_G = 0x10
CTRL_REG6_XL = 0x20
OUT_X_L_G = 0x18
OUT_X_L_XL = 0x28

WHO_AM_I_M = 0x0F
CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
OUT_X_L_M = 0x28

ACC_SCALE = 0.000061
GYRO_SCALE = 0.00875
MAG_SCALE = 0.00014
GRAVITY = 9.80665
DEG2RAD = math.pi / 180.0


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class BerryIMUDriver(Node):
    def __init__(self):
        super().__init__('berryimu_driver')

        self.declare_parameter('gyro_bias_x', 0.0)
        self.declare_parameter('gyro_bias_y', 0.0)
        self.declare_parameter('gyro_bias_z', 0.0)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)
        self.declare_parameter('accel_scale_x', 1.0)
        self.declare_parameter('accel_scale_y', 1.0)
        self.declare_parameter('accel_scale_z', 1.0)
        self.declare_parameter('mag_offset_x', 0.0)
        self.declare_parameter('mag_offset_y', 0.0)
        self.declare_parameter('mag_offset_z', 0.0)
        # 25 Hz: el EKF solo usa la velocidad angular z y corre a 15 / 10 Hz; cada
        # mensaje de /imu/data cuesta ~3 ms de CPU a cada suscriptor rclpy de la Pi
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('auto_calibrate_gyro', False)

        self.declare_parameter('frame_id', 'BerryIMU_Link')
        # Signos del magnetometro respecto al marco del acelerometro. Para el
        # LSM9DS1 el codigo de ozzmaker (berryIMU.py, rama LSM9DS1) usa los
        # valores crudos sin invertir ningun eje. Confirmar con la prueba de rumbo.
        self.declare_parameter('mag_sign_x', 1.0)
        self.declare_parameter('mag_sign_y', 1.0)
        self.declare_parameter('mag_sign_z', 1.0)
        # Filtro complementario: correccion por muestra a gain_ref_hz. Con otra
        # frecuencia se escala con el dt real (imu_filter.py), asi la constante de
        # tiempo no depende de publish_rate_hz (accel ~1 s, mag ~2 s)
        self.declare_parameter('accel_gain', 0.02)
        self.declare_parameter('mag_gain', 0.01)
        self.declare_parameter('gain_ref_hz', GAIN_REF_HZ)
        self.declare_parameter('accel_gate', 2.0)  # m/s2 de desvio respecto a g
        # Covarianzas [rad^2] y [(rad/s)^2]
        self.declare_parameter('roll_pitch_variance', 0.01)
        self.declare_parameter('yaw_variance', 0.1)
        self.declare_parameter('yaw_variance_no_mag', 1.0e3)
        self.declare_parameter('gyro_variance', 4.0e-4)
        self.declare_parameter('accel_variance', 0.01)

        self.gx_bias = self.get_parameter('gyro_bias_x').value
        self.gy_bias = self.get_parameter('gyro_bias_y').value
        self.gz_bias = self.get_parameter('gyro_bias_z').value
        self.accel_offset = [
            self.get_parameter('accel_offset_x').value,
            self.get_parameter('accel_offset_y').value,
            self.get_parameter('accel_offset_z').value,
        ]
        self.accel_scale = [
            self.get_parameter('accel_scale_x').value,
            self.get_parameter('accel_scale_y').value,
            self.get_parameter('accel_scale_z').value,
        ]
        self.mag_offset = [
            self.get_parameter('mag_offset_x').value,
            self.get_parameter('mag_offset_y').value,
            self.get_parameter('mag_offset_z').value,
        ]
        self.mag_sign = [
            self.get_parameter('mag_sign_x').value,
            self.get_parameter('mag_sign_y').value,
            self.get_parameter('mag_sign_z').value,
        ]
        self.frame_id = self.get_parameter('frame_id').value
        self.accel_gain = self.get_parameter('accel_gain').value
        self.mag_gain = self.get_parameter('mag_gain').value
        self.gain_ref_hz = self.get_parameter('gain_ref_hz').value
        self.accel_gate = self.get_parameter('accel_gate').value
        self.rp_var = self.get_parameter('roll_pitch_variance').value
        self.yaw_var = self.get_parameter('yaw_variance').value
        self.yaw_var_no_mag = self.get_parameter('yaw_variance_no_mag').value
        self.gyro_var = self.get_parameter('gyro_variance').value
        self.accel_var = self.get_parameter('accel_variance').value

        # Estado del filtro de orientacion (ENU, x adelante, y izquierda, z arriba)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.orientation_ready = False
        self.last_t = None

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.bus = smbus2.SMBus(1)
        self.sensors_status = {'imu': False, 'mag': False}

        self.init_sensors()

        if self.sensors_status['imu'] and self.get_parameter('auto_calibrate_gyro').value:
            self.calibrate_gyro()

        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.publish_all)
        self.get_logger().info(f'--- BerryIMU Driver a {rate} Hz, frame {self.frame_id} ---')

    def init_sensors(self):
        try:
            who_ag = self.bus.read_byte_data(AG_ADDR, WHO_AM_I_AG)
            if who_ag != 0x68:
                self.get_logger().warn(f'WHO_AM_I inesperado en 0x{AG_ADDR:02X}: 0x{who_ag:02X}')
            self.bus.write_byte_data(AG_ADDR, CTRL_REG1_G, 0xC0)
            self.bus.write_byte_data(AG_ADDR, CTRL_REG6_XL, 0xC0)
            self.sensors_status['imu'] = True
            self.get_logger().info(f'IMU (0x{AG_ADDR:02X}): OK, WHO_AM_I=0x{who_ag:02X}')
        except Exception as e:
            self.get_logger().error(f'IMU (0x{AG_ADDR:02X}): NO DETECTADA — {e}')

        try:
            who_m = self.bus.read_byte_data(M_ADDR, WHO_AM_I_M)
            if who_m != 0x3D:
                self.get_logger().warn(f'WHO_AM_I_M inesperado en 0x{M_ADDR:02X}: 0x{who_m:02X}')
            self.bus.write_byte_data(M_ADDR, CTRL_REG1_M, 0x70)
            self.bus.write_byte_data(M_ADDR, CTRL_REG2_M, 0x00)
            self.bus.write_byte_data(M_ADDR, CTRL_REG3_M, 0x00)
            self.sensors_status['mag'] = True
            self.get_logger().info(f'Magnetometro (0x{M_ADDR:02X}): OK, WHO_AM_I=0x{who_m:02X}')
        except Exception as e:
            self.get_logger().error(f'Magnetometro (0x{M_ADDR:02X}): NO DETECTADO — {e}')

    def calibrate_gyro(self, samples=500):
        self.get_logger().info(f'Calibrando giroscopio ({samples} muestras)...')
        sum_x = sum_y = sum_z = 0
        valid = 0
        for _ in range(samples):
            gx, gy, gz = self.read_raw_block(AG_ADDR, OUT_X_L_G)
            time.sleep(0.005)  # ~ODR del giroscopio; evita muestras repetidas
            sum_x += gx
            sum_y += gy
            sum_z += gz
            valid += 1
        if valid > 0:
            self.gx_bias = sum_x / valid
            self.gy_bias = sum_y / valid
            self.gz_bias = sum_z / valid
            self.get_logger().info(
                f'Bias gyro RAW: gx={self.gx_bias:.2f}, gy={self.gy_bias:.2f}, gz={self.gz_bias:.2f}'
            )

    def read_raw_block(self, addr, reg, count=6):
        try:
            data = self.bus.read_i2c_block_data(addr, reg | 0x80, count)
            vals = []
            for i in range(0, count, 2):
                word = (data[i + 1] << 8) | data[i]
                vals.append(word - 65536 if word >= 0x8000 else word)
            return vals
        except Exception as e:
            self.get_logger().error(f'I2C error 0x{addr:02X} reg 0x{reg:02X}: {e}')
            if addr == AG_ADDR:
                self.sensors_status['imu'] = False
            elif addr == M_ADDR:
                self.sensors_status['mag'] = False
            return [0, 0, 0]

    # ── Orientacion ──────────────────────────────────────────

    @staticmethod
    def accel_roll_pitch(ax, ay, az):
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return roll, pitch

    @staticmethod
    def mag_yaw(mx, my, mz, roll, pitch):
        """Yaw ENU (0 = mirando al este, antihorario positivo) con compensacion de inclinacion."""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        xh = mx * cp + my * sr * sp + mz * cr * sp
        yh = my * cr - mz * sr
        return math.atan2(xh, yh)

    def update_orientation(self, accel, gyro, mag, dt):
        ax, ay, az = accel
        gx, gy, gz = gyro

        roll_acc, pitch_acc = self.accel_roll_pitch(ax, ay, az)

        if not self.orientation_ready:
            self.roll, self.pitch = roll_acc, pitch_acc
            self.yaw = self.mag_yaw(*mag, self.roll, self.pitch) if mag else 0.0
            self.orientation_ready = True
            return

        # Integracion del giroscopio (tasas de Euler ZYX)
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        cp = max(math.cos(self.pitch), 0.1)
        tp = math.tan(self.pitch)
        self.roll += (gx + sr * tp * gy + cr * tp * gz) * dt
        self.pitch += (cr * gy - sr * gz) * dt
        self.yaw += ((sr * gy + cr * gz) / cp) * dt

        # Correccion roll/pitch con el acelerometro, solo si no hay aceleracion fuerte
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if abs(a_norm - GRAVITY) < self.accel_gate:
            k = rate_scaled_gain(self.accel_gain, dt, self.gain_ref_hz)
            self.roll += k * wrap_pi(roll_acc - self.roll)
            self.pitch += k * wrap_pi(pitch_acc - self.pitch)

        # Correccion de yaw con el magnetometro
        if mag:
            yaw_m = self.mag_yaw(*mag, self.roll, self.pitch)
            k = rate_scaled_gain(self.mag_gain, dt, self.gain_ref_hz)
            self.yaw += k * wrap_pi(yaw_m - self.yaw)

        self.roll = wrap_pi(self.roll)
        self.pitch = wrap_pi(self.pitch)
        self.yaw = wrap_pi(self.yaw)

    def publish_all(self):
        stamp = self.get_clock().now()
        now = stamp.to_msg()
        t = time.monotonic()
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.2)
        self.last_t = t

        mag_body = None
        if self.sensors_status['mag']:
            mx, my, mz = self.read_raw_block(M_ADDR, OUT_X_L_M)
            mag_gauss = [
                (mx * MAG_SCALE - self.mag_offset[0]) * self.mag_sign[0],
                (my * MAG_SCALE - self.mag_offset[1]) * self.mag_sign[1],
                (mz * MAG_SCALE - self.mag_offset[2]) * self.mag_sign[2],
            ]
            mag_body = mag_gauss

            msg = MagneticField()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.magnetic_field.x = mag_gauss[0] * 1e-4
            msg.magnetic_field.y = mag_gauss[1] * 1e-4
            msg.magnetic_field.z = mag_gauss[2] * 1e-4
            msg.magnetic_field_covariance[0] = 1e-6
            msg.magnetic_field_covariance[4] = 1e-6
            msg.magnetic_field_covariance[8] = 1e-6
            self.mag_pub.publish(msg)

        if not self.sensors_status['imu']:
            return

        ax, ay, az = self.read_raw_block(AG_ADDR, OUT_X_L_XL)
        gx, gy, gz = self.read_raw_block(AG_ADDR, OUT_X_L_G)
        gx -= self.gx_bias
        gy -= self.gy_bias
        gz -= self.gz_bias

        accel = (
            (ax * ACC_SCALE - self.accel_offset[0]) * self.accel_scale[0] * GRAVITY,
            (ay * ACC_SCALE - self.accel_offset[1]) * self.accel_scale[1] * GRAVITY,
            (az * ACC_SCALE - self.accel_offset[2]) * self.accel_scale[2] * GRAVITY,
        )
        gyro = (
            gx * GYRO_SCALE * DEG2RAD,
            gy * GYRO_SCALE * DEG2RAD,
            gz * GYRO_SCALE * DEG2RAD,
        )

        self.update_orientation(accel, gyro, mag_body, dt)

        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro

        qx, qy, qz, qw = euler_to_quat(self.roll, self.pitch, self.yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance[0] = self.rp_var
        msg.orientation_covariance[4] = self.rp_var
        msg.orientation_covariance[8] = (
            self.yaw_var if self.sensors_status['mag'] else self.yaw_var_no_mag)
        msg.angular_velocity_covariance[0] = self.gyro_var
        msg.angular_velocity_covariance[4] = self.gyro_var
        msg.angular_velocity_covariance[8] = self.gyro_var
        msg.linear_acceleration_covariance[0] = self.accel_var
        msg.linear_acceleration_covariance[4] = self.accel_var
        msg.linear_acceleration_covariance[8] = self.accel_var

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BerryIMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
