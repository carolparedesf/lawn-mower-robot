#!/usr/bin/env python3
"""Control de motores de corte con BTS7960.
Un solo sentido de giro: el LPWM de cada driver va fijo a GND y solo se usa RPWM.
Topics:
  /cutter/cmd   std_msgs/Float32  0.0 - 1.0 (duty)
  /trimmer/cmd  std_msgs/Float32  0.0 - 1.0 (duty)
  /estop        std_msgs/Bool     True = corte inmediato
"""
import pigpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

PWM_FREQ = 2000       # Hz
RAMP_PER_S = 0.8      # variacion de duty por segundo al acelerar (arranque suave)
WATCHDOG_S = 0.5      # sin comandos en este tiempo -> motor a 0
PERIOD_S = 0.02       # lazo a 50 Hz

# nombre: (GPIO con PWM por hardware, GPIO de EN)
MOTORS = {
    'cutter': (12, 5),
    'trimmer': (13, 6),
}


class CuttingMotors(Node):
    def __init__(self):
        super().__init__('cutting_motors')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpiod no esta corriendo (sudo systemctl start pigpiod)')

        now = self.get_clock().now()
        self.target = {n: 0.0 for n in MOTORS}
        self.duty = {n: 0.0 for n in MOTORS}
        self.last_cmd = {n: now for n in MOTORS}
        self.estop = False

        for name, (pwm, en) in MOTORS.items():
            self.pi.hardware_PWM(pwm, PWM_FREQ, 0)
            self.pi.set_mode(en, pigpio.OUTPUT)
            self.pi.write(en, 1)
            self.create_subscription(
                Float32, f'/{name}/cmd',
                lambda msg, n=name: self.on_cmd(n, msg), 10)

        self.create_subscription(Bool, '/estop', self.on_estop, 10)
        self.create_timer(PERIOD_S, self.update)
        self.get_logger().info('cutting_motors listo')

    def set_duty(self, name, value):
        self.duty[name] = value
        rc = self.pi.hardware_PWM(MOTORS[name][0], PWM_FREQ, int(value * 1_000_000))
        if rc != 0:
            self.get_logger().error(f'hardware_PWM {name} fallo: {pigpio.error_text(rc)}')

    def on_cmd(self, name, msg):
        self.target[name] = min(max(msg.data, 0.0), 1.0)
        self.last_cmd[name] = self.get_clock().now()

    def on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            for name in MOTORS:
                self.set_duty(name, 0.0)

    def update(self):
        now = self.get_clock().now()
        for name in MOTORS:
            stale = (now - self.last_cmd[name]).nanoseconds * 1e-9 > WATCHDOG_S
            goal = 0.0 if (self.estop or stale) else self.target[name]
            cur = self.duty[name]
            # sube con rampa, baja de inmediato
            new = min(goal, cur + RAMP_PER_S * PERIOD_S) if goal > cur else goal
            if new != cur:
                self.set_duty(name, new)

    def shutdown(self):
        for name, (pwm, en) in MOTORS.items():
            self.pi.hardware_PWM(pwm, PWM_FREQ, 0)
            self.pi.write(en, 0)
        self.pi.stop()


def main():
    rclpy.init()
    node = CuttingMotors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
