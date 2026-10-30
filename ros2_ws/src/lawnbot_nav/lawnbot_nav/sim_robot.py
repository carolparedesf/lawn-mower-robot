"""Robot diferencial simulado (sin dependencias de ROS). Lo usan fake_robot y los tests."""
import math


class SimRobot:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, base_width=0.355,
                 wheel_deadzone=0.0, tau=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.base_width = base_width
        self.wheel_deadzone = wheel_deadzone   # [m/s] por debajo, la rueda no gira
        self.tau = tau                         # constante de tiempo del lazo de velocidad [s]
        self.v = 0.0
        self.w = 0.0

    def step(self, v_cmd, w_cmd, dt):
        # Como el firmware: comandos casi nulos detienen el robot
        if abs(v_cmd) < 0.01 and abs(w_cmd) < 0.01:
            v_cmd = w_cmd = 0.0
        half = self.base_width / 2.0
        vr = v_cmd + w_cmd * half
        vl = v_cmd - w_cmd * half
        if abs(vr) < self.wheel_deadzone:
            vr = 0.0
        if abs(vl) < self.wheel_deadzone:
            vl = 0.0
        v_target = (vr + vl) / 2.0
        w_target = (vr - vl) / self.base_width
        if self.tau > 0.0:
            k = dt / (self.tau + dt)
            self.v += k * (v_target - self.v)
            self.w += k * (w_target - self.w)
        else:
            self.v, self.w = v_target, w_target
        mid = self.yaw + self.w * dt / 2.0
        self.x += self.v * math.cos(mid) * dt
        self.y += self.v * math.sin(mid) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt),
                              math.cos(self.yaw + self.w * dt))
