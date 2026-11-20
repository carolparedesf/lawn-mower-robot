#include "DiffDrive.h"

DiffDrive::DiffDrive(Motor& right, Motor& left,
                     Encoder& enc_right, Encoder& enc_left,
                     const DiffDriveConfig& cfg)
    : right_(right), left_(left),
      enc_right_(enc_right), enc_left_(enc_left),
      cfg_(cfg), cmd_v_(0), cmd_w_(0),
      wr_filt_(0), wl_filt_(0),
      pwm_r_(0), pwm_l_(0), buf_idx_(0), last_update_us_(0)
{
    odom_ = {0, 0, 0, 0, 0};
    memset(buf_r_, 0, sizeof(buf_r_));
    memset(buf_l_, 0, sizeof(buf_l_));
}

void DiffDrive::begin()
{
    right_.begin();
    left_.begin();
    // El PID satura en el mismo limite que se aplica a la salida (anti-windup)
    right_.setOutputLimit(DIFFDRIVE_PWM_LIMIT);
    left_.setOutputLimit(DIFFDRIVE_PWM_LIMIT);
}

void DiffDrive::setCmdVel(float linear_x, float angular_z)
{
    cmd_v_ = linear_x;
    cmd_w_ = angular_z;
}

void DiffDrive::stop()
{
    cmd_v_ = 0;
    cmd_w_ = 0;
    right_.stop();
    left_.stop();
    right_.resetPID();
    left_.resetPID();
    pwm_r_ = 0;
    pwm_l_ = 0;
}

void DiffDrive::update(bool drive_enabled)
{
    // ── dt real (micros) ────────────────────────────
    unsigned long now_us = micros();
    float dt_real = (now_us - last_update_us_) * 1e-6f;
    bool  first   = (last_update_us_ == 0);

    // Llamada demasiado seguida: esperar (los ticks se acumulan al siguiente)
    if (!first && dt_real < DIFFDRIVE_DT_MIN_S) return;
    last_update_us_ = now_us ? now_us : 1;

    // Primera llamada o lazo detenido: consumir los ticks acumulados y no
    // controlar este ciclo (dividirlos por un dt inventado daria un pico).
    if (first || dt_real > DIFFDRIVE_DT_MAX_S) {
        enc_right_.getRadPerSec(1.0f);
        enc_left_.getRadPerSec(1.0f);
        if (!drive_enabled) {
            pwm_r_ = idleWheel(right_);
            pwm_l_ = idleWheel(left_);
        }
        return;
    }
    // ── Cinemática inversa ──────────────────────────
    float vr = cmd_v_ + (cfg_.base_width * 0.5f) * cmd_w_;
    float vl = cmd_v_ - (cfg_.base_width * 0.5f) * cmd_w_;

    float wr_ref = vr / cfg_.wheel_radius;
    float wl_ref = vl / cfg_.wheel_radius;

    // ── Leer encoders via librería Encoder ──────────
    float wr_raw = enc_right_.getRadPerSec(dt_real);
    float wl_raw = enc_left_.getRadPerSec(dt_real);

    // ── Filtro media móvil ──────────────────────────
    buf_idx_++;
    wr_filt_ = movingAvg(buf_r_, wr_raw);
    wl_filt_ = movingAvg(buf_l_, wl_raw);

    // ── PID ─────────────────────────────────────────
    // Orden 0 (o accionamiento deshabilitado): PWM 0 e integral 0. El PWM
    // minimo (zona muerta) solo se aplica cuando la orden no es 0.
    pwm_r_ = (drive_enabled && fabsf(wr_ref) >= DIFFDRIVE_REF_ZERO_RADS)
                 ? runWheel(right_, wr_ref, wr_filt_, dt_real) : idleWheel(right_);
    pwm_l_ = (drive_enabled && fabsf(wl_ref) >= DIFFDRIVE_REF_ZERO_RADS)
                 ? runWheel(left_, wl_ref, wl_filt_, dt_real) : idleWheel(left_);

    // ── Odometría ────────────────────────────────────
    updateOdometry(wr_filt_ * cfg_.wheel_radius,
                   wl_filt_ * cfg_.wheel_radius,
                   dt_real);
}

int DiffDrive::runWheel(Motor& m, float ref, float meas, float dt)
{
    float u = m.compute(ref, meas, dt);
    u = constrain(u, -DIFFDRIVE_PWM_LIMIT, DIFFDRIVE_PWM_LIMIT);
    int pwm = m.applyDeadzone(u);
    m.setPWM(pwm);
    return pwm;
}

int DiffDrive::idleWheel(Motor& m)
{
    m.stop();
    m.resetPID();
    return 0;
}

void DiffDrive::updateOdometry(float vr_ms, float vl_ms, float dt)
{
    float vx = (vr_ms + vl_ms) / 2.0f;
    float wz = (vr_ms - vl_ms) / cfg_.base_width;

    float dtheta = wz * dt;
    float dx     = vx * cosf(odom_.theta + dtheta / 2.0f) * dt;
    float dy     = vx * sinf(odom_.theta + dtheta / 2.0f) * dt;

    odom_.x     += dx;
    odom_.y     += dy;
    odom_.theta += dtheta;
    odom_.vx     = vx;
    odom_.wz     = wz;

    while (odom_.theta >  PI) odom_.theta -= 2.0f * PI;
    while (odom_.theta < -PI) odom_.theta += 2.0f * PI;
}

float DiffDrive::movingAvg(float* buf, float val)
{
    buf[buf_idx_ % DIFFDRIVE_FILT_N] = val;
    float sum = 0;
    for (int i = 0; i < DIFFDRIVE_FILT_N; i++) sum += buf[i];
    return sum / DIFFDRIVE_FILT_N;
}
