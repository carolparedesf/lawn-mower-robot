#include "DiffDrive.h"

DiffDrive::DiffDrive(Motor& right, Motor& left,
                     Encoder& enc_right, Encoder& enc_left,
                     const DiffDriveConfig& cfg)
    : right_(right), left_(left),
      enc_right_(enc_right), enc_left_(enc_left),
      cfg_(cfg), cmd_v_(0), cmd_w_(0),
      wr_filt_(0), wl_filt_(0),
      pwm_r_(0), pwm_l_(0), buf_idx_(0), last_update_ms_(0)
{
    odom_ = {0, 0, 0, 0, 0};
    memset(buf_r_, 0, sizeof(buf_r_));
    memset(buf_l_, 0, sizeof(buf_l_));
}

void DiffDrive::begin()
{
    right_.begin();
    left_.begin();
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
}

void DiffDrive::update()
{
    // ── dt real ─────────────────────────────────────
    unsigned long now = millis();
    float dt_real = (now - last_update_ms_) / 1000.0f;
    last_update_ms_ = now;
    
    // Protección contra dt muy pequeño o muy grande
    if (dt_real < 0.005f || dt_real > 0.1f) dt_real = cfg_.dt;
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
    float u_r = right_.compute(wr_ref, wr_filt_);
    float u_l = left_.compute(wl_ref, wl_filt_);

    u_r = constrain(u_r, -DIFFDRIVE_PWM_LIMIT, DIFFDRIVE_PWM_LIMIT);
    u_l = constrain(u_l, -DIFFDRIVE_PWM_LIMIT, DIFFDRIVE_PWM_LIMIT);

    pwm_r_ = right_.applyDeadzone(u_r);
    pwm_l_ = left_.applyDeadzone(u_l);

    right_.setPWM(pwm_r_);
    left_.setPWM(pwm_l_);

    // ── Odometría ────────────────────────────────────
    updateOdometry(wr_filt_ * cfg_.wheel_radius,
                   wl_filt_ * cfg_.wheel_radius);
}

void DiffDrive::updateOdometry(float vr_ms, float vl_ms)
{
    float vx = (vr_ms + vl_ms) / 2.0f;
    float wz = (vr_ms - vl_ms) / cfg_.base_width;

    float dtheta = wz * cfg_.dt;
    float dx     = vx * cosf(odom_.theta + dtheta / 2.0f) * cfg_.dt;
    float dy     = vx * sinf(odom_.theta + dtheta / 2.0f) * cfg_.dt;

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
