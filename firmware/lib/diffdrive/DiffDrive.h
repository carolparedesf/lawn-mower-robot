#ifndef DIFFDRIVE_H
#define DIFFDRIVE_H

#include <Arduino.h>
#include "Motor.h"
#include "encoder.h"

#define DIFFDRIVE_FILT_N    4
#define DIFFDRIVE_PWM_LIMIT 160
// Referencia de rueda por debajo de este valor (rad/s) se considera "orden 0"
#define DIFFDRIVE_REF_ZERO_RADS 0.02f
// Ventana valida para el dt real (s): por debajo se espera al siguiente ciclo,
// por encima el lazo estuvo detenido y se resincroniza sin controlar.
#define DIFFDRIVE_DT_MIN_S 0.005f
#define DIFFDRIVE_DT_MAX_S 0.25f

struct DiffDriveConfig {
    float wheel_radius;  // m
    float base_width;    // m
    float dt;            // s
};

struct Odometry {
    float x;      // m
    float y;      // m
    float theta;  // rad
    float vx;     // m/s
    float wz;     // rad/s
};

class DiffDrive {
public:
    DiffDrive(Motor& right, Motor& left,
              Encoder& enc_right, Encoder& enc_left,
              const DiffDriveConfig& cfg);

    void begin();
    void setCmdVel(float linear_x, float angular_z);
    void stop();
    // drive_enabled = false (agente caido, e-stop, bateria baja...): PWM 0 e
    // integral 0, pero se siguen leyendo encoders y la odometria.
    void update(bool drive_enabled = true);

    Odometry getOdometry()   const { return odom_; }
    float    getRightSpeed() const { return wr_filt_; }
    float    getLeftSpeed()  const { return wl_filt_; }
    int      getRightPWM()   const { return pwm_r_; }
    int      getLeftPWM()    const { return pwm_l_; }

private:
    Motor&          right_;
    Motor&          left_;
    Encoder&        enc_right_;
    Encoder&        enc_left_;
    DiffDriveConfig cfg_;
    Odometry        odom_;

    float cmd_v_;
    float cmd_w_;
    float wr_filt_;
    float wl_filt_;
    int   pwm_r_;
    int   pwm_l_;

    float buf_r_[DIFFDRIVE_FILT_N];
    float buf_l_[DIFFDRIVE_FILT_N];
    int   buf_idx_;
    
    unsigned long last_update_us_;

    int   runWheel(Motor& m, float ref, float meas, float dt);
    int   idleWheel(Motor& m);
    float movingAvg(float* buf, float val);
    void  updateOdometry(float vr_ms, float vl_ms, float dt);
};

#endif
