#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "pid.h"

#define MOTOR_PWM_FREQ 25000
#define MOTOR_PWM_RES  8

class Motor {
public:
    Motor(uint8_t pwm_a, uint8_t pwm_b, uint8_t ch_a, uint8_t ch_b,
          int pwm_min, int pwm_max,
          float kp, float ki, float kd,
          float dt, float integral_max);

    void  begin();
    void  setPWM(int pwm);
    void  stop();
    int   applyDeadzone(float u);
    float compute(float ref_rads, float meas_rads);
    void  updateGains(float kp, float ki, float kd);

private:
    uint8_t pwm_a_;
    uint8_t pwm_b_;
    uint8_t ch_a_;
    uint8_t ch_b_;
    int     pwm_max_;
    PID     pid_;
};

#endif
