#include "Motor.h"

Motor::Motor(uint8_t pwm_a, uint8_t pwm_b, uint8_t ch_a, uint8_t ch_b,
             int pwm_min, int pwm_max,
             float kp, float ki, float kd,
             float dt, float integral_max)
    : pwm_a_(pwm_a), pwm_b_(pwm_b), 
      ch_a_(ch_a), ch_b_(ch_b), 
      pwm_max_(pwm_max),
      pid_(pwm_min, pwm_max, kp, ki, kd, dt, integral_max)
{}

void Motor::begin()
{
    /*ledcAttach(pwm_a_, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttach(pwm_b_, MOTOR_PWM_FREQ, MOTOR_PWM_RES);*/
    ledcSetup(ch_a_, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(pwm_a_, ch_a_);
    ledcSetup(ch_b_, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(pwm_b_, ch_b_);
    stop();
}

void Motor::setPWM(int pwm)
{
    pwm = constrain(pwm, -pwm_max_, pwm_max_);
    if (pwm > 0) {
        ledcWrite(ch_a_, pwm);
        ledcWrite(ch_b_, 0);
    } else if (pwm < 0) {
        ledcWrite(ch_a_, 0);
        ledcWrite(ch_b_, abs(pwm));
    } else {
        stop();
    }
}

void Motor::stop()
{
    ledcWrite(ch_a_, 0);
    ledcWrite(ch_b_, 0);
}

float Motor::compute(float ref_rads, float meas_rads)
{
    return pid_.compute(ref_rads, meas_rads);
}

int Motor::applyDeadzone(float u)
{
    return pid_.apply_deadzone(u);
}

void Motor::updateGains(float kp, float ki, float kd)
{
    pid_.updateConstants(kp, ki, kd);
}
