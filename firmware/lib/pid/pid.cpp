// codigo .cpp
#include "Arduino.h"
#include "pid.h"

// Constructor
PID::PID(int min_pwm, int max_pwm, float kp, float ki, float kd, float dt, float integral_max_pwm):
	minPWM_(min_pwm),
	maxPWM_(max_pwm),
	kp_(kp),
	ki_(ki),
	kd_(kd),
	dt_(dt),
	integralMaxPwm_(integral_max_pwm),
	integralMax_(0.0f),
	outLimit_((float)max_pwm),
	integral_(0.0f),
	derivative_(0.0f),
	prev_measured_(0.0f),
	has_prev_(false)
{
	updateIntegralLimit();
}

// tope de la integral [rad] para que Ki*integral no supere integralMaxPwm_
void PID::updateIntegralLimit()
{
	integralMax_ = (ki_ > 0.0f) ? (integralMaxPwm_ / ki_) : 0.0f;
}

void PID::setOutputLimit(float limit)
{
	outLimit_ = limit;
}

// compute - calcula salida del PID con el dt real medido [s]
// (el llamador no lo invoca con orden 0: ahi hace PWM 0 + reset())
float PID::compute(float reference, float measured, float dt)
{
	if (dt <= 0.0f) dt = dt_;   // dt_ nominal solo como respaldo
	float error = reference - measured;

	// Acción derivativa sobre la medida (0 en la primera muestra tras reset)
	derivative_ = has_prev_ ? (measured - prev_measured_) / dt : 0.0f;
	prev_measured_ = measured;
	has_prev_ = true;

	float p = kp_ * error;
	float d = kd_ * derivative_;

	// Integral candidata, limitada para que su termino no pase de integralMaxPwm_
	float integral_new = constrain(integral_ + error * dt, -integralMax_, integralMax_);

	// Anti-windup (integracion condicional): si la salida ya esta saturada y el
	// error empuja mas hacia la saturacion, no acumular.
	float out_unsat = p + ki_ * integral_new + d;
	bool pushing_sat = (fabsf(out_unsat) > outLimit_) && (out_unsat * error > 0.0f);
	if (!pushing_sat) integral_ = integral_new;

	float out = p + ki_ * integral_ + d;
	return constrain(out, -outLimit_, outLimit_);
}

// suma umbral minimo para vencer friccion
int PID::apply_deadzone(float out) 
{
  if (out >  0.5f) return constrain((int)out + minPWM_,  0, maxPWM_);
  if (out < -0.5f) return constrain((int)out - minPWM_, -maxPWM_, 0);
  return 0;
}

// limpia estado interno del controlador
void PID::reset()
{
	integral_ = 0.0f;
	prev_measured_ = 0.0f;
	has_prev_ = false;
}

// ajuste en tiempo real de ganancias
void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
	updateIntegralLimit();
	reset();
}