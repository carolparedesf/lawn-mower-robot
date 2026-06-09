// codigo .cpp
#include "Arduino.h"
#include "pid.h"

// Constructor
PID::PID(int min_pwm, int max_pwm, float kp, float ki, float kd, float dt, float integral_max):
	minPWM_(min_pwm),
	maxPWM_(max_pwm),
	kp_(kp),
	ki_(ki),
	kd_(kd),
	dt_(dt),
	integralMax_(integral_max),
	integral_(0.0f),
	derivative_(0.0f),
	prev_measured_(0.0f)
{}

// compute - calcula salida del PID
float PID::compute(float reference, float measured)
{
	float error;
	float out;
	
	error = reference - measured;
	integral_ += error*dt_;
	// Acción integral con anti-windup
	integral_ = constrain(integral_, -integralMax_, integralMax_);
	// Acción derivativa sobre la medida
	derivative_= (measured - prev_measured_)/dt_;
	prev_measured_ = measured;
	
	if(reference == 0.0f && fabsf(measured) <= 0.01f)
	{
		reset();
	}
	
	out = (kp_*error) + (ki_*integral_) + (kd_*derivative_);
	
	return out;
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
}

// ajuste en tiempo real de ganancias
void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
	reset();
}