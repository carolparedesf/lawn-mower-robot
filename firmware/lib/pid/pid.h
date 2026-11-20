// codigo .h
#ifndef PID_H
#define PID_H
#include "Arduino.h"
class PID
{
	public:
		// integral_max_pwm: tope del TERMINO integral (Ki*integral) en unidades de PWM
		PID(int min_pwm, int max_pwm, float kp, float ki, float kd, float dt, float integral_max_pwm);
		float compute(float reference, float measured, float dt);   // dt real [s]
		int apply_deadzone(float out);
		void reset();
		void updateConstants(float kp, float ki, float kd);
		// Limite de la salida de compute(); se usa para detectar saturacion
		void setOutputLimit(float limit);
	private:
		void updateIntegralLimit();
		int minPWM_;
		int maxPWM_;
		float kp_;
		float ki_;
		float kd_;
		float dt_;
		float integralMaxPwm_;   // tope del termino integral [PWM]
		float integralMax_;      // tope de la integral [rad], = integralMaxPwm_/ki_
		float outLimit_;
		float integral_;
		float derivative_;
		float prev_measured_;
		bool  has_prev_;
};
#endif
